/*
*  <legal_notice>
*  * BSD License 2.0
*  *
*  * Copyright (c) 2021, MaxLinear, Inc.
*  *
*  * Redistribution and use in source and binary forms, with or without
*  * modification, are permitted provided that the following conditions are met:
*  * 1. Redistributions of source code must retain the above copyright notice, 
*  *    this list of conditions and the following disclaimer.
*  * 2. Redistributions in binary form must reproduce the above copyright notice, 
*  *    this list of conditions and the following disclaimer in the documentation 
*  *    and/or other materials provided with the distribution.
*  * 3. Neither the name of the copyright holder nor the names of its contributors 
*  *    may be used to endorse or promote products derived from this software 
*  *    without specific prior written permission.
*  *
*  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
*  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
*  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
*  * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
*  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
*  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
*  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
*  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
*  * POSSIBILITY OF SUCH DAMAGE.
*  </legal_notice>
*/

/**
 * @addtogroup vector_boost
 * @{
 **/

/**
 * @file vb_LCMP_com.c
 * @brief Communications over LCMP implemented to vector boost
 *
 * @internal
 *
 * @author
 * @date 07/01/2015
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>
#include <linux/if_arp.h>
#include <errno.h>
#include <pthread.h>

#include "vb_LCMP_com.h"
#include "vb_LCMP_dbg.h"
#include "vb_log.h"
#include "vb_LCMP_socket.h"
#include "vb_thread.h"
#include "vb_priorities.h"

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */


/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define LCMP_THREAD_NAME      ("LCMPRx")
#define ETH_LCMP_PROTOCOL     (0x22E3)    /* Every packet (be careful!!!) */

/* see linux/if_ether.h */
#define ETH_HEADER_LEN        (ETH_HLEN)      /* Total octets in header.       */
#define ETH_MAX_FRAME_LEN     (ETH_FRAME_LEN) /* Max. octets in frame sans FCS */

#define BUF_SIZE              (ETH_MAX_FRAME_LEN)
#define SRCMACOFFSET          (ETH_ALEN)

#define SOCKET_RETRY_TIMEOUT  (1000) //ms

typedef struct __attribute__ ((packed))
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT32U Length : 12         __attribute__ ((packed));
  INT32U OPCODE : 12         __attribute__ ((packed));
  INT32U stdVersion: 8       __attribute__ ((packed));
  INT32U numberSegments : 4  __attribute__ ((packed));
  INT32U segment : 4         __attribute__ ((packed));
  INT32U seqNumber : 16      __attribute__ ((packed));
  INT32U repNumber : 4       __attribute__ ((packed));
  INT32U fsb : 1             __attribute__ ((packed));
  INT32U : 3                 __attribute__ ((packed));
#else
  INT32U stdVersion: 8       __attribute__ ((packed));
  INT32U OPCODE : 12         __attribute__ ((packed));
  INT32U Length : 12         __attribute__ ((packed));
  INT32U : 3                 __attribute__ ((packed));
  INT32U fsb : 1             __attribute__ ((packed));
  INT32U repNumber : 4       __attribute__ ((packed));
  INT32U seqNumber : 16      __attribute__ ((packed));
  INT32U segment : 4         __attribute__ ((packed));
  INT32U numberSegments : 4  __attribute__ ((packed));
#endif
} t_MMH;

#define MMH_OFFSET                  (ETH_HEADER_LEN)
#define MMH_SIZE                    (sizeof(t_MMH))
#define MMPL_OFFSET                 (MMH_OFFSET + MMH_SIZE)
#define MMPL_MAX_LENGTH             (1492)

#define TIME_TO_DISCARD_SEGMENTED_FRAME (50)//ms

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct s_CallkacsLists{
  INT16U  NumCallbacks;
  t_Callbacks *head;
  t_Callbacks *tail;
} t_CallbacksLists;

typedef struct s_Segments{
  INT16U ValueLength;
  INT8U *Value;
  INT8U numSegment;
  struct s_Segments *nextSegment;
} t_Segments;

typedef struct s_SegmentedFrame{
  INT16U numSeq;
  INT8U srcMAC[ETH_ALEN];
  INT16U LCMP_Opcode;
  struct timespec timeStamp;
  t_Segments  *Segments;
  INT8U numSegmentsSaved;
  struct s_SegmentedFrame *nextSegmentedFrame;
  struct s_SegmentedFrame *prevSegmentedFrame;
} t_SegmentedFrame;

typedef struct s_SegmentedFramesLists{
  INT16U  SegmentedFrames;
  t_SegmentedFrame *head;
  t_SegmentedFrame *tail;
} t_SegmentedFramesLists;

typedef struct s_LcmpParsedInfo
{
  INT8U  *valuePtr;
  INT16U  valueLen;
  BOOLEAN isVbParam;
  BOOLEAN isForMe;
  BOOLEAN notifAck;
} t_LcmpParsedInfo;

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_t lcmpReceiveThread;
static BOOL lcmpReceiveThreadRunning;
//Sockets
static INT32S lcmpSc;

static t_CallbacksLists *lcmpCallbacksList;
static pthread_mutex_t lcmpMutexCallbacksList = PTHREAD_MUTEX_INITIALIZER;
static t_SegmentedFramesLists *lcmpSegmentedFramesLists;
static char ifLcmp[IFNAMSIZ] = {" "};

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/**
 * @brief Creates a callbacks list
 * @return Pointer to new callbacks list
**/
static t_CallbacksLists *LcmpCallbackListCreate( void );

/**
 * @brief Insert a callback in callbak list
 * @param[in/out] newCallback
**/
static void LcmpCallbackInsert( t_Callbacks *newCallback );

/**
 * @brief Execute a read operation on LCMP
 * @param[in] Callback Function which implement the callback
 * @param[in] OpcodeFilter Opcode to receive
 * @param[in] ReceivedValues Pointer to values list
 * @param[in] expectedHGFOpcode HGFOpcode to receive
 * @param[in] transactionId Transaction Id (if != 0 only Rx messages with this transaction Id will be processed)
 * @param[in] markTimeStamp Indication to activate timestamps in this frames
 * @return Pointer to new callback.
**/
static t_Callbacks *LcmpCallbackAdd( BOOL (* callback)(const INT8U *, INT16U, t_CallbackData *),
    t_LCMP_OPCODE opcodeFilter,
    t_HGF_TLV expectedHgfOpcode, INT8U paramID, INT16U transactionId, BOOL markTimeStamp);

/**
 * @brief Delete a callback from callbacks list
 * @param[in-out] Callback Callback to delete
 * @return error code
**/
static t_HGF_LCMP_ErrorCode LcmpCallbackDelete( t_Callbacks *callback );

/**
 * @brief Free the reserved memory for callbacks List
 * @param[in] CallbackList
 * @return error code
**/
static t_HGF_LCMP_ErrorCode LcmpCallbackListDestroy( void );

/**
 * @brief Execute the correct callback for frame
 * @param[in] opcodeReceived
 * @param[in] data
 * @param[in] lentgh
 * @param[in] srcMac
**/
static void LcmpCallbacksExecute( t_LCMP_OPCODE opcodeReceived, const INT8U *data,
    INT16U length, const INT8U *srcMac);

/**
 * @brief Thread responsible for receive and parse frames
 * @param[in] arg pointer to input arguments
**/
void *LcmpReceiveThread(void *arg);

/**
 * @brief Create a segmented frames list
 * @return pointer to segmented frames list or NULL if error
**/
static t_SegmentedFramesLists *LcmpSegmentedFramesListsCreate( void );

/**
 * @brief Open a segmented frame saved to insert a new segment, if it does not exist, creates one new
 * @param[in] numSeq Received sequence number
 * @param[in] srcMAC Source MAC of received frame
 * @param[in] LCMP_Opcode Opcode of received frame
 * @return Pointer to segmented frame
**/
static t_SegmentedFrame *LcmpSegmentedFrameOpen( INT16U numSeq, const INT8U *srcMac,
    INT16U lcmpOpcode );

/**
 * @brief Create a segmented frame to insert a new segment
 * @param[in] numSeq Received sequence number
 * @param[in] srcMAC Source MAC of received frame
 * @param[in] LCMP_Opcode Opcode of received frame
 * @return Pointer to segmented frame
**/
static t_SegmentedFrame *LcmpSegmentedFrameCreate( INT16U numSeq, const INT8U *srcMac,
    INT16U LcmpOpcode );

/**
 * @brief Add a new segment into a frame segmented
 * @param[in] numSeq Received sequence number
 * @param[in] srcMAC Source MAC of received frame
 * @param[in] LCMP_Opcode Opcode of received frame
 * @param[in] ValueLength Segment length
 * @param[in] Value Segment data
 * @param[in] numSegment Segment number
 * @param[out] SegmentedFrame Pointer to segmented frame
 * @return code error
**/
static t_HGF_LCMP_ErrorCode LcmpSegmentAdd(INT16U numSeq, const INT8U *srcMac, INT16U LcmpOpcode,
    INT16U valueLength, const INT8U *value, INT8U numSegment, t_SegmentedFrame **segmentedFrame );

/**
 * @brief Insert a new segment on frame's ordered segments list. It drops duplicates.
 * @param[in/out] SegmentedFrame
 * @param[in] numSegment Segment number
 * @param[in] ValueLength Segment length
 * @param[in] Value Segment data
 * @return code error
**/
static t_HGF_LCMP_ErrorCode LcmpSegmentOnSegmentedFrameInsert( t_SegmentedFrame *segmentedFrame,
                                    INT8U numSegment, INT16U valueLength, const INT8U *value );

/**
 * @brief Extract a frame from a segmented frames list
 * @param[in/out] SegmentedFrame
 * @param[out] ValueLength Frame length
 * @param[out] Value Segment data
 * @return code error
**/
static t_HGF_LCMP_ErrorCode LcmpSegmentedFrameExtract( t_SegmentedFrame *segmentedFrame,
    INT16U *valueLength, INT8U **value );

/**
 * @brief Delete a segmented frame from a segmented frames list and free its memory
 * @param[in-out] SegmentedFrame
 * @return code error
**/
static t_HGF_LCMP_ErrorCode LcmpSegmentedFrameDelete( t_SegmentedFrame *segmentedFrame );
/**
 * @brief Free segmented frame memory
 * @param[in-out] SegmentedFrame
 * @return code error
**/
static t_HGF_LCMP_ErrorCode LcmpSegmentedFrameDestroy( t_SegmentedFrame *segmentedFrame );

/**
 * @brief Free segmented frame list memory
 * @return code error
**/
static t_HGF_LCMP_ErrorCode LcmpSegmentedFramesListsDestroy( void );

/**
 * @brief Clean a segmented frame list erasing the segmented frames expired
 * @param[in] SegmentedFramesLists
 * @return code error
**/
static t_HGF_LCMP_ErrorCode LcmpSegmentedFramesListClean( void );

/**
 * @brief This fucntion stops the reception thread
**/
static void LcmpReceiveThreadStop( void );

/**
 * @brief This fucntion runs the reception thread
**/
static void LcmpReceiveThreadExecute( void );

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void LcmpStateSet(BOOL running)
{
  lcmpReceiveThreadRunning = running;
}

/*******************************************************************/

static BOOL LcmpStateGet(void)
{
  return lcmpReceiveThreadRunning;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpSegmentedFramesListsDestroy( void )
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if(lcmpSegmentedFramesLists != NULL)
  {
    t_SegmentedFrame *segmented_frame;

    segmented_frame = lcmpSegmentedFramesLists->head;
    while(segmented_frame != NULL)
    {
      LcmpSegmentedFrameDelete(segmented_frame);
      segmented_frame = (lcmpSegmentedFramesLists)->head;
    }
    free(lcmpSegmentedFramesLists);
    lcmpSegmentedFramesLists = NULL;
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }

  return result;
}

/*******************************************************************/

static t_SegmentedFramesLists *LcmpSegmentedFramesListsCreate( void )
{
  t_SegmentedFramesLists *new_segmented_frames_lists = NULL;

  new_segmented_frames_lists = (t_SegmentedFramesLists *)calloc(1, sizeof(t_SegmentedFramesLists));
  if(new_segmented_frames_lists != NULL)
  {
    new_segmented_frames_lists->SegmentedFrames = 0;
    new_segmented_frames_lists->head = NULL;
    new_segmented_frames_lists->tail = NULL;
  }
  return new_segmented_frames_lists;
}

/*******************************************************************/

static t_SegmentedFrame *LcmpSegmentedFrameCreate( INT16U numSeq, const INT8U *srcMac, INT16U LcmpOpcode )
{
  t_SegmentedFrame *this_segmented_frame = NULL;

  if((lcmpSegmentedFramesLists != NULL) && (srcMac != NULL))
  {
    this_segmented_frame = (t_SegmentedFrame *) calloc(1, sizeof(t_SegmentedFrame));

    if(this_segmented_frame != NULL)
    {
      this_segmented_frame->numSeq = numSeq;
      memcpy(this_segmented_frame->srcMAC,srcMac, ETH_ALEN);
      this_segmented_frame->LCMP_Opcode = LcmpOpcode;
      clock_gettime(CLOCK_MONOTONIC, &this_segmented_frame->timeStamp);
      this_segmented_frame->numSegmentsSaved = 0;
      this_segmented_frame->Segments = NULL;
      this_segmented_frame->nextSegmentedFrame = NULL;
      this_segmented_frame->prevSegmentedFrame = lcmpSegmentedFramesLists->tail;

      if (lcmpSegmentedFramesLists->tail != NULL)
      {
        lcmpSegmentedFramesLists->tail->nextSegmentedFrame = this_segmented_frame;
      }

      lcmpSegmentedFramesLists->tail = this_segmented_frame;
      if(lcmpSegmentedFramesLists->head == NULL)
      {
        lcmpSegmentedFramesLists->head = this_segmented_frame;
      }
      lcmpSegmentedFramesLists->SegmentedFrames++;
    }
  }
  return this_segmented_frame;
}
/*******************************************************************/

static t_SegmentedFrame *LcmpSegmentedFrameOpen( INT16U numSeq, const INT8U *srcMac,
    INT16U lcmpOpcode )
{
  t_SegmentedFrame *this_segmented_frame = NULL;
  BOOL frameFound = FALSE;

  if( (lcmpSegmentedFramesLists != NULL) && (srcMac != NULL) )
  {
    this_segmented_frame = lcmpSegmentedFramesLists->head;
    while((this_segmented_frame != NULL)&&(!frameFound))
    {
      if(
          (this_segmented_frame->numSeq == numSeq)
          &&
          (memcmp(srcMac,this_segmented_frame->srcMAC, ETH_ALEN) == 0)
          &&
          (this_segmented_frame->LCMP_Opcode == lcmpOpcode)
         )
      {
        frameFound = TRUE;
      }
      else
      {
        this_segmented_frame = this_segmented_frame->nextSegmentedFrame;
      }
    }
    if(this_segmented_frame == NULL)
    {
      this_segmented_frame = LcmpSegmentedFrameCreate(numSeq, srcMac, lcmpOpcode);
    }
  }
  return this_segmented_frame;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpSegmentedFrameDestroy(t_SegmentedFrame *segmentedFrame)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
  t_Segments *this_segment;
  t_Segments *next_segment;

  if(segmentedFrame != NULL)
  {
    this_segment = segmentedFrame->Segments;
    while(this_segment != NULL)
    {
      next_segment = this_segment->nextSegment;
      free(this_segment->Value);
      this_segment->Value = NULL;
      free(this_segment);
      this_segment = NULL;
      this_segment = next_segment;
    }
    free(segmentedFrame);
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return result;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpSegmentedFrameDelete( t_SegmentedFrame *segmentedFrame )
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if((lcmpSegmentedFramesLists != NULL) && (segmentedFrame!=NULL))
  {
    if(segmentedFrame->prevSegmentedFrame == NULL)
    {
      lcmpSegmentedFramesLists->head = segmentedFrame->nextSegmentedFrame;
    }
    else
    {
      segmentedFrame->prevSegmentedFrame->nextSegmentedFrame = segmentedFrame->nextSegmentedFrame;
    }

    if(segmentedFrame->nextSegmentedFrame == NULL)
    {
      lcmpSegmentedFramesLists->tail = segmentedFrame->prevSegmentedFrame;
    }
    else
    {
      segmentedFrame->nextSegmentedFrame->prevSegmentedFrame = segmentedFrame->prevSegmentedFrame;
    }

    lcmpSegmentedFramesLists->SegmentedFrames--;

    result = LcmpSegmentedFrameDestroy(segmentedFrame);
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return result;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpSegmentedFrameExtract( t_SegmentedFrame *segmentedFrame,
    INT16U *valueLength, INT8U **value )
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
  INT16U offsetValue;
  t_Segments  *thissegment;
  INT8U num_segments = 0;

  if((lcmpSegmentedFramesLists != NULL) && (segmentedFrame!=NULL) && (valueLength!=NULL))
  {
    (*valueLength) = 0;
    (*value) = NULL;
    if(segmentedFrame->numSegmentsSaved > 0)
    {
      thissegment = segmentedFrame->Segments;

      num_segments = 0;
      while((thissegment != NULL) && ( num_segments < segmentedFrame->numSegmentsSaved))
      {
        (*valueLength) += thissegment->ValueLength;
        thissegment = thissegment->nextSegment;
        num_segments++;
      }
      if((*valueLength) > 0)
      {
        (*value) = (INT8U *) calloc(1, (*valueLength));
        if((*value) != NULL)
        {
          thissegment = segmentedFrame->Segments;
          offsetValue = 0;
          num_segments = 0;
          while((thissegment != NULL) && ( num_segments < segmentedFrame->numSegmentsSaved))
          {
            memcpy(&((*value)[offsetValue]),thissegment->Value,thissegment->ValueLength);

            offsetValue += thissegment->ValueLength;
            thissegment = thissegment->nextSegment;
            num_segments++;
          }
          result = LcmpSegmentedFrameDelete(segmentedFrame);
        }
        else
        {
          result = HGF_LCMP_ERROR_MALLOC;
        }
      }
    }
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return result;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpSegmentOnSegmentedFrameInsert( t_SegmentedFrame *segmentedFrame,
                                    INT8U numSegment, INT16U valueLength, const INT8U *value )
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
  t_Segments  *thissegment;
  t_Segments  *segment;
  BOOL positionFound = FALSE;
  BOOL duplicate = FALSE;

  if( (segmentedFrame != NULL) && (value != NULL))
  {
    t_Segments  *prevsegment;
    thissegment = segmentedFrame->Segments;
    prevsegment = NULL;
    while((thissegment != NULL) && (!duplicate) && (!positionFound))
    {
      if(thissegment->numSegment == numSegment)
      {
        duplicate = TRUE;
      }
      else if(thissegment->numSegment > numSegment)
      {
        positionFound = TRUE;
      }
      else
      {
        prevsegment = thissegment;
        thissegment = thissegment->nextSegment;
      }
    }
    if(!duplicate)
    {
      segment = (t_Segments *) calloc (1, sizeof(t_Segments));
      if(segment == NULL)
      {
        result = HGF_LCMP_ERROR_MALLOC;
      }
      else
      {
        segment->Value = (INT8U *) calloc(1, valueLength);
        if(segment->Value == NULL)
        {
          result = HGF_LCMP_ERROR_MALLOC;
          VbLogPrint(VB_LOG_ERROR,"Malloc error creating segment frame");
        }
        else
        {
          segment->numSegment = numSegment;
          segment->ValueLength = valueLength;
          memcpy(segment->Value,value,valueLength);

        }

        if(prevsegment == NULL)
        {
          segment->nextSegment = thissegment;
          segmentedFrame->Segments = segment;
        }
        else
        {
          segment->nextSegment = thissegment;
          prevsegment->nextSegment = segment;
        }
        segmentedFrame->numSegmentsSaved++;
        clock_gettime(CLOCK_MONOTONIC, &segmentedFrame->timeStamp);
      }
    }
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return result;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpSegmentAdd(INT16U numSeq, const INT8U *srcMac, INT16U LcmpOpcode,
    INT16U valueLength, const INT8U *value, INT8U numSegment, t_SegmentedFrame **segmentedFrame )
{

  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if((srcMac != NULL) && (value != NULL) && (segmentedFrame != NULL))
  {
    if(lcmpSegmentedFramesLists == NULL)
    {
      lcmpSegmentedFramesLists = LcmpSegmentedFramesListsCreate();
    }
    if(lcmpSegmentedFramesLists != NULL)
    {
      (*segmentedFrame) = LcmpSegmentedFrameOpen(numSeq, srcMac, LcmpOpcode);
      if((*segmentedFrame) == NULL)
      {
        result =  HGF_LCMP_ERROR_MALLOC;
      }
      else
      {
        result =
            LcmpSegmentOnSegmentedFrameInsert((*segmentedFrame), numSegment, valueLength, value);
      }
    }
    else
    {
      result =  HGF_LCMP_ERROR_MALLOC;
    }
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return result;
}

/*******************************************************************/

static void LcmpReceiveThreadExecute( void )
{
  LcmpReceiveThreadStop();

  VbLogPrint(VB_LOG_INFO, "Starting %s thread", LCMP_THREAD_NAME);

  LcmpStateSet(TRUE);
  if (FALSE == VbThreadCreate(LCMP_THREAD_NAME, LcmpReceiveThread, NULL, VB_DRIVER_LCMP_THREAD_PRIORITY, &lcmpReceiveThread))
  {
    VbLogPrint(VB_LOG_ERROR,"Can't create receiveThread");
    LcmpStateSet(FALSE);
  }
}

/*******************************************************************/

static void LcmpReceiveThreadStop()
{
  if (LcmpStateGet() == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", LCMP_THREAD_NAME);

    LcmpStateSet(FALSE);

    if (lcmpSc >= 0)
    {
      shutdown(lcmpSc, SHUT_RDWR);
      close(lcmpSc);
    }

    VbThreadJoin(lcmpReceiveThread, LCMP_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", LCMP_THREAD_NAME);
  }
}

/*******************************************************************/

static t_CallbacksLists *LcmpCallbackListCreate( void )
{
  t_CallbacksLists *callback_list = NULL;

  callback_list = (t_CallbacksLists *)calloc(1, sizeof(t_CallbacksLists));
  if(callback_list != NULL)
  {
    callback_list->NumCallbacks = 0;
    callback_list->head = NULL;
    callback_list->tail = NULL;
  }
  return callback_list;
}

/*******************************************************************/

static t_Callbacks *LcmpCallbackAdd( BOOL (* callback)(const INT8U *, INT16U, t_CallbackData *),
    t_LCMP_OPCODE opcodeFilter,t_HGF_TLV expectedHgfOpcode,
    INT8U paramId, INT16U transactionId, BOOL markTimeStamp)
{
  t_Callbacks       *new_callback = NULL;
  pthread_condattr_t attr_con_var;

  if (lcmpCallbacksList == NULL)
  {
    lcmpCallbacksList = LcmpCallbackListCreate();
  }

  if (lcmpCallbacksList != NULL)
  {
    new_callback = (t_Callbacks *)calloc(1, sizeof(t_Callbacks));

    if (new_callback != NULL)
    {
      bzero(new_callback, sizeof(t_Callbacks));
      new_callback->Callback = callback;
      new_callback->filter = opcodeFilter;
      new_callback->transactionID = transactionId;
      new_callback->CallbackData.ReceivedValues = NULL;
      new_callback->CallbackData.expectedHGFOpcode = expectedHgfOpcode;
      new_callback->CallbackData.paramID = paramId;
      pthread_mutex_init(&(new_callback->CallbackData.mutex), NULL);
      pthread_condattr_init(&attr_con_var);
      pthread_cond_init(&(new_callback->CallbackData.condition),&attr_con_var);
      new_callback->CallbackData.frameReceived = FALSE;
      new_callback->CallbackData.markTimeStamp = markTimeStamp;
      memset(new_callback->CallbackData.srcMAC, 0, ETH_ALEN);
      LcmpCallbackInsert( new_callback );
    }
  }

  return new_callback;
}

/*******************************************************************/

static void LcmpCallbackInsert( t_Callbacks *newCallback )
{

  if((lcmpCallbacksList != NULL) && (newCallback != NULL))
  {
    if(lcmpCallbacksList->tail != NULL)
    {
      lcmpCallbacksList->tail->nextCallback = newCallback;
    }
    newCallback->prevCallback = lcmpCallbacksList->tail;
    lcmpCallbacksList->tail = newCallback;
    if(lcmpCallbacksList->head == NULL)
    {
      lcmpCallbacksList->head = newCallback;
      newCallback->prevCallback = NULL;
    }
    newCallback->nextCallback = NULL;
    lcmpCallbacksList->NumCallbacks++;
  }
}

/******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpCallbackDelete( t_Callbacks *callback )
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if ((lcmpCallbacksList == NULL) || (callback == NULL))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    if (0 != pthread_mutex_lock(&(callback->CallbackData.mutex)))
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_mutex_lock failed!");
    }

    VbDatamodelHtlvsListValueDestroy(&(callback->CallbackData.ReceivedValues));
    callback->CallbackData.frameReceived = FALSE;

    if (0 != pthread_cond_signal(&(callback->CallbackData.condition)))
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_cond_signal failed!");
    }

    if (0 != pthread_mutex_unlock(&(callback->CallbackData.mutex)))
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_mutex_lock failed!");
    }

    if (0 != pthread_mutex_lock(&(callback->CallbackData.mutex)))
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_mutex_lock failed!");
    }

    pthread_cond_destroy(&(callback->CallbackData.condition));

    if (0 != pthread_mutex_unlock(&(callback->CallbackData.mutex)))
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_mutex_lock failed!");
    }

    pthread_mutex_destroy(&(callback->CallbackData.mutex));

    if(callback->nextCallback != NULL)
    {
      callback->nextCallback->prevCallback = callback->prevCallback;
    }
    else
    {
      lcmpCallbacksList->tail = callback->prevCallback;
    }
    if(callback->prevCallback!= NULL)
    {
      callback->prevCallback->nextCallback = callback->nextCallback;
    }
    else
    {
      lcmpCallbacksList->head = callback->nextCallback;
    }

    free(callback);
    lcmpCallbacksList->NumCallbacks--;
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return result;

}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpCallbackListDestroy( void )
{
  t_Callbacks *this_callback;
  t_Callbacks *next_callback;
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if(lcmpCallbacksList != NULL)
  {
    this_callback = lcmpCallbacksList->head;
    while(this_callback != NULL)
    {
      next_callback = this_callback->nextCallback;
      LcmpCallbackDelete(this_callback);
      this_callback = next_callback;
    }
    free(lcmpCallbacksList);
    lcmpCallbacksList = NULL;
  }
  else
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return result;
}

/*******************************************************************/

static BOOLEAN LcmpMsgIsForMe(t_LCMP_OPCODE opcodeReceived, INT16U transactionId, t_Callbacks *callback, const INT8U *mac)
{
  BOOLEAN is_for_me = FALSE;

  if (callback == NULL)
  {
    is_for_me = FALSE;
  }
  else
  {
    if ((opcodeReceived == callback->filter) &&
        ((callback->transactionID == 0) || (transactionId == callback->transactionID)))
    {
      is_for_me = TRUE;
    }
    else
    {
      is_for_me = FALSE;
    }
  }

  return is_for_me;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpRxFrameParse(t_LCMP_OPCODE opcodeReceived, const INT8U *data,
    INT16U length, t_Callbacks *callback, const INT8U *mac, t_LcmpParsedInfo *parsedInfo)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;
  INT16U               lcmp_length = 0;
  INT16U               lcmp_value_length = 0;
  INT16U               lcmp_vaue_hdr_length = 0;
  INT8U                control;
  BOOLEAN              notif_ack = FALSE;
  INT8U               *lcmp_value;
  INT16U               transaction_id = 0;

  if ((data == NULL) || (parsedInfo == NULL))
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    switch (opcodeReceived)
    {
      case LCMP_READ_REQ:
      case LCMP_WRITE_REQ:
      case LCMP_CTRL_REQ:
      {
        lcmp_value_length = _ntohs_ghn(((t_LCMP_req_Header *)data)->length);
        control = ((t_LCMP_req_Header *)data)->control;
        transaction_id =  _ntohs_ghn(((t_LCMP_req_Header *)data)->transactionId);

        lcmp_value = (INT8U *)(&data[LCMP_REQ_HEADER_SIZE]);
        lcmp_length = lcmp_value_length;
        lcmp_vaue_hdr_length = LCMP_REQ_HEADER_SIZE;

        break;
      }
      case LCMP_NOTIFY_RSP:
      {
        lcmp_value_length = _ntohs_ghn(((t_LCMP_rsp_Header *)data)->length);
        control = ((t_LCMP_rsp_Header *)data)->control;
        transaction_id =  _ntohs_ghn(((t_LCMP_rsp_Header *)data)->transactionId);

        lcmp_value = (INT8U *)(&data[LCMP_RSP_HEADER_SIZE]);
        lcmp_length = lcmp_value_length;
        lcmp_vaue_hdr_length = LCMP_RSP_HEADER_SIZE;

        break;
      }
      case LCMP_NOTIFY_IND:
      {
        lcmp_value_length = _ntohs_ghn(((t_LCMP_ind_Header *)data)->length);
        control = ((t_LCMP_ind_Header *)data)->control;
        notif_ack = (BOOLEAN)(((t_LCMP_ind_Header *)data)->notifAck);
        transaction_id =  _ntohs_ghn(((t_LCMP_ind_Header *)data)->transactionId);

        lcmp_value = (INT8U *)(&data[LCMP_IND_HEADER_SIZE]);
        lcmp_length = lcmp_value_length;
        lcmp_vaue_hdr_length = LCMP_IND_HEADER_SIZE;

        break;
      }
      case LCMP_WRITE_CNF:
      case LCMP_CTRL_CNF:
      case LCMP_READ_CNF:
      {
        lcmp_value_length = _ntohs_ghn(((t_LCMP_cnf_Header *)data)->length);
        control = ((t_LCMP_cnf_Header *)data)->control;
        transaction_id =  _ntohs_ghn(((t_LCMP_cnf_Header *)data)->transactionId);

        lcmp_value = (INT8U *)(&data[LCMP_CNF_HEADER_SIZE]);
        lcmp_length = lcmp_value_length;
        lcmp_vaue_hdr_length = LCMP_CNF_HEADER_SIZE;

        break;
      }
      default:
      {
        ret = HGF_LCMP_ERROR_BAD_ARGS;
        break;
      }
    }
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    // Check control value and consistent length
    if ((control == LCMP_CONTROL_VB) &&
        (length >= (lcmp_length + lcmp_vaue_hdr_length)))
    {
      parsedInfo->isVbParam = TRUE;
    }
    else
    {
      parsedInfo->isVbParam = FALSE;
    }

    parsedInfo->notifAck = notif_ack;
    parsedInfo->isForMe = LcmpMsgIsForMe(opcodeReceived, transaction_id, callback, mac);
    parsedInfo->valuePtr = lcmp_value;
    parsedInfo->valueLen = lcmp_length;
  }

  return ret;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpDbgRxMsgAdd(t_LCMP_OPCODE opcodeReceived, const INT8U *data, INT16U length, const INT8U *srcMac)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;
  t_LcmpParsedInfo     parsed_info = {0};
  INT8U                param_id;

  ret = LcmpRxFrameParse(opcodeReceived, data, length, NULL, srcMac, &parsed_info);

  if ((ret == HGF_LCMP_ERROR_NONE) &&
      (parsed_info.isVbParam == TRUE))
  {
    ret = VbLcmpParamIdGet(parsed_info.valuePtr, parsed_info.valueLen, &param_id);

    if (ret == HGF_LCMP_ERROR_NONE)
    {
      ret = LcmpDbgMsgAdd(srcMac, FALSE, FALSE, opcodeReceived, param_id);
    }
  }
  else
  {
    ret = LcmpDbgNoVbMsgAdd(srcMac);
  }

  if (ret != HGF_LCMP_ERROR_NONE)
  {
    if (ret == HGF_LCMP_ERROR_PARAM_ERROR)
    {
      // LCMP Confirmation not received
    }
    else
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d inserting LCMP in debug list (srcMac " MAC_PRINTF_FORMAT "; vbParam %u opcode 0x%X; param_id 0x%X)",
          ret, MAC_PRINTF_DATA(srcMac), parsed_info.isVbParam, opcodeReceived, param_id);
    }
  }

  return ret;
}

/*******************************************************************/

static void LcmpCallbacksExecute( t_LCMP_OPCODE opcodeReceived, const INT8U *data,
    INT16U length, const INT8U *srcMac)
{
  t_HGF_LCMP_ErrorCode err = HGF_LCMP_ERROR_NONE;
  t_Callbacks         *this_callback;
  t_LcmpParsedInfo     parsed_info = {0};

  if ((data != NULL) && (srcMac != NULL))
  {
    pthread_mutex_lock( &lcmpMutexCallbacksList );

    if (lcmpCallbacksList != NULL)
    {
      this_callback = lcmpCallbacksList->head;

      while (this_callback != NULL)
      {
        // Parse received LCMP frame
        err = LcmpRxFrameParse(opcodeReceived, data, length, this_callback, srcMac, &parsed_info);

        if (err != HGF_LCMP_ERROR_NONE)
        {
          VbLogPrint(VB_LOG_ERROR, "Error %d parsing LCMP frame (opcode 0x%X)", err, opcodeReceived);
        }

        if ((err == HGF_LCMP_ERROR_NONE) &&
            (parsed_info.isForMe == TRUE) &&
            (parsed_info.isVbParam == TRUE))
        {
          // Frame is for me, call callback

          this_callback->CallbackData.sendack = parsed_info.notifAck;
          this_callback->CallbackData.rxLcmpOpcode = opcodeReceived;
          MACAddrClone(this_callback->CallbackData.srcMAC, srcMac);

          this_callback->Callback(parsed_info.valuePtr, parsed_info.valueLen, &(this_callback->CallbackData));
        }

        // Get next callback
        this_callback = this_callback->nextCallback;
      }
    }

    pthread_mutex_unlock( &lcmpMutexCallbacksList );

    // Insert message in debug list
    err = LcmpDbgRxMsgAdd(opcodeReceived, data, length, srcMac);
  }
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpSegmentedFramesListClean( void )
{

  struct timespec tnow;
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
  t_SegmentedFrame *segmented_frame;
  t_SegmentedFrame *next_segmented_frame;
  INT32S elapsed_ms;

  if(lcmpSegmentedFramesLists != NULL)
  {
    if(lcmpSegmentedFramesLists != NULL)
    {
      clock_gettime(CLOCK_MONOTONIC, &tnow);
      segmented_frame = lcmpSegmentedFramesLists->head;
      while(segmented_frame != NULL)
      {
        elapsed_ms = VbUtilElapsetimeTimespecMs( segmented_frame->timeStamp, tnow );
        next_segmented_frame = segmented_frame->nextSegmentedFrame;
        if(elapsed_ms > TIME_TO_DISCARD_SEGMENTED_FRAME)
        {
          LcmpSegmentedFrameDelete(segmented_frame);
        }
        segmented_frame = next_segmented_frame;
      }
    }
  }
  return result;

}

/*******************************************************************/

void *LcmpReceiveThread(void *arg)
{

  INT8U *buffer;
  int bytes;
  INT32S select_ret;
  t_MMH *mmh;
  INT16U mmpl_length;
  INT16U lcmp_opcode;
  INT16U num_seq;
  INT8U* mmpl = NULL;
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
#if(0)
  pthread_t th_id;
  pthread_attr_t th_attr;
  struct sched_param param;
  INT32U policy = 0;
#endif
  INT8U num_segments;
  INT8U segment;
  INT8U *srcMac;
  t_SegmentedFrame *segmented_frame;
  INT8U *full_frame_data_mmpl = NULL;
  struct timeval tv;
  fd_set rfds;
  INT32U temp_field_endianness;
  INT32U *mmh_ptr = NULL;

  if(lcmpSc == -1)
  {
    VbLogPrint(VB_LOG_ERROR,"Socket error [%s]",strerror(errno));
  }
  else
  {
    buffer = (INT8U *) calloc(1, BUF_SIZE);
    if (buffer == NULL)
    {
      VbLogPrint(VB_LOG_ERROR,"Malloc error to create buffer");
    }
    else
    {
#if(0)
      // Configure max priority to this thread
      th_id = pthread_self();
      pthread_attr_init(&th_attr);
      pthread_attr_getschedpolicy(&th_attr, (int *)&policy);
      param.sched_priority = sched_get_priority_max(policy);
      pthread_setschedparam(th_id, policy,&param);
      pthread_attr_destroy(&th_attr);
      // Configure max priority to this thread
#endif

      while(LcmpStateGet() == TRUE)
      {
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        FD_ZERO(&rfds);
        FD_SET(lcmpSc, &rfds);

        while ((select_ret = select(lcmpSc + 1, &rfds, NULL, NULL, &tv)) > 0)
        {
          bytes = recv(lcmpSc, buffer, BUF_SIZE, 0);

          if(bytes > MMPL_OFFSET)
          {
            srcMac = &buffer[SRCMACOFFSET];

            // Apply endianness transformation to MMH (2 words)
            mmh_ptr = (INT32U*)&buffer[MMH_OFFSET];

            temp_field_endianness = _ntohl_ghn(*mmh_ptr);
            *mmh_ptr = temp_field_endianness;

            temp_field_endianness = _ntohl_ghn(*(mmh_ptr + 1));
            *(mmh_ptr + 1) = temp_field_endianness;
            // End of apply endianness transformation

            mmh         = (t_MMH *)&buffer[MMH_OFFSET];
            num_segments = mmh->numberSegments;
            segment     = mmh->segment;
            mmpl_length = mmh->Length;
            lcmp_opcode = mmh->OPCODE;
            num_seq      = mmh->seqNumber;

            mmpl = &buffer[MMPL_OFFSET];
            if(num_segments == 0)
            {
              LcmpCallbacksExecute( lcmp_opcode, mmpl, mmpl_length, srcMac );
            }
            else
            {
              segmented_frame = NULL;
              result = LcmpSegmentAdd( num_seq, srcMac, lcmp_opcode,
                  mmpl_length, mmpl, segment, &segmented_frame);

              if(result == HGF_LCMP_ERROR_NONE)
              {
                if(segmented_frame != NULL)
                {
                  if(segmented_frame->numSegmentsSaved == (num_segments + 1))
                  {
                    result = LcmpSegmentedFrameExtract( segmented_frame, &mmpl_length,
                                                        &full_frame_data_mmpl);
                    if(result == HGF_LCMP_ERROR_NONE)
                    {
                      LcmpCallbacksExecute( lcmp_opcode, full_frame_data_mmpl, mmpl_length, srcMac );
                    }

                    if (full_frame_data_mmpl != NULL)
                    {
                      free(full_frame_data_mmpl);
                      full_frame_data_mmpl = NULL;
                    }
                  }
                }
              }
            }
          }
          result= LcmpSegmentedFramesListClean();
          if(result != HGF_LCMP_ERROR_NONE)
          {
            VbLogPrint(VB_LOG_ERROR,"Segmented frames list clean error [%d]", result);
          }
        }
      }
      free(buffer);
      buffer = NULL;
    }
  }
  pthread_mutex_lock( &lcmpMutexCallbacksList );
  LcmpCallbackListDestroy();
  pthread_mutex_unlock( &lcmpMutexCallbacksList );
  LcmpSegmentedFramesListsDestroy();

  return NULL;
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/******************************************************************/

t_VB_comErrorCode LcmpCallBackUninstall(t_Callbacks *callback)
{
  t_VB_comErrorCode result =  VB_COM_ERROR_NONE;

  if(callback != NULL)
  {
    pthread_mutex_lock( &lcmpMutexCallbacksList );

    if (HGF_LCMP_ERROR_NONE != LcmpCallbackDelete(callback))
    {
      result = VB_COM_ERROR_PARAM_ERROR;
    }

    pthread_mutex_unlock( &lcmpMutexCallbacksList );
  }
  else
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }
  return result;
}

/******************************************************************/

t_Callbacks *LcmpCallBackInstall(BOOL  (*callback)(const INT8U*,INT16U, t_CallbackData *),
                                    t_LCMP_OPCODE opcodeFilter, t_HGF_TLV expectedHgfOpcode,
                                    INT8U paramId, INT16U transactionId, BOOL markTimeStamp)
{

  t_Callbacks *returnValue;

  pthread_mutex_lock( &lcmpMutexCallbacksList );
  returnValue =
      LcmpCallbackAdd(
          callback,
          opcodeFilter,
          expectedHgfOpcode,
          paramId,
          transactionId,
          markTimeStamp);

  pthread_mutex_unlock( &lcmpMutexCallbacksList );

  return returnValue;

}

/******************************************************************/

t_HTLVsLists *LcmpCallBackReceiveGet(t_Callbacks *callback, BOOLEAN *frameReceived)
{

  t_HTLVsLists *returnValue = NULL;

  if (callback != NULL)
  {
    pthread_mutex_lock( &lcmpMutexCallbacksList );

    returnValue = callback->CallbackData.ReceivedValues;

    if (frameReceived != NULL)
    {
      *frameReceived = callback->CallbackData.frameReceived;
    }

    callback->CallbackData.ReceivedValues = NULL;
    callback->CallbackData.frameReceived = FALSE;

    pthread_mutex_unlock( &lcmpMutexCallbacksList );
  }

  return returnValue;
}

/*******************************************************************/

t_VB_comErrorCode LcmpMacGet( INT8U *myMac )
{

  INT16U j;
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  struct ifreq ifr;

  if (lcmpSc==-1)
  {
    result = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }
  else
  {
    strncpy(ifr.ifr_name, (char *)ifLcmp, IFNAMSIZ);
    // Retrieve corresponding MAC
    if (ioctl(lcmpSc, SIOCGIFHWADDR, &ifr) == -1)
    {
      //ERROR: error getting source mac
      result = VB_COM_ERROR_ETH_IF;
    }
    else
    {

      for (j = 0; j < ETH_ALEN; j++)
      {
        myMac[j] = ifr.ifr_hwaddr.sa_data[j];
      }
    }
  }
  return result;
}

/*******************************************************************/

void LcmpEnd ( void )
{
  LcmpReceiveThreadStop();
  if(lcmpSc >= 0)
  {
    close(lcmpSc);
  }
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode LcmpExecute( void )
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
  const int broadcast = 1;
  int flags;

  if (lcmpSc == -1)
  {
    lcmpSc = socket(PF_PACKET, SOCK_RAW, _htons(ETH_LCMP_PROTOCOL));

    if (setsockopt(lcmpSc, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)))
    {
      VbLogPrint(VB_LOG_ERROR,"Socket error [%s]", strerror(errno));
    }

    flags = fcntl(lcmpSc, F_GETFL,0);
    fcntl(lcmpSc, F_SETFL, flags | O_NONBLOCK);
  }
  if (lcmpSc == -1)
  {
    result = HGF_LCMP_ERROR_SOCKET_NOT_OPENED;
  }
  else
  {
    LcmpReceiveThreadExecute();
  }

  return result;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode LcmpHeaderBuild(INT32U length, t_LCMP_OPCODE opcode, INT32U numSegments, INT32U segmNumber, t_MMH *mmh)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;

  if (mmh == NULL)
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    INT32U *mmh_ptr = NULL;
    INT32U temp_field_endianness;

    bzero(mmh, sizeof(*mmh));
    mmh->Length = length;
    mmh->OPCODE = opcode;
    mmh->numberSegments = numSegments;
    mmh->segment = segmNumber;
    mmh->repNumber = 0;
    mmh->seqNumber = 0;
    mmh->fsb = 1; // Force start of sequence to consider all vectorboost LCMP messages as "NEW"

    // Apply endianness transformation to MMH (2 words)
    mmh_ptr = (INT32U *)mmh;
    temp_field_endianness = _htonl_ghn(*mmh_ptr);
    *mmh_ptr = temp_field_endianness;

    temp_field_endianness = _htonl_ghn(*(mmh_ptr + 1));
    *(mmh_ptr + 1) = temp_field_endianness;
    // End of apply endianness transformation
  }

  return ret;
}

/******************************************************************/

t_HGF_LCMP_ErrorCode LcmpPacketSend(const INT8U *dstMac,t_LCMP_OPCODE lcmpOpcodes,INT16U mmplLength,
    const INT8U *mmpl)
{
  INT8U  *buffer = NULL;
  struct ethhdr *eh;
  int sent;
  t_MMH mmh;

  t_HGF_LCMP_ErrorCode returnvalue = HGF_LCMP_ERROR_NONE;
  struct sockaddr_ll socket_address;
  INT8U local_mac[ETH_ALEN];
  struct ifreq ifr;
  INT16U j;

  if((dstMac != NULL) && (mmpl != NULL))
  {
    if (lcmpSc==-1)
    {
      returnvalue = HGF_LCMP_ERROR_SOCKET_NOT_OPENED;
    }
    else
    {

      // Prepare sockaddr_ll
      bzero(&socket_address, sizeof(socket_address));
      socket_address.sll_family = PF_PACKET;
      socket_address.sll_protocol = htons(ETH_LCMP_PROTOCOL);
      socket_address.sll_hatype = ARPHRD_ETHER;
      socket_address.sll_pkttype = PACKET_OTHERHOST;
      socket_address.sll_halen = ETH_ALEN;
      socket_address.sll_addr[0] = dstMac[0];
      socket_address.sll_addr[1] = dstMac[1];
      socket_address.sll_addr[2] = dstMac[2];
      socket_address.sll_addr[3] = dstMac[3];
      socket_address.sll_addr[4] = dstMac[4];
      socket_address.sll_addr[5] = dstMac[5];
      socket_address.sll_addr[6] = 0x00;
      socket_address.sll_addr[7] = 0x00;

      memcpy(ifr.ifr_name, (char *)ifLcmp, IFNAMSIZ);

      // Retrieve corresponding MAC
      if (ioctl(lcmpSc, SIOCGIFHWADDR, &ifr) == -1)
      {
        //ERROR: error getting source mac
        returnvalue = HGF_LCMP_ERROR_ETH_IF;
      }
      else
      {
        for (j = 0; j < ETH_ALEN; j++)
        {
          local_mac[j] = ifr.ifr_hwaddr.sa_data[j];
        }
        if (ioctl(lcmpSc, SIOCGIFINDEX, &ifr) == -1)
        {
          //ERROR: error getting source mac
          returnvalue = HGF_LCMP_ERROR_ETH_IF;
        }
        else
        {
          socket_address.sll_ifindex = ifr.ifr_ifru.ifru_ivalue;

          buffer = (INT8U *) calloc(1, BUF_SIZE); // Buffer for ethernet frame
          if (buffer == NULL)
          {
            returnvalue = HGF_LCMP_ERROR_MALLOC;
          }
          else
          {
            eh = (struct ethhdr *) buffer;   // Pointer to ethernet header
            bzero(eh, sizeof(struct ethhdr));

            // Prepare buffer
            memcpy((void*) buffer, (void*) dstMac, ETH_ALEN);
            memcpy((void*) (buffer + ETH_ALEN), (void*)local_mac, ETH_ALEN);
            eh->h_proto = _htons(ETH_LCMP_PROTOCOL);

            if (mmplLength <= MMPL_MAX_LENGTH)
            {
              // Build header to send a single fragment
              returnvalue = LcmpHeaderBuild(mmplLength, lcmpOpcodes, 0, 0, &mmh);

              if (returnvalue == HGF_LCMP_ERROR_NONE)
              {
                // Copy MMH
                memcpy((void*) (buffer + MMH_OFFSET), (void*)&mmh, MMH_SIZE);

                // Copy MMPL
                memcpy((void*) (buffer + MMPL_OFFSET), (void*)mmpl, mmplLength);

                sent = sendto(lcmpSc, buffer, ETH_HEADER_LEN + MMH_SIZE + mmplLength,
                     0 , (struct sockaddr*) &socket_address, sizeof(socket_address));

                if (sent == -1)
                {
                  VbLogPrint(VB_LOG_ERROR,"Sendto error [%s]", strerror(errno));
                  //ERROR
                  returnvalue = HGF_LCMP_ERROR_SENDTO;
                  if((errno == EAGAIN)||(errno == EWOULDBLOCK))
                  {
                    // Socket is unavailable, lets wait a bit a try again later
                    VbThreadSleep(SOCKET_RETRY_TIMEOUT);
                  }
                }
              }
              else
              {
                VbLogPrint(VB_LOG_ERROR, "Error [%d] building MMH", returnvalue);
              }
            }
            else
            {
              // Message must be segmented

              const INT8U    *ptr_mmpl;
              INT32U          num_segments = 0;
              INT32U          segment_number = 0;
              INT32U          remaining_bytes = 0;  // Remaining bytes to transmit
              INT32U          bytes_to_send = 0;  // Remaining bytes to transmit

              // Calculating the needed number of segments
              num_segments = CEIL(mmplLength, MMPL_MAX_LENGTH);

              // Setting remaining bytes to transmit as the total length
              remaining_bytes = mmplLength;

              // Reset the segment number
              segment_number = 0;

              // Set the pointer to the beginning of MMPL
              ptr_mmpl = mmpl;

              // Send segments while there are remaining bytes
              while ((remaining_bytes > 0) && (segment_number < num_segments) &&
                     (returnvalue == HGF_LCMP_ERROR_NONE))
              {
                if (remaining_bytes > MMPL_MAX_LENGTH)
                {
                  bytes_to_send = MMPL_MAX_LENGTH;
                }
                else
                {
                  bytes_to_send = remaining_bytes;
                }

                // Building the header (use num_segments -1 as codified in ITU G.hn)
                returnvalue = LcmpHeaderBuild(bytes_to_send, lcmpOpcodes, num_segments - 1, segment_number, &mmh);

                if (returnvalue != HGF_LCMP_ERROR_NONE)
                {
                  VbLogPrint(VB_LOG_ERROR, "Error [%d] building MMH", returnvalue);
                }
                else
                {
                  // Copy MMH
                  memcpy((void*) (buffer + MMH_OFFSET), (void*)&mmh, MMH_SIZE);

                  // Copy MMPL
                  memcpy((void*) (buffer + MMPL_OFFSET), (void*)ptr_mmpl, bytes_to_send);

                  sent = sendto(lcmpSc, buffer, ETH_HEADER_LEN + MMH_SIZE + bytes_to_send,
                       0 , (struct sockaddr*) &socket_address, sizeof(socket_address));

                  if (sent == -1)
                  {
                    VbLogPrint(VB_LOG_ERROR,"Sendto error [%s]", strerror(errno));
                    //ERROR
                    returnvalue = HGF_LCMP_ERROR_SENDTO;

                    if((errno == EAGAIN)||(errno == EWOULDBLOCK))
                    {
                      // Socket is unavailable, lets wait a bit a try again later
                      VbThreadSleep(SOCKET_RETRY_TIMEOUT);
                    }
                  }
                  else
                  {
                    // Increase the segment number
                    segment_number++;

                    // Count the transmitted bytes
                    remaining_bytes -= bytes_to_send;

                    // Update the chunk to transmit
                    ptr_mmpl += bytes_to_send;
                  }
                }
              }
            }

            free(buffer);
            buffer = NULL;
          }
        }
      }
    }
  }
  else
  {
    returnvalue = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  return returnvalue;
}

/******************************************************************/

void LcmpInit(const char *ifeth)
{
  lcmpReceiveThread = 0;
  LcmpStateSet(FALSE);

  lcmpSc = -1;
  lcmpCallbacksList = NULL;

  lcmpSegmentedFramesLists = NULL;
  strcpy(ifLcmp,ifeth);
}

/*******************************************************************/

/**
 * @}
 **/
