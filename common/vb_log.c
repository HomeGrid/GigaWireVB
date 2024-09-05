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
 * @file vb_log.c
 * @brief Implements log feature
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <mqueue.h>
#include <libgen.h>

#if (_WITH_SYSLOG_ == 1)
#include <syslog.h>
#endif

#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_log.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_util.h"
#include "vb_ea_communication.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#if (_WITH_SYSLOG_ == 0)
#define VB_LOG_THREAD_NAME      ("Log")
#endif

// Single line log limits
//
#define VB_LOG_MAX_MODE_LEN     7
#define VB_LOG_MAX_FILE_LEN     30
#define VB_LOG_MAX_LINENUM_LEN  5
#define VB_LOG_MAX_FUNC_LEN     40
#define VB_LOG_MAX_EXTRA_LEN    24 // [][][][][xx:xx:xx xxxms]
#define VB_LOG_MAX_HDR_LEN      (VB_LOG_MAX_FILE_LEN + VB_LOG_MAX_LINENUM_LEN + \
                                 VB_LOG_MAX_FUNC_LEN + VB_LOG_MAX_MODE_LEN    + \
                                 VB_LOG_MAX_EXTRA_LEN)
#define VB_LOG_FMT              "[%7s][%30s][%5s][%40s][%02d:%02d:%02d %03dms]"

#define VB_LOG_MAX_DRIVER_HDR_LEN (VB_EA_DRIVER_ID_MAX_SIZE + 2) // Take into account [] (+2)
#define VB_LOG_MAX_EXT_HDR_LEN  (VB_LOG_MAX_HDR_LEN + VB_LOG_MAX_DRIVER_HDR_LEN)
#define VB_LOG_DRIVER_ID_FMT    "[%-20s]"

#define VB_LOG_MAX_LINE_LEN     200

#define VB_LOG_ENTRY_LEN        (VB_LOG_MAX_HDR_LEN + VB_LOG_MAX_LINE_LEN)
#define VB_LOG_EXT_ENTRY_LEN    (VB_LOG_MAX_EXT_HDR_LEN + VB_LOG_MAX_LINE_LEN)

#if (_WITH_SYSLOG_ == 0)
// Chuck of data log (to file) limits
//
#define VB_LOG_FILENAME_LEN     99
#endif

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

#if (_WITH_SYSLOG_ == 0)
typedef struct
{
  INT8U         type;            // One from "t_vbLogType"

  t_vbLogLevel  verboseLevel;    // Verbose level

  CHAR          *msg;            // The message to send to the Log subsystem. It
                                 // must be a pointer to a dynamically allocated
                                 // buffer which is later automatically "free()'ed"
                                 // by the Log subsystem once the message has been
                                 // processed

  // The next ones are only used when type == VB_LOG_EVENT_SAVETOFILE
  //
  CHAR          *file_name;      // File where data will be written. It must be a
                                 // a pointer to a dynamically allocated buffer
                                 // which is later automatically "free()'ed" by
                                 // the Log subsystem once the message has been
                                 // processed
  const CHAR    *access_mode;    // Either "w+"  or "a+" for overwritting the whole
                                 // file or appending the data at the end
                                 // respectively. It must be a pointer to a
                                 // rodata string that is never changed or freed.
} t_vbLogMsg;


typedef enum {
  VB_LOG_EVENT_PRINT      = 0xa1,  // Message is sent using the default Log
                                   // subsystem output mechanism (ie. socket)
  VB_LOG_EVENT_SAVETOFILE = 0xa2,  // Message is saved to a specific file in the
                                   // file system
  VB_LOG_EVENT_CLOSE      = 0xa3,  // Special event to end the processing of
                                   // future messages and close the queue
} t_vbLogType;

typedef struct
{
  INT32U              numLines;
  t_vbLogLevel        verboseLevel;
  INT32U              currLine;
  INT32U              overflows;
  CHAR               *buffer;
  BOOLEAN             circular;
} t_vbLogPersistent;
#endif

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

#if (_WITH_SYSLOG_ == 0)
static pthread_t    vbLogThread = 0;
#endif
static t_vbLogLevel vbLogVerbose;
#if (_WITH_SYSLOG_ == 0)
static BOOL         vbLogThreadRunning;
static mqd_t        vbLogQueue = { 0 };
static mqd_t        vbLogQueueBlock = { 0 };
static const CHAR   *vbQueueName = NULL;
static CHAR         vbOutputFolder[VB_LOG_FILENAME_LEN+1] = { 0 };
static t_vbLogPersistent vbLogPersistent = { 0 };
#endif

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

// Print a message to stderr, used (internally) when the regular log mechanism
// cannot be used (ex: errors reported by the Log subsystem itself)
static void VbLogErrorPrint(const char *fmt, ...);


#if (_WITH_SYSLOG_ == 0)
// Send the message to the default Log subsystem output (typically a socket, but
// this might change in the future)
static void VbLogWriteToDefault(const char *str);

// Save the message to a file (instead of sending it to the default output).
static void VbLogWriteToFile(const char *file_name, const char *mode, const char *str);
#endif

// Build the "log header" string (that is pre-pended to all messages sent to the
// default log output)
static void VbLogHdrBuild(CHAR *dst, t_vbLogLevel mode, const char *file, INT16U line, const char *function);

#if (_WITH_SYSLOG_ == 0)
// Log subsystem thread
static void *VBLogProcess(void *arg);

// Close opened resources (queues, mallocs...)
static void VbLogDestroy(void *buffer);
#endif

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

#if (_WITH_SYSLOG_ == 0)
static void VbLogStateSet(BOOL running)
{
  vbLogThreadRunning = running;
}

/*******************************************************************/

static BOOL VbLogStateGet(void)
{
  return vbLogThreadRunning;
}
#endif

/*******************************************************************/

static void VbLogErrorPrint(const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);

  fprintf(stderr, "\n");
}

/*******************************************************************/

#if (_WITH_SYSLOG_ == 0)
static void VbLogWriteToDefault(const char *str)
{
  if (str != NULL)
  {
      printf("%s\n", str);
      fflush(stdout);
  }
}

/*******************************************************************/

static void VbLogWriteToFile(const char *file_name, const char *mode, const char *str)
{
  FILE *fd;
  char *aux;

  // Create parent folders
  //
  if((file_name != NULL) && (mode != NULL) && (str != NULL))
  {
    aux = strdup(file_name);
    if(aux != NULL)
    {
      VbUtilCreateFolderAndParents(dirname(aux));
      free(aux);

      fd = fopen(file_name,mode);

      if(fd != NULL)
      {
        fprintf(fd, "%s", str);
        fclose(fd);
      }
      else
      {
        VbLogPrint(VB_LOG_ERROR, "Could not open file %s (%s)", file_name, strerror(errno));
      }
    }
    else
    {
      VbLogPrint(VB_LOG_ERROR, "No Memory for file %s (%s)", file_name, strerror(errno));
    }
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Bad arg (%s)", strerror(errno));
  }
}
#endif

/*******************************************************************/

static void VbLogHdrBuild(CHAR *dst, t_vbLogLevel mode, const char *file, INT16U line, const char *function)
{
  CHAR              mode_str[VB_LOG_MAX_MODE_LEN+1] = { 0 };
  CHAR              file_name_str[VB_LOG_MAX_FILE_LEN+1] = { 0 };
  CHAR              linenum_name_str[VB_LOG_MAX_LINENUM_LEN+1] = { 0 };
  CHAR              func_name_str[VB_LOG_MAX_FUNC_LEN+1] = { 0 };

  time_t            t;
  struct            tm *tmu;
  struct            timeval tv;

  if ((dst != NULL) && (mode < VB_LOG_LAST) && (file != NULL) && (function != NULL))
  {
    t = time(NULL);
    tmu = localtime(&t);

    if (tmu != NULL)
    {
      gettimeofday(&tv, NULL);

      // Copy level string
      snprintf(mode_str, VB_LOG_MAX_MODE_LEN+1, "%7s", VbVerboseLevelToStr(mode));
      mode_str[VB_LOG_MAX_MODE_LEN] = '\0';

      // Copy file name
      strncpy(file_name_str, file, VB_LOG_MAX_FILE_LEN+1);
      file_name_str[VB_LOG_MAX_FILE_LEN] = '\0';

      // Copy line number
      snprintf(linenum_name_str, VB_LOG_MAX_LINENUM_LEN+1, "%05u", line);
      linenum_name_str[VB_LOG_MAX_LINENUM_LEN] = '\0';

      // Copy func name
      strncpy(func_name_str, function, VB_LOG_MAX_FUNC_LEN+1);
      func_name_str[VB_LOG_MAX_FUNC_LEN] = '\0';

      {
        // Limit msec to 999 to avoid a compilation warning of GCC-7
        INT32U msec = tv.tv_usec / 1000;

        if (msec >= 1000)
        {
          msec = 999;
        }

        // Build the whole header
        snprintf(dst, VB_LOG_MAX_HDR_LEN+1, VB_LOG_FMT,
            mode_str, file_name_str, linenum_name_str, func_name_str,
            tmu->tm_hour, tmu->tm_min, tmu->tm_sec, msec);
      }
    }
  }
}

/*******************************************************************/

#if (_WITH_SYSLOG_ == 0)
static inline CHAR* PersistentLogLineGet(INT32U lineIdx)
{
  CHAR *ret;

  if (lineIdx < vbLogPersistent.numLines)
  {
    ret = vbLogPersistent.buffer + (lineIdx * VB_LOG_EXT_ENTRY_LEN);
  }
  else
  {
    ret = NULL;
  }

  return ret;
}

/*******************************************************************/

static void PersistentLogInsert(t_vbLogMsg *vbLogMsg)
{
  if ((vbLogMsg != NULL) &&
      (vbLogMsg->verboseLevel <= vbLogPersistent.verboseLevel) &&
      (vbLogMsg->msg != NULL) &&
      (vbLogPersistent.buffer != NULL))
  {
    if ((vbLogPersistent.circular == TRUE) ||
        (vbLogPersistent.currLine < vbLogPersistent.numLines))
    {
      CHAR *ptr_to_write = PersistentLogLineGet(vbLogPersistent.currLine);

      if (ptr_to_write != NULL)
      {
        // Copy string to buffer
        strncpy(ptr_to_write, vbLogMsg->msg, VB_LOG_EXT_ENTRY_LEN * sizeof(CHAR));

        // Update index
        vbLogPersistent.currLine++;

        if ((vbLogPersistent.circular == TRUE) &&
            (vbLogPersistent.currLine >= vbLogPersistent.numLines))
        {
          vbLogPersistent.currLine = 0;
          vbLogPersistent.overflows++;
        }
      }
    }
    else
    {
      vbLogPersistent.overflows++;
    }
  }
}

/*******************************************************************/

static void *VBLogProcess(void *arg)
{
  INT32S err = 0;
  ssize_t numRead;
  t_vbLogMsg *vb_log_msg;
  struct mq_attr attr;
  void *buffer = NULL;

  if (mq_getattr(vbLogQueueBlock, &attr) == -1)
  {
    VbLogErrorPrint("Error mq_getattr. errno %s", strerror(errno));
    err = -1;
  }

  if (err == 0)
  {
    buffer = calloc(1, attr.mq_msgsize);

    if (buffer == NULL)
    {
      VbLogErrorPrint("Error malloc. errno %s", strerror(errno));
      err = -1;
    }
  }

  while ((VbLogStateGet() == TRUE) && (err == 0))
  {
    numRead = mq_receive(vbLogQueueBlock, buffer, attr.mq_msgsize, NULL);
    if(numRead < 0)
    {
      //Fatal error thread
      VbLogErrorPrint("Log mq_receive error. errno %s",strerror(errno));
    }
    else if(numRead != sizeof(t_vbLogMsg))
    {
      VbLogErrorPrint("Message size error, numbytes received %d",numRead);
    }
    else
    {
      vb_log_msg = (t_vbLogMsg *)buffer;

#if (_VALGRIND_ == 1)
      ANNOTATE_HAPPENS_AFTER(vb_log_msg->msg);
#endif

      switch(vb_log_msg->type)
      {
        case VB_LOG_EVENT_PRINT:
        {
          if (vb_log_msg->msg != NULL)
          {
            // Insert in persistent buffer (if needed)
            PersistentLogInsert(vb_log_msg);

            if (vb_log_msg->verboseLevel <= vbLogVerbose)
            {
              VbLogWriteToDefault((const char *)vb_log_msg->msg);
            }

            free(vb_log_msg->msg);
            vb_log_msg->msg = NULL;
          }
          break;
        }
        case VB_LOG_EVENT_SAVETOFILE:
        {
          if (vb_log_msg->msg != NULL)
          {
            VbLogWriteToFile(vb_log_msg->file_name, vb_log_msg->access_mode, (const char *)vb_log_msg->msg);

            free(vb_log_msg->msg);
            vb_log_msg->msg = NULL;

            free(vb_log_msg->file_name);
            vb_log_msg->file_name = NULL;
          }
          break;
        }
        case VB_LOG_EVENT_CLOSE:
        {
          err = -1; // To abort "while" loop and finish thread
          break;
        }
        default:
        {
          VbLogErrorPrint("Unknown message type received (%d)",vb_log_msg->type);
          break;
        }
      }
    }
  }

  if (buffer != NULL)
  {
    VbLogDestroy(buffer);
    free(buffer);
    buffer = NULL;
  }

  return NULL;
}

/*******************************************************************/

static void VbLogDestroy(void *buffer)
{
  struct mq_attr attr;
  ssize_t numRead = 0;
  t_vbLogMsg *vb_log_msg;
  BOOL flushing = TRUE;

  printf("Closing Log Queue...\n");

  if (mq_getattr(vbLogQueue, &attr) == -1)
  {
    VbLogErrorPrint("Error mq_getattr. errno %s", strerror(errno));
  }

  while (flushing)
  {
    numRead = mq_receive(vbLogQueue, buffer, attr.mq_msgsize, NULL);
    if(numRead <= 0)
    {
      flushing = FALSE;
    }
    else if(numRead != sizeof(t_vbLogMsg))
    {
      VbLogErrorPrint("Cleanning queue numbytes received error");
    }
    else
    {
      vb_log_msg = (t_vbLogMsg *)buffer;
      if(vb_log_msg->msg != NULL)
      {
        free(vb_log_msg->msg);
        vb_log_msg = NULL;
      }
    }
  }

  mq_close(vbLogQueue);
  mq_close(vbLogQueueBlock);
  mq_unlink(vbQueueName);

  printf("Closed Log Queue...\n");
}
#endif

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

INT32S VbLogInit(const char *queueName, t_vbLogLevel verboseLevel, CHAR *outputFolder, INT32U persLogNumLines, t_vbLogLevel persLogVerbose, BOOLEAN circular)
{
#if (_WITH_SYSLOG_ == 0)
  INT32S ret = 0;
  struct mq_attr attr;

  vbLogVerbose              = verboseLevel;
  vbQueueName               = queueName;
  VbLogStateSet(FALSE);

  strncpy(vbOutputFolder, outputFolder, VB_LOG_FILENAME_LEN+1);
  vbOutputFolder[VB_LOG_FILENAME_LEN] = '\0';

  bzero(&attr, sizeof(attr));
  attr.mq_maxmsg  = 200;
  attr.mq_msgsize = sizeof(t_vbLogMsg);

  mq_unlink(vbQueueName);
  vbLogQueue      = mq_open(vbQueueName, O_CREAT | O_RDWR | O_NONBLOCK, 0666, &attr);
  vbLogQueueBlock = mq_open(vbQueueName, O_CREAT | O_RDWR, 0666, &attr);

  vbLogPersistent.numLines     = persLogNumLines;
  vbLogPersistent.verboseLevel = persLogVerbose;
  vbLogPersistent.circular     = circular;

  if (persLogNumLines > 0)
  {
    INT32U size = persLogNumLines * VB_LOG_EXT_ENTRY_LEN * sizeof(CHAR);
    vbLogPersistent.buffer     = (CHAR *)malloc(size);

    if (vbLogPersistent.buffer == NULL)
    {
      ret = -1;
      printf("No available memory to allocate persistent log (required %u bytes)\n", size);
    }
  }
  else
  {
    vbLogPersistent.buffer     = NULL;
  }

  VbLogPersistentReset();

  return ret;
#else
  vbLogVerbose = verboseLevel;

  return 0;
#endif
}

/*******************************************************************/

BOOLEAN VbLogRun()
{
#if (_WITH_SYSLOG_ == 0)
  BOOLEAN return_value;

  VbLogStop();

  VbLogPrint(VB_LOG_INFO, "Starting %s thread", VB_LOG_THREAD_NAME);

  VbLogStateSet(TRUE);
  return_value = VbThreadCreate(VB_LOG_THREAD_NAME, VBLogProcess, NULL, VB_LOG_THREAD_PRIORITY, &vbLogThread);

  if (return_value == FALSE)
  {
    VbLogErrorPrint("Can't create %s thread", VB_LOG_THREAD_NAME);
    VbLogStateSet(FALSE);
  }

  return return_value;
#else
  return TRUE;
#endif
}

/*******************************************************************/

void VbLogStop()
{
#if (_WITH_SYSLOG_ == 0)
  if (VbLogStateGet() == TRUE)
  {
    t_vbLogMsg        vb_log_msg;

    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...\n", VB_LOG_THREAD_NAME);

    bzero(&vb_log_msg, sizeof(vb_log_msg));
    vb_log_msg.type = VB_LOG_EVENT_CLOSE;

    if(0 != (mq_send(vbLogQueue, ((const char *)(&vb_log_msg)), sizeof(t_vbLogMsg), VB_THREADMSG_PRIORITY)))
    {
      VbLogErrorPrint("Error posting CLOSE msg to Log queue [%s]", strerror(errno));
    }

    VbThreadJoin(vbLogThread , VB_LOG_THREAD_NAME);

    printf("Stopped %s thread!\n", VB_LOG_THREAD_NAME);
  }
#endif
}

/*******************************************************************/

void VbLogPrint_helper(const char *current_file_name, INT16U current_line_number, const char *current_function_name, t_vbLogLevel verboseLevel, const char *fmt, ...)
{
  va_list           args;
  CHAR             *log_line = NULL;
  CHAR             *dst = NULL;
#if (_WITH_SYSLOG_ == 0)
  t_vbLogMsg        vb_log_msg;
#endif

  if ((verboseLevel <= vbLogVerbose)
#if (_WITH_SYSLOG_ == 0)
      || (verboseLevel <= vbLogPersistent.verboseLevel)
#endif
      )
  {
    log_line = (CHAR *)calloc(1, VB_LOG_ENTRY_LEN+1);

    if (log_line == NULL)
    {
      VbLogErrorPrint("No memory to allocate log_line (%d bytes)", VB_LOG_ENTRY_LEN+1);
    }
    else
    {
      // Build header
      VbLogHdrBuild(log_line, verboseLevel, current_file_name, current_line_number, current_function_name);

      // Locate pointer after header
      dst = log_line + VB_LOG_MAX_HDR_LEN;

      va_start(args, fmt);
      vsnprintf(dst, VB_LOG_MAX_LINE_LEN, fmt, args);
      va_end(args);

#if (_WITH_SYSLOG_ == 0)
      bzero(&vb_log_msg, sizeof(vb_log_msg));
      vb_log_msg.type = VB_LOG_EVENT_PRINT;
      vb_log_msg.msg = log_line;
      vb_log_msg.verboseLevel = verboseLevel;

      if (vb_log_msg.msg != NULL)
      {
#if (_VALGRIND_ == 1)
        ANNOTATE_HAPPENS_BEFORE(vb_log_msg.msg);
#endif
        if(0 != (mq_send(vbLogQueue, ((const char *)(&vb_log_msg)), sizeof(t_vbLogMsg), VB_THREADMSG_PRIORITY)))
        {
          free(vb_log_msg.msg);
          vb_log_msg.msg = NULL;

          VbLogErrorPrint("Error posting PRINT msg to Log queue [%s]", strerror(errno));
        }
      }
#else
      syslog(LOG_INFO, "%s", log_line);
#endif
    }
  }
}

/*******************************************************************/

void VbLogPrintExt_helper(const char *current_file_name, INT16U current_line_number, const char *current_function_name, t_vbLogLevel verboseLevel, const char *driverId, const char *fmt, ...)
{
  va_list           args;
  CHAR             *log_line = NULL;
  CHAR             *dst = NULL;
#if (_WITH_SYSLOG_ == 0)
  t_vbLogMsg        vb_log_msg;
#endif
  CHAR              driver_id_str[VB_EA_DRIVER_ID_MAX_SIZE];

  if ((verboseLevel <= vbLogVerbose)
#if (_WITH_SYSLOG_ == 0)
      || (verboseLevel <= vbLogPersistent.verboseLevel)
#endif
     )
  {
    log_line = (CHAR *)calloc(1, VB_LOG_EXT_ENTRY_LEN+1);

    if (log_line == NULL)
    {
      VbLogErrorPrint("No memory to allocate log_line (%d bytes)", VB_LOG_EXT_ENTRY_LEN+1);
    }
    else
    {
      // Build header
      VbLogHdrBuild(log_line, verboseLevel, current_file_name, current_line_number, current_function_name);

      // Locate pointer after base header
      dst = log_line + VB_LOG_MAX_HDR_LEN;

      // Copy driver Id
      strncpy(driver_id_str, driverId, VB_EA_DRIVER_ID_MAX_SIZE);
      driver_id_str[VB_EA_DRIVER_ID_MAX_SIZE - 1] = '\0';

      // Dump DriverId
      snprintf(dst, VB_LOG_MAX_DRIVER_HDR_LEN, VB_LOG_DRIVER_ID_FMT, driver_id_str);

      // Locate pointer after driverId header
      dst += VB_LOG_MAX_DRIVER_HDR_LEN - 1;  // -1 to locate ptr just in '\0' character

      va_start(args, fmt);
      vsnprintf(dst, VB_LOG_MAX_LINE_LEN, fmt, args);
      va_end(args);

#if (_WITH_SYSLOG_ == 0)
      bzero(&vb_log_msg, sizeof(vb_log_msg));
      vb_log_msg.type = VB_LOG_EVENT_PRINT;
      vb_log_msg.msg = log_line;
      vb_log_msg.verboseLevel = verboseLevel;

#if (_VALGRIND_ == 1)
      ANNOTATE_HAPPENS_BEFORE(vb_log_msg.msg);
#endif
      if(0 != (mq_send(vbLogQueue, ((const char *)(&vb_log_msg)), sizeof(t_vbLogMsg), VB_THREADMSG_PRIORITY)))
      {
        free(vb_log_msg.msg);
        vb_log_msg.msg = NULL;

        VbLogErrorPrint("Error posting PRINT msg to Log queue [%s]", strerror(errno));
      }
#else
      syslog(LOG_INFO, "%s", log_line);
#endif
    }
  }
}

/*******************************************************************/

void VbLogSaveToTextFile(const char *file_name, const char *access_mode, int maxLen, const char *fmt, ...)
{
#if (_WITH_SYSLOG_ == 0)
  va_list           args;
  CHAR             *fn = NULL;
  CHAR             *log_data = NULL;
  t_vbLogMsg        vb_log_msg;
  INT16U            file_name_length;

  log_data = (CHAR *)malloc(maxLen);

  if (log_data == NULL)
  {
    VbLogErrorPrint("No memory to allocate log_data (%d bytes)", maxLen);
  }
  else
  {
    va_start(args, fmt);
    vsnprintf(log_data, maxLen, fmt, args);
    va_end(args);

    file_name_length = strlen(file_name);

    if (file_name_length > 0)
    {
      file_name_length += strlen(vbOutputFolder) + 3;

      fn = (CHAR *)calloc(1, file_name_length);

      if (fn == NULL)
      {
        VbLogErrorPrint("No memory to allocate fn (%d bytes)", file_name_length);
        free(log_data);
      }
      else
      {
        //strncpy(fn, file_name, VB_LOG_FILENAME_LEN+1);
        snprintf(fn, file_name_length, "%s/%s", vbOutputFolder, file_name);

        bzero(&vb_log_msg, sizeof(vb_log_msg));
        vb_log_msg.type        = VB_LOG_EVENT_SAVETOFILE;
        vb_log_msg.msg         = log_data;
        vb_log_msg.file_name   = fn;
        vb_log_msg.access_mode = access_mode;

        if (vb_log_msg.msg != NULL)
        {
          if(0 != (mq_send(vbLogQueueBlock, ((const char *)(&vb_log_msg)), sizeof(t_vbLogMsg),VB_THREADMSG_PRIORITY)))
          {
            free(vb_log_msg.msg);
            vb_log_msg.msg = NULL;

            free(vb_log_msg.file_name);
            vb_log_msg.file_name = NULL;

            VbLogErrorPrint("Error posting SAVETODISK msg to Log queue [%s]", strerror(errno));
          }
        }
      }
    }
    else
    {
      free(log_data);
    }
  }
#endif
}

/*******************************************************************/

t_vbLogLevel VbLogVerboseLevelGet(void)
{
  return vbLogVerbose;
}

/*******************************************************************/

#if (_WITH_SYSLOG_ == 0)
void VbLogPersistentReset(void)
{
  vbLogPersistent.currLine = 0;
  vbLogPersistent.overflows = 0;
}
#endif

/*******************************************************************/

void VbLogPersistentDump(t_writeFun writeFun)
{
#if (_WITH_SYSLOG_ == 0)
  if (writeFun != NULL)
  {
    INT32U   num_lines;
    time_t   t;
    struct   tm *tmu;
    struct   timeval tv;

    if (vbLogPersistent.overflows > 0)
    {
      num_lines = vbLogPersistent.numLines;
    }
    else
    {
      num_lines = vbLogPersistent.currLine;
    }

    t = time(NULL);
    tmu = localtime(&t);
    gettimeofday(&tv, NULL);

    writeFun("[%02d:%02d:%02d %03dms]\n", tmu->tm_hour, tmu->tm_min, tmu->tm_sec, (int)(tv.tv_usec/ 1000));
    writeFun("Circular mode = %s\n", vbLogPersistent.circular?"ENABLED":"DISABLED");
    writeFun("Stored lines  = %u\n", num_lines);
    writeFun("Overflows     = %u\n", vbLogPersistent.overflows);

    if (vbLogPersistent.buffer != NULL)
    {
      INT32U line_idx;
      INT32U iters     = 0;
      INT32U line_idx_start;
      INT32U line_idx_stop;

      if ((vbLogPersistent.circular == TRUE) &&
          (vbLogPersistent.overflows > 0))
      {
        line_idx_start = vbLogPersistent.currLine;
        line_idx_stop = vbLogPersistent.currLine - 1;

        if (line_idx_stop >= vbLogPersistent.numLines)
        {
          line_idx_stop = vbLogPersistent.numLines - 1;
        }
      }
      else
      {
        line_idx_start = 0;
        line_idx_stop = vbLogPersistent.currLine;
      }

      line_idx = line_idx_start;
      while ((line_idx_start != line_idx_stop) &&
             (iters < num_lines) &&
             (line_idx < vbLogPersistent.numLines))
      {
        writeFun("[%4u] %s\n", iters, PersistentLogLineGet(line_idx));

        // Increase line index
        line_idx++;

        if (line_idx >= vbLogPersistent.numLines)
        {
          line_idx = 0;
        }

        // Update iters
        iters++;
      }
    }
  }
#endif
}

/*******************************************************************/

BOOL VbLogConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
#if (_WITH_SYSLOG_ == 0)
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] == NULL)
  {
    VbLogPersistentDump(writeFun);
    ret = TRUE;
  }
  else
  {
    if (!strcmp(cmd[1], "r"))
    {
      VbLogPersistentReset();
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
    }
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("log h           : Shows this help\n");
    writeFun("log             : Dumps log buffer\n");
    writeFun("log r           : Resets log buffer\n");
  }

  return ret;
#else
  return FALSE;
#endif
}

/*******************************************************************/

/**
 * @}
 **/
