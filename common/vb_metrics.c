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
 * @file vb_metrics.c
 * @brief Generic metrics used inside vector boost driver and engine
 *
 * @internal
 *
 * @author pablom
 * @date 13/1/2017
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <mqueue.h>
#include <sys/sysinfo.h>
#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_log.h"
#include "vb_thread.h"
#include "vb_util.h"
#include "vb_metrics.h"
#include "vb_console.h"
#include "vb_priorities.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define METRICS_THREAD_NAME                ("Metrics")

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

#define VB_METRICS_MAX_NR_REPORTS      (16)
#define VB_METRICS_FIXED_NAME_SIZE     (44)
#define VB_METRICS_OUTPUT_FOLDER       "%s/Metrics_%04d-%02d-%02d/"
#define VB_METRICS_MSGQ_SIZE           (200)
#define VB_METRICS_DEFAULT_BUFFER_MODE EVENTS_BUFF_CIRCULAR

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

const CHAR *metricsEventsDescription[VB_METRICS_MAX_NR_EVENT_TYPES] =
{
    "No event",
    "Start low channel capacity calculation", //VB_METRICS_EVENT_START_LOW_CHANN_CAP, // 1
    "End low channel capacity calculation",   //VB_METRICS_EVENT_END_LOW_CHANN_CAP,   // 2
    "Start full channel capacity calculation",//VB_METRICS_EVENT_START_FULL_CHANN_CAP,// 3
    "End full channel capacity calculation",  //VB_METRICS_EVENT_END_FULL_CHANN_CAP,  // 4
    "Enter Boosted mode",                     //VB_METRICS_EVENT_BOOSTING_START,      // 5
    "Exit Boosted mode",                      //VB_METRICS_EVENT_BOOSTING_STOP,       // 6
    "Start SNR SISO calculation",             //VB_METRICS_EVENT_START_SNR_SISO_CALC, // 7
    "End SNR SISO calculation",               //VB_METRICS_EVENT_END_SNR_SISO_CALC,   // 8
    "Start SNR MIMO calculation",             //VB_METRICS_EVENT_START_SNR_MIMO_CALC, // 9
    "End SNR MIMO calculation",               //VB_METRICS_EVENT_END_SNR_MIMO_CALC,   // 10
    "Time Markers report",                    //VB_METRICS_EVENT_TIME_MARKERS,        // 11
    "Generic traffic report",                 //VB_METRICS_EVENT_TRAFFIC_REPORT,      // 12 - Generic traffic event
    "Traffic went over 105 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_105,    // 13
    "Traffic went over 205 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_205,    // 14
    "Traffic went over 305 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_305,    // 15
    "Traffic went over 405 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_405,    // 16
    "Traffic went over 505 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_505,    // 17
    "Traffic went over 605 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_605,    // 18
    "Traffic went over 705 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_705,    // 19
    "Traffic went over 805 Mbps",             //VB_METRICS_EVENT_TRAFFIC_OVER_805,    // 20
    "Traffic went under 95 Mbps",             //VB_METRICS_EVENT_TRAFFIC_UNDER_95,    // 21
    "Traffic went under 195 Mbps",            //VB_METRICS_EVENT_TRAFFIC_UNDER_195,   // 22
    "Traffic went under 295 Mbps",            //VB_METRICS_EVENT_TRAFFIC_UNDER_295,   // 23
    "Traffic went under 395 Mbps",            //VB_METRICS_EVENT_TRAFFIC_UNDER_395,   // 24
    "Traffic went under 495 Mbps",            //VB_METRICS_EVENT_TRAFFIC_UNDER_495,   // 25
    "Traffic went under 595 Mbps",            //VB_METRICS_EVENT_TRAFFIC_UNDER_595,   // 26
    "Traffic went under 695 Mbps",            //VB_METRICS_EVENT_TRAFFIC_UNDER_695,   // 27
    "Traffic went under 795 Mbps",            //VB_METRICS_EVENT_TRAFFIC_UNDER_795,   // 28
    "Start measurement plan calculations",    //VB_METRICS_EVENT_START_MEAS_PLAN,     // 29
    "End measurement plan calculations",      //VB_METRICS_EVENT_END_MEAS_PLAN,       // 30
    "New Line detected",                      //31
    "Line lost detection",                    //32
    "Start alignment process",                //33
    "Restart alignment process",              //34
    "Alignment check start process",          //35
    "Alignment check end process",            //36
    "Alignment process info",                 //37
    "Stop alignment process",                 //38
};

static BOOL (*metricsEventsProcess[VB_METRICS_MAX_NR_EVENT_TYPES])(t_VB_MetricsEvent *) = { 0 };

static t_VB_MetricsEvent *vbMetricsEventsList = NULL;
static INT32U vbMetricsListLength;
static INT32U vbMetricsCurrentWrIndex;
static INT32U vbMetricsCurrentRdIndex;
static BOOLEAN vbMetricsIndexOverflow; // Whether the buffer already overflowed and started again
static t_VbMetricsBufferType vbMetricsBufferMode; // Circular buffer or normal buffer
static BOOLEAN vbMetricsEventTypeEnable[VB_METRICS_MAX_NR_EVENT_TYPES];
static BOOLEAN vbMetricsRunning = FALSE; // Enable or disable metrics
static struct timespec vbMetricsStartupTime;
static time_t vbMetricsBootTime;

// Array with externally defined report functions
static t_VB_MetricsReport vbMetricsReportsList[VB_METRICS_MAX_NR_REPORTS];
// Array used to calculate average time values of high frequent events
static t_VBMetricsTimeMarker vbMetricsTimeMarkersList[VB_METRICS_MAX_NR_TIME_MARKERS];

static CHAR  vbMetricsCurrentPath[VB_ENGINE_METRICS_MAX_PATH_LEN+VB_METRICS_FIXED_NAME_SIZE]; // this path is re-generated with Start
static CHAR  vbOutputPath[VB_ENGINE_METRICS_MAX_PATH_LEN]; // engine reports path

static pthread_mutex_t vbMetricsEventsListMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       vbMetricsThread;
static mqd_t           vbMetricsQueue;
static CHAR           *vbMetricsQueueName;
static BOOL            vbMetricsThreadRunning;

/*
 ************************************************************************
 ** Private function declaration
 ************************************************************************
 */

static t_VbMetricsErrorType VbMetricsInsertEvent(t_VB_MetricsEvent *event);
static void VbMetricsCloseMQueue(void *buffer);
static void *VbThreadMetrics(void *arg);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

/**
 * @brief Insert a new event in the metrics buffer. This function is called
 * from the metrics thread. Not all events are saved in the buffer.
 * @param[in] event Event containig all info
 * @return @ref t_VbMetricsErrorType
 *
 * @pre data must be allocated with malloc, since the module will free it
 * automatically if needed
 **/
static t_VbMetricsErrorType VbMetricsInsertEvent(t_VB_MetricsEvent *event)
{
  t_VbMetricsErrorType error = VB_METRICS_NO_ERROR;

  if(event == NULL)
  {
    error = VB_METRICS_ERROR_MEMORY;
  }
  // Check if Metrics storage is enabled
  else if(vbMetricsRunning == FALSE)
  {
    // Just discard event
    error = VB_METRICS_ERROR_STOPPED; // These are not really errors
    free(event->eventData);
    event->eventData = NULL;
  }
  else
  {
    // Check array index boundaries
    if(vbMetricsCurrentWrIndex < vbMetricsListLength)
    {
      pthread_mutex_lock( &vbMetricsEventsListMutex );

      if(vbMetricsEventsList[vbMetricsCurrentWrIndex].eventData != NULL)
      {
        // Free the overwritten event
        free(vbMetricsEventsList[vbMetricsCurrentWrIndex].eventData);
        vbMetricsEventsList[vbMetricsCurrentWrIndex].eventData = NULL;
      }

      memcpy(&(vbMetricsEventsList[vbMetricsCurrentWrIndex]), event, sizeof(t_VB_MetricsEvent));

      // Increment pointer
      if(vbMetricsBufferMode == EVENTS_BUFF_CIRCULAR)
      {
        // Check array index boundaries
        if(++vbMetricsCurrentWrIndex >= vbMetricsListLength)
        {
          // Buffer full event
          pthread_mutex_unlock( &vbMetricsEventsListMutex );
          // dump information to files
          VbMetricsDumpReportsToFiles();

          pthread_mutex_lock( &vbMetricsEventsListMutex );
          vbMetricsIndexOverflow = 1;
          vbMetricsCurrentWrIndex = 0;
        }
      }
      else
      {
        // Check array index boundaries
        if(++vbMetricsCurrentWrIndex >= vbMetricsListLength)
        {
          vbMetricsIndexOverflow = 1;
        }
        // We catched the Rd pointer - this should not happen
        if(vbMetricsCurrentWrIndex == vbMetricsCurrentRdIndex)
        {
          vbMetricsCurrentRdIndex++;
        }
      }
      pthread_mutex_unlock( &vbMetricsEventsListMutex );
    }
  }

  return error;
}

/*******************************************************************/

static void VbMetricsCloseMQueue(void *buffer)
{
  struct mq_attr attr;
  t_VB_MetricsEvent *vb_event;

  VbLogPrint(VB_LOG_INFO, "Closing Metrics Queue...");

  if (mq_getattr(vbMetricsQueue, &attr) == -1)
  {
    VbLogPrint(VB_LOG_ERROR,"Error mq_getattr. errno %s", strerror(errno));
  }

  if (attr.mq_curmsgs != 0)
  {
    // There are some messages on this queue....eat em

    // First set the queue to not block any calls
    attr.mq_flags = O_NONBLOCK;
    mq_setattr (vbMetricsQueue, &attr, NULL);

    // Now eat all of the messages
    while (mq_receive (vbMetricsQueue, buffer, sizeof(t_VB_MetricsEvent), VB_THREADMSG_PRIORITY) != -1)
    {
      vb_event = (t_VB_MetricsEvent *)buffer;
      // Free data
      free(vb_event->eventData);
    }
  }

  mq_close(vbMetricsQueue);
  mq_unlink(vbMetricsQueueName);

  VbLogPrint(VB_LOG_INFO, "Closed Metrics Queue...");
}

/*******************************************************************/

static void *VbThreadMetrics(void *arg)
{
  ssize_t numRead;
  t_VB_MetricsEvent *vb_event;
  struct mq_attr attr;
  void *buffer = NULL;
  BOOL insert;
  t_VbMetricsErrorType error;

  if (mq_getattr(vbMetricsQueue, &attr) == -1)
  {
    VbLogPrint(VB_LOG_ERROR,"Error mq_getattr. errno %s", strerror(errno));
    vbMetricsThreadRunning = FALSE;
  }

  if(vbMetricsThreadRunning)
  {
    buffer = calloc(1, attr.mq_msgsize);
    if (buffer == NULL)
    {
      VbLogPrint(VB_LOG_ERROR,"Error malloc. errno %s", strerror(errno)); // ??
      vbMetricsThreadRunning = FALSE;
    }
  }

  while(vbMetricsThreadRunning)
  {
    numRead = mq_receive(vbMetricsQueue, buffer, attr.mq_msgsize, NULL);
    if(numRead < 0)
    {
      //Fatal error thread
      VbLogPrint(VB_LOG_ERROR,"Metrics mq_receive error. errno %s",strerror(errno));
    }
    else if(numRead != sizeof(t_VB_MetricsEvent))
    {
      VbLogPrint(VB_LOG_ERROR,"Message size error, numbytes received %d",numRead);
      vbMetricsThreadRunning = FALSE;
    }
    else
    {
      vb_event = (t_VB_MetricsEvent *)buffer;

#if (_VALGRIND_ == 1)
      ANNOTATE_HAPPENS_AFTER(vb_event->eventData);
#endif

      if(vb_event->eventType == VB_METRICS_EVENT_CLOSE)
      {
        vbMetricsThreadRunning = FALSE;
      }
      else
      {
        insert = TRUE;

        // Call process function if it exists
        //
        if((vb_event->eventType < VB_METRICS_MAX_NR_EVENT_TYPES) &&
            (metricsEventsProcess[vb_event->eventType] != NULL))
        {
          insert = metricsEventsProcess[vb_event->eventType](vb_event);
        }

        // Insert event
        if(insert)
        {
          error = VbMetricsInsertEvent(vb_event);

          if (error != VB_METRICS_NO_ERROR)
          {
            VbLogPrint(VB_LOG_ERROR, "Error %d inserting event", error);
          }
        }
        else
        {
          free(vb_event->eventData);
        }
      }
    }
  };

  if (buffer != NULL)
  {
    // Close MQ
    VbMetricsCloseMQueue(buffer);
    free(buffer);
  }

  return NULL;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_VbMetricsErrorType VbMetricsInit(CHAR *outputPath, CHAR *queueName)
{
  t_VbMetricsErrorType res = VB_METRICS_NO_ERROR;
  struct mq_attr attr;
  int i;

  // Avoid multiple initializations
  if(vbMetricsEventsList != NULL)
  {
    res = VB_METRICS_ERROR_ALREADY_INITIALIZED;
  }
  else
  {
    // Allocate the memory for the events list
    vbMetricsEventsList = (t_VB_MetricsEvent *)calloc(VB_METRICS_EVENTS_BUFFER_LENGTH, sizeof(t_VB_MetricsEvent));

    if(vbMetricsEventsList == NULL)
    {
      res = VB_METRICS_ERROR_MEMORY;
    }
  }

  if(res == VB_METRICS_NO_ERROR)
  {
    // Initialize variables
    //
    vbMetricsCurrentWrIndex = 0;
    vbMetricsCurrentRdIndex = 0;
    vbMetricsListLength = VB_METRICS_EVENTS_BUFFER_LENGTH;
    vbMetricsBufferMode = VB_METRICS_DEFAULT_BUFFER_MODE;

    memset(vbMetricsReportsList, 0, sizeof(t_VB_MetricsReport) * VB_METRICS_MAX_NR_REPORTS);

    // Enable all events as default
    // or enable some events, statically/from xml params
    for(i=0; i<VB_METRICS_MAX_NR_EVENT_TYPES; i++)
    {
      vbMetricsEventTypeEnable[i] = TRUE;
    }

    bzero(&attr, sizeof(attr));
    attr.mq_maxmsg  = VB_METRICS_MSGQ_SIZE;
    attr.mq_msgsize = sizeof(t_VB_MetricsEvent);

    vbMetricsQueueName = queueName;
    mq_unlink(vbMetricsQueueName);
    vbMetricsQueue = mq_open(vbMetricsQueueName, O_CREAT | O_RDWR, 0666, &attr);

    if(vbMetricsQueue == -1)
    {
      res = VB_METRICS_ERROR_MEMORY;
      VbLogPrint(VB_LOG_ERROR,"Cannot create metrics Messages Queue");
    }

    if(outputPath != NULL)
    {
      strncpy(vbOutputPath, outputPath, VB_ENGINE_METRICS_MAX_PATH_LEN-1 );
      vbOutputPath[VB_ENGINE_METRICS_MAX_PATH_LEN - 1] = '\0';
    }
    else
    {
      strcpy(vbOutputPath, ".");
    }
  }

  if (res == VB_METRICS_NO_ERROR)
  {
    VbLogPrint(VB_LOG_INFO, "Starting %s thread", METRICS_THREAD_NAME);

    vbMetricsThreadRunning = TRUE;

    if (FALSE == VbThreadCreate(METRICS_THREAD_NAME, VbThreadMetrics, NULL, VB_CONSOLE_THREAD_PRIORITY, &vbMetricsThread))
    {
      res = VB_METRICS_ERROR_MEMORY;
      VbLogPrint(VB_LOG_ERROR,"Can't create %s thread", METRICS_THREAD_NAME);
      vbMetricsThreadRunning = FALSE;
    }
  }

  return res;
}

/*******************************************************************/

void VbMetricsExit(void)
{
  t_VB_MetricsEvent vb_event = {0, {0,0}, NULL};

  if (vbMetricsThreadRunning == TRUE)
  {
    vb_event.eventType = VB_METRICS_EVENT_CLOSE;

    if(0 != (mq_send(vbMetricsQueue, ((const char *)(&vb_event)), sizeof(t_VB_MetricsEvent), VB_THREADMSG_PRIORITY)))
    {
      VbLogPrint(VB_LOG_ERROR,"Error posting CLOSE msg to Log queue [%s]", strerror(errno));
    }

    VbThreadJoin(vbMetricsThread , METRICS_THREAD_NAME);
  }
}

/*******************************************************************/

void VbMetricsDestroy(void)
{
  INT32U i;

  for(i=0; i<vbMetricsListLength; i++)
  {
    free(vbMetricsEventsList[i].eventData);
  }
  free(vbMetricsEventsList);

  // Free reports list contents
  for(i=0 ; i<VB_METRICS_MAX_NR_REPORTS; i++)
  {
    if(vbMetricsReportsList[i].description != NULL)
    {
      free(vbMetricsReportsList[i].description);
    }

    if(vbMetricsReportsList[i].file != NULL)
    {
      free(vbMetricsReportsList[i].file);
    }
  }
}

/*******************************************************************/

void VbMetricsGetElapsedTime(struct timespec *timeFromStart)
{
  struct timespec tv;
  // Get the current elapsed time from startup
  clock_gettime(CLOCK_MONOTONIC,&tv);

  tv.tv_sec += vbMetricsBootTime;

  VbUtilTimespecSubtract(&vbMetricsStartupTime, &tv, timeFromStart);
}

/*******************************************************************/

void VbMetricsCalculateTimeDiffFromStart(struct timespec *timeRef, struct timespec *timeDiff)
{
  VbUtilTimespecSubtract(&vbMetricsStartupTime, timeRef, timeDiff);
}

/*******************************************************************/

t_VbMetricsErrorType VbMetricsReportEvent(t_VBMetricsEventType type, void *data)
{
  t_VbMetricsErrorType error = VB_METRICS_NO_ERROR;
  t_VB_MetricsEvent event;

  // Check if event type is enabled
  if(vbMetricsEventTypeEnable[type] == FALSE)
  {
    // Just discard event
    error = VB_METRICS_ERROR_EV_TYPE_DISABLED;
    free(data);
  }
  // Check if Metrics storage is enabled
  else if(vbMetricsRunning == FALSE)
  {
    // Just discard event
    error = VB_METRICS_ERROR_STOPPED; // These are not really errors
    free(data);
  }
  else
  {
    bzero(&event, sizeof(t_VB_MetricsEvent));

    event.eventType = type;
    event.eventData = data;
    clock_gettime(CLOCK_MONOTONIC,&event.eventTime);

    event.eventTime.tv_sec += vbMetricsBootTime;

    if(0 != (mq_send(vbMetricsQueue, ((const char *)(&event)), sizeof(t_VB_MetricsEvent), VB_THREADMSG_PRIORITY)))
    {
      free(event.eventData);

      VbLogPrint(VB_LOG_ERROR,"Error inserting metrics event to msg queue [%s]", strerror(errno));
    }
  }

  return error;
}

/*******************************************************************/

t_VbMetricsErrorType VbMetricsInsertCurrentTimeMarkersValues(void)
{
  t_VbMetricsErrorType error = VB_METRICS_NO_ERROR;
  t_VBMetricsTimeMarker *values;

  // Allocate memory to save a copy of the time markers list
  values = (t_VBMetricsTimeMarker *)malloc(sizeof(t_VBMetricsTimeMarker) * VB_METRICS_MAX_NR_TIME_MARKERS);

  if(values == NULL)
  {
    error = VB_METRICS_ERROR_MEMORY;
  }

  if(error == VB_METRICS_NO_ERROR)
  {
    // Copy the current status of the counters
    memcpy(values, vbMetricsTimeMarkersList, sizeof(t_VBMetricsTimeMarker) * VB_METRICS_MAX_NR_TIME_MARKERS);
  }

  if(error == VB_METRICS_NO_ERROR)
  {
    // Insert new event with these values
    error = VbMetricsReportEvent(VB_METRICS_EVENT_TIME_MARKERS, values);
  }

  return error;
}

/*******************************************************************/

t_VbMetricsErrorType VbMetricsGetCurrentTimeMarkersValues(t_VBMetricsTimeMarker *values)
{
  t_VbMetricsErrorType error = VB_METRICS_NO_ERROR;

  if(values == NULL)
  {
    error = VB_METRICS_ERROR_MEMORY;
  }

  if(error == VB_METRICS_NO_ERROR)
  {
    // Copy the current status of the counters
    memcpy(values, vbMetricsTimeMarkersList, sizeof(t_VBMetricsTimeMarker) * VB_METRICS_MAX_NR_TIME_MARKERS);
  }

  return error;
}

/*******************************************************************/

t_VbMetricsErrorType VbMetricsInsertTimeMarker(t_VBMetricsTimeMarkerType type)
{
  t_VbMetricsErrorType error = VB_METRICS_NO_ERROR;
  INT64S time_diff;
  struct timespec time_now;
  INT32U index;
  BOOL start_marker;

  // Check if Metrics storage is enabled
  if(vbMetricsRunning == FALSE)
  {
    // Just discard event
    error = VB_METRICS_ERROR_STOPPED;
  }
  else
  {
    // We don't use mutex here because we want this function to consume
    // as few time as possible. Furthermore, if we call try to insert the
    // same time marker from different threads, it will cause wrong results
    // Calculate the index inside the list
    index = (INT32U)type/2;
    start_marker = ((type & 0x1) == 0);

    // Check array index boundaries
    if(index < VB_METRICS_MAX_NR_TIME_MARKERS)
    {
      // Get the current elapsed time from startup
      clock_gettime(CLOCK_MONOTONIC, &time_now);

      time_now.tv_sec += vbMetricsBootTime;

      if(start_marker)
      {
        vbMetricsTimeMarkersList[index].lastBeginTime = time_now;
      }
      else
      {
        time_diff = VbUtilElapsetimeTimespecNs(&vbMetricsTimeMarkersList[index].lastBeginTime, &time_now);
        if(time_diff >= 0)
        {
          vbMetricsTimeMarkersList[index].sumatory += time_diff;
          vbMetricsTimeMarkersList[index].counter++;
        }
        else
        {
          error = VB_METRICS_ERROR_INPUT_PARAM;
        }
      }
    }
  }

  return error;
}

/*******************************************************************/

t_VbMetricsErrorType VbMetricsInsertTimeMarkerAndSetNext(t_VBMetricsTimeMarkerType type, t_VBMetricsTimeMarkerType next)
{
  t_VbMetricsErrorType error = VB_METRICS_NO_ERROR;
  INT64S time_diff;
  struct timespec time_now;
  INT32U index;
  BOOL start_marker;

  // Check if Metrics storage is enabled
  if(vbMetricsRunning == FALSE)
  {
    // Just discard event
    error = VB_METRICS_ERROR_STOPPED;
  }

  if(error == VB_METRICS_NO_ERROR)
  {
    index = (INT32U)type/2;
    start_marker = ((type & 0x1) == 0);

    // It only works with end marks
    if(start_marker || index >= VB_METRICS_MAX_NR_TIME_MARKERS)
    {
      error = VB_METRICS_ERROR_INPUT_PARAM;
    }
  }

  if(error == VB_METRICS_NO_ERROR)
  {
    // Check array index boundaries
    // Get the current elapsed time from startup
    clock_gettime(CLOCK_MONOTONIC, &time_now);

    time_now.tv_sec += vbMetricsBootTime;

    time_diff = VbUtilElapsetimeTimespecNs(&vbMetricsTimeMarkersList[index].lastBeginTime, &time_now);

    if(time_diff > 0)
    {
      vbMetricsTimeMarkersList[index].sumatory += time_diff;
      vbMetricsTimeMarkersList[index].counter++;
    }
    else
    {
      error = VB_METRICS_ERROR_INPUT_PARAM;
    }
  }

  if(error == VB_METRICS_NO_ERROR)
  {
    index = (INT32U)next/2;
    start_marker = ((next & 0x1) == 0);

    // It only works with end marks
    if(index >= VB_METRICS_MAX_NR_TIME_MARKERS)
    {
      error = VB_METRICS_ERROR_INPUT_PARAM;
    }
    else
    {
      vbMetricsTimeMarkersList[index].lastBeginTime = time_now;
    }
  }

  return error;
}

/*******************************************************************/

INT32S VbMetricsGetMarkerAvgTime(t_VBMetricsTimeMarker *timeMarker)
{
  INT32S res = 0;

  if(timeMarker->counter > 0)
  {
    res = (INT32S)(timeMarker->sumatory / timeMarker->counter);
  }
  else
  {
    res = 0;
  }

  // Time in nanoseconds
  return res;
}

/*******************************************************************/

void VbMetricsResetTimeMarkers(void)
{
  int i;

  for(i=0; i<VB_METRICS_MAX_NR_TIME_MARKERS; i++)
  {
    vbMetricsTimeMarkersList[i].sumatory = 0;
    vbMetricsTimeMarkersList[i].counter = 0;
  }
}

/*******************************************************************/

void VbMetricsResetList(void)
{
  INT32U i;

  VbStopMetrics();

  for(i=0; i<vbMetricsListLength; i++)
  {
    free(vbMetricsEventsList[i].eventData);
    vbMetricsEventsList[i].eventData = NULL;
    vbMetricsEventsList[i].eventType = 0;
  }

  vbMetricsIndexOverflow = 0;
  vbMetricsCurrentWrIndex = 0;
  vbMetricsCurrentRdIndex = 0;

  VbStartMetrics();
}

/*******************************************************************/

void VbStartMetrics(void)
{
  struct tm tmu;
  struct sysinfo info;

  if(vbMetricsRunning == FALSE)
  {
    sysinfo(&info);
    vbMetricsBootTime = time(NULL) - info.uptime;

    // Get reference time
    clock_gettime(CLOCK_MONOTONIC, &vbMetricsStartupTime);

    vbMetricsStartupTime.tv_sec += vbMetricsBootTime;

    // Init Logs file path
    localtime_r(&(vbMetricsStartupTime.tv_sec), &tmu);
    snprintf(
        vbMetricsCurrentPath,
        sizeof(vbMetricsCurrentPath)-1,
        VB_METRICS_OUTPUT_FOLDER,
        vbOutputPath,
        tmu.tm_year + 1900, tmu.tm_mon + 1, tmu.tm_mday
    );
  }

  vbMetricsRunning = TRUE;
}

/*******************************************************************/

void VbStopMetrics(void)
{
  vbMetricsRunning = FALSE;
}

/*******************************************************************/

BOOL VbMetricsGetStatus(void)
{
  return vbMetricsRunning;
}

/*******************************************************************/

time_t VbMetricsGetBootTime(void)
{
  return vbMetricsBootTime;
}

/*******************************************************************/

CHAR *VbMetricsGetReportsPath(void)
{
  return vbMetricsCurrentPath;
}

/*******************************************************************/

void VBMetricsEnableEvent(t_VBMetricsEventType type, BOOL enable)
{
  vbMetricsEventTypeEnable[type] = enable;
}

/*******************************************************************/

INT32S VbMetricsGetAllEventsByType(t_VB_MetricsEvent*** eventArr, t_VBMetricsEventType type)
{
  INT32U i, tmp_index;
  INT32U buffer_start_index = 0;
  INT32U number_of_events = 0;
  INT32S res = VB_METRICS_NO_ERROR;

  if(eventArr == NULL)
  {
    res = -VB_METRICS_ERROR_INPUT_PARAM;
  }

  if(res == VB_METRICS_NO_ERROR)
  {
    // In case the base index is not 0
    if((vbMetricsIndexOverflow == 1) &&
       (vbMetricsBufferMode == EVENTS_BUFF_CIRCULAR))
    {
      buffer_start_index = vbMetricsCurrentWrIndex;
    }
    else
    {
      buffer_start_index = 0;
    }
  }

  if(res == VB_METRICS_NO_ERROR)
  {
    for (i = 0; i<vbMetricsListLength; i++)
    {
      if(vbMetricsEventsList[i].eventType == type)
      {
        number_of_events++;
      }
    }
  }

  if ( (res == VB_METRICS_NO_ERROR) && (number_of_events > 0) )
  {
    *eventArr = (t_VB_MetricsEvent **)malloc(sizeof(t_VB_MetricsEvent *) * number_of_events);
    if(*eventArr == NULL)
    {
      number_of_events = -VB_METRICS_ERROR_MEMORY;
    }
    else
    {
      number_of_events = 0;
      for (i = 0, tmp_index = buffer_start_index; i<vbMetricsListLength; i++, tmp_index++)
      {
        tmp_index %= vbMetricsListLength;

        if(vbMetricsEventsList[tmp_index].eventType == type)
        {
          (*eventArr)[number_of_events] = &vbMetricsEventsList[tmp_index];
          number_of_events++;
        }
      }
    }
    res = number_of_events;
  }
  else
  {
    res = 0;
  }

  return res;
}

/*******************************************************************/

INT32S VbMetricsGetNextEventByType(t_VB_MetricsEvent** event, t_VBMetricsEventType type, INT32U offset)
{
  INT32S ev_index = -VB_METRICS_ERROR_EVENT_NOT_FOUND; // Invalid index
  INT32U i, tmp_index;
  INT32U buffer_start_index;

  if(event == NULL || offset > vbMetricsListLength)
  {
    ev_index = -VB_METRICS_ERROR_INPUT_PARAM;
  }
  else
  {
    // In case the base index is not 0
    if((vbMetricsIndexOverflow == 1) &&
      (vbMetricsBufferMode == EVENTS_BUFF_CIRCULAR))
    {
      buffer_start_index = vbMetricsCurrentWrIndex;
    }
    else
    {
      buffer_start_index = 0;
    }

    for (i = 0; i < (vbMetricsListLength-offset); i++)
    {
      tmp_index = buffer_start_index + offset + i; // Begin the search at buffer_start_index + index
      tmp_index = (tmp_index >= vbMetricsListLength) ? (tmp_index - vbMetricsListLength) : tmp_index;

      if(vbMetricsEventsList[tmp_index].eventType == type)
      {
        *event = &vbMetricsEventsList[tmp_index];
        // return relative index so we can use it for the next search
        ev_index = tmp_index - buffer_start_index;
        ev_index = (ev_index < 0)? ev_index + vbMetricsListLength : ev_index;
        break;
      }
    }
  }

  return ev_index;
}

/*******************************************************************/

t_VbMetricsErrorType VbMetricsGetEventByIndex(t_VB_MetricsEvent** event, INT32U index)
{
  t_VbMetricsErrorType res;
  INT32U tmp_index;
  INT32U buffer_start_index;

  if((event == NULL) ||
      (index >= vbMetricsListLength) ||
      (index >= vbMetricsCurrentWrIndex && vbMetricsIndexOverflow == 0))
  {
    res = VB_METRICS_ERROR_INPUT_PARAM;
  }
  else
  {
    // In case the base index is not 0
    if((vbMetricsIndexOverflow == 1) &&
      (vbMetricsBufferMode == EVENTS_BUFF_CIRCULAR))
    {
      buffer_start_index = vbMetricsCurrentWrIndex ;
    }
    else
    {
      buffer_start_index = 0;
    }

    // Index is relative to the start_index
    tmp_index = buffer_start_index + index;
    // Check overflow boundaries
    tmp_index = (tmp_index >= vbMetricsListLength) ? (tmp_index - vbMetricsListLength) : tmp_index;

    *event = &vbMetricsEventsList[tmp_index];
    res = VB_METRICS_NO_ERROR;
  }

  return res;
}

/*******************************************************************/

t_VbMetricsErrorType VbMetricsReadNextEvent(t_VB_MetricsEvent** event)
{
  t_VbMetricsErrorType res;

  if((event == NULL) ||
      (vbMetricsCurrentRdIndex >= vbMetricsListLength))
  {
    res = VB_METRICS_ERROR_INPUT_PARAM;
  }
  else if(vbMetricsCurrentRdIndex == vbMetricsCurrentWrIndex)
  {
    res = VB_METRICS_ERROR_EVENT_NOT_FOUND;
  }
  else
  {
    *event = &vbMetricsEventsList[vbMetricsCurrentRdIndex];
    res = VB_METRICS_NO_ERROR;

    // Increment Read index
    vbMetricsCurrentRdIndex++;
    // Overflow?
    if((vbMetricsCurrentRdIndex >= vbMetricsListLength) && (vbMetricsIndexOverflow == 1))
    {
      vbMetricsCurrentRdIndex = 0;
    }
  }

  return res;
}

/*******************************************************************/

INT64S VbMetricsGetTimeInterval(t_VBMetricsEventType beginType, t_VBMetricsEventType endType, INT32U offset, void **pData)
{
  t_VB_MetricsEvent *start_event, *end_event;
  INT32S start_index;
  INT64S diff = -VB_METRICS_ERROR_EVENT_NOT_FOUND;

  // Get start index
  start_index = VbMetricsGetNextEventByType(&start_event, beginType, 0);

  if (start_index >= 0)
  {
    *pData = start_event->eventData;
    // Get end index
    if(VbMetricsGetNextEventByType(&end_event, endType, start_index) >= 0)
    {
      diff = VbUtilElapsetimeTimespecNs(&(start_event->eventTime), &(end_event->eventTime));
    }
  }

  return diff;
}

/*******************************************************************/

INT32S VbMetricsAddReportHandler(const CHAR *description, t_VbMetricsReportHandler handler, const CHAR *file)
{
  INT32S i;
  INT32S index = -VB_METRICS_ERROR_LIST_FULL;

  if(handler != NULL)
  {
    for(i=0 ; i<VB_METRICS_MAX_NR_REPORTS; i++)
    {
      if(vbMetricsReportsList[i].handler == NULL)
      {
        vbMetricsReportsList[i].handler = handler;

        if(description != NULL)
        {
          vbMetricsReportsList[i].description = strdup(description);
        }
        else
        {
          vbMetricsReportsList[i].description = NULL;
        }

        if(file != NULL)
        {
          vbMetricsReportsList[i].file = strdup(file);
        }
        else
        {
          vbMetricsReportsList[i].file = NULL;
        }

        index = i;
        break;
      }
    }
  }

  return index;
}

/*******************************************************************/

void VbMetricsDumpReportsToFiles(void)
{
  INT32S i;
  CHAR  *buffer;
  CHAR   logsFilePath[VB_ENGINE_METRICS_MAX_PATH_LEN];
  INT32U remaining_size;
  CHAR  *ptr_to_write;

  buffer = (CHAR *)malloc(VB_METRICS_OUTPUT_BUFFER_SIZE);

  if(buffer != NULL)
  {
    for(i=0 ; i<VB_METRICS_MAX_NR_REPORTS; i++)
    {
      if(vbMetricsReportsList[i].handler != NULL)
      {
        if(vbMetricsReportsList[i].file != NULL)
        {
          pthread_mutex_lock( &vbMetricsEventsListMutex );

          ptr_to_write = buffer;
          remaining_size = VB_METRICS_OUTPUT_BUFFER_SIZE;

          // Generate text
          (vbMetricsReportsList[i].handler)(&ptr_to_write, &remaining_size);

          pthread_mutex_unlock( &vbMetricsEventsListMutex );

          // Handler decrements remaining size when writing something to buffer
          if (remaining_size != VB_METRICS_OUTPUT_BUFFER_SIZE)
          {
            // Handler has written something to buffer, dump buffer to file

            strncpy(logsFilePath, vbMetricsCurrentPath, (VB_ENGINE_METRICS_MAX_PATH_LEN-1));
            logsFilePath[VB_ENGINE_METRICS_MAX_PATH_LEN - 1] = '\0';

            strncat(logsFilePath, vbMetricsReportsList[i].file, (VB_ENGINE_METRICS_MAX_PATH_LEN-1));

            // Save to file
            VbLogSaveBufferToTextFile(logsFilePath, "a+", VB_METRICS_OUTPUT_BUFFER_SIZE, buffer); // Append text
          }
        }
      }
    }
    free(buffer);
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR,"Memory allocation error inside VbMetricsDumpReportsToFiles");
  }
}

/*******************************************************************/

void VbMetricsAddProcessHandler(t_VBMetricsEventType index, BOOL (*handler)(t_VB_MetricsEvent *))
{
  if((handler != NULL) && (index < VB_METRICS_MAX_NR_EVENT_TYPES))
  {
    if(metricsEventsProcess[index] == NULL)
    {
      metricsEventsProcess[index] = handler;
    }
  }
}

/*******************************************************************/

void VbMetricsCallReport(INT32S id,  void (*write_fun)(const char *fmt, ...))
{
  CHAR  *buffer;
  CHAR  *ptr_to_write;
  INT32U remaining_size;
  INT32U written_size;
  INT32U offset;

  // Check valid event value
  if((id < VB_METRICS_MAX_NR_REPORTS) && (vbMetricsReportsList[id].handler != NULL))
  {
    buffer = (CHAR *)malloc(VB_METRICS_OUTPUT_BUFFER_SIZE);

    if(buffer != NULL)
    {
      pthread_mutex_lock( &vbMetricsEventsListMutex );

      remaining_size = VB_METRICS_OUTPUT_BUFFER_SIZE;
      ptr_to_write = buffer;

      (vbMetricsReportsList[id].handler)(&ptr_to_write, &remaining_size);

      pthread_mutex_unlock( &vbMetricsEventsListMutex );

      // written size
      written_size = VB_METRICS_OUTPUT_BUFFER_SIZE - remaining_size;

      // Write to console
      offset = 0;
      do
      {
        write_fun("%s", buffer+offset);
        offset += VB_CONSOLE_BUFFER_SIZE-1;
      }while(written_size > offset);

      free(buffer);
    }
  }
  else
  {
    write_fun("Invalid report id. Choose one of the following reports:\n");
    VbMetricsPrintReportList(write_fun);
  }
}

/*******************************************************************/

void VbMetricsPrintReportList(void (*write_fun)(const char *fmt, ...))
{
  INT32S i;

  // Print reports list
  for(i=0 ; i<VB_METRICS_MAX_NR_REPORTS; i++)
  {
    if((vbMetricsReportsList[i].handler != NULL) && (vbMetricsReportsList[i].description != NULL))
    {
      write_fun("\t\t %d %s \n", i, vbMetricsReportsList[i].description);
    }
  }
}

/*******************************************************************/

void VbMetricsPrintEventsDescription(void (*write_fun)(const char *fmt, ...))
{
  INT32S i;

  // Print reports list
  for(i=1 ; i<VB_METRICS_MAX_NR_EVENT_TYPES; i++)
  {
    write_fun("\t %d > %s > %s\n", i, metricsEventsDescription[i], (vbMetricsEventTypeEnable[i] ? "Enabled" : "Disabled"));
  }
}

/*******************************************************************/

void VbMetricsPrintEventsList(t_VBMetricsEventType type,  void (*write_fun)(const char *fmt, ...))
{
  INT32U i;
  INT32S count;
  // We use an array of pointers to point the events on the list
  t_VB_MetricsEvent** startEvent = NULL;

  // Check valid event value
  if(type < VB_METRICS_MAX_NR_EVENT_TYPES)
  {
    count = VbMetricsGetAllEventsByType(&startEvent, type);

    if(count > 0)
    {
      write_fun("\n| Index | Type | Time (sec)\t| pData |");
      write_fun("\n------------------------------------");

      for(i=0; i<count; i++)
      {
        write_fun("\n|  %d |  %d  | %d,%09d\t| 0x%08p |", i, (startEvent[i])->eventType,
            (startEvent[i])->eventTime.tv_sec, (startEvent[i])->eventTime.tv_nsec,
            ((startEvent[i])->eventData));
      }

      write_fun("\n------------------------------------");
    }
    else if(count == 0)
    {
      write_fun("\n--------  No results to show  -------\n");
    }
    else
    {
      write_fun("\nError performing the search %d\n", count);
    }

    // Release allocated memory
    if (startEvent != NULL)
    {
      free(startEvent);
    }
  }
  else
  {
    write_fun("\nError performing the search. Invalid event type\n");
  }
}

/*******************************************************************/

void VbMetricsDumpAllEventsList(void (*write_fun)(const char *fmt, ...))
{
  INT32U i, j, count;
  BOOL   metrics_running;
  t_VBMetricsTimeMarker *values;

  metrics_running = vbMetricsRunning;
  // Avoid modifications on the list while we are printing
  VbStopMetrics();

  write_fun("\nMetrics internal buffer contents");
  write_fun("\n---------------------------------\n");

  write_fun("Event types:\n");
  VbMetricsPrintEventsDescription(write_fun);

  write_fun("\n|Index\t|Type\t|Time (sec)\t| ptrData |");
  write_fun("\n---------------------------------------");

  count = 0;
  pthread_mutex_lock( &vbMetricsEventsListMutex );
  for(i=0; i<vbMetricsListLength; i++)
  {
    if(vbMetricsEventsList[i].eventType == VB_METRICS_EVENT_CLOSE)
    {
      continue;
    }
    else if(vbMetricsEventsList[i].eventType == VB_METRICS_EVENT_TIME_MARKERS)
    {
      write_fun("\n| %d\t| %d \t| %d,%09d\t| 0x%08p |", i, vbMetricsEventsList[i].eventType,
          vbMetricsEventsList[i].eventTime.tv_sec, vbMetricsEventsList[i].eventTime.tv_nsec,
          vbMetricsEventsList[i].eventData);

      values = (t_VBMetricsTimeMarker *)vbMetricsEventsList[i].eventData;
      for(j=0; j<VB_METRICS_MAX_NR_TIME_MARKERS; j++)
      {
        if(values[j].counter > 0)
        {
          write_fun("\n| %d\t| TM%d\t| total: %u nsec Cnt: %d Avg: %d nsec\t|",i , j,
              (unsigned int)values[j].sumatory, values[j].counter, VbMetricsGetMarkerAvgTime(&values[j]));
        }
      }
      count++;
    }
    else
    {
      write_fun("\n| %d\t| %d \t| %d,%09d\t| 0x%08p |", i, vbMetricsEventsList[i].eventType,
          vbMetricsEventsList[i].eventTime.tv_sec, vbMetricsEventsList[i].eventTime.tv_nsec,
          vbMetricsEventsList[i].eventData);
      count++;
    }
  }
  pthread_mutex_unlock( &vbMetricsEventsListMutex );

  if(count == 0)
  {
    write_fun("\n------  Events list is empty  ------");
  }

  write_fun("\n------------------------------------");

  // Extra information
  write_fun("\n Metrics list length     : %u", vbMetricsListLength);
  write_fun("\n Metrics current index   : %u", vbMetricsCurrentWrIndex);
  write_fun("\n Metrics buffer overflow : %u", vbMetricsIndexOverflow);
  write_fun("\n Metrics buffer mode     : %s", (vbMetricsBufferMode ? "Bucket" : "Circular"));
  write_fun("\n Metrics status          : %s\n", (metrics_running ? "Running" : "Stopped"));
  write_fun("\n Events enabled :");

  write_fun("\n");

  // Since capture is stopped, we can avoid using the mutex
  vbMetricsRunning = metrics_running;
}

/*******************************************************************/

INT32U VbMetricsEventsListSizeGet(void)
{
  return vbMetricsListLength;
}

/*******************************************************************/

/**
 * @}
 **/
