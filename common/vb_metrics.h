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
 * @file vb_metrics.h
 *
 * @brief This module implements functions to register events and measure time between
 * those events.
 * @internal
 *
 * @author pablom
 * @date Jan 13, 2017
 *
 **/
#include <time.h>

#ifndef VB_METRICS_H_
#define VB_METRICS_H_

/*
 ************************************************************************
 * To use metrics you just have to insert a call to VbMetricsInsertEvent in the
 * code you want to monitorize. Indicate the type of event and you can add
 * extra information that can be retrieved afterwards when you retrieve the
 * posted events on the events list.
 * You can query the registered events from the console, dumping the raw content
 * of the events list or creating a report function to show the information
 * in a nice formated way.
 * For very frequent events you can use Time Markers instead of normal events.
 * Time markers store the total elapsed time over many iterations and the number
 * of iterations, so you can afterward calculate the average elapsed time
 * between two different markers.
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define VB_METRICS_EVENTS_BUFFER_LENGTH (1500)
#define VB_METRICS_MAX_NR_TIME_MARKERS  (6)
#define VB_METRICS_OUTPUT_BUFFER_SIZE   (250 * VB_METRICS_EVENTS_BUFFER_LENGTH)
#define VB_ENGINE_METRICS_MAX_PATH_LEN  (128)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef enum {
  VB_METRICS_EVENT_CLOSE = 0,
  VB_METRICS_EVENT_START_LOW_CHANN_CAP, // 1
  VB_METRICS_EVENT_END_LOW_CHANN_CAP,   // 2
  VB_METRICS_EVENT_START_FULL_CHANN_CAP,// 3
  VB_METRICS_EVENT_END_FULL_CHANN_CAP,  // 4
  VB_METRICS_EVENT_BAND_CHANGE,         // 5
  VB_METRICS_EVENT_UD_RATE_CHANGE,      // 6
  VB_METRICS_EVENT_START_SNR_LOW_CALC,  // 7
  VB_METRICS_EVENT_END_SNR_LOW_CALC,    // 8
  VB_METRICS_EVENT_START_SNR_HIGH_CALC, // 9
  VB_METRICS_EVENT_END_SNR_HIGH_CALC,   // 10
  VB_METRICS_EVENT_TIME_MARKERS,        // 11 - Inserted automatically when calling VbMetricsInsertCurrentTimeMarkersValues
  VB_METRICS_EVENT_TRAFFIC_REPORT,      // 12 - Generic traffic event
  VB_METRICS_EVENT_TRAFFIC_OVER_105,    // 13
  VB_METRICS_EVENT_TRAFFIC_OVER_205,    // 14
  VB_METRICS_EVENT_TRAFFIC_OVER_305,    // 15
  VB_METRICS_EVENT_TRAFFIC_OVER_405,    // 16
  VB_METRICS_EVENT_TRAFFIC_OVER_505,    // 17
  VB_METRICS_EVENT_TRAFFIC_OVER_605,    // 18
  VB_METRICS_EVENT_TRAFFIC_OVER_705,    // 19
  VB_METRICS_EVENT_TRAFFIC_OVER_805,    // 20
  VB_METRICS_EVENT_TRAFFIC_UNDER_95,    // 21
  VB_METRICS_EVENT_TRAFFIC_UNDER_195,   // 22
  VB_METRICS_EVENT_TRAFFIC_UNDER_295,   // 23
  VB_METRICS_EVENT_TRAFFIC_UNDER_395,   // 24
  VB_METRICS_EVENT_TRAFFIC_UNDER_495,   // 25
  VB_METRICS_EVENT_TRAFFIC_UNDER_595,   // 26
  VB_METRICS_EVENT_TRAFFIC_UNDER_695,   // 27
  VB_METRICS_EVENT_TRAFFIC_UNDER_795,   // 28
  VB_METRICS_EVENT_START_MEAS_PLAN,     // 29
  VB_METRICS_EVENT_END_MEAS_PLAN,       // 30
  VB_METRICS_EVENT_NEW_LINE,            // 31
  VB_METRICS_EVENT_LINE_LOST,           // 32
  VB_METRICS_EVENT_ALIGN_START,         // 33
  VB_METRICS_EVENT_ALIGN_RESTART,       // 34
  VB_METRICS_EVENT_ALIGN_CHECK_START,   // 35
  VB_METRICS_EVENT_ALIGN_CHECK_END,     // 36
  VB_METRICS_EVENT_ALIGN_INFO,          // 37
  VB_METRICS_EVENT_ALIGN_DONE,          // 38
  VB_METRICS_MAX_NR_EVENT_TYPES,        // 39
} t_VBMetricsEventType;

typedef enum {
  VB_METRICS_TM_EMPTY = -1,
  VB_METRICS_TM_BEGIN_1 = 0,
  VB_METRICS_TM_END_1   = 1,
  VB_METRICS_TM_BEGIN_2 = 2,
  VB_METRICS_TM_END_2   = 3,
  VB_METRICS_TM_BEGIN_3 = 4,
  VB_METRICS_TM_END_3   = 5,
  VB_METRICS_TM_BEGIN_4 = 6,
  VB_METRICS_TM_END_4   = 7,
  VB_METRICS_TM_BEGIN_5 = 8,
  VB_METRICS_TM_END_5   = 9,
  VB_METRICS_TM_BEGIN_6 = 10,
  VB_METRICS_TM_END_6   = 11,
} t_VBMetricsTimeMarkerType;

typedef enum
{
  VB_METRICS_NO_ERROR = 0,
  VB_METRICS_ERROR_INPUT_PARAM,     // 1
  VB_METRICS_ERROR_MEMORY,          // 2
  VB_METRICS_ERROR_EV_TYPE_DISABLED,// 3
  VB_METRICS_ERROR_STOPPED,         // 4
  VB_METRICS_ERROR_EVENT_NOT_FOUND, // 5
  VB_METRICS_ERROR_LIST_FULL,       // 6
  VB_METRICS_ERROR_ALREADY_INITIALIZED, // 7
  VB_METRICS_ERROR_LAST,            // 8
}t_VbMetricsErrorType;

typedef enum
{
  EVENTS_BUFF_CIRCULAR = 0,
  EVENTS_BUFF_BUCKET,
  EVENTS_BUFF_LAST,
}t_VbMetricsBufferType;

typedef struct
{
  t_VBMetricsEventType eventType;
  struct timespec eventTime;
  void *eventData;
}t_VB_MetricsEvent;

typedef struct
{
  struct timespec lastBeginTime;
  INT64U sumatory;
  INT32U counter;
}t_VBMetricsTimeMarker;

typedef void (*t_VbMetricsReportHandler)(CHAR **buffer, INT32U *buffSize);

typedef struct
{
  t_VbMetricsReportHandler handler;
  CHAR                    *description;
  CHAR                    *file;
}t_VB_MetricsReport;

/*
 ************************************************************************
 ** Public function declaration
 ************************************************************************
 */

/**
 * @brief Initialize metrics module
 * @param[in] outputPath Path to dump metrics files
 * @param[in] queueName Unique name for posix queue
 * @return @ref t_VbMetricsErrorType
 *
 **/
t_VbMetricsErrorType VbMetricsInit(CHAR *outputPath, CHAR *queueName);

/**
 * @brief Stop the Metrics thread and close the MsgQ
 **/
void VbMetricsExit(void);

/**
 * @brief Free all memory allocated
 **/
void VbMetricsDestroy(void);

/**
 * @brief Calculate the time elapsed from the engine start up
 * @param[out] timeFromStart time elapsed
 **/
void VbMetricsGetElapsedTime(struct timespec *timeFromStart);

/**
 * @brief Calculate the time elapsed from the engine start up
 * @param[out] timeFromStart time elapsed
 **/
void VbMetricsCalculateTimeDiffFromStart(struct timespec *timeRef, struct timespec *timeDiff);

/**
 * @brief Report a metrics event. This function just saves the timestamp of
 * the event and passes it to the metrics thread to be processed and eventually,
 * inserted in the buffer.
 * @param[in] type Event type
 * @param[in] data Data associated to the event
 **/
t_VbMetricsErrorType VbMetricsReportEvent(t_VBMetricsEventType type, void *data);

/**
 * @brief Start metrics module. After calling this function all events reported
 * will be saved to the events buffer
 **/
void VbStartMetrics(void);

/**
 * @brief Stop metrics module. After calling this function all events reported
 * will be discarded and not saved to the events buffer
 **/
void VbStopMetrics(void);

/**
 * @brief Get running status
 * @return TRUE: metrics module is running; FALSE: otherwise
 **/
BOOL VbMetricsGetStatus(void);

/**
 * @brief
 * @return Reports path
 **/
CHAR *VbMetricsGetReportsPath(void);

/**
 * @brief Get the seconds elapsed from the metrics start time
 * @return Seconds
 **/
time_t VbMetricsGetBootTime(void);

/**
 * @brief Reset metrics list content. All pointed memory regions will be
 * freed.
 **/
void VbMetricsResetList(void);

/**
 * @brief Enable or disable the capture of a given event type
 * @param[in] type Event type
 * @param[in] enable TRUE: enable capture of given event type; FALSE: disable capture
 **/
void VBMetricsEnableEvent(t_VBMetricsEventType type, BOOL enable);

/**
 * @brief Get a list of events on the list matching a given type. The caller
 * function has free the returned eventArr
 * @param[out] eventArr List of events matching the given type. Pointer to array of pointer.
 * @param[in] type Event type searched
 * @return number of events found, or a negative value if an error occurred
 *
 * @pre Metrics module has to be stopped while running this function
 * Note: eventArr has to be freed by the caller function once it finished using it
 **/
INT32S VbMetricsGetAllEventsByType(t_VB_MetricsEvent*** eventArr, t_VBMetricsEventType type);

/**
 * @brief Get the next event on the list with the given type, starting
 * from index (Index is relative to vbMetricsCurrentWrIndex)
 * @param[out] event Pointer to the event on the events list
 * @param[in] type Event type searched
 * @param[in] offset index offset to start the search
 * @return index of the event found
 *
 * @pre Metrics module has to be stopped while running this function
 **/
INT32S VbMetricsGetNextEventByType(t_VB_MetricsEvent** event, t_VBMetricsEventType type, INT32U offset);

/**
 * @brief Get the event on the at a given index
 * @param[out] event Pointer to the event on the events list
 * @param[in] index index of the event requested
 * @return number of events found, or a negative value if an error occurred
 *
 * @pre Metrics module has to be stopped while running this function
 **/
t_VbMetricsErrorType VbMetricsGetEventByIndex(t_VB_MetricsEvent** event, INT32U index);

t_VbMetricsErrorType VbMetricsReadNextEvent(t_VB_MetricsEvent** event);

/**
 * @brief Calculate the time difference between the two first events found in
 * the events list with the events types defined. The eventData pointer is
 * returned
 * @param[in] beginType
 * @param[in] endType
 * @param[in] offset
 * @param[out] pData pointer to the eventData pointer of the start event
 * @return index of the event found
 *
 * @pre Metrics module should be stopped while running this function
 **/
INT64S VbMetricsGetTimeInterval(t_VBMetricsEventType beginType, t_VBMetricsEventType endType, INT32U offset, void **pData);

/**
 * @brief Add a new report function to the list of available reports
 * @param[in] description Short description of the report (31 chars)
 * @param[in] handler Report function called from the console
 * @return index of the report
 **/
INT32S VbMetricsAddReportHandler(const CHAR *description, t_VbMetricsReportHandler handler, const CHAR *file);

/**
 * @brief Add a new process function to the list of available reports
 * @param[in] index event type to which it is related
 * @param[in] handler Process function called when a event of type index is received
 * @return index of the report
 **/
void VbMetricsAddProcessHandler(t_VBMetricsEventType index, BOOL (*handler)(t_VB_MetricsEvent *));

void VbMetricsPrintReportList(void (*write_fun)(const char *fmt, ...));

void VbMetricsPrintEventsDescription(void (*write_fun)(const char *fmt, ...));

/**
 * @brief Register a new time marker.
 * @param[in] type Time Marker type
 * @return @ref t_VbMetricsErrorType
 **/
t_VbMetricsErrorType VbMetricsInsertTimeMarker(t_VBMetricsTimeMarkerType type);

/**
 * @brief Insert two time markers using the same time stamp. They can be one end marker
 * and one begin marker or two end markers
 * @param[in] type First Time Marker type, it must be an end marker
 * @param[in] next Second Time Marker type
 * @return @ref t_VbMetricsErrorType
 **/
t_VbMetricsErrorType VbMetricsInsertTimeMarkerAndSetNext(t_VBMetricsTimeMarkerType type, t_VBMetricsTimeMarkerType next);

/**
 * @brief Insert new metrics event with the content of the time markers
 * list. So we can retrieve these values afterwards.
 * @return @ref t_VbMetricsErrorType
 **/
t_VbMetricsErrorType VbMetricsInsertCurrentTimeMarkersValues(void);

t_VbMetricsErrorType VbMetricsGetCurrentTimeMarkersValues(t_VBMetricsTimeMarker *values);

/**
 * @brief Calculate the average time of a given time marker
 * @param[in] timeMarker pointer to a time marker
 * @return @ref Average time elapsed between the calls to begin and end of
 * this time marker. Time in nanoseconds
 *
 * @pre
 **/
INT32S VbMetricsGetMarkerAvgTime(t_VBMetricsTimeMarker *timeMarker);

/**
 * @brief Reset counters and summatories of all time markers
 **/
void VbMetricsResetTimeMarkers(void);

void VbMetricsDumpReportsToFiles(void);

void VbMetricsDumpAllEventsList(void (*write_fun)(const char *fmt, ...));

void VbMetricsPrintEventsList(t_VBMetricsEventType type,  void (*write_fun)(const char *fmt, ...));

void VbMetricsCallReport(INT32S id,  void (*write_fun)(const char *fmt, ...));

/**
 * @brief Gets events list maximum size
 **/
INT32U VbMetricsEventsListSizeGet(void);

#endif /* VB_METRICS_H_ */

/**
 * @}
 **/
