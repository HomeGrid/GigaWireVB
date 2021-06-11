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
 * @file vb_engine_metrics_reports.h
 *
 * @brief
 * @internal
 *
 * @author pablom
 * @date Feb 1, 2017
 *
 **/

#ifndef VB_ENGINE_METRICS_REPORTS_H_
#define VB_ENGINE_METRICS_REPORTS_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_engine_datamodel.h"
#include "vb_metrics.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

// Used to capture information on when capturing the event
typedef struct SNRMetricsData
{
  INT8U mac[6];
  t_VBDriver *driver;
  t_VBMetricsTimeMarker vbMetricsTimeMarkersList[VB_METRICS_MAX_NR_TIME_MARKERS];
}t_SNRMetricsData;

// Used to pass information when capturing the event
typedef struct DeviceMetricsData
{
  INT8U mac[6];
  CHAR driverId[VB_EA_DRIVER_ID_MAX_SIZE];
  BOOL DMNodeType;
  INT32U data1;
  INT32U data2;
}t_DeviceMetricsData;


/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Initializes engine metrics
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineMetricsInit(void);

/**
 * @brief Starts engine metrics
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineMetricsRun(void);

/**
 * @brief Report an event passing generic data (pointer and size)
 * @return @ref t_VB_engineErrorCode
 **/
t_VbMetricsErrorType VbMetricsReportGenericEvent(t_VBMetricsEventType eventType, void *data, INT32U size);

/**
 * @brief Report an event related to a given node. Event types VB_METRICS_EVENT_BAND_CHANGE,
 * VB_METRICS_EVENT_UD_RATE_CHANGE or VB_METRICS_EVENT_TRAFFIC_REPORT
 * @return @ref t_VB_engineErrorCode
 **/
t_VbMetricsErrorType VbMetricsReportDeviceEvent(t_VBMetricsEventType eventType, INT8U *mac, BOOL DMNodeType, t_VBDriver *driver, INT32U value1, INT32U value2);

void VbEngineMetricsStart(void);

void VbEngineMetricsStop(void);

void VbEngineMetricsResetMetrics(void);

INT32U VbMetricsGetMaxLogSize();

BOOL VbMetricsGetSaveMetricsToDisk();

BOOL VbMetricsGetEnableCPUTimingMetrics();

BOOL VbMetricsGetEnableBoostTrafficMetrics();

void VbMetricsSetEnableCPUTimingMetrics(BOOL value);

void VbMetricsSetEnableBoostTrafficMetrics(BOOL value);

void VbMetricsSetMaxLogSize(INT32U value);

void VbMetricsSetSaveMetricsToDisk(BOOL value);

void VbEngineDumpMetricsToFiles();

void VbMetricsCPUCalculationReport(CHAR **buffer, INT32U *buffSize);

/**
 * @brief Print all boost and traffic logs to the passed buffer
 **/
void VbMetricsBoostedLogReport( CHAR **buffer, INT32U *buffSize);

/**
 * @brief Print all boosted state information to the passed buffer
 **/
void VbMetricsBoostedLogFileReport(CHAR **buffer, INT32U *buffSize);

/**
 * @brief Print Boosting events list
 **/
void VbMetricsTrafficHistReport(CHAR **buffer, INT32U *buffSize);

void VbMetricsBoostLineStatusReport(CHAR **buffer, INT32U *buffSize);

/**
 * @brief Calculates consumed times and write them to t_VBMetricsMeasTimes
 **/
BOOL VbMetricsMeasPlanProcess(t_VB_MetricsEvent *event);

BOOL VbMetricsBoostedStatsProcess(t_VB_MetricsEvent *event);

BOOL VbMetricsLostLineProcess(t_VB_MetricsEvent *event);

BOOL VbMetricsTrafficProcess(t_VB_MetricsEvent *event);

void vbMetricsDumpAndCloseLists(void);

#endif /* VB_ENGINE_METRICS_REPORTS_H_ */

/**
 * @}
**/
