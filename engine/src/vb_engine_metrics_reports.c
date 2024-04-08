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
 * @file vb_engine_metrics_reports.c
 * @brief Metrics reports. These can be output to the console or write in a file
 *
 * @internal
 *
 * @author pablom
 * @date 1/2/2017
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <string.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "types.h"
#include "vb_util.h"
#include "vb_metrics.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_conf.h"
#include "vb_engine_alignment_metrics.h"
#include "vb_log.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_ENGINE_METRICS_MAX_NUMBER_LINES  (24)
#define VB_ENGINE_METRICS_HISTOGRAM_STEPS   (20)
#define VB_ENGINE_METRICS_LOG_FILE          "traffic_n_boost_log.txt"
#define VB_ENGINE_METRICS_QUEUE_NAME        "/VbEngineMetricsQ"
#define VB_ENGINE_METRICS_TRAFFIC_HYST      (5) // In Mbps
#define VB_ENGINE_METRICS_TRAFFIC_MAX       (800)
#define VB_ENGINE_METRICS_TRAFFIC_MIN       (100)
#define VB_ENGINE_METRICS_TRAFFIC_STEP      (100)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct MeasCPUTimes
{
  t_linkedMACElement l;
  BOOL DMNode;
  CHAR driverId[VB_EA_DRIVER_ID_MAX_SIZE];
  INT64S fullChanCapCalc; //in nanoseconds
  INT64S lowChanCapCalc;  //in nanoseconds
  INT64S SNRLowXtalkCalc;         //in nanoseconds
  INT64S SNRHighXtalkCalc;        //in nanoseconds
  t_VBMetricsTimeMarker timeMarkers[VB_METRICS_MAX_NR_TIME_MARKERS]; // Intermediate steps
  struct timespec SNRLowBeginTime;
  struct timespec SNRFullBeginTime;
  struct timespec fullChanBeginTime;
  struct timespec lowChanBeginTime;
}t_VBMetricsMeasTimes;

typedef struct nodeTraffic
{
  t_linkedMACElement l;
  CHAR driverId[VB_EA_DRIVER_ID_MAX_SIZE];
  struct timespec firstEventTime;
  struct timespec lastEventTime;
  INT32U previousTrafficValue;
  INT32U reportCounter;
  INT32U trafficHistogram[VB_ENGINE_METRICS_HISTOGRAM_STEPS]; // each counter represents traffic between [50*n, 50*(n+1)]
}t_VBMetricsNodeTraffic;

typedef struct MeasPlan
{
  struct timespec measTimestamp;
  INT64S totalMeasPlan;   //in nanoseconds
  struct timespec measPlanBeginTime;
  INT8U planId;
}t_VBMetricsMeasPlanInfo;

typedef struct BoostingStatsLine
{
  t_linkedMACElement l;
  BOOL DMNode;
  CHAR driverId[VB_EA_DRIVER_ID_MAX_SIZE];
  INT32U currentBand;
  INT32U currentRate;
  INT64U totalTimeBoosted[VB_PSD_NUM_BANDS]; // in ms
  struct timespec lastBandBeginTime[VB_PSD_NUM_BANDS];
}t_VBMetricsBoostingStatsLine;

typedef struct BoostingStatsGlobal
{
  INT32U currentHigherBand;                               // Higher band with DMs using it
  INT64U currentNrOfNodesInBand[VB_PSD_NUM_BANDS];        // Nr of DMs on each band
  INT64U totalTimeWithNodesInBand[VB_PSD_NUM_BANDS];      // Total time using each band on DS
  struct timespec lastTimeWithNodesInBand; //[VB_PSD_NUM_BANDS];
  INT32U currentNrNodesinFullBand;                               // Number of nodes in full band
  INT64U totalTimeWithNodesInFullBand[VB_ENGINE_METRICS_MAX_NUMBER_LINES];  // Total time with nr of nodes using 5 bands
  struct timespec lastTimeWithNodesInFullBand; //[VB_ENGINE_METRICS_MAX_NUMBER_LINES];
}t_VBMetricsBoostingStatsGlobal;

// Linked list head
static t_VBMetricsMeasTimes           *vbMetricsCPUMeasurements = NULL;
static t_VBMetricsMeasPlanInfo         vbMetricsMeasPlanInfo;
static t_VBMetricsBoostingStatsGlobal  vbMetricsBandStatsGlobalDM = {0, {0}, {0}, {0}, 0, {0}, {0}};
static t_VBMetricsBoostingStatsGlobal  vbMetricsBandStatsGlobalEP = {0, {0}, {0}, {0}, 0, {0}, {0}};
static t_VBMetricsBoostingStatsLine   *vbMetricsBoostingStatsLine = NULL;
static t_VBMetricsNodeTraffic         *vbMetricsHistogramsList = NULL;
static BOOL    vbMetricsEnableCPUTimingMetrics    = FALSE;
static BOOL    vbMetricsEnableBoostTrafficMetrics = TRUE;
static BOOL    vbMetricsSaveMetricsToDisk         = TRUE;
static BOOL    vbMetricsLogFileHeaderWritten      = FALSE;
static INT32U  vbMetricsMaxLogSize                = 0;
static struct timespec vbMetricsStopTime = {0,0};
static struct timespec vbMetricsStartTime = {0,0};
static t_vbQueueName vbMetricsQueueName;
static INT16U  vbEngineMetricsNumBands;
static t_vbEngineQosRate vbEngineMetricsLastUSRate; // Used to generate the log
/*
 ************************************************************************
 ** Private function declaration
 ************************************************************************
 */

static void vbMetricsResetBoostList(void);
static BOOL VbMetricsCheckEnoughSpaceForMetrics(void);
static t_VBMetricsBoostingStatsLine *GetBoostStructByMac(INT8U *mac);
static t_VBMetricsMeasTimes *GetMeasStructByMac(INT8U *mac);
static t_VBMetricsNodeTraffic *GetTrafficStructByMac(INT8U *mac, struct timespec *eventTime);
static void vbMetricsResetHistogramList(struct timespec timeNow);
static void DrawHistogram(CHAR **buffer, INT32U *buffSize, t_VBMetricsNodeTraffic *node);
static BOOL VbMetricsBandChangeStatsProcess(t_VB_MetricsEvent *event);
static BOOL VbMetricsNewNodeProcess(t_VB_MetricsEvent *event);
static void vbMetricsUpdateBandsCounters(t_VBMetricsBoostingStatsLine **devs_list, INT32U *numPerBand, INT8U *mac, INT32U newBand);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/************************************************************************************/

static BOOL VbMetricsCheckEnoughSpaceForMetrics(void)
{
  DIR           *d;
  struct dirent *dir;
  struct stat    st;
  CHAR           dir_path[VB_ENGINE_METRICS_MAX_PATH_LEN];
  CHAR           file_path[2 * VB_ENGINE_METRICS_MAX_PATH_LEN + 1];
  INT32U         space;
  BOOL           res;

  res = TRUE;

  if(vbMetricsMaxLogSize > 0)
  {
    snprintf(
        dir_path,
        (VB_ENGINE_METRICS_MAX_PATH_LEN-1),
        "%s/%s",
        VbEngineConfOutputPathGet(), VbMetricsGetReportsPath()
        );

    d = opendir(dir_path);
    space = 0;

    if (d)
    {
      while ((dir = readdir(d)) != NULL)
      {
        if(strcmp(dir->d_name, ".") == 0)
        {
          continue;
        }
        //Compose path and file names
        strcpy(file_path, dir_path);
        strncat(file_path, dir->d_name, sizeof(file_path) - 1);

        if (stat(file_path, &st) == 0)
        {
          space+=st.st_size;
        }
      }
      closedir(d);

      // Size in KB
      space /= 1024;
    }
    else
    {
      // Directory not found
      res = FALSE;
    }

    if(space >= vbMetricsMaxLogSize)
    {
      res = FALSE;
    }
  }

  return res;
}

/*************************************************************************/
/**
 * @brief Function used to calculate the time percentage rounding the result.
 **/

INT32U VbMetricsCalculateRoundPercentage(INT64U dividend, INT64U divisor)
{
  INT32U result;

  result = (dividend * 1000) / divisor;

  if(result%10 >= 5)
  {
    result /= 10;
    result++;
  }
  else
  {
    result /= 10;
  }

  return result;
}

/************************************************************************************/
/**
 * @brief Print Boosting events list
 **/
static void DrawHistogram(CHAR **buffer, INT32U *buffSize, t_VBMetricsNodeTraffic *node)
{
  INT32S i, j;
  INT32U percent, decimals, length;

  VbUtilStringToBuffer(buffer, buffSize, "\n Mbps ");

  for(i=(VB_ENGINE_METRICS_HISTOGRAM_STEPS-1); i>=0; i--)
  {
    percent = VbMetricsCalculateRoundPercentage(node->trafficHistogram[i], node->reportCounter);
    decimals = ((node->trafficHistogram[i]*100) % node->reportCounter)*100 / node->reportCounter;
    length = percent*10/25;

    VbUtilStringToBuffer(buffer, buffSize,
                      "\n %3u-%4u |", (unsigned int)(i*50), (unsigned int)((i+1)*50));

    for (j=0; j<length; j++)
    {
      VbUtilStringToBuffer(buffer, buffSize, "X");
    }

    VbUtilStringToBuffer(buffer, buffSize,
                      " %2u.%02u%%", (unsigned int)(percent), (unsigned int)(decimals));
  }

  VbUtilStringToBuffer(buffer, buffSize,
      "\n          L-------------------+-------------------->\n");
}


/************************************************************************************/

static t_VBMetricsMeasTimes *GetMeasStructByMac(INT8U *mac)
{
  t_VBMetricsMeasTimes *times_st;
  t_linkedMACElement *linked_el;

  if(GetElementByMAC((t_linkedMACElement *)vbMetricsCPUMeasurements, &linked_el, mac))
  {
    times_st = (t_VBMetricsMeasTimes*)linked_el;
  }
  else
  {
    times_st = (t_VBMetricsMeasTimes*)calloc(1, sizeof(t_VBMetricsMeasTimes));

    if(times_st != NULL)
    {
      //Add to the list
      AppendElement((t_linkedElement **)&vbMetricsCPUMeasurements, (t_linkedElement *) times_st);

      memcpy(times_st->l.mac, mac, sizeof(times_st->l.mac));
    }
  }
  return times_st;
}

/************************************************************************************/

/**
 * @brief Updates t_VBMetricsBoostingStats
 **/
static t_VBMetricsBoostingStatsLine *GetBoostStructByMac(INT8U *mac)
{
  t_VBMetricsBoostingStatsLine *boost_st;
  t_linkedMACElement *linked_el;

  if(GetElementByMAC((t_linkedMACElement *)vbMetricsBoostingStatsLine, &linked_el, mac))
  {
    boost_st = (t_VBMetricsBoostingStatsLine*)linked_el;
  }
  else
  {
    boost_st = (t_VBMetricsBoostingStatsLine*)calloc(1, sizeof(t_VBMetricsBoostingStatsLine));

    if(boost_st != NULL)
    {
      //Add to the list
      AppendElement((t_linkedElement **)&vbMetricsBoostingStatsLine, (t_linkedElement *) boost_st);

      MACAddrClone(boost_st->l.mac, mac);
      boost_st->l.l.next = NULL;
    }
  }
  return boost_st;
}

/************************************************************************************/

/**
 * @brief Updates t_VBMetricsBoostingStats
 **/
static void vbMetricsUpdateBandsCounters(t_VBMetricsBoostingStatsLine **devs_list, INT32U *numPerBand, INT8U *mac, INT32U newBand)
{
  t_VBMetricsBoostingStatsLine *boost_st;
  t_linkedMACElement *linked_el;

  if(GetElementByMAC((t_linkedMACElement *)(*devs_list), &linked_el, mac))
  {
    boost_st = (t_VBMetricsBoostingStatsLine*)linked_el;
    if(numPerBand[boost_st->currentBand] > 0)
    {
      numPerBand[boost_st->currentBand]--;
    }
  }
  else
  {
    boost_st = (t_VBMetricsBoostingStatsLine*)calloc(1, sizeof(t_VBMetricsBoostingStatsLine));

    if(boost_st != NULL)
    {
      //Add to the list
      AppendElement((t_linkedElement **)devs_list, (t_linkedElement *) boost_st);

      MACAddrClone(boost_st->l.mac, mac);
      boost_st->l.l.next = NULL;
    }
  }

  if(boost_st != NULL)
  {
    boost_st->currentBand = (newBand > 0)?(--newBand):0;
    numPerBand[boost_st->currentBand]++;
  }
}

/************************************************************************************/

static void vbMetricsResetBoostList(void)
{
  t_VBMetricsBoostingStatsLine *boost_st;
  t_linkedElement *current;
  t_linkedElement *head;

  head = (t_linkedElement *)vbMetricsBoostingStatsLine;

  // Loop over the list
  LIST_FOREACH(head,current)
  {
    boost_st = (t_VBMetricsBoostingStatsLine *)current;
    memset(&(boost_st->lastBandBeginTime), 0, sizeof(struct timespec)*VB_PSD_NUM_BANDS);
    memset(boost_st->totalTimeBoosted, 0, sizeof(INT64U)*VB_PSD_NUM_BANDS);
    boost_st->currentBand = 0;
    boost_st->currentRate = VbCdtaDefaultQosRateGet() * 10;
  }
}

/**************************************************************************/

static t_VBMetricsNodeTraffic *GetTrafficStructByMac(INT8U *mac, struct timespec *eventTime)
{
  t_VBMetricsNodeTraffic *traffic_st;
  t_linkedMACElement *linked_el;

  if(GetElementByMAC((t_linkedMACElement *)vbMetricsHistogramsList, &linked_el, mac))
  {
    traffic_st = (t_VBMetricsNodeTraffic*)linked_el;
  }
  else
  {
    traffic_st = (t_VBMetricsNodeTraffic*)calloc(1, sizeof(t_VBMetricsNodeTraffic));

    if(traffic_st != NULL)
    {
      //Add to the list
      AppendElement((t_linkedElement **)&vbMetricsHistogramsList, (t_linkedElement *) traffic_st);

      memcpy(traffic_st->l.mac, mac, sizeof(traffic_st->l.mac));
      traffic_st->l.l.next = NULL;
      traffic_st->firstEventTime = *eventTime;
    }
  }
  return traffic_st;
}

/**************************************************************************/

static void vbMetricsResetHistogramList(struct timespec timeNow)
{
  t_VBMetricsNodeTraffic *tr_st;
  t_linkedElement *current;
  t_linkedElement *head;

  head = (t_linkedElement *)vbMetricsHistogramsList;

  // Search the given mac
  LIST_FOREACH(head,current)
  {
    tr_st = (t_VBMetricsNodeTraffic *)current;
    memset(&(tr_st->lastEventTime), 0, sizeof(struct timespec));
    memset(tr_st->trafficHistogram,0, sizeof(tr_st->trafficHistogram));
    tr_st->reportCounter = 0;
    tr_st->firstEventTime = timeNow;
  }
}

/************************************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*************************************************************************/

t_VB_engineErrorCode VbEngineMetricsInit(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VbMetricsErrorType metric_err;

  VbMetricsSetEnableBoostTrafficMetrics(VbEngineConfTrafficMetricsEnabledGet());
  VbMetricsSetMaxLogSize(VbEngineConfMaxMetricsLogSizeGet());
  VbMetricsSetSaveMetricsToDisk(VbEngineConfSaveMetricsEnabledGet());

  // Build the metrics queue name
  VbUtilQueueNameBuild(VB_ENGINE_METRICS_QUEUE_NAME, VbEngineConfEngineIdGet(), vbMetricsQueueName);

  metric_err =VbMetricsInit(VB_ENGINE_MEASURES_FOLDER, vbMetricsQueueName);

  if (metric_err != VB_METRICS_NO_ERROR)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error calling VbMetricsInit (%d)", metric_err);
    ret = VB_ENGINE_ERROR_NOT_READY;
  }

  vbEngineMetricsNumBands = VbEngineConfNumPSDBandAllocationGet(VB_TX_MODE_200_MHZ);
  vbEngineMetricsLastUSRate = VbCdtaDefaultQosRateGet();

  return ret;
}

/*************************************************************************/

t_VB_engineErrorCode VbEngineMetricsRun(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  VbStartMetrics();

  // Initialize vbMetricsStartTime
  clock_gettime(CLOCK_MONOTONIC,&vbMetricsStartTime);
  vbMetricsStartTime.tv_sec += VbMetricsGetBootTime();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Initialize Metrics. Buff length=%d", VB_METRICS_EVENTS_BUFFER_LENGTH);

  if (VbMetricsAddReportHandler("CPU calculation times", VbMetricsCPUCalculationReport, NULL) < 0)
  {
    ret = VB_ENGINE_ERROR_METRICS;
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error adding metrics report");
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbMetricsAddReportHandler("Boosted status", VbMetricsBoostLineStatusReport, NULL) < 0)
    {
      ret = VB_ENGINE_ERROR_METRICS;
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error adding metrics report");
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init Logs file path
    if (VbMetricsAddReportHandler("Boosting & traffic log", VbMetricsBoostedLogReport, NULL) < 0)
    {
      ret = VB_ENGINE_ERROR_METRICS;
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error adding metrics report");
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbMetricsAddReportHandler("Traffic histogram", VbMetricsTrafficHistReport, NULL) < 0)
    {
      ret = VB_ENGINE_ERROR_METRICS;
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error adding metrics report");
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init Logs file path
    if (VbMetricsAddReportHandler(NULL, VbMetricsBoostedLogFileReport, VB_ENGINE_METRICS_LOG_FILE) < 0)
    {
      ret = VB_ENGINE_ERROR_METRICS;
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error adding metrics report");
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Assign process functions
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_TRAFFIC_REPORT,      VbMetricsTrafficProcess );
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_BAND_CHANGE,         VbMetricsBandChangeStatsProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_NEW_LINE,            VbMetricsNewNodeProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_START_LOW_CHANN_CAP, VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_END_LOW_CHANN_CAP,   VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_START_FULL_CHANN_CAP,VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_END_FULL_CHANN_CAP,  VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_START_SNR_LOW_CALC,  VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_END_SNR_LOW_CALC,    VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_START_SNR_HIGH_CALC, VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_END_SNR_HIGH_CALC,   VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_START_MEAS_PLAN,     VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_END_MEAS_PLAN,       VbMetricsMeasPlanProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_LINE_LOST,           VbMetricsLostLineProcess);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineAlignMetricsStart();
  }

  return ret;
}

/*************************************************************************/

t_VbMetricsErrorType VbMetricsReportDeviceEvent(t_VBMetricsEventType eventType, INT8U *mac, BOOL DMNodeType, t_VBDriver *driver, INT32U value1, INT32U value2)
{
  t_DeviceMetricsData *info;
  t_VbMetricsErrorType res;

  if(VbMetricsGetStatus())
  {
    info = (t_DeviceMetricsData *)malloc(sizeof(t_DeviceMetricsData));
    if(info)
    {
      if(mac != NULL)
      {
        MACAddrClone(info->mac, mac);
      }
      else
      {
        memset(info->mac, 0, ETH_ALEN);
      }
      info->DMNodeType = DMNodeType;
      info->data1 = value1;
      info->data2 = value2;
      if(driver != NULL)
      {
        strncpy(info->driverId, driver->vbDriverID, sizeof(info->driverId));
      }
      else
      {
        strncpy(info->driverId, "All drivers", sizeof(info->driverId));
      }
      info->driverId[sizeof(info->driverId)-1] = '\0';
#if (_VALGRIND_ == 1)
      ANNOTATE_HAPPENS_BEFORE(value1);
#endif
      res = VbMetricsReportEvent(eventType, info);
    }
    else
    {
      res = VB_METRICS_ERROR_MEMORY;
    }
  }
  else
  {
    res = VB_METRICS_ERROR_STOPPED;
  }

  return res;
}

/*************************************************************************/

t_VbMetricsErrorType VbMetricsReportGenericEvent(t_VBMetricsEventType eventType, void *data, INT32U size)
{
  INT8U *data_ev = NULL;
  t_VbMetricsErrorType res = VB_METRICS_NO_ERROR;

  if(VbMetricsGetStatus())
  {
    if(size > 0)
    {
      data_ev = (INT8U *)memdup(data, size);
#if (_VALGRIND_ == 1)
      ANNOTATE_HAPPENS_BEFORE(data_ev);
#endif
      if(data_ev == NULL)
      {
        res = VB_METRICS_ERROR_MEMORY;
      }
    }
    if(res == VB_METRICS_NO_ERROR)
    {
      res = VbMetricsReportEvent(eventType, data_ev);
    }
  }
  else
  {
    res = VB_METRICS_ERROR_STOPPED;
  }

  return res;
}

/*************************************************************************/

void VbEngineMetricsResetMetrics(void)
{

  VbStopMetrics();

  // Initialize lastChangeTime
  // Initialize vbMetricsStartTime
  clock_gettime(CLOCK_MONOTONIC,&vbMetricsStartTime);
  vbMetricsStartTime.tv_sec += VbMetricsGetBootTime();

  // Clear saved data
  memset(&vbMetricsMeasPlanInfo, 0, sizeof(t_VBMetricsMeasPlanInfo));
  memset(&vbMetricsBandStatsGlobalDM, 0, sizeof(t_VBMetricsBoostingStatsGlobal));
  memset(&vbMetricsBandStatsGlobalEP, 0, sizeof(t_VBMetricsBoostingStatsGlobal));
  memset(&vbMetricsStopTime, 0, sizeof(struct timespec));

  ClearList((t_linkedElement *)vbMetricsCPUMeasurements);
  vbMetricsCPUMeasurements = NULL;

  vbMetricsResetBoostList();
  vbMetricsResetHistogramList(vbMetricsStartTime);

  // Reset alignment metrics
  VbEngineAlignMetricsReset();

  // Reset and start metrics
  VbMetricsResetList();
}

/*************************************************************************/

void VbEngineMetricsStart(void)
{

  if(VbMetricsGetStatus() == FALSE)
  {
    // Initialize lastChangeTime
    // Initialize vbMetricsStartTime
    clock_gettime(CLOCK_MONOTONIC,&vbMetricsStartTime);
    vbMetricsStartTime.tv_sec += VbMetricsGetBootTime();

    // Clear saved data
    memset(&vbMetricsMeasPlanInfo, 0, sizeof(t_VBMetricsMeasPlanInfo));
    memset(&vbMetricsBandStatsGlobalDM, 0, sizeof(t_VBMetricsBoostingStatsGlobal));
    memset(&vbMetricsBandStatsGlobalEP, 0, sizeof(t_VBMetricsBoostingStatsGlobal));
    memset(&vbMetricsStopTime, 0, sizeof(struct timespec));

    ClearList((t_linkedElement *)vbMetricsCPUMeasurements);
    vbMetricsCPUMeasurements = NULL;

    vbMetricsResetBoostList();
    vbMetricsResetHistogramList(vbMetricsStartTime);

    // Reset and start metrics
    VbMetricsResetList();
  }
}

/*************************************************************************/

void VbEngineMetricsStop(void)
{
  INT64S diff;

  if(VbMetricsGetStatus() == TRUE)
  {
    // Get reference time
    clock_gettime(CLOCK_MONOTONIC, &vbMetricsStopTime);

    vbMetricsStopTime.tv_sec += VbMetricsGetBootTime();

    // Update global info
    diff = VbUtilElapsetimeTimespecMs(
        vbMetricsBandStatsGlobalDM.lastTimeWithNodesInBand, //[vbMetricsBandStatsGlobalDM.currentHigherBand],
        vbMetricsStopTime);
    vbMetricsBandStatsGlobalDM.totalTimeWithNodesInBand[vbMetricsBandStatsGlobalDM.currentHigherBand] += diff;
    vbMetricsBandStatsGlobalDM.lastTimeWithNodesInBand = vbMetricsStopTime;

    diff = VbUtilElapsetimeTimespecMs(
        vbMetricsBandStatsGlobalEP.lastTimeWithNodesInBand, //[vbMetricsBandStatsGlobalEP.currentHigherBand],
        vbMetricsStopTime);
    vbMetricsBandStatsGlobalEP.totalTimeWithNodesInBand[vbMetricsBandStatsGlobalEP.currentHigherBand] += diff;
    vbMetricsBandStatsGlobalEP.lastTimeWithNodesInBand = vbMetricsStopTime;

    diff = VbUtilElapsetimeTimespecMs(
        vbMetricsBandStatsGlobalDM.lastTimeWithNodesInFullBand, //[vbMetricsBandStatsGlobalDM.currentNrNodesinFullBand],
        vbMetricsStopTime);
    vbMetricsBandStatsGlobalDM.totalTimeWithNodesInFullBand[vbMetricsBandStatsGlobalDM.currentNrNodesinFullBand] += diff;
    vbMetricsBandStatsGlobalDM.lastTimeWithNodesInFullBand = vbMetricsStopTime;

    diff = VbUtilElapsetimeTimespecMs(
        vbMetricsBandStatsGlobalEP.lastTimeWithNodesInFullBand, //[vbMetricsBandStatsGlobalEP.currentNrNodesinFullBand],
        vbMetricsStopTime);
    vbMetricsBandStatsGlobalEP.totalTimeWithNodesInFullBand[vbMetricsBandStatsGlobalEP.currentNrNodesinFullBand] += diff;
    vbMetricsBandStatsGlobalEP.lastTimeWithNodesInFullBand = vbMetricsStopTime;

    VbStopMetrics();
  }
}

/*************************************************************************/

INT32U VbMetricsGetMaxLogSize()
{
  return vbMetricsMaxLogSize;
}

/*************************************************************************/

BOOL VbMetricsGetSaveMetricsToDisk()
{
  return vbMetricsSaveMetricsToDisk;
}

/*************************************************************************/

BOOL VbMetricsGetEnableCPUTimingMetrics()
{
  return  vbMetricsEnableCPUTimingMetrics;
}

/*************************************************************************/

BOOL VbMetricsGetEnableBoostTrafficMetrics()
{
  return  vbMetricsEnableBoostTrafficMetrics;
}

/*************************************************************************/

void VbMetricsSetEnableCPUTimingMetrics(BOOL value)
{
  VBMetricsEnableEvent(VB_METRICS_EVENT_START_LOW_CHANN_CAP, value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_END_LOW_CHANN_CAP,   value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_START_FULL_CHANN_CAP,value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_END_FULL_CHANN_CAP,  value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_START_SNR_LOW_CALC, value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_END_SNR_LOW_CALC,   value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_START_SNR_HIGH_CALC, value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_END_SNR_HIGH_CALC,   value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_START_MEAS_PLAN,     value);
  VBMetricsEnableEvent(VB_METRICS_EVENT_END_MEAS_PLAN,       value);

  vbMetricsEnableCPUTimingMetrics = value;
}

/*************************************************************************/

void VbMetricsSetEnableBoostTrafficMetrics(BOOL value)
{
  VBMetricsEnableEvent(VB_METRICS_EVENT_TRAFFIC_REPORT, value );
  VBMetricsEnableEvent(VB_METRICS_EVENT_BAND_CHANGE, value );
  VBMetricsEnableEvent(VB_METRICS_EVENT_UD_RATE_CHANGE, value );
  VBMetricsEnableEvent(VB_METRICS_EVENT_LINE_LOST, value);

  vbMetricsEnableBoostTrafficMetrics = value;
}

/*************************************************************************/

void VbMetricsSetMaxLogSize(INT32U value)
{
  vbMetricsMaxLogSize = value;
}

/*************************************************************************/

void VbMetricsSetSaveMetricsToDisk(BOOL value)
{
  vbMetricsSaveMetricsToDisk = value;
}

/*************************************************************************/

void VbMetricsBoostLineStatusReport(CHAR **buffer, INT32U *buffSize)
{
  t_VBMetricsBoostingStatsLine *current;
  t_linkedElement              *elem;
  INT32U                        average = 0;
  INT32U i, percentage;
  INT32U totalDMs, totalEPs;
  INT32U hours, minutes, seconds, total_sec;
  INT64S total_time, accum_time, curr_time;
  INT64U band_time[VB_PSD_NUM_BANDS] = {0};
  INT64U line_time[VB_ENGINE_METRICS_MAX_NUMBER_LINES] = {0};
  struct timespec timeNow;
  struct tm       tmu;

  if ((buffer != NULL) && (*buffer != NULL) && (buffSize != NULL))
  {
    // Initialize vbMetricsStartTime
    clock_gettime(CLOCK_MONOTONIC,&timeNow);
    timeNow.tv_sec += VbMetricsGetBootTime();

    localtime_r(&timeNow.tv_sec, &tmu);

    total_time = VbUtilElapsetimeTimespecMs(vbMetricsStartTime, timeNow);

    VbUtilStringToBuffer(buffer, buffSize,
        "\nLines Boosting status report\n"
        "============================\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "Report date   [%4d/%02d/%02d %02d:%02d:%02d]\n", (tmu.tm_year+1900), tmu.tm_mon + 1,
        tmu.tm_mday, tmu.tm_hour, tmu.tm_min, tmu.tm_sec);

    VbUtilStringToBuffer(buffer, buffSize,
        "\n=============================================================================================================\n"
        "|        Node       | Type |       Driver id      | %Time in band 1 |  2  |  3  |  4  |  5  | Avg Bands used|\n"
        "=============================================================================================================\n");

    totalDMs = 0;
    totalEPs = 0;
    LIST_FOREACH((t_linkedElement *)vbMetricsBoostingStatsLine,elem)
    {
      current = (t_VBMetricsBoostingStatsLine *)elem;

      VbUtilStringToBuffer(buffer, buffSize,
          "| %02X:%02X:%02X:%02X:%02X:%02X |  %s  | %20s |            ",
          current->l.mac[0], current->l.mac[1], current->l.mac[2],
          current->l.mac[3], current->l.mac[4], current->l.mac[5],
          (current->DMNode) ? "DM" : "EP",
          current->driverId);

      average = 0;
      for(i = 0; i<vbEngineMetricsNumBands; i++)
      {
        if(current->currentBand == i)
        {
          curr_time = VbUtilElapsetimeTimespecMs(current->lastBandBeginTime[i], timeNow);
        }
        else
        {
          curr_time = 0;
        }
        percentage = (unsigned int)(((current->totalTimeBoosted[i] + curr_time)*100)/total_time);
        average += (i+1)*percentage;

        VbUtilStringToBuffer(buffer, buffSize, "%3u%% |", percentage);
      }
      VbUtilStringToBuffer(buffer, buffSize, "      %2u.%02u    |\n", (unsigned int)((average/100)), (unsigned int)(average%100));

      if(current->DMNode)
      {
        totalDMs++;
      }
      else
      {
        totalEPs++;
      }
    }

    VbUtilStringToBuffer(buffer, buffSize,
        "=============================================================================================================\n");

    // Get the time elapsed since the metrics started
    // Metrics are running, no stop time
    if(vbMetricsStopTime.tv_sec == 0)
    {
      total_time = VbUtilElapsetimeTimespecMs( vbMetricsStartTime, timeNow );
    }
    else
    {
      total_time = VbUtilElapsetimeTimespecMs( vbMetricsStartTime, vbMetricsStopTime);
    }

    total_sec = total_time / 1000;
    hours   = total_sec / 3600;
    minutes = (total_sec % 3600)/60;
    seconds = (total_sec % 60);

    VbUtilStringToBuffer(buffer, buffSize,
        "\nTime per Boosting lines report\n"
        "==============================\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "Total monitoring time : %02u:%02u:%02u s \n", (unsigned int)hours, (unsigned int)minutes, (unsigned int)seconds);

    VbUtilStringToBuffer(buffer, buffSize,
        "Bands used now     DM : %u  EP : %u\n",
        (unsigned int)vbMetricsBandStatsGlobalDM.currentHigherBand+1,
        (unsigned int)vbMetricsBandStatsGlobalEP.currentHigherBand+1);

    VbUtilStringToBuffer(buffer, buffSize,
        "\nDOWNSTREAM ================>\n");
    VbUtilStringToBuffer(buffer, buffSize,
        "\n==========================================\n"
        "| Max Boost Level   | %% over total time  |\n"
        "==========================================\n");

    accum_time = 0;
    for(i=0; i<vbEngineMetricsNumBands; i++)
    {
      band_time[i] = vbMetricsBandStatsGlobalDM.totalTimeWithNodesInBand[i];
      if(vbMetricsBandStatsGlobalDM.currentHigherBand == i)
      {
        if(vbMetricsBandStatsGlobalDM.lastTimeWithNodesInBand.tv_sec != 0)
        {
          // Get the time since last change
          band_time[i] += VbUtilElapsetimeTimespecMs( vbMetricsBandStatsGlobalDM.lastTimeWithNodesInBand, timeNow );
        }
        else
        {
          band_time[i] += total_time;
        }
      }
      accum_time += band_time[i];
    }

    for(i=0; i<vbEngineMetricsNumBands; i++)
    {
      percentage = VbMetricsCalculateRoundPercentage(band_time[i], accum_time);

      VbUtilStringToBuffer(buffer, buffSize,
          "|        %2u         |        %3u %%       |\n",
          (unsigned int)(i+1),
          (unsigned int)percentage);
    }

    VbUtilStringToBuffer(buffer, buffSize,
        "==========================================\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "\n============================================\n"
        "|    DMs in fullband  | %% over total time  |\n"
        "============================================\n");

    accum_time = 0;
    for(i=0; i<=totalDMs; i++)
    {
      line_time[i] = vbMetricsBandStatsGlobalDM.totalTimeWithNodesInFullBand[i];
      if(i == vbMetricsBandStatsGlobalDM.currentNrNodesinFullBand)
      {
        if(vbMetricsBandStatsGlobalDM.lastTimeWithNodesInFullBand.tv_sec != 0)
        {
          line_time[i] += VbUtilElapsetimeTimespecMs( vbMetricsBandStatsGlobalDM.lastTimeWithNodesInFullBand, timeNow );
        }
        else
        {
          line_time[i] += total_time;
        }
      }
      accum_time += line_time[i];
    }


    for(i=0; i<=totalDMs; i++)
    {
      percentage = VbMetricsCalculateRoundPercentage(line_time[i], accum_time);

      VbUtilStringToBuffer(buffer, buffSize,
          "|         %2u          |        %3u %%       |\n",
          (unsigned int)(i),
          (unsigned int)percentage);
    }

    VbUtilStringToBuffer(buffer, buffSize,
        "============================================\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "\n\nUPSTREAM <================\n");
    VbUtilStringToBuffer(buffer, buffSize,
        "\n==========================================\n"
        "|  Max Boost Level  | %% over total time  |\n"
        "==========================================\n");

    accum_time = 0;
    memset(band_time, 0, sizeof(INT64U)*VB_PSD_NUM_BANDS);
    for(i = 0; i<vbEngineMetricsNumBands; i++)
    {
      band_time[i] = vbMetricsBandStatsGlobalEP.totalTimeWithNodesInBand[i];
      if(vbMetricsBandStatsGlobalEP.currentHigherBand == i)
      {
        if(vbMetricsBandStatsGlobalEP.lastTimeWithNodesInBand.tv_sec != 0)
        {
          // Get the time since last change
          band_time[i] += VbUtilElapsetimeTimespecMs( vbMetricsBandStatsGlobalEP.lastTimeWithNodesInBand, timeNow );
        }
        else
        {
          band_time[i] += total_time;
        }
      }
      accum_time += band_time[i];
    }

    for(i = 0; i<vbEngineMetricsNumBands; i++)
    {
      percentage = VbMetricsCalculateRoundPercentage(band_time[i], accum_time);

      VbUtilStringToBuffer(buffer, buffSize,
          "|        %2u         |        %3u %%       |\n",
          (unsigned int)(i+1),
          (unsigned int)percentage);
    }

    VbUtilStringToBuffer(buffer, buffSize,
        "==========================================\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "\n============================================\n"
        "|    EPs in fullband  | %% over total time  |\n"
        "============================================\n");

    accum_time = 0;
    for(i=0; i<=totalEPs; i++)
    {
      line_time[i] = vbMetricsBandStatsGlobalEP.totalTimeWithNodesInFullBand[i];
      if(i == vbMetricsBandStatsGlobalEP.currentNrNodesinFullBand)
      {
        if(vbMetricsBandStatsGlobalEP.lastTimeWithNodesInFullBand.tv_sec != 0)
        {
          line_time[i] += VbUtilElapsetimeTimespecMs( vbMetricsBandStatsGlobalEP.lastTimeWithNodesInFullBand, timeNow );
        }
        else
        {
          line_time[i] += total_time;
        }
      }
      accum_time += line_time[i];
    }


    for(i=0; i<=totalEPs; i++)
    {
      percentage = VbMetricsCalculateRoundPercentage(line_time[i], accum_time);

      VbUtilStringToBuffer(buffer, buffSize,
          "|         %2u          |        %3u %%       |\n",
          (unsigned int)(i),
          (unsigned int)percentage);
    }

    VbUtilStringToBuffer(buffer, buffSize,
        "============================================\n");

  }
}

/*************************************************************************/

void VbMetricsBoostedLogFileReport(CHAR **buffer, INT32U *buffSize)
{
  t_VbMetricsErrorType   res;
  t_VB_MetricsEvent     *event;
  INT32S                 index;
  INT32U                 traffic;
  INT32U                 us_rate;
  t_DeviceMetricsData   *tr_data;
  CHAR                  *ev_desc;
  CHAR                  *drv_id;
  INT8U                 *mac;
  BOOL                   print_log;
  struct tm              tmu;
  t_VBMetricsBoostingStatsLine *devs_list = NULL;
  INT32U                 num_devs_us_band[VB_PSD_NUM_BANDS] = {0};
  INT32U                 num_devs_ds_band[VB_PSD_NUM_BANDS] = {0};
  CHAR                  *tr_ev_description[]= {
  "TR_OVER_100 ",
  "TR_OVER_200 ",
  "TR_OVER_300 ",
  "TR_OVER_400 ",
  "TR_OVER_500 ",
  "TR_OVER_600 ",
  "TR_OVER_700 ",
  "TR_OVER_800 ",
  "TR_UNDER_100",
  "TR_UNDER_200",
  "TR_UNDER_300",
  "TR_UNDER_400",
  "TR_UNDER_500",
  "TR_UNDER_600",
  "TR_UNDER_700",
  "TR_UNDER_800"
      };

  if ((buffer != NULL) && (*buffer != NULL) && (buffSize != NULL))
  {
    if(vbMetricsSaveMetricsToDisk)
    {
      res = VbMetricsReadNextEvent(&event);
    }
    else
    {
      // Exit
      res = VB_METRICS_ERROR_STOPPED;
    }

    if(res == VB_METRICS_NO_ERROR)
    {
      us_rate = vbEngineMetricsLastUSRate * 10;

      if(!vbMetricsLogFileHeaderWritten)
      {
        time_t t;

        t = time(NULL);
        localtime_r(&t, &tmu);

        VbUtilStringToBuffer(buffer, buffSize,
            "Report date   [%4d/%02d/%02d]\n", (tmu.tm_year+1900), tmu.tm_mon +1, tmu.tm_mday);

        VbUtilStringToBuffer(buffer, buffSize,
          "\n                                                                                            dev using DS Bands ||   devs using US Bands\n"
            "=========================================================================================================================================================\n"
            "|   Timestamp   |        Node       |       Driver id      |     Event     |  Inst Traffic | 1 | 2 | 3 | 4 | 5 || 1 | 2 | 3 | 4 | 5 | Up Rate |Down Rate|\n"
            "=========================================================================================================================================================\n");

        vbMetricsLogFileHeaderWritten = TRUE;
      }

      // Get all boosted status events from metrics
      do
      {
        print_log = TRUE;
        switch(event->eventType)
        {
        case VB_METRICS_EVENT_BAND_CHANGE:
          tr_data  = (t_DeviceMetricsData *)event->eventData;
          traffic = tr_data->data1;
          mac     = tr_data->mac;
          drv_id  = tr_data->driverId;
          ev_desc = "BAND CHANGE ";
          if(tr_data->DMNodeType)
          {
            vbMetricsUpdateBandsCounters(&devs_list, num_devs_ds_band, tr_data->mac, tr_data->data2);
          }
          else
          {
            vbMetricsUpdateBandsCounters(&devs_list, num_devs_us_band, tr_data->mac, tr_data->data2);
          }
          break;

        case VB_METRICS_EVENT_UD_RATE_CHANGE:
          tr_data  = (t_DeviceMetricsData *)event->eventData;
          traffic = tr_data->data1;
          us_rate = tr_data->data2*10;
          mac     = tr_data->mac;
          drv_id  = tr_data->driverId;
          ev_desc = "UD RATE CHNG";
          break;

          case VB_METRICS_EVENT_NEW_LINE:
            tr_data  = (t_DeviceMetricsData *)event->eventData;
            traffic = 0;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            ev_desc = " NEW NODE   ";
            break;

          case VB_METRICS_EVENT_LINE_LOST:
            tr_data  = (t_DeviceMetricsData *)event->eventData;
            traffic = 0;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            ev_desc = " LINE LOST  ";
            break;


          case VB_METRICS_EVENT_TRAFFIC_OVER_105:
          case VB_METRICS_EVENT_TRAFFIC_OVER_205:
          case VB_METRICS_EVENT_TRAFFIC_OVER_305:
          case VB_METRICS_EVENT_TRAFFIC_OVER_405:
          case VB_METRICS_EVENT_TRAFFIC_OVER_505:
          case VB_METRICS_EVENT_TRAFFIC_OVER_605:
          case VB_METRICS_EVENT_TRAFFIC_OVER_705:
          case VB_METRICS_EVENT_TRAFFIC_OVER_805:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_95:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_195:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_295:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_395:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_495:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_595:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_695:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_795:
            tr_data = (t_DeviceMetricsData *)event->eventData;
            traffic = tr_data->data1;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            index = (INT32S)(event->eventType - VB_METRICS_EVENT_TRAFFIC_OVER_105);
            ev_desc = tr_ev_description[index];
            break;

          default:
            print_log = FALSE;
            break;
        }

        if(print_log)
        {
          localtime_r(&(event->eventTime.tv_sec), &tmu);

          VbUtilStringToBuffer(buffer, buffSize,
              "|%2d:%02d:%02d %03dms | %02X:%02X:%02X:%02X:%02X:%02X | %20s |  %s |      %3u Mbps |%2u |%2u |%2u |%2u |%2u ||%2u |%2u |%2u |%2u |%2u |  %3u    |  %3u    |\n",
              tmu.tm_hour, tmu.tm_min, tmu.tm_sec,
              (int)(event->eventTime.tv_nsec/1000000),
              mac[0], mac[1], mac[2],
              mac[3], mac[4], mac[5],
              drv_id,
              ev_desc,
              (unsigned int)traffic,
              (unsigned int)num_devs_ds_band[0],(unsigned int)num_devs_ds_band[1],
              (unsigned int)num_devs_ds_band[2],(unsigned int)num_devs_ds_band[3],
              (unsigned int)num_devs_ds_band[4],
              (unsigned int)num_devs_us_band[0],(unsigned int)num_devs_us_band[1],
              (unsigned int)num_devs_us_band[2],(unsigned int)num_devs_us_band[3],
              (unsigned int)num_devs_us_band[4],
              (unsigned int)us_rate,
              (unsigned int)(100 - us_rate));
        }

        res = VbMetricsReadNextEvent(&event);

      }while(res == VB_METRICS_NO_ERROR);

      vbEngineMetricsLastUSRate = us_rate / 10;
    }
  }
}

/*************************************************************************/

void VbMetricsBoostedLogReport( CHAR **buffer, INT32U *buffSize)
{
  t_VbMetricsErrorType   res;
  INT32S                 current_offset;
  t_VB_MetricsEvent     *event;
  INT32S                 index;
  INT32U                 traffic;
  INT32U                 us_rate;
  INT32U                 num_devs_us_band[VB_PSD_NUM_BANDS] = {0};
  INT32U                 num_devs_ds_band[VB_PSD_NUM_BANDS] = {0};
  t_DeviceMetricsData   *tr_data;
  CHAR                  *ev_desc;
  CHAR                  *drv_id;
  INT8U                 *mac;
  BOOL                   print_log;
  struct tm              tmu;
  t_VBMetricsBoostingStatsLine *devs_list = NULL;
  CHAR                  *tr_ev_description[]= {
  "TR_OVER_100 ",
  "TR_OVER_200 ",
  "TR_OVER_300 ",
  "TR_OVER_400 ",
  "TR_OVER_500 ",
  "TR_OVER_600 ",
  "TR_OVER_700 ",
  "TR_OVER_800 ",
  "TR_UNDER_100",
  "TR_UNDER_200",
  "TR_UNDER_300",
  "TR_UNDER_400",
  "TR_UNDER_500",
  "TR_UNDER_600",
  "TR_UNDER_700",
  "TR_UNDER_800"
      };

  if ((buffer != NULL) && (*buffer != NULL) && (buffSize != NULL))
  {
    // Save last saved offset

    us_rate = VbCdtaDefaultQosRateGet() * 10;

    current_offset = 0;
    res = VbMetricsGetEventByIndex(&event, current_offset);

    if(res == VB_METRICS_NO_ERROR)
    {
      VbUtilStringToBuffer(buffer, buffSize,
        "\n                                                                                            dev using DS Bands ||   devs using US Bands\n"
          "=========================================================================================================================================================\n"
          "|   Timestamp   |        Node       |       Driver id      |     Event     |  Inst Traffic | 1 | 2 | 3 | 4 | 5 || 1 | 2 | 3 | 4 | 5 | Up Rate |Down Rate|\n"
          "=========================================================================================================================================================\n");

      // Get all boosted status events from metrics
      do
      {
        print_log = TRUE;
        switch(event->eventType)
        {
          case VB_METRICS_EVENT_BAND_CHANGE:
            tr_data  = (t_DeviceMetricsData *)event->eventData;
            traffic = tr_data->data1;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            ev_desc = "BAND CHANGE ";
            if(tr_data->DMNodeType)
            {
              vbMetricsUpdateBandsCounters(&devs_list, num_devs_ds_band, tr_data->mac, tr_data->data2);
            }
            else
            {
              vbMetricsUpdateBandsCounters(&devs_list, num_devs_us_band, tr_data->mac, tr_data->data2);
            }
            break;

          case VB_METRICS_EVENT_UD_RATE_CHANGE:
            tr_data  = (t_DeviceMetricsData *)event->eventData;
            traffic = tr_data->data1;
            us_rate = tr_data->data2*10;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            ev_desc = "UD RATE CHNG";
            break;

          case VB_METRICS_EVENT_NEW_LINE:
            tr_data  = (t_DeviceMetricsData *)event->eventData;
            traffic = 0;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            ev_desc = "NEW NODE    ";
            if(tr_data->DMNodeType)
            {
              vbMetricsUpdateBandsCounters(&devs_list, num_devs_ds_band, tr_data->mac, 0);
            }
            else
            {
              vbMetricsUpdateBandsCounters(&devs_list, num_devs_us_band, tr_data->mac, 0);
            }
            break;

          case VB_METRICS_EVENT_LINE_LOST:
            tr_data  = (t_DeviceMetricsData *)event->eventData;
            traffic = 0;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            ev_desc = "LINE LOST   ";
            break;


          case VB_METRICS_EVENT_TRAFFIC_OVER_105:
          case VB_METRICS_EVENT_TRAFFIC_OVER_205:
          case VB_METRICS_EVENT_TRAFFIC_OVER_305:
          case VB_METRICS_EVENT_TRAFFIC_OVER_405:
          case VB_METRICS_EVENT_TRAFFIC_OVER_505:
          case VB_METRICS_EVENT_TRAFFIC_OVER_605:
          case VB_METRICS_EVENT_TRAFFIC_OVER_705:
          case VB_METRICS_EVENT_TRAFFIC_OVER_805:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_95:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_195:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_295:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_395:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_495:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_595:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_695:
          case VB_METRICS_EVENT_TRAFFIC_UNDER_795:
            tr_data = (t_DeviceMetricsData *)event->eventData;
            traffic = tr_data->data1;
            mac     = tr_data->mac;
            drv_id  = tr_data->driverId;
            index = (INT32S)(event->eventType - VB_METRICS_EVENT_TRAFFIC_OVER_105);
            ev_desc = tr_ev_description[index];
            break;

          default:
            print_log = FALSE;
            break;
        }

        if(print_log)
        {
          localtime_r(&(event->eventTime.tv_sec), &tmu);

          VbUtilStringToBuffer(buffer, buffSize,
              "|%2d:%02d:%02d %03dms | %02X:%02X:%02X:%02X:%02X:%02X | %20s |  %s |      %3u Mbps |%2u |%2u |%2u |%2u |%2u ||%2u |%2u |%2u |%2u |%2u |  %3u    |  %3u    |\n",
              tmu.tm_hour, tmu.tm_min, tmu.tm_sec,
              (int)(event->eventTime.tv_nsec/1000000),
              mac[0], mac[1], mac[2],
              mac[3], mac[4], mac[5],
              drv_id,
              ev_desc,
              (unsigned int)traffic,
              (unsigned int)num_devs_ds_band[0],(unsigned int)num_devs_ds_band[1],
              (unsigned int)num_devs_ds_band[2],(unsigned int)num_devs_ds_band[3],
              (unsigned int)num_devs_ds_band[4],
              (unsigned int)num_devs_us_band[0],(unsigned int)num_devs_us_band[1],
              (unsigned int)num_devs_us_band[2],(unsigned int)num_devs_us_band[3],
              (unsigned int)num_devs_us_band[4],
              (unsigned int)us_rate,
              (unsigned int)(100 - us_rate));
        }

        current_offset++;
        res = VbMetricsGetEventByIndex(&event, current_offset);

      }while(res == VB_METRICS_NO_ERROR);

      VbUtilStringToBuffer(buffer, buffSize,
          "=========================================================================================================================================================\n");
    }
    else
    {
      VbUtilStringToBuffer(buffer, buffSize,
          "\n Logs buffer is empty.\n");
    }
  }
}

/*******************************************************************/

void VbMetricsTrafficHistReport(CHAR **buffer, INT32U *buffSize)
{
  struct timespec timeFromStart;
  struct tm       tmu;
  time_t t;
  t_VBMetricsNodeTraffic *curr_node;
  t_linkedElement *elem;
  t_linkedMACElement *elemMAC;

  if ((buffer != NULL) && (*buffer != NULL) && (buffSize != NULL))
  {
    VbMetricsGetElapsedTime(&timeFromStart);

    t = time(NULL);
    localtime_r(&t, &tmu);

    VbUtilStringToBuffer(buffer, buffSize,
        "\nTraffic Histograms report\n"
        "===========================\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "Report time       : [%02d:%02d:%02d]\n", tmu.tm_hour, tmu.tm_min, tmu.tm_sec);

    LIST_FOREACH((t_linkedElement *)vbMetricsHistogramsList, elem)
    {
      curr_node = (t_VBMetricsNodeTraffic *)elem;
      elemMAC = (t_linkedMACElement *)elem;

      VbUtilStringToBuffer(buffer, buffSize,
          "\nNode              : %02X:%02X:%02X:%02X:%02X:%02X",
          elemMAC->mac[0], elemMAC->mac[1], elemMAC->mac[2],
          elemMAC->mac[3], elemMAC->mac[4], elemMAC->mac[5]);

      VbUtilStringToBuffer(buffer, buffSize,
          "\nNumber of reports : %2u \n", (unsigned int)curr_node->reportCounter);

      DrawHistogram(buffer, buffSize, curr_node);
    }
  }
}

/*******************************************************************/

void VbMetricsCPUCalculationReport(CHAR **buffer, INT32U *buffSize)
{
  t_VBMetricsMeasTimes *curr_meas;
  t_linkedElement *elem;
  CHAR *driver_id;
  struct tm         tmu;

  if ((buffer != NULL) && (*buffer != NULL) && (buffSize != NULL))
  {
    localtime_r(&(vbMetricsMeasPlanInfo.measTimestamp.tv_sec), &tmu);

    VbUtilStringToBuffer(buffer, buffSize,
        "\nCPU timing report\n"
        "=================\n");

    // This parameters are the same for each MAC
    VbUtilStringToBuffer(buffer, buffSize,
        "\nMeasurement time   : [%2d:%02d:%02d %03dms]\n", tmu.tm_hour, tmu.tm_min, tmu.tm_sec,
        (int)(vbMetricsMeasPlanInfo.measTimestamp.tv_nsec/ 1000000));

    VbUtilStringToBuffer(buffer, buffSize,
        "Plan ID            : %u\n", (unsigned int)vbMetricsMeasPlanInfo.planId);

    VbUtilStringToBuffer(buffer, buffSize,
        "Meas total time    : %d.%06u ms\n", (unsigned int)vbMetricsMeasPlanInfo.totalMeasPlan/1000000,
        (unsigned int)(vbMetricsMeasPlanInfo.totalMeasPlan % 1000000));


    VbUtilStringToBuffer(buffer, buffSize,
        "\n====================================================================================================================================================================\n"
        "|        Node       | Type |       Driver id      | Full Chann cap calc time | Low Chann cap calc time |   SNR Low Xtalk Calc time  |      SNR High Xtalk Calc time  |\n"
        "======================================================================================================================================================================\n");


    LIST_FOREACH((t_linkedElement *)vbMetricsCPUMeasurements,elem)
    {
      curr_meas = (t_VBMetricsMeasTimes *)elem;
      driver_id = curr_meas->driverId;

      VbUtilStringToBuffer(buffer, buffSize,
          "| %02X:%02X:%02X:%02X:%02X:%02X |  %s  | %20s |        %10d ns     |      %10d ns      |      %10d ns      |      %10d ns      |\n",
          curr_meas->l.mac[0], curr_meas->l.mac[1], curr_meas->l.mac[2],
          curr_meas->l.mac[3], curr_meas->l.mac[4], curr_meas->l.mac[5],
          (curr_meas->DMNode) ? "DM" : "EP",
              driver_id,
              (int)curr_meas->fullChanCapCalc,
              (int)curr_meas->lowChanCapCalc,
              (int)curr_meas->SNRLowXtalkCalc,
              (int)curr_meas->SNRHighXtalkCalc);

    }

    VbUtilStringToBuffer(buffer, buffSize,
        "====================================================================================================================================================================\n");
  }
}

/*******************************************************************/

void VbEngineDumpMetricsToFiles()
{
  CHAR   file_path[VB_ENGINE_METRICS_MAX_PATH_LEN];
  CHAR  *buffer;
  CHAR  *ptr_to_write;
  INT32U remaining_size;

  if (vbMetricsEnableBoostTrafficMetrics || vbMetricsEnableCPUTimingMetrics)
  {
    buffer = (CHAR *)malloc(VB_METRICS_OUTPUT_BUFFER_SIZE);

    if (buffer != NULL)
    {
      if (vbMetricsEnableBoostTrafficMetrics)
      {

        snprintf(
            file_path,
            (VB_ENGINE_METRICS_MAX_PATH_LEN -1),
            "%sboost_mode_report.txt",
            VbMetricsGetReportsPath()
        );

        ptr_to_write = buffer;
        remaining_size = VB_METRICS_OUTPUT_BUFFER_SIZE;

        // Generate text
        VbMetricsBoostLineStatusReport(&ptr_to_write, &remaining_size);
        VbMetricsTrafficHistReport(&ptr_to_write, &remaining_size);

        // Save to file
        VbLogSaveBufferToTextFile(file_path, "w+", VB_METRICS_OUTPUT_BUFFER_SIZE, buffer);
      }

      if (vbMetricsEnableCPUTimingMetrics)
      {
        snprintf(
            file_path,
            (VB_ENGINE_METRICS_MAX_PATH_LEN -1),
            "%sCPU_time_measurements.txt",
            VbMetricsGetReportsPath()
        );

        ptr_to_write = buffer;
        remaining_size = VB_METRICS_OUTPUT_BUFFER_SIZE;

        // TODO: Anyadir hora local?
        VbMetricsCPUCalculationReport(&ptr_to_write, &remaining_size);

        // Save to file
        VbLogSaveBufferToTextFile(file_path, "w+", VB_METRICS_OUTPUT_BUFFER_SIZE, buffer);
      }
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Cannot dump metrics to file. Memory allocation error");
    }

    free(buffer);
  }

  if(VbMetricsCheckEnoughSpaceForMetrics())
  {
    // Save boost logs
    VbMetricsDumpReportsToFiles();
  }
  else
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Not enough sapce in disk to write log file!");
  }
}

/************************************************************************************/

BOOL VbMetricsMeasPlanProcess(t_VB_MetricsEvent *event)
{
  t_VBMetricsMeasTimes *times_st;
  t_SNRMetricsData     *SNR_data;
  t_DeviceMetricsData  *chann_data;
  INT8U *mac;
  BOOL insert_in_buffer;

  insert_in_buffer = FALSE;

  switch(event->eventType)
  {
    case VB_METRICS_EVENT_START_MEAS_PLAN:
      vbMetricsMeasPlanInfo.measPlanBeginTime = event->eventTime;
      vbMetricsMeasPlanInfo.measTimestamp = event->eventTime;
      vbMetricsMeasPlanInfo.planId = *((INT8U *)event->eventData);
      insert_in_buffer = TRUE;
      break;

    case VB_METRICS_EVENT_END_MEAS_PLAN:
      vbMetricsMeasPlanInfo.totalMeasPlan = VbUtilElapsetimeTimespecNs(&(vbMetricsMeasPlanInfo.measPlanBeginTime), &(event->eventTime));
      insert_in_buffer = TRUE;
      break;

    case VB_METRICS_EVENT_START_LOW_CHANN_CAP:
      chann_data = (t_DeviceMetricsData*)event->eventData;
      times_st = GetMeasStructByMac(chann_data->mac);
      if(times_st)
      {
        times_st->lowChanBeginTime = event->eventTime;
        times_st->DMNode = chann_data->DMNodeType;
        strncpy(times_st->driverId, chann_data->driverId, VB_EA_DRIVER_ID_MAX_SIZE);
      }
      break;

    case VB_METRICS_EVENT_END_LOW_CHANN_CAP:
      mac = event->eventData;
      times_st = GetMeasStructByMac(mac);
      if(times_st)
        times_st->lowChanCapCalc = VbUtilElapsetimeTimespecNs(&(times_st->lowChanBeginTime), &(event->eventTime));
      break;

    case VB_METRICS_EVENT_START_FULL_CHANN_CAP:
      mac = event->eventData;
      times_st = GetMeasStructByMac(mac);
      if(times_st)
        times_st->fullChanBeginTime = event->eventTime;
      break;

    case VB_METRICS_EVENT_END_FULL_CHANN_CAP:
      mac = event->eventData;
      times_st = GetMeasStructByMac(mac);
      if(times_st)
        times_st->fullChanCapCalc = VbUtilElapsetimeTimespecNs(&(times_st->fullChanBeginTime), &(event->eventTime));
      break;

    case VB_METRICS_EVENT_START_SNR_LOW_CALC:
      mac = event->eventData;
      times_st = GetMeasStructByMac(mac);
      if(times_st)
        times_st->SNRLowBeginTime = event->eventTime;
      break;

    case VB_METRICS_EVENT_END_SNR_LOW_CALC:
      SNR_data = (t_SNRMetricsData *)event->eventData;
      times_st = GetMeasStructByMac(SNR_data->mac);
      if(times_st)
      {
        times_st->SNRLowXtalkCalc = VbUtilElapsetimeTimespecNs(&(times_st->SNRLowBeginTime), &(event->eventTime));
        memcpy(times_st->timeMarkers, SNR_data->vbMetricsTimeMarkersList,
            sizeof(t_VBMetricsTimeMarker) * VB_METRICS_MAX_NR_TIME_MARKERS);
      }
      break;

    case VB_METRICS_EVENT_START_SNR_HIGH_CALC:
      mac = event->eventData;
      times_st = GetMeasStructByMac(mac);
      if(times_st)
        times_st->SNRFullBeginTime = event->eventTime;
      break;

    case VB_METRICS_EVENT_END_SNR_HIGH_CALC:
      SNR_data = (t_SNRMetricsData *)event->eventData;
      times_st = GetMeasStructByMac(SNR_data->mac);
      if(times_st)
      {
        times_st->SNRHighXtalkCalc = VbUtilElapsetimeTimespecNs(&(times_st->SNRFullBeginTime), &(event->eventTime));
        memcpy(times_st->timeMarkers, SNR_data->vbMetricsTimeMarkersList,
            sizeof(t_VBMetricsTimeMarker) * VB_METRICS_MAX_NR_TIME_MARKERS);
      }
      break;

    default:
      break;
  }

  return insert_in_buffer;
}

/**************************************************************************/

static void VbMetricsUpdateGlobalBandStats(BOOL DMNode, INT32U currBand, INT32U newBand, struct timespec eventTime)
{
  t_VBMetricsBoostingStatsGlobal *band_stats;
  INT32S i;

  if(DMNode)
  {
    band_stats = &vbMetricsBandStatsGlobalDM;
  }
  else
  {
    band_stats = &vbMetricsBandStatsGlobalEP;
  }

  if(band_stats->lastTimeWithNodesInBand.tv_sec != 0 ||
      band_stats->lastTimeWithNodesInBand.tv_nsec != 0)
  {
    band_stats->totalTimeWithNodesInBand[band_stats->currentHigherBand] +=
        VbUtilElapsetimeTimespecMs(band_stats->lastTimeWithNodesInBand, eventTime);
  }
  else
  {
    // First event, calculate time from metrics start
    band_stats->totalTimeWithNodesInBand[band_stats->currentHigherBand] +=
        VbUtilElapsetimeTimespecMs(vbMetricsStartTime, eventTime);
  }

  band_stats->currentNrOfNodesInBand[currBand] = (band_stats->currentNrOfNodesInBand[currBand] == 0) ? 0 : band_stats->currentNrOfNodesInBand[currBand]-1;
  band_stats->currentNrOfNodesInBand[newBand]++;

  // Find higher band
  for(i = (vbEngineMetricsNumBands - 1); i >= 0; i--)
  {
    if(band_stats->currentNrOfNodesInBand[i] > 0)
    {
      band_stats->lastTimeWithNodesInBand = eventTime;
      band_stats->currentHigherBand = i;
      break;
    }
  }

  // Did we enter or exit from fullband ?
  if(currBand == (vbEngineMetricsNumBands-1) || newBand == (vbEngineMetricsNumBands-1))
  {
    if(band_stats->lastTimeWithNodesInFullBand.tv_sec != 0 || //[band_stats->currentNrNodesinFullBand]
        band_stats->lastTimeWithNodesInFullBand.tv_nsec != 0)
    {
      band_stats->totalTimeWithNodesInFullBand[band_stats->currentNrNodesinFullBand] +=
          VbUtilElapsetimeTimespecMs(band_stats->lastTimeWithNodesInFullBand, eventTime);
    }
    else
    {
      // First event, calculate time from metrics start
      band_stats->totalTimeWithNodesInFullBand[band_stats->currentNrNodesinFullBand] +=
          VbUtilElapsetimeTimespecMs(vbMetricsStartTime, eventTime);
    }

    band_stats->lastTimeWithNodesInFullBand = eventTime;

    if(currBand == (vbEngineMetricsNumBands-1) && newBand < (vbEngineMetricsNumBands-1))
    {
      band_stats->currentNrNodesinFullBand = (band_stats->currentNrNodesinFullBand == 0)? 0 : (band_stats->currentNrNodesinFullBand-1);
    }
    else if(currBand < (vbEngineMetricsNumBands-1) && newBand == (vbEngineMetricsNumBands-1))
    {
      band_stats->currentNrNodesinFullBand++;
    }
  }
}

/**************************************************************************/

static BOOL VbMetricsBandChangeStatsProcess(t_VB_MetricsEvent *event)
{
  t_DeviceMetricsData *data;
  t_VBMetricsBoostingStatsLine *boost_st;
  INT32U new_band;
  struct timespec timeFromStart;

  data = (t_DeviceMetricsData *)event->eventData;

  // Check boundaries
  new_band = MIN(MAX(0, (data->data2 - 1)), (VB_PSD_NUM_BANDS - 1));

  // Update line info
  boost_st = GetBoostStructByMac(data->mac);

  if(boost_st != NULL)
  {
    if(boost_st->currentBand == new_band)
    {
      // We only register changes
      return FALSE;
    }

    // Accumulate the time in the current band
    if(boost_st->lastBandBeginTime[boost_st->currentBand].tv_sec != 0 ||
        boost_st->lastBandBeginTime[boost_st->currentBand].tv_nsec != 0)
    {
      boost_st->totalTimeBoosted[boost_st->currentBand] +=
          VbUtilElapsetimeTimespecMs(boost_st->lastBandBeginTime[boost_st->currentBand], event->eventTime);
    }
    else
    {
      timeFromStart = vbMetricsStartTime;
      boost_st->totalTimeBoosted[boost_st->currentBand] =
          VbUtilElapsetimeTimespecMs(timeFromStart, event->eventTime);
    }

    VbMetricsUpdateGlobalBandStats(boost_st->DMNode, boost_st->currentBand, new_band, event->eventTime);

    // Update current band -------------------------
    boost_st->currentBand = new_band;

    boost_st->lastBandBeginTime[boost_st->currentBand] = event->eventTime;
  }
  else
  {
    return FALSE;
  }

  return TRUE;
}

/**************************************************************************/

static BOOL VbMetricsNewNodeProcess(t_VB_MetricsEvent *event)
{
  t_DeviceMetricsData *data;
  t_VBMetricsBoostingStatsLine *boost_st;

  data = (t_DeviceMetricsData *)event->eventData;

  // Add new node struct
  boost_st = GetBoostStructByMac(data->mac);

  if(boost_st != NULL)
  {
    // All new nodes start in the low band
    boost_st->currentBand = 0;
    boost_st->DMNode = data->DMNodeType;
    strncpy(boost_st->driverId, data->driverId, VB_EA_DRIVER_ID_MAX_SIZE);
    // It is a completely new node, no present before
    if(boost_st->lastBandBeginTime[boost_st->currentBand].tv_sec == 0 && boost_st->lastBandBeginTime[boost_st->currentBand].tv_nsec == 0)
    {
      boost_st->lastBandBeginTime[0] = vbMetricsStartTime;
    }
  }
  else
  {
    return FALSE;
  }

  return TRUE;
}


/**************************************************************************/

BOOL VbMetricsLostLineProcess(t_VB_MetricsEvent *event)
{
  t_VBMetricsBoostingStatsLine *boost_st;
  INT64S diff;
  INT8U *mac;
  struct timespec timeFromStart;

  mac = (INT8U *)event->eventData;
  boost_st = GetBoostStructByMac(mac);

  if(boost_st != NULL)
  {
    // Update global info
    VbMetricsUpdateGlobalBandStats(boost_st->DMNode, boost_st->currentBand, 0, event->eventTime);

    // Update line info
    if(boost_st->lastBandBeginTime[boost_st->currentBand].tv_sec != 0 && boost_st->lastBandBeginTime[boost_st->currentBand].tv_nsec != 0)
    {
      diff = VbUtilElapsetimeTimespecMs(boost_st->lastBandBeginTime[boost_st->currentBand], event->eventTime);
      boost_st->totalTimeBoosted[boost_st->currentBand] += diff;
    }
    else
    {
      timeFromStart = vbMetricsStartTime;
      boost_st->totalTimeBoosted[boost_st->currentBand] =
          VbUtilElapsetimeTimespecMs(timeFromStart, event->eventTime);
    }

    boost_st->currentBand = 0;
    boost_st->lastBandBeginTime[boost_st->currentBand] = event->eventTime;
  }
  else
  {
    return FALSE;
  }

  return TRUE;
}

/**************************************************************************/

// Called when VB_METRICS_EVENT_TRAFFIC_REPORT event is received. It updates
// the histogram counters and detects if the traffic crossed a defined threshold.
// If it crossed a threshold, it changes the event type with the correspondent one:
// -New Line, Line Lost
// -Traffic went over 105, 205, 305, 405, 505, 605, 705 and 805 Mbps
// -Traffic went under 95, 195, 295, 395, 495, 595, 695 and 795 Mbps
BOOL VbMetricsTrafficProcess(t_VB_MetricsEvent *event)
{
  t_VBMetricsNodeTraffic  *traffic_st;
  t_DeviceMetricsData *tr_data;
  INT32U i;
  INT32U j;
  BOOL res;
  t_VBMetricsEventType eventsOver[] = {
      VB_METRICS_EVENT_TRAFFIC_OVER_805,    // 20
      VB_METRICS_EVENT_TRAFFIC_OVER_705,    // 19
      VB_METRICS_EVENT_TRAFFIC_OVER_605,    // 18
      VB_METRICS_EVENT_TRAFFIC_OVER_505,    // 17
      VB_METRICS_EVENT_TRAFFIC_OVER_405,    // 16
      VB_METRICS_EVENT_TRAFFIC_OVER_305,    // 15
      VB_METRICS_EVENT_TRAFFIC_OVER_205,    // 14
      VB_METRICS_EVENT_TRAFFIC_OVER_105,    // 13
  };

  t_VBMetricsEventType eventsUnder[] = {
      VB_METRICS_EVENT_TRAFFIC_UNDER_95,    // 21
      VB_METRICS_EVENT_TRAFFIC_UNDER_195,   // 22
      VB_METRICS_EVENT_TRAFFIC_UNDER_295,   // 23
      VB_METRICS_EVENT_TRAFFIC_UNDER_395,   // 24
      VB_METRICS_EVENT_TRAFFIC_UNDER_495,   // 25
      VB_METRICS_EVENT_TRAFFIC_UNDER_595,   // 26
      VB_METRICS_EVENT_TRAFFIC_UNDER_695,   // 27
      VB_METRICS_EVENT_TRAFFIC_UNDER_795,   // 28
  };

  tr_data = (t_DeviceMetricsData *)event->eventData;
  traffic_st = GetTrafficStructByMac(tr_data->mac, &event->eventTime);

  res = FALSE;

  if (traffic_st != NULL)
  {
    // Traffic increases
    if((traffic_st->previousTrafficValue < tr_data->data1) &&
       (tr_data->data1 > (VB_ENGINE_METRICS_TRAFFIC_MIN + VB_ENGINE_METRICS_TRAFFIC_HYST)))
    {
      for (i = VB_ENGINE_METRICS_TRAFFIC_MAX, j = 0; i >= VB_ENGINE_METRICS_TRAFFIC_MIN; i -= VB_ENGINE_METRICS_TRAFFIC_STEP, j++)
      {
        // did we cross this limit ?
        if ((tr_data->data1 >= (i + VB_ENGINE_METRICS_TRAFFIC_HYST)) &&
            (traffic_st->previousTrafficValue <= (i - VB_ENGINE_METRICS_TRAFFIC_HYST)))
        {
          event->eventType = eventsOver[j];
          res = TRUE;
          break;
        }
      }
    }
    else if(traffic_st->previousTrafficValue > tr_data->data1)
    {
      for (i = VB_ENGINE_METRICS_TRAFFIC_MIN, j = 0; i <= VB_ENGINE_METRICS_TRAFFIC_MAX; i += VB_ENGINE_METRICS_TRAFFIC_STEP, j++)
      {
        // did we cross this limit ?
        if ((tr_data->data1 <= (i - VB_ENGINE_METRICS_TRAFFIC_HYST)) &&
            (traffic_st->previousTrafficValue >= (i + VB_ENGINE_METRICS_TRAFFIC_HYST)))
        {
          event->eventType = eventsUnder[j];
          res = TRUE;
          break;
        }
      }
    }
    else
    {
      // No cross event, don't insert the event
      res = FALSE;
    }

    if (res == TRUE)
    {
      traffic_st->previousTrafficValue = tr_data->data1;
    }

    traffic_st->reportCounter++;

    // Update histogram counters
    i = tr_data->data1/50;
    // Check overflow
    if (i >= VB_ENGINE_METRICS_HISTOGRAM_STEPS)
    {
      i = VB_ENGINE_METRICS_HISTOGRAM_STEPS - 1 ;
    }
    traffic_st->trafficHistogram[i]++;
  }

  return res;
}

/**************************************************************************/

void vbMetricsDumpAndCloseLists(void)
{
  // Write files
  if (vbMetricsSaveMetricsToDisk)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Saving Metrics Reports to Files...");
    VbEngineDumpMetricsToFiles();
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Saved Metrics Reports to Files!");
  }

  ClearList((t_linkedElement *)vbMetricsCPUMeasurements);
  ClearList((t_linkedElement *)vbMetricsBoostingStatsLine);
  ClearList((t_linkedElement *)vbMetricsHistogramsList);
}

/**************************************************************************/

/**
 * @}
 **/
