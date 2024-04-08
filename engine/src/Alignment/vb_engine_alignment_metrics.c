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
 * @addtogroup vector_boost_engine
 * @{
 **/

/**
 * @file vb_engine_alignment_metrics.c
 * @brief VB Drivers alignment metrics
 *
 * @internal
 *
 * @author V.Grau
 * @date 2019-11-13
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "types.h"

#include "vb_log.h"
#include "vb_metrics.h"
#include "vb_util.h"
#include "vb_ea_communication.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_conf.h"

#include "vb_engine_alignment.h"
#include "vb_engine_alignment_metrics.h"

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

#define ENGINE_ALIGN_LOG_FILE                "alignment_log.txt"

#define ENGINE_ALIGN_TIME_FMT                "[%02d:%02d:%02d %03dms]"
#define ENGINE_ALIGN_DRIVER_ID_FMT           "[%-20s]"
#define ENGINE_ALIGN_CLUSTER_ID_FMT          "[%010u]"
#define ENGINE_ALIGN_ENGCONF_ID_FMT          "[%03u]"
#define ENGINE_ALIGN_STEP_FMT                "[%-10s]"

#define ENGINE_ALIGN_TIME_LEN                (17) // [xx:xx:xx xxxms]
#define ENGINE_ALIGN_MAX_DRIVER_HDR_LEN      (VB_EA_DRIVER_ID_MAX_SIZE + 2) // Take into account [] (+2)
#define ENGINE_ALIGN_CLUSTER_ID_LEN          (13)
#define ENGINE_ALIGN_ENGCONF_ID_LEN          (6)
#define ENGINE_ALIGN_STEP_LEN                (13)
#define ENGINE_ALIGN_LINE_SIZE               (200)
#define ENGINE_ALIGN_TOTAL_LINE_SIZE         (ENGINE_ALIGN_LINE_SIZE +                 \
                                              ENGINE_ALIGN_MAX_DRIVER_HDR_LEN +        \
                                              ENGINE_ALIGN_TIME_LEN +                  \
                                              ENGINE_ALIGN_CLUSTER_ID_LEN +            \
                                              ENGINE_ALIGN_ENGCONF_ID_LEN +            \
                                              ENGINE_ALIGN_STEP_LEN)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  struct timespec startTime;
  INT64S          lastElapsedTime;
  INT64S          maxElapsedTime;
  INT64S          minElapsedTime;
  INT64S          avgElapsedTime;
  INT32U          numCompleted;
  INT32U          numRestarts;
  BOOLEAN         started;
} t_alignTimings;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_alignTimings alignTimings;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

static BOOL AlignTimingsEvProcess(t_VB_MetricsEvent *event)
{
  BOOL insert_in_buffer = FALSE;

  if (event != NULL)
  {
    switch(event->eventType)
    {
      case (VB_METRICS_EVENT_ALIGN_START):
      {
        insert_in_buffer = TRUE;
        alignTimings.started = TRUE;
        alignTimings.startTime = event->eventTime;
        break;
      }
      case (VB_METRICS_EVENT_ALIGN_RESTART):
      {
        insert_in_buffer = TRUE;
        alignTimings.numRestarts++;
        break;
      }
      case (VB_METRICS_EVENT_ALIGN_DONE):
      {
        insert_in_buffer = TRUE;

        if (alignTimings.started == TRUE)
        {
          alignTimings.started = FALSE;
          alignTimings.lastElapsedTime = VbUtilElapsetimeTimespecMs(alignTimings.startTime, event->eventTime);
          alignTimings.avgElapsedTime = ((alignTimings.avgElapsedTime * alignTimings.numCompleted) + alignTimings.lastElapsedTime) / (alignTimings.numCompleted + 1);
          alignTimings.maxElapsedTime = MAX(alignTimings.lastElapsedTime, alignTimings.maxElapsedTime);
          alignTimings.minElapsedTime = MIN(alignTimings.lastElapsedTime, alignTimings.minElapsedTime);
          alignTimings.numCompleted++;
        }
        break;
      }
      default:
      {
        break;
      }
    }
  }

  return insert_in_buffer;
}

/*******************************************************************/

static void VbAlignMetricsReport(CHAR **buffer, INT32U *buffSize)
{
  if ((buffer != NULL) && (*buffer != NULL) && (buffSize != NULL))
  {
    time_t               t;
    struct               tm *tmu;
    FP32                 restarts_per_process;
    INT32U               event_idx = 0;
    INT32U               event_list_size = VbMetricsEventsListSizeGet();
    t_VbMetricsErrorType metrics_err;

    t = time(NULL);
    tmu = localtime(&t);

    VbUtilStringToBuffer(buffer, buffSize,
        "\nAlignment report\n"
        "===========================\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "Report time                                  : [%02d:%02d:%02d]\n", tmu->tm_hour, tmu->tm_min, tmu->tm_sec);

    VbUtilStringToBuffer(buffer, buffSize,
        "Number of completed alignments               : %lu\n",
        alignTimings.numCompleted);

    if (alignTimings.numCompleted > 0)
    {
      restarts_per_process = (alignTimings.numRestarts / alignTimings.numCompleted);
    }
    else
    {
      restarts_per_process = 0;
    }
    VbUtilStringToBuffer(buffer, buffSize,
        "Avg number of restarts per alignment process : %.2f\n",
        restarts_per_process);

    VbUtilStringToBuffer(buffer, buffSize,
        "Alignment elapsed times (last/max/min/avg)   : %lld/%lld/%lld/%lld ms\n",
        alignTimings.lastElapsedTime,
        alignTimings.maxElapsedTime,
        alignTimings.minElapsedTime,
        alignTimings.avgElapsedTime
        );

    VbUtilStringToBuffer(buffer, buffSize,
        "\nHistory:\n");

    VbUtilStringToBuffer(buffer, buffSize,
        "[%-14s][%-20s][%-10s][%s][%-10s]Message...\n",
        "Timestamp", "Driver Id", "Cluster Id", "AId", "Step");

    do
    {
      t_VB_MetricsEvent    *event;

      // Get next event
      metrics_err = VbMetricsGetEventByIndex(&event, event_idx);

      if (metrics_err == VB_METRICS_NO_ERROR)
      {
        // Update index
        event_idx++;

        if (event->eventType == VB_METRICS_EVENT_ALIGN_START)
        {
          VbUtilStringToBuffer(buffer, buffSize, "\n\n");
        }
        else if ((event->eventType == VB_METRICS_EVENT_ALIGN_CHECK_START) ||
                 (event->eventType == VB_METRICS_EVENT_ALIGN_RESTART))
        {
          VbUtilStringToBuffer(buffer, buffSize, "\n");
        }

        if (((event->eventType == VB_METRICS_EVENT_ALIGN_START) ||
             (event->eventType == VB_METRICS_EVENT_ALIGN_RESTART) ||
             (event->eventType == VB_METRICS_EVENT_ALIGN_CHECK_START) ||
             (event->eventType == VB_METRICS_EVENT_ALIGN_CHECK_END) ||
             (event->eventType == VB_METRICS_EVENT_ALIGN_INFO) ||
             (event->eventType == VB_METRICS_EVENT_ALIGN_DONE)) &&
            (event->eventData != NULL))
        {
          VbUtilStringToBuffer(buffer, buffSize, "%s\n", event->eventData);
        }

        if (event->eventType == VB_METRICS_EVENT_ALIGN_CHECK_END)
        {
          VbUtilStringToBuffer(buffer, buffSize, "\n");
        }
      }
    } while ((metrics_err == VB_METRICS_NO_ERROR) && (event_idx < event_list_size));
  }
}

/*******************************************************************/

static t_VB_engineErrorCode NodeInfoReportCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "Conf",
      "Domain %s - DID %3u - Role %10s - Hops %3u - HasBeenCandidate %1u - Seed %4u",
      domain->dm.MACStr,
      domain->dm.devID,
      VbEngineAlignRoleStringGet(domain->dm.nodeAlignInfo.role),
      domain->dm.nodeAlignInfo.hops,
      domain->dm.nodeAlignInfo.hasBeenCandidate,
      domain->dm.addInfo1.extSeed);

  return ret;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

t_VB_engineErrorCode VbEngineAlignMetricsStart(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32S               metrics_err;

  if (VbEngineConfAlignMetricsEnabled() == TRUE)
  {
    // Reset metrics
    VbEngineAlignMetricsReset();

    // Install callbacks
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_ALIGN_START, AlignTimingsEvProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_ALIGN_RESTART, AlignTimingsEvProcess);
    VbMetricsAddProcessHandler(VB_METRICS_EVENT_ALIGN_DONE, AlignTimingsEvProcess);

    metrics_err = VbMetricsAddReportHandler("Alignment report", VbAlignMetricsReport, ENGINE_ALIGN_LOG_FILE);

    if (metrics_err < 0)
    {
      ret = VB_ENGINE_ERROR_METRICS;
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error adding metrics report");
    }
  }

  return ret;
}

/*******************************************************************/

void VbEngineAlignMetricsReset(void)
{
  memset(&alignTimings, 0, sizeof(alignTimings));
  alignTimings.minElapsedTime = MAX_INT64S;
}

/*******************************************************************/

void VbEngineAlignMetricsEvReport(t_VBMetricsEventType event, const char *driverId, INT32U clusterId, const char *step, const char *fmt, ...)
{
  if ((VbEngineConfAlignMetricsEnabled() == TRUE) &&
      (driverId != NULL) &&
      ((event == VB_METRICS_EVENT_ALIGN_START) ||
       (event == VB_METRICS_EVENT_ALIGN_RESTART) ||
       (event == VB_METRICS_EVENT_ALIGN_CHECK_START) ||
       (event == VB_METRICS_EVENT_ALIGN_CHECK_END) ||
       (event == VB_METRICS_EVENT_ALIGN_INFO) ||
       (event == VB_METRICS_EVENT_ALIGN_DONE)))
  {
    time_t            t;
    struct            tm *tmu;
    struct            timeval tv;
    CHAR             *log_line = NULL;

    // alloc buffer
    log_line = (CHAR *)calloc(1, ENGINE_ALIGN_TOTAL_LINE_SIZE + 1);

    t = time(NULL);
    tmu = localtime(&t);
    gettimeofday(&tv, NULL);

    if ((log_line != NULL) && (tmu != NULL))
    {
      INT8U             align_id = 0;
      CHAR             *dst      = NULL;
      va_list           args;
      INT32U            msec;

      dst = log_line;

      // Limit msec to 999 to avoid a compilation warning of GCC-7
      msec = tv.tv_usec / 1000;

      if (msec >= 1000)
      {
        msec = 999;
      }

      VbEngineAlignmentIdGet(clusterId, NULL, &align_id);

      // Dump timestamp
      snprintf(dst, ENGINE_ALIGN_TIME_LEN, ENGINE_ALIGN_TIME_FMT,
          tmu->tm_hour, tmu->tm_min, tmu->tm_sec, msec);
      dst += ENGINE_ALIGN_TIME_LEN - 1; // -1 to locate ptr just in '\0' character

      // Dump DriverId
      snprintf(dst, ENGINE_ALIGN_MAX_DRIVER_HDR_LEN, ENGINE_ALIGN_DRIVER_ID_FMT, driverId);
      dst += ENGINE_ALIGN_MAX_DRIVER_HDR_LEN - 1;  // -1 to locate ptr just in '\0' character

      // Dump ClusterId
      snprintf(dst, ENGINE_ALIGN_CLUSTER_ID_LEN, ENGINE_ALIGN_CLUSTER_ID_FMT, clusterId);
      dst += ENGINE_ALIGN_CLUSTER_ID_LEN - 1;  // -1 to locate ptr just in '\0' character

      // Dump AlignId
      snprintf(dst, ENGINE_ALIGN_ENGCONF_ID_LEN, ENGINE_ALIGN_ENGCONF_ID_FMT, align_id);
      dst += ENGINE_ALIGN_ENGCONF_ID_LEN - 1;  // -1 to locate ptr just in '\0' character

      // Dump AlignId
      snprintf(dst, ENGINE_ALIGN_STEP_LEN, ENGINE_ALIGN_STEP_FMT, step);
      dst += ENGINE_ALIGN_STEP_LEN - 1;  // -1 to locate ptr just in '\0' character

      va_start(args, fmt);
      vsnprintf(dst, ENGINE_ALIGN_LINE_SIZE, fmt, args);
      va_end(args);

      VbMetricsReportEvent(event, log_line);
    }
  }
}

/*******************************************************************/

void VbEngineAlignMetricsNodeInfoReport(INT32U clusterId)
{
  t_VB_engineErrorCode       err;

  if (VbEngineConfAlignMetricsEnabled() == TRUE)
  {
    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
        "Domains configuration for cluster Id %u:", clusterId);

    err = VbEngineDatamodelClusterXAllDomainsLoop(NodeInfoReportCb, clusterId, NULL);

    if (err != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Error %d reporting alignment node info", err);
    }
  }
}

/*******************************************************************/

/**
 * @}
 **/
