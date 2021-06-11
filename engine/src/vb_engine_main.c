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
 * @file vb_engine_main.c
 * @brief Execute vector boost engine functionality
 *
 * @internal
 *
 * @author
 * @date 22/12/2014
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <string.h>
#include <errno.h>
#if (_MEMORY_DEBUG_ == 1)
#include <mcheck.h>
#endif

#include "vb_engine_datamodel.h"
#include "vb_log.h"
#include "vb_engine_process.h"
#include "vb_thread.h"
#include "vb_console.h"
#include "vb_engine_conf.h"
#include "vb_engine_console.h"
#include "vb_counters.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_clock.h"
#include "vb_engine_socket_alive.h"
#include "vb_engine_main_timer.h"
#include "vb_engine_main.h"
#include "vb_timer.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_measure.h"
#include "vb_engine_alignment.h"
#include "vb_util.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_ENGINE_MAIN_MAX_QUEUE_MESSAGES  (5)
#define VB_ENGINE_MAIN_MSG_SIZE            (sizeof(t_VBMainMsg))
#define VB_ENGINE_MAIN_QUEUE_NAME          "/VbEngineMainQ"
#define VB_ENGINE_LOG_QUEUE_NAME           "/VbEngineLogQ"

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static const char *VB_COUNTERS_NAME[] =
    {
        "EV_KILL_ALL",
        "EV_KILL",
        "EV_CONNECT",
        "EV_DISCONNECT",
        "EV_BOOST_UPDATE",
        "EV_BOOST_UPDATE_END_OK",
        "EV_BOOST_UPDATE_END_KO",
        "EV_ALIGN_CHECK_RESTART",
        "EV_ALIGN_PERIODIC_CHECK",
        "EV_ALIGN_CHECK",
        "EV_ALIGN_DONE",
        "EV_ALIGN_DONE_SKIP_MEAS",
        "EV_ALIGN_CHANGE",
        "EV_ALIGN_CHANGE_SYNC",
        "EV_ALIGN_WAIT",
        "EV_ALIGN_RESTART",
        "EV_ALIGN_CONF_SYNC",
        "EV_DISC_DOMAIN_SYNC",
        "EV_CLOCK_REQ",
        "EV_CLOCK_REQ_SYNC",
        "EV_CLOCK_FORCE_REQ",
        "EV_MEASPLAN_FAIL",
        "EV_MEASPLAN_RSP_SYNC",
        "EV_MEASURE_END_SYNC",
        "EV_MEAS_GENERAL_FAILURE",
        "EV_COMPUTATION_OK",
        "EV_COMPUTATION_KO",
        "EV_MEAS_FORCE",
        "EV_SNR_PROBES_MEAS_FORCE",
        "EV_PSD_UPDATE_FORCE",
        "EV_NETWORK_CHANGE",
        "EV_MEAS_COLLECT_END_NO_LINES",
        "EV_DISC_VERS_TO",
        "EV_DISC_STATE_TO",
        "EV_DOMAINS_SYNC_TO",
        "EV_CLOCK_RSP_TO",
        "EV_ALIGN_CHECK_TO",
        "EV_ALIGN_CHANGE_TO",
        "EV_ALIGN_SYNC_TO",
        "EV_MEASPLAN_RSP_TO",
        "EV_MEAS_COMPLETE_TO",
        "EV_MEAS_COLLECT_TO",
        "EV_MEASPLAN_CANCEL_TO",
        "EV_BOOST_UPDATE_END_TO",
        "EV_BOOST_ALG_RUN_TO",
        "EV_ALIGN_CONF_TO",
        "EV_RX_NETWORK_CHANGE_TRG",
        "EV_RX_VERSION_RSP",
        "EV_RX_STATE_RSP",
        "EV_RX_DOMAIN_RSP",
        "EV_RX_EMPTY_DOMAIN_RSP",
        "EV_RX_CYCQUERY_RSP",
        "EV_RX_CYCQUERY_RSP_KO",
        "EV_RX_CYCQUERY_RSP_INV",
        "EV_RX_CYCCHANGE_RSP",
        "EV_RX_CYCCHANGE_RSP_KO",
        "EV_MEASPLAN_RSP",
        "EV_MEASPLAN_CANCEL_RSP",
        "EV_MEASPLAN_ERR_TRG",
        "EV_MEASURE_BGN_RSP",
        "EV_RX_MEASURE_CFR_RSP",
        "EV_RX_MEAS_COLLECT_END_TRG",
        "EV_RX_TRAFFIC_AWARENESS_TRG",
        "EV_RX_CLOCK_RSP",
        "EV_RX_PSD_SHAPE_RSP",
        "EV_RX_MEASURE_SNRPROBES_RSP",
        "EV_RX_CDTA_RSP",
        "EV_RX_ALIGNMODE_RSP",
        "EV_RX_ALIGNMODE_RSP_KO",
        "EV_RX_CLUSTER_STOP_RSP",
        "EV_RX_CLUSTER_STOP_RSP_KO",
        "EV_RX_ALIGN_SYNC_LOST_TRG",
        "DISCONNECTED_STATUS",
        "BOOSTING_SEND_PSD_SHAPE",
        "LINE_LOST",
        "NEW_LINE",
        "COMMUNICATION_DRIVER_LOST",
        "MEASURE_PLAN_REQUESTED",
        "MEASURE_PLAN_FAILED",
        "MEASURE_PLAN_REJECTED",
        "MEASURE_PLAN_SUCCESS",
        "MEASURE_PLAN_CANCELLED",
        "CDTA_CFG",
        "CDTA_FORCE_NO_CHANGE",
        "ERR_CREATING_NEW_DRIVER",
    };

static CHAR  engineMsgBuffer[VB_ENGINE_MAIN_MSG_SIZE];
static t_vbQueueName vbEngineMainQueueName;
static t_vbQueueName vbEngineLogQueueName;
static mqd_t vbEngineQueue = (mqd_t) 0;

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

/*******************************************************************/

static void MainQueueFlush(void)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  struct mq_attr      attr;
  ssize_t              bytes_read;
  BOOLEAN              queue_is_empty = FALSE;

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Flushing vbQueue...");

  if (mq_getattr(vbEngineQueue, &attr) == -1)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Error mq_getattr main");
    err = VB_ENGINE_ERROR_QUEUE;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Config nonBlock queue
    attr.mq_flags = O_NONBLOCK;

    if (mq_setattr(vbEngineQueue, &attr, NULL) == -1)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error mq_setattr main");
      err = VB_ENGINE_ERROR_QUEUE;
    }
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    while (queue_is_empty == FALSE)
    {
      bytes_read = mq_receive(vbEngineQueue, engineMsgBuffer, VB_ENGINE_MAIN_MSG_SIZE, NULL);

      if (bytes_read <= 0)
      {
        queue_is_empty = TRUE;
      }
      else if (bytes_read != VB_ENGINE_MAIN_MSG_SIZE)
      {
        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Cleanning queue numbytes received error");
      }
    }
  }

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Flushed vbQueue...");
}

/*******************************************************************/

static t_VB_engineErrorCode MainQueueInit(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  struct mq_attr      attr;

  // Build the queue name
  VbUtilQueueNameBuild(VB_ENGINE_MAIN_QUEUE_NAME, VbEngineConfEngineIdGet(), vbEngineMainQueueName);

  // Destroy previous posix queue
  mq_unlink(vbEngineMainQueueName);

  attr.mq_maxmsg = VB_ENGINE_MAIN_MAX_QUEUE_MESSAGES;
  attr.mq_msgsize = VB_ENGINE_MAIN_MSG_SIZE;

  vbEngineQueue = mq_open(vbEngineMainQueueName, O_CREAT | O_RDWR , 0666, &attr);

  if (vbEngineQueue == (mqd_t) -1)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error to create mqueue. errno %s", strerror(errno));
    ret = VB_ENGINE_ERROR_QUEUE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode MainQueueRelease(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  // Flush queue
  MainQueueFlush();

  // Close and release queue
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Closing main queue...");
  mq_close(vbEngineQueue);
  mq_unlink(vbEngineMainQueueName);
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Closed main queue...");

  return ret;
}

/*******************************************************************/

static void VbEngineSignalHandler(INT32U signum)
{
  if (signum == SIGINT)
  {
    VbEngineKill();
  }
}

/*******************************************************************/

static t_vb_counter_error VbEngineMainCountersInit( void )
{
  t_vb_counter_error err;
  INT8U i;

  err = VbCountersInit(NULL);

  if(err == VB_COUNTERS_ERROR_NONE)
  {
    for(i = 0 ; i < VB_ENGINE_COUNTERS_NUM ; i++)
    {
      err = VbCounterInstall(i, VB_COUNTERS_NAME[i]);
      if(err != VB_COUNTERS_ERROR_NONE)
      {
        break;
      }
    }
  }

  return err;
}

/*******************************************************************/

static t_VB_engineErrorCode ComponentsInit(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32S               err;

  // Install counter for debug
  VbEngineMainCountersInit();

  // Init threads
  VbThreadInit();

  // Init timers
  VbTimerInit();

  // Init Log
  // Build the log queue name
  VbUtilQueueNameBuild(VB_ENGINE_LOG_QUEUE_NAME, VbEngineConfEngineIdGet(), vbEngineLogQueueName);

  err = VbLogInit(vbEngineLogQueueName,
                  VbEngineConfVerboseLevelGet(),
                  VbEngineConfOutputPathGet(),
                  VbEngineConfPeristentLogNumLinesGet(),
                  VbEngineConfPeristentLogVerboseLevelGet(),
                  VbEngineConfPersistentLogIsCircular());

  if (err != 0)
  {
    ret = VB_ENGINE_ERROR_NOT_STARTED;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init clock monitor
    VbEngineClockMonitorInit();

    // Init Console
    ret = VbEngineConsoleInit(VbEngineConfConsolePortGet());
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init process thread
    VbEngineProcessThreadInit();

    // Init Metrics
    ret = VbEngineMetricsInit();
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ComponentsStart(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  BOOLEAN              running;

  running = VbLogRun();

  if (running == FALSE)
  {
    ret = VB_ENGINE_ERROR_NOT_STARTED;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbEngineDatamodelStart();
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Create the timer thread in charge of saving Engine Data stats to a file every few seconds
    running = VbEngineMainTimerRun();

    if (running == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineMetricsRun();
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    running = VbEngineClockMonitorRun();

    if (running == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if(VbEngineConfSocketAliveEnableGet() == TRUE)
    {
      running = VbEngineSocketAliveMonitorRun();

      if (running == FALSE)
      {
        ret = VB_ENGINE_ERROR_NOT_STARTED;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Start signal processing thread
    running = VbThreadHandleSignalsStart(VbEngineSignalHandler);

    if (running == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if ((ret == VB_ENGINE_ERROR_NONE) &&
      (VbEngineConfConsolePortGet() != 0))
  {
    // Start console thread
    VbConsoleStart();
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Start Processing thread
    ret = VbEngineProcessThreadExecute();
  }

  return ret;
}

/*******************************************************************/

static void ComponentsStop(void)
{
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Closing Timer thread...");
  VbEngineMainTimerStop();
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Process Timer closed!");

  VbEngineClockMonitorStop();

  VbEngineSocketAliveMonitorStop();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Closing Process thread...");
  VbEngineProcessProtocolThreadStop();
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Process thread closed!");

  // Stop console thread
  VbConsoleStop();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Closing metrics thread...");
  VbMetricsExit();
  vbMetricsDumpAndCloseLists();
  VbMetricsDestroy();
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Metrics thread closed!");

  // Stop timers associated to clusters
  VbEngineClusterStopTimers();
  // Free clusters and associated memory
  VbEngineCltListClustersDestroy();

  // Stop Log thread
  VbLogStop();
  // From this point we should use "printf" instead of VbLogPrint

  // Stop signal handler thread
  VbThreadHandleSignalsStop();

  // Release VBEngineAlign resources
  VbEngineAlignReleaseResources();

  // Release VBEngineConf resources
  VbEngineConfReleaseResources();
  
  // Release datamodel resources
  VbEngineDatamodelStop();
}

/*******************************************************************/

static void EngineMainLoop(void)
{
  t_VB_engineErrorCode ret;
  BOOLEAN              run;
  ssize_t              bytes_read;

  run = TRUE;

  while (run)
  {
    ret = VB_ENGINE_ERROR_NONE;
    bytes_read = mq_receive(vbEngineQueue, engineMsgBuffer, VB_ENGINE_MAIN_MSG_SIZE, NULL);

    if (bytes_read < 0)
    {
      // Fatal error, finish execution
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Main mq_receive error. errno %s", strerror(errno));

      run = FALSE;

      ret = VB_ENGINE_ERROR_QUEUE;
    }

    if ((ret == VB_ENGINE_ERROR_NONE) && (bytes_read != VB_ENGINE_MAIN_MSG_SIZE))
    {
      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "MAIN ---- Message size error, numbytes received %d ", bytes_read);
      ret = VB_ENGINE_ERROR_QUEUE;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      t_VBMainMsg *vbMsg = (t_VBMainMsg *)engineMsgBuffer;

      switch (vbMsg->vbEvent)
      {
        case VB_MAIN_EVENT_MAIN_END:
        {
          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "MAIN ---- Main end");
          run = FALSE;
          break;
        }
        default:
        {
          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Unexpected event %u", vbMsg->vbEvent);
          break;
        }
      }
    }
  } // while (run)
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

int main(int argc,  char **argv)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

#if (_MEMORY_DEBUG_ == 1)
  mtrace();
#endif

  printf("VectorBoost Compute Engine version: %s\n", VB_ENGINE_VERSION);

  // Take care of CTRL+C
  if (VbBlockSignals() == FALSE)
  {
    err = VB_ENGINE_ERROR_NOT_STARTED;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Parse configuration and create drivers structures
    err = VbEngineConfParse(argc, argv);
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Destroy previous posix queue and create a new one
    err = MainQueueInit();
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Init components
    err = ComponentsInit();
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Start components
    err = ComponentsStart();
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Write engine version to file
    VbLogSaveStringToTextFile(VB_ENGINE_VERSION_FILE, "w+", "VBEngineVersion=%s\n", VB_ENGINE_VERSION);
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Entering main loop
    EngineMainLoop();
  }

  printf("Exiting...\n");

  // Release queue
  MainQueueRelease();

  // Signal all threads to finish
  ComponentsStop();

#if (_MEMORY_DEBUG_ == 1)
  muntrace();
#endif

  return 0;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineKill(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  mqd_t                vb_queue;

  // Connect to main queue
  vb_queue = mq_open(vbEngineMainQueueName, O_WRONLY);

  VbEngineMainQEvSend(VB_MAIN_EVENT_MAIN_END, vb_queue, NULL);

  mq_close(vb_queue);

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
