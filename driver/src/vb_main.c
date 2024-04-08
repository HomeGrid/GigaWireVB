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
 * @file vb_main.c
 * @brief Execute vector boost functionality
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
#include <pthread.h>
#include <arpa/inet.h>
#include <string.h>

#if (_MEMORY_DEBUG_ == 1)
#include <mcheck.h>
#endif

#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_alignment.h"
#include "vb_LCMP_com.h"
#include "vb_LCMP_socket.h"
#include "vb_log.h"
#include "vb_EA_interface.h"
#include "vb_measurement.h"
#include "vb_traffic.h"
#include "vb_psdShaping.h"
#include "vb_domainsMonitor.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_counters.h"
#include "vb_main.h"
#include "vb_main_console.h"
#include "vb_main_timer.h"
#include "vb_driver_conf.h"
#include "vb_timer.h"
#include "vb_cdta.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define DRIVER_NUM_QUEUE_ELEMS                       (50)
#define DRIVER_MSG_SIZE                              (sizeof(t_driverMsg))
#define DRIVER_LOG_QUEUE_NAME                        ("/VbDriverLogQ")
#define DRIVER_WAIT_TO_IDLE                          (1000) // in ms
#define DRIVER_WAIT_TO_RETRY_CONN                    (1000) // in ms
#define DRIVER_MAX_TCP_REDIRECT_PORT                 (0x10000)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  t_driverEvent event;
  t_vbEAMsg    *eaMsg;
  void         *args;
} t_driverMsg;

typedef struct
{
  INT32U              nextState;
  t_VB_comErrorCode (*transitionFunc) (t_driverMsg *msg, t_driverState currStt, void *data);
} t_driverFSMStep;

/*
 ************************************************************************
 ** Private function prototypes
 ************************************************************************
 */

/**
 * @brief Define the function to be called when ctrl-c (SIGINT) signal is sent to process
 **/
static void VbDriverSignalHandler(INT32U signum);

/**
 * @brief Implement the close of system, free resources
 **/
static void ComponentsStop(void);

/**
 * @brief This function executes the main loop to receive messages from other threads
 **/
static void VbMainLoop(void);

/**
 * @brief Removes all pending messages from queue
 **/
static void MainQueueFlush(void);

/**
 * @brief Initializes driver counters
 * @return @ref t_vb_counter_error
 **/
static t_vb_counter_error VbMainDriverCountersInit(void);

/// FSM transition functions
static t_VB_comErrorCode FSMDisconnectTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMConnectTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMIdleTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMKillTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMAlignCheckTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMAlignCheckEndTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMAlignChangeTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMAlignChangeEndTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasuringTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMPsdShapingTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMCdtaTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMTrafficErrorTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMSyncLostMonitorErrorTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMDomainReqRxTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMGenericRxFrameTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasurePlanFailedTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasurePlanMeasRspOkTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasurePlanMeasRspKoTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMPsdShapeEndOkTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMPsdShapeEndKoTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMPsdShapeAlreadyRunningTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMCdtaEndOkTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMCdtaEndKoTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMCdtaAlreadyRunningTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasureCollectStartTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasureCollectNodeEndTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasureCollectAllNodesEndTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMMeasureCancelTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMSNRProbeRequestTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMRedirectTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMEngineConfTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMClusterStopRequestTransition(t_driverMsg *msg, t_driverState currStt, void *data);
static t_VB_comErrorCode FSMNetChangeReportTransition(t_driverMsg *msg, t_driverState currStt, void *data);

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static const char *driverSttString[DRIVER_STT_LAST] =
    {
        "IDLE",
        "DISCONNECTED",
        "PRECONNECTED",
        "CONNECTED",
        "ALIGNMENT_CHECK",
        "ALIGNMENT_CHANGE",
        "MEASURING",
        "MEAS_COLLECT",
        "PSDSHAPING",
    };

static const char *driverEvString[DRIVER_EV_LAST] =
    {
        "START",
        "IDLE",
        "KILL",
        "CONNECT",
        "DISCONNECT",
        "NETCHANGE",
        "TRAFFIC_KO",
        "ALIGNCHECK_END",
        "ALIGNCHANGE_END",
        "PLAN_FAILED",
        "PLAN_CNF_OK",
        "MEAS_COLLECT_NODE_END",
        "MEAS_COLLECT_KO",
        "MEAS_COLLECT_ALL_END",
        "PSD_SHAPE_END_OK",
        "PSD_SHAPE_END_KO",
        "SYNC_LOST_KO",
        "RX_MEASPLAN_REQ",
        "RX_VERSION_REQ",
        "RX_STATE_REQ",
        "RX_DOMAINS_REQ",
        "RX_CYCQUERY_REQ",
        "RX_CLOCK_REQ",
        "RX_CYCCHANGE_REQ",
        "RX_MEASCOLLECT_REQ",
        "RX_MEASPLAN_CANCEL_REQ",
        "RX_PSD_SHAPE_REQ",
        "RX_SNRPROBES_REQ",
        "RX_CDTA_REQ",
        "RX_CDTA_END_OK",
        "RX_CDTA_END_KO",
        "RX_REDIRECT_REQ",
        "RX_ENGINECONF_REQ",
        "RX_CLUSTER_STOP_REQ",
        "RX_SOCKET_ALIVE_REQ"
    };

static const char *VB_COUNTERS_NAME[VB_DRIVER_COUNTERS_NUM] =
    {
        "EV_START",
        "EV_IDLE",
        "EV_KILL",
        "EV_CONNECT",
        "EV_DISCONNECT",
        "EV_NETCHANGE",
        "EV_TRAFFIC_KO",
        "EV_ALIGNCHECK_END",
        "EV_ALIGNCHANGE_END",
        "EV_PLAN_FAILED",
        "EV_PLAN_CNF_OK",
        "EV_MEAS_COLLECT_NODE_END",
        "EV_MEAS_COLLECT_KO",
        "EV_MEAS_COLLECT_ALL_END",
        "EV_PSD_SHAPE_END_OK",
        "EV_PSD_SHAPE_END_KO",
        "EV_SYNC_LOST_KO",
        "EV_RX_MEASPLAN_REQ",
        "EV_RX_VERSION_REQ",
        "EV_RX_STATE_REQ",
        "EV_RX_DOMAINS_REQ",
        "EV_RX_CYCQUERY_REQ",
        "EV_RX_CLOCK_REQ",
        "EV_RX_CYCCHANGE_REQ",
        "EV_RX_MEASCOLLECT_REQ",
        "EV_RX_MEASPLAN_CANCEL_REQ",
        "EV_RX_PSD_SHAPE_REQ",
        "EV_RX_SNRPROBES_REQ",
        "EV_RX_CDTA_REQ",
        "EV_CDTA_END_OK",
        "EV_CDTA_END_KO",
        "EV_RX_ENGINECONF_REQ",
        "NETWORK_DISCOVERY_ERROR",
        "MEASURE_BGN_SUCCESS",
        "MEASURE_BGN_RETRIES",
        "MEASURE_BGN_ERROR",
        "MEASURE_CFR_SUCCESS",
        "MEASURE_CFR_RETRIES",
        "MEASURE_CFR_ERROR",
        "MEASURE_SNR_SUCCESS",
        "MEASURE_SNR_ERROR",
        "LCMP_REQ_TX_ERROR",
        "LCMP_REQ_NO_RESPONSE",
        "LCMP_REQ_NOT_ALL_DEV",
        "LCMP_REQ_UNKNOWN_ERROR",
        "LCMP_CNF_PARAMID_NOT_FOUND",
        "LCMP_CNF_ERROR",
        "LCMP_NOTIFY_REQ_ERROR",
        "LCMP_NOTIFY_TIMEOUT",
        "LCMP_NOTIFY_OTHER_ERROR",
        "KEEP_ALIVE_TX",
    };

// Lookup array to convert frame opcodes -> FSM Events
static const t_driverEvent eaLookupEvents[VB_EA_OPCODE_LAST] =
{
    DRIVER_EV_RX_MEASPLAN_REQ,         //VB_EA_OPCODE_MEASURE_PLAN_REQ           = 0x00,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_MEASURE_PLAN_RESP          = 0x01,
    DRIVER_EV_RX_DOMAINS_REQ,          //VB_EA_OPCODE_DOMAIN_REQ                 = 0x02,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_DOMAIN_RESP                = 0x03,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_BGN_RESP                   = 0x04,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_CFR_RESP                   = 0x05,
    DRIVER_EV_RX_SNRPROBES_REQ,        //VB_EA_OPCODE_SNRPROBES_REQ              = 0x06,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_SNRPROBES_RESP             = 0x07,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_PSD_REQ                    = 0x08,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_PSD_RESP                   = 0x09,
    DRIVER_EV_RX_PSD_SHAPE_REQ,        //VB_EA_OPCODE_PSD_SHAPE_CFG              = 0x0A,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_PSD_SHAPE_CFM              = 0x0B,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_TRAFFIC_AWARENESS_TRG      = 0x0C,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_NETWORK_CHANGE_TRG         = 0x0D,
    DRIVER_EV_RX_VERSION_REQ,          //VB_EA_OPCODE_VERSION_REQ                = 0x0E,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_VERSION_RESP               = 0x0F,
    DRIVER_EV_RX_STATE_REQ,            //VB_EA_OPCODE_VBDRIVER_STATE_REQ         = 0x10,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_VBDRIVER_STATE_RESP        = 0x11,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_MEASURE_END_TRG            = 0x12,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_VBDRIVER_STATE_TRG         = 0x13,
    DRIVER_EV_RX_CLOCK_REQ,            //VB_EA_OPCODE_CLOCK_REQ                  = 0x14,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_CLOCK_RSP                  = 0x15,
    DRIVER_EV_RX_CYCQUERY_REQ,         //VB_EA_OPCODE_CYCQUERY_REQ               = 0x16,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_CYCQUERY_RSP               = 0x17,
    DRIVER_EV_RX_CYCCHANGE_REQ,        //VB_EA_OPCODE_CYCCHANGE_REQ              = 0x18,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_CYCCHANGE_RSP              = 0x19,
    DRIVER_EV_RX_MEASCOLLECT_REQ,      //VB_EA_OPCODE_MEAS_COLLECT_REQ           = 0x1A,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_MEAS_COLLECT_RSP           = 0x1B,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_MEAS_COLLECT_END           = 0x1C,
    DRIVER_EV_RX_MEASPLAN_CANCEL_REQ,  //VB_EA_OPCODE_MEASURE_PLAN_CANCEL_REQ    = 0x1D,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_MEASURE_PLAN_CANCEL_RSP    = 0x1E,
    DRIVER_EV_RX_CDTA_REQ,             //VB_EA_OPCODE_CDTA_CFG                   = 0x1F,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_CDTA_CFM                   = 0x20,
    DRIVER_EV_RX_REDIRECT_REQ,         //VB_EA_OPCODE_REDIRECT_REQ               = 0x21,
    DRIVER_EV_RX_ENGINE_CONF_REQ,      //VB_EA_OPCODE_ENGINE_CONF_REQ            = 0x22,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_ENGINE_CONF_RSP            = 0x23,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_ALIGN_SYNC_LOST_TRG        = 0x24,
    DRIVER_EV_RX_CLUSTER_STOP_REQ,     //VB_EA_OPCODE_ALIGN_CLUSTER_STOP_REQ     = 0x25,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_ALIGN_CLUSTER_STOP_RSP     = 0x26,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_NETWORK_CHANGE_REPORT      = 0x27,
    DRIVER_EV_RX_SOCKET_ALIVE_REQ,     //VB_EA_OPCODE_SOCKET_ALIVE_REQUEST       = 0x28,
    DRIVER_EV_LAST,                    //VB_EA_OPCODE_SOCKET_ALIVE_RESP          = 0x29
};

static t_driverState driverState = DRIVER_STT_IDLE;
static CHAR          driverMsgBuffer[DRIVER_MSG_SIZE];
static BOOL          vbMainRunMAIN;
static mqd_t         vbQueue = (mqd_t) 0;
static BOOL          vbMainRedirected = FALSE;

static t_driverFSMStep driverFSMTransition[DRIVER_STT_LAST][DRIVER_EV_LAST];

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/************************************************************************/

static void MainThreadPrioConf(void)
{
  pthread_t           th_id;
  pthread_attr_t      th_attr;
  struct sched_param param;
  INT32U              policy = 0;

  th_id = pthread_self();
  pthread_attr_init(&th_attr);
  pthread_attr_getschedpolicy(&th_attr, (int *)&policy);
  param.sched_priority = sched_get_priority_max(policy);;
  pthread_setschedparam(th_id, policy, &param);
  pthread_attr_destroy(&th_attr);
}

/************************************************************************/

static const CHAR *FSMSttToStrGet(t_driverState state)
{
  const CHAR *state_str = "UNKNOWN";

  if (state < DRIVER_STT_LAST)
  {
    state_str = driverSttString[state];
  }

  return state_str;
}

/************************************************************************/

static const CHAR *FSMEvToStrGet(t_driverEvent event)
{
  const CHAR *event_str = "UNKNOWN";

  if (event < DRIVER_EV_LAST)
  {
    event_str = driverEvString[event];
  }

  return event_str;
}

/*******************************************************************/

static void FSMInit(void)
{
  INT32U stt_idx;
  INT32U event_idx;

  for (stt_idx = 0; stt_idx < DRIVER_STT_LAST; stt_idx++)
  {
    for (event_idx = 0; event_idx < DRIVER_EV_LAST; event_idx++)
    {
      driverFSMTransition[stt_idx][event_idx] = (t_driverFSMStep){DRIVER_STT_LAST, NULL};
    }
  }

  //                  CURRENT_STATE            EVENT                                                 NEXT_STATE               TRANSITION_FUNCTION
  /*** DRIVER_STT_IDLE ***/
  /*
   * Driver does not accept requests from engine in this state.
   *
   * Expected events:
   * - DRIVER_EV_START: try to establish a connection with engine. Move to next state.
   * - DRIVER_EV_KILL: release resources and close driver.
   */
  driverFSMTransition[DRIVER_STT_IDLE]          [DRIVER_EV_START]                     = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_IDLE]          [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};

  /*** DRIVER_STT_DISCONNECTED ***/
  /*
   * Trying to establish a new connection with engine.
   *
   * Expected events:
   * - DRIVER_EV_IDLE: stop all threads and move to DRIVER_STT_IDLE (debug purposes).
   * - DRIVER_EV_CONNECT: connection with engine established. Move to next state.
   */
  driverFSMTransition[DRIVER_STT_DISCONNECTED]  [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_DISCONNECTED]  [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_DISCONNECTED]  [DRIVER_EV_CONNECT]                   = (t_driverFSMStep){DRIVER_STT_PRECONNECTED, NULL};
  driverFSMTransition[DRIVER_STT_DISCONNECTED]  [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};

  /*** DRIVER_STT_PRECONNECTED ***/
  /*
   * Trying to establish a new connection with engine only (no LCMP for now).
   *
   * Expected events:
   * - DRIVER_EV_IDLE: stop all threads and move to DRIVER_STT_IDLE (debug purposes).
   * - DRIVER_EV_CONNECT: connection with engine established. Move to next state.
   */
  driverFSMTransition[DRIVER_STT_PRECONNECTED]  [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_PRECONNECTED]  [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_PRECONNECTED]  [DRIVER_EV_CONNECT]                   = (t_driverFSMStep){DRIVER_STT_PRECONNECTED, NULL};
  driverFSMTransition[DRIVER_STT_PRECONNECTED]  [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_PRECONNECTED]  [DRIVER_EV_RX_VERSION_REQ]            = (t_driverFSMStep){DRIVER_STT_PRECONNECTED, FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_PRECONNECTED]  [DRIVER_EV_RX_SOCKET_ALIVE_REQ]       = (t_driverFSMStep){DRIVER_STT_PRECONNECTED, FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_PRECONNECTED]  [DRIVER_EV_RX_STATE_REQ]              = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMConnectTransition};

  /*** DRIVER_STT_CONNECTED ***/
  /*
   * Driver connected to engine.
   * Requests from engine are accepted.
   * Network is monitored to detect changes and inform engine.
   * Traffic reports from G.hn nodes shall be accepted.
   *
   * Expected events:
   * - DRIVER_EV_DISCONNECT: engine is disconnected. Start from scratch and try to connect to engine again.
   * - DRIVER_EV_RX_CYCQUERY_REQ: start a new alignment check process.
   * - DRIVER_EV_RX_CYCCHANGE_REQ: perform alignment corrections.
   * - DRIVER_EV_NETCHANGE: network change detected.
   * - DRIVER_EV_RX_MEASPLAN_REQ: new measure plan requested.
   * - DRIVER_EV_RX_PSD_SHAPE_REQ: new resource allocation requested.
   * - DRIVER_EV_TRAFFIC_KO: error detected in traffic reports. Try to reconfigure G.hn nodes to send traffic reports.
   * - DRIVER_EV_SYNC_LOST_KO: error starting sync lost monitor. Try starting monitor again.
   * - DRIVER_EV_RX_VERSION_REQ: version and driver Id requested.
   * - DRIVER_EV_RX_STATE_REQ: driver state requested.
   * - DRIVER_EV_RX_DOMAINS_REQ: domains list requested.
   * - DRIVER_EV_RX_CLOCK_REQ: clock requested.
   * - DRIVER_EV_RX_MEASCOLLECT_REQ: start the measure collection process.
   * - DRIVER_EV_RX_MEASPLAN_CANCEL_REQ: measure plan cancellation requested.
   * - DRIVER_EV_RX_SNRPROBES_REQ: SNR measure with PROBEs requested.
   * - DRIVER_EV_MEAS_COLLECT_KO: error detected while collecing measures from G.hn nodes.
   * - DRIVER_EV_MEAS_COLLECT_NODE_END: if measuring process was cancelled, we can receive this event out of expected FSM state. Ignore it.
   * - DRIVER_EV_MEAS_COLLECT_ALL_END: if measuring process was cancelled, we can receive this event out of expected FSM state. Ignore it.
   */
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_CYCQUERY_REQ]           = (t_driverFSMStep){DRIVER_STT_ALIGNCHECK,   FSMAlignCheckTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_CYCCHANGE_REQ]          = (t_driverFSMStep){DRIVER_STT_ALIGNCHANGE,  FSMAlignChangeTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_NETCHANGE]                 = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMNetChangeReportTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_MEASPLAN_REQ]           = (t_driverFSMStep){DRIVER_STT_MEASURING,    FSMMeasuringTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_PSD_SHAPE_REQ]          = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMPsdShapingTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_CDTA_REQ]               = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMCdtaTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_TRAFFIC_KO]                = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMTrafficErrorTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_SYNC_LOST_KO]              = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMSyncLostMonitorErrorTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_VERSION_REQ]            = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_SOCKET_ALIVE_REQ]       = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_STATE_REQ]              = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_DOMAINS_REQ]            = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMDomainReqRxTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_CLOCK_REQ]              = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_MEASCOLLECT_REQ]        = (t_driverFSMStep){DRIVER_STT_MEASCOLLECT,  FSMMeasureCollectStartTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_MEASPLAN_CANCEL_REQ]    = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasureCancelTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_SNRPROBES_REQ]          = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMSNRProbeRequestTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_MEAS_COLLECT_KO]           = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasurePlanFailedTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_MEAS_COLLECT_NODE_END]     = (t_driverFSMStep){DRIVER_STT_CONNECTED,    NULL};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_MEAS_COLLECT_ALL_END]      = (t_driverFSMStep){DRIVER_STT_CONNECTED,    NULL};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_REDIRECT_REQ]           = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMRedirectTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_ENGINE_CONF_REQ]        = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMEngineConfTransition};
  driverFSMTransition[DRIVER_STT_CONNECTED]     [DRIVER_EV_RX_CLUSTER_STOP_REQ]       = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMClusterStopRequestTransition};

  /*** DRIVER_STT_ALIGNCHECK ***/
  /*
   * Wait until time published in EACycQuery.req and
   * send a CycQuery.ind LCMP frame to G.hn devices to gather
   * alignment info.
   *
   * Expected events:
   * - DRIVER_EV_ALIGNCHECK_END: alignment check process is finished.
   * - DRIVER_EV_RX_CYCQUERY_REQ: new CycQuery request received. Abort current process and start a new one.
   */
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_NETCHANGE]                 = (t_driverFSMStep){DRIVER_STT_ALIGNCHECK,   FSMNetChangeReportTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_ALIGNCHECK_END]            = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMAlignCheckEndTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_TRAFFIC_KO]                = (t_driverFSMStep){DRIVER_STT_ALIGNCHECK,   FSMTrafficErrorTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_RX_CLOCK_REQ]              = (t_driverFSMStep){DRIVER_STT_ALIGNCHECK,   FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_RX_SOCKET_ALIVE_REQ]       = (t_driverFSMStep){DRIVER_STT_ALIGNCHECK,   FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_RX_CLUSTER_STOP_REQ]       = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMClusterStopRequestTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHECK]    [DRIVER_EV_RX_CYCQUERY_REQ]           = (t_driverFSMStep){DRIVER_STT_ALIGNCHECK,   FSMAlignCheckTransition};

  /*** DRIVER_STT_ALIGNCHANGE ***/
  /*
   * Translate alignment changes requested by engine to LCMP requests.
   *
   * Expected events:
   * - DRIVER_EV_ALIGNCHANGE_END: alignment change process is finished.
   */
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_NETCHANGE]                 = (t_driverFSMStep){DRIVER_STT_ALIGNCHANGE,  FSMNetChangeReportTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_ALIGNCHANGE_END]           = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMAlignChangeEndTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_TRAFFIC_KO]                = (t_driverFSMStep){DRIVER_STT_ALIGNCHANGE,  FSMTrafficErrorTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_RX_CLOCK_REQ]              = (t_driverFSMStep){DRIVER_STT_ALIGNCHANGE,  FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_RX_SOCKET_ALIVE_REQ]       = (t_driverFSMStep){DRIVER_STT_ALIGNCHANGE,  FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_ALIGNCHANGE]   [DRIVER_EV_RX_CLUSTER_STOP_REQ]       = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMClusterStopRequestTransition};

  /*** DRIVER_STT_MEASURING ***/
  /*
   * New measure plan requested.
   * Send measure plan to G.hn devices and wait their confirmation.
   *
   * Expected events:
   * - DRIVER_EV_PLAN_FAILED: measure plan not confirmed by all nodes. Inform engine and stop measuring thread.
   * - DRIVER_EV_PLAN_CNF_OK: measure plan confirmed by all nodes. Inform engine and stop measuring thread.
   * - DRIVER_EV_RX_MEASPLAN_CANCEL_REQ: measure plan cancel requested by engine. Inform G.hn devices.
   * - DRIVER_EV_RX_MEASPLAN_REQ: a new measure plan requested while another one is on-going. Send error to engine.
   */
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_NETCHANGE]                 = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMNetChangeReportTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_TRAFFIC_KO]                = (t_driverFSMStep){DRIVER_STT_MEASURING,    FSMTrafficErrorTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_RX_CLOCK_REQ]              = (t_driverFSMStep){DRIVER_STT_MEASURING,    FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_RX_SOCKET_ALIVE_REQ]       = (t_driverFSMStep){DRIVER_STT_MEASURING,    FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_PLAN_FAILED]               = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasurePlanFailedTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_PLAN_CNF_OK]               = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasurePlanMeasRspOkTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_RX_MEASPLAN_CANCEL_REQ]    = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasureCancelTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_RX_MEASPLAN_REQ]           = (t_driverFSMStep){DRIVER_STT_MEASURING,    FSMMeasurePlanMeasRspKoTransition};
  driverFSMTransition[DRIVER_STT_MEASURING]     [DRIVER_EV_RX_CLUSTER_STOP_REQ]       = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMClusterStopRequestTransition};

  /*** DRIVER_STT_MEASCOLLECT ***/
  /*
   * Measure collection requested by engine.
   * Start one thread per node to request its related measures.
   *
   * Expected events:
   * - DRIVER_EV_RX_MEASCOLLECT_REQ: a new measure collection requested. Try to accommodate the request if enough resources.
   * - DRIVER_EV_MEAS_COLLECT_KO: error detected while collecting measures. Inform engine.
   * - DRIVER_EV_MEAS_COLLECT_NODE_END: measure collection finished for a given node. Stop its related thread. Inform engine if all nodes have finished.
   * - DRIVER_EV_MEAS_COLLECT_ALL_END: whole measure collection process is finished.
   * - DRIVER_EV_RX_MEASPLAN_CANCEL_REQ: measure plan cancel requested by engine. Inform G.hn devices.
   * - DRIVER_EV_RX_MEASPLAN_REQ: a new measure plan requested while another one is on-going. Send error to engine.
   */
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_NETCHANGE]                 = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMNetChangeReportTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_TRAFFIC_KO]                = (t_driverFSMStep){DRIVER_STT_MEASCOLLECT,  FSMTrafficErrorTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_RX_CLOCK_REQ]              = (t_driverFSMStep){DRIVER_STT_MEASCOLLECT,  FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_RX_SOCKET_ALIVE_REQ]       = (t_driverFSMStep){DRIVER_STT_MEASCOLLECT,  FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_RX_MEASCOLLECT_REQ]        = (t_driverFSMStep){DRIVER_STT_MEASCOLLECT,  FSMMeasureCollectStartTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_MEAS_COLLECT_KO]           = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasurePlanFailedTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_MEAS_COLLECT_NODE_END]     = (t_driverFSMStep){DRIVER_STT_MEASCOLLECT,  FSMMeasureCollectNodeEndTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_MEAS_COLLECT_ALL_END]      = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasureCollectAllNodesEndTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_RX_MEASPLAN_CANCEL_REQ]    = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMMeasureCancelTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_RX_MEASPLAN_REQ]           = (t_driverFSMStep){DRIVER_STT_MEASCOLLECT,  FSMMeasurePlanMeasRspKoTransition};
  driverFSMTransition[DRIVER_STT_MEASCOLLECT]   [DRIVER_EV_RX_CLUSTER_STOP_REQ]       = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMClusterStopRequestTransition};


  /*** DRIVER_STT_PSDSHAPING ***/
  /*
   * New PSD shaping requested.
   * Translate to LCMP requests.
   *
   * Expected events:
   * - DRIVER_EV_RX_PSD_SHAPE_REQ: a new PSD shaping requested while running a previous one. Send an error to engine.
   * - DRIVER_EV_PSD_SHAPE_END_OK: PSD shaping process finished successfully. Inform engine.
   * - DRIVER_EV_PSD_SHAPE_END_KO: error detected while configuring new PSD shape. Send error to engine.
   * - DRIVER_EV_RX_MEASPLAN_REQ: a new measure plan requested while another one is on-going. Send error to engine.
   */
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_IDLE]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMIdleTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_KILL]                      = (t_driverFSMStep){DRIVER_STT_IDLE,         FSMKillTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_DISCONNECT]                = (t_driverFSMStep){DRIVER_STT_DISCONNECTED, FSMDisconnectTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_NETCHANGE]                 = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMNetChangeReportTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_TRAFFIC_KO]                = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMTrafficErrorTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_CLOCK_REQ]              = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_SOCKET_ALIVE_REQ]       = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMGenericRxFrameTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_MEASPLAN_REQ]           = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMMeasurePlanMeasRspKoTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_PSD_SHAPE_REQ]          = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMPsdShapeAlreadyRunningTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_CDTA_REQ]               = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMCdtaAlreadyRunningTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_SNRPROBES_REQ]          = (t_driverFSMStep){DRIVER_STT_PSDSHAPING,   FSMSNRProbeRequestTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_PSD_SHAPE_END_OK]          = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMPsdShapeEndOkTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_PSD_SHAPE_END_KO]          = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMPsdShapeEndKoTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_CDTA_END_OK]            = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMCdtaEndOkTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_CDTA_END_KO]            = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMCdtaEndKoTransition};
  driverFSMTransition[DRIVER_STT_PSDSHAPING]    [DRIVER_EV_RX_CLUSTER_STOP_REQ]       = (t_driverFSMStep){DRIVER_STT_CONNECTED,    FSMClusterStopRequestTransition};

}

/************************************************************************/

static t_VB_comErrorCode FSMEventDo(t_driverMsg *msg, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_driverFSMStep   state_transition = { 0, NULL };

  // Check valid args
  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Check valid event
    if (msg->event >= DRIVER_EV_LAST)
    {
      ret = VB_COM_ERROR_BAD_ARGS;
      VbLogPrint(VB_LOG_ERROR, "Invalid event (%u)", msg->event);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (driverState >= DRIVER_STT_LAST)
    {
      ret = VB_COM_ERROR_INVALID_STATE;
      VbLogPrint(VB_LOG_ERROR, "Invalid state (%u)", driverState);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Increase related counter
    VbCounterIncrease(VbDatamodelEvToCounter(msg->event));
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Get state related transition
    state_transition = driverFSMTransition[driverState][msg->event];

    if (state_transition.nextState == DRIVER_STT_LAST)
    {
      ret = VB_COM_ERROR_INVALID_TRANSITION;
      VbLogPrint(VB_LOG_ERROR, "Invalid transition (driverState %s; event %s)",
          FSMSttToStrGet(driverState), FSMEvToStrGet(msg->event));
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (state_transition.transitionFunc != NULL)
    {
      // Do transition actions
      ret = state_transition.transitionFunc(msg, driverState, data);

      if (ret != VB_COM_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d running state transition (driverState %s; event %s)",
            ret, FSMSttToStrGet(driverState), FSMEvToStrGet(msg->event));
      }
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_INFO, "FSM Transition : STT_%-20s -> EV_%-20s -> STT_%-20s",
        FSMSttToStrGet(driverState), FSMEvToStrGet(msg->event),FSMSttToStrGet(state_transition.nextState));

    if (driverState != state_transition.nextState)
    {
      // Notify state change to VB Engine
      VbEAStateTrgSend(FSMSttToStrGet(state_transition.nextState));
    }

    // Update current state
    driverState = state_transition.nextState;
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode MainQueueInit(void)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  struct mq_attr   attr;

  // Destroy previous posix queue
  mq_unlink(VBQUEUENAME);

  attr.mq_maxmsg = DRIVER_NUM_QUEUE_ELEMS;
  attr.mq_msgsize = DRIVER_MSG_SIZE;

  vbQueue = mq_open(VBQUEUENAME, O_CREAT | O_RDWR , 0666, &attr);

  if (vbQueue == (mqd_t) -1)
  {
    VbLogPrint(VB_LOG_ERROR, "Error to create mqueue. errno %s", strerror(errno));
    ret = VB_COM_ERROR_QUEUE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode MainQueueRelease(void)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  // Flush queue and release allocated memory
  MainQueueFlush();

  // Close and release queue
  VbLogPrint(VB_LOG_INFO, "Closing vbQueue...");
  mq_close(vbQueue);
  mq_unlink(VBQUEUENAME);
  VbLogPrint(VB_LOG_INFO, "Closed vbQueue...");

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode ComponentsInit(void)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32S            err;

  VbThreadInit();
  VbTimerInit();
  VbPsdShapeInit();
  VbDomainsMonitorInit();
  VbDatamodelInit();
  VbMainDriverCountersInit();

  err = VbLogInit(DRIVER_LOG_QUEUE_NAME,
                  VbDriverConfVerboseLevelGet(),
                  VbDriverConfOutputPathGet(),
                  VbDriverConfPeristentLogNumLinesGet(),
                  VbDriverConfPeristentLogVerboseLevelGet(),
                  VbDriverConfPersistentLogIsCircular());

  if (err != 0)
  {
    ret = VB_COM_ERROR_NOT_STARTED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    VbLcmpInit(VbDriverConfLcmpIfGet());
    VbEAInit(VbDriverConfEaIfGet(), VbDriverConfEaPortGet(), VbDriverConfRemoteIPGet(), VbDriverConfServerModeGet(), VbDriverConfFamilyGet());
    VbTrafficInit();
    VbMainConsoleInit(VbDriverConfConsolePortGet());
    VbMeasurementInit();
    vbMainRedirected = FALSE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode ComponentsStart(void)
{
  t_VB_comErrorCode    ret = VB_COM_ERROR_NONE;
  t_HGF_LCMP_ErrorCode lcmp_err;
  BOOL                 run;

  // Start signal processing thread
  run = VbThreadHandleSignalsStart(VbDriverSignalHandler);

  if (run == FALSE)
  {
    ret = VB_COM_ERROR_THREAD;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Start lcmp thread
    lcmp_err = LcmpExecute();

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      ret = VB_COM_ERROR_THREAD;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Start log
    run = VbLogRun();

    if (run == FALSE)
    {
      ret = VB_COM_ERROR_THREAD;
    }
  }

  if ((ret == VB_COM_ERROR_NONE) &&
      (VbDriverConfConsolePortGet() != 0))
  {
    // Start console
    VbConsoleStart();
  }

  return ret;
}

/*******************************************************************/

static void ComponentsStop()
{
  // Stop Console thread
  VbConsoleStop();

  // Stop LCMP thread
  LcmpEnd();

  // Stop measure plan & collect measure
  VbMeasurementStop();

  // Release datamodel memory
  VbDatamodelAllDomainsListDestroy();

  VbMainTimerStop();

  // Stop socket alive timer
  VbMainSocketAliveTimerStop();

  // Stop Log thread
  VbLogStop();
  // From this point we should use "printf" instead of VbLogPrint

  // Stop signal handler thread
  VbThreadHandleSignalsStop();

  // Release connection resources
  VbEADestroy();
}

/*******************************************************************/

static t_VB_comErrorCode FSMDisconnectTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  // Start from scratch
  FSMIdleTransition(msg, currStt, data);

  if(vbMainRedirected == TRUE)
  {
    VbEAInit(VbDriverConfEaIfGet(), VbDriverConfEaPortGet(), VbDriverConfRemoteIPGet(), VbDriverConfServerModeGet(), VbDriverConfFamilyGet());
  }

  // Wait some time to avoid too many retries per second
  VbThreadSleep(DRIVER_WAIT_TO_RETRY_CONN);

  // Restart EA interface and wait for new connections
  VbEAStart();

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMConnectTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  BOOL              run;

  if (ret == VB_COM_ERROR_NONE)
  {
    // Process State request frame
    VbEAFrameRxProcess(msg->eaMsg);
  }

  // Force first Alive Msg
  VbMainKeepAliveSend();

  // Start timer
  run = VbMainTimerRun();

  if (run == FALSE)
  {
    ret = VB_COM_ERROR_THREAD;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Start G.hn domains monitor to check for network changes
    VbDomainsMonitorRun();

    // Start traffic awareness
    VbTrafficAwarenessRun();

    // Start sync lost monitor
    run = VbAlignmentSyncLostMonitorRun();

    if (run == FALSE)
    {
      ret = VB_COM_ERROR_THREAD;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMIdleTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  // Stop nodes transmissions
  VbAlignmentLCMPClusterStopSend(TRUE);

  // Stop align check thread
  VbAlignmentCheckProcessStop();

  // Stop align change thread
  VbAlignmentChangeProcessStop();

  // Stop PSD thread
  VbPsdShapeStop();

  // Stop Domains Monitor thread
  VbDomainsMonitorStop();

  // Stop Measure Plan & collection measures thread
  VbMeasurementStop();

   // Stop traffic awareness thread
  VbTrafficAwarenessStop();

  // Stop sync lost monitor
  VbAlignmentSyncLostMonitorStop();

  // Stop EA interface
  VbEAStop();

  // Stop main timer thread
  VbMainTimerStop();

  // Stop main timer thread
  VbMainSocketAliveTimerStop();

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMKillTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  vbMainRunMAIN = FALSE;

  // Stop nodes transmissions
  VbAlignmentLCMPClusterStopSend(TRUE);

  // Stop threads
  FSMIdleTransition(msg, currStt, data);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMRedirectTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  CHAR      ip_addr[INET6_ADDRSTRLEN];

  CHAR      driver_id[VB_EA_DRIVER_ID_MAX_SIZE];
  INT32U    port;
  struct    in6_addr remoteIp;
  int       ip_err;
  const INT32U PORT_MAX_LENGTH = 6;

  if(msg->eaMsg->eaPayload.msgLen < (INET6_ADDRSTRLEN + PORT_MAX_LENGTH + VB_EA_DRIVER_ID_MAX_SIZE))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if(ret == VB_COM_ERROR_NONE)
  {
    // Get ip addr
    strncpy(ip_addr, (CHAR *)msg->eaMsg->eaPayload.msg, INET6_ADDRSTRLEN);
    ip_addr[INET6_ADDRSTRLEN - 1] = '\0';
    ip_err = inet_pton(AF_INET6, VbUtilTrimWhiteSpace(ip_addr), &remoteIp); //ip_addr

    if (ip_err != 1)
    {
      ret = VB_COM_ERROR_BAD_ARGS;
      VbLogPrint(VB_LOG_ERROR, "Wrong IP values. Err %d", ip_err);
    }
  }

  if(ret == VB_COM_ERROR_NONE)
  {
    // Get port
    port = atoi((CHAR *)&msg->eaMsg->eaPayload.msg[INET6_ADDRSTRLEN]);

    if(port >= DRIVER_MAX_TCP_REDIRECT_PORT)
    {
      ret = VB_COM_ERROR_BAD_ARGS;
      VbLogPrint(VB_LOG_ERROR, "Port out of limits, %s", (CHAR *)&msg->eaMsg->eaPayload.msg[INET6_ADDRSTRLEN]);
    }
  }

  if(ret == VB_COM_ERROR_NONE)
  {
    // Add driver id
    strncpy(driver_id, (CHAR *)&(msg->eaMsg->eaPayload.msg[INET6_ADDRSTRLEN + PORT_MAX_LENGTH]), VB_EA_DRIVER_ID_MAX_SIZE);
    driver_id[VB_EA_DRIVER_ID_MAX_SIZE - 1] = '\0';
    if(strncmp(driver_id, VbDriverConfDriverIdGet(), VB_EA_DRIVER_ID_MAX_SIZE) != 0)
    {
      ret = VB_COM_ERROR_BAD_ARGS;
      VbLogPrint(VB_LOG_ERROR, "Incorrect driver id, %s. Expected %s", (CHAR *)&msg->eaMsg->eaPayload.msg[INET6_ADDRSTRLEN + PORT_MAX_LENGTH], VbDriverConfDriverIdGet());
    }
  }

  if(ret == VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_INFO, "Redirected to %s : %u", ip_addr, (unsigned int)port);
    // Start from scratch
    FSMIdleTransition(msg, currStt, data);

    // Change IP and port
    VbEAInit( VbDriverConfEaIfGet(), port, ip_addr, FALSE, VbDriverConfFamilyGet());

    vbMainRedirected = TRUE;

    // Restart EA interface and wait for new connections
    VbEAStart();
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMAlignCheckTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbAlignmentEACycQueryReqProcess(msg->eaMsg->eaPayload.msg);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMAlignCheckEndTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Ensure that align check thread is stopped
    VbAlignmentCheckProcessStop();
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMAlignChangeTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbAlignmentEACycChangeReqProcess(msg->eaMsg->eaPayload.msg);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMEngineConfTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbAlignmentModeProcess(msg->eaMsg->eaPayload.msg);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMClusterStopRequestTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    VbMeasurementStop();

    ret = VbAlignmentClusterStopProcess(msg->eaMsg->eaPayload.msg);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMAlignChangeEndTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Ensure that align change thread is stopped
    VbAlignmentChangeProcessStop();
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMNetChangeReportTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  // Stop measurement plan & measures collection
  VbMeasurementStop();

  // Start traffic awareness (only if domains present > 0)
  VbTrafficAwarenessRun();

  if (ret == VB_COM_ERROR_NONE)
  {
    // Sent network change trigger to VB Engine
    ret = VbEANetworkChangeReportTrgSend();
  }

  if(ret == VB_COM_ERROR_NONE)
  {
    // Tell Domain Monitor thread that sending has been successful
    VbDomainsMonitorSignal();
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasuringTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAMeasRspErrorCode err_code;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbMeasurePlanReqRun(msg->eaMsg->eaPayload);
  }

  if (ret == VB_COM_ERROR_ALREADY_RUNNING)
  {
    err_code = VB_EA_MEAS_RSP_ERR_ALREADY_RUNNING;
  }
  else if (ret != VB_COM_ERROR_NONE)
  {
    err_code = VB_EA_MEAS_RSP_ERR_UNKNOWN;
  }

  if (ret != VB_COM_ERROR_NONE)
  {
    INT8U plan_id;

    if(msg == NULL)
    {
      plan_id = 0;
    }
    else
    {
      plan_id = VBMeasurementPlanIDFromMsgGet(msg->eaMsg->eaPayload.msg);
    }

    VbLogPrint(VB_LOG_ERROR, "Error in FSMMeasuringTransition ret (%d), Err %d, plan Id %d", ret, err_code,plan_id);

    ret = VbEAMeasurePlanErrorSend(plan_id, err_code);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasurePlanFailedTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret;
  t_vbEAMeasRspErrorCode reported_error = *(t_vbEAMeasRspErrorCode *)data;

  // Send message before stopping measurement process to get proper PlanId
  ret = VbEAMeasurePlanErrorSend(VBMeasurementPlanIDGet(), reported_error);

  VbMeasurementStop();

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasurePlanMeasRspOkTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT8U plan_id;

  // Join measurement thread
  VbMeasurePlanStop();

  // Send confirmation to Engine
  plan_id = VBMeasurementPlanIDGet();
  VbEAMeasurePlanRspSend(plan_id, VB_EA_MEAS_RSP_ERR_NONE);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasurePlanMeasRspKoTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    INT8U plan_id;

    plan_id = VBMeasurementPlanIDFromMsgGet(msg->eaMsg->eaPayload.msg);

    // Send error to Engine
    ret = VbEAMeasurePlanRspSend(plan_id, VB_EA_MEAS_RSP_ERR_PROCESS);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMPsdShapeEndOkTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret;

  // Stop PSD shape thread
  VbPsdShapeStop();

  // Send PSD shape response to engine
  ret = VbEAPsdShapeCnfSend(VB_EA_PSD_SHAPE_STATUS_OK);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMPsdShapeEndKoTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret;

  // Stop PSD shape thread
  VbPsdShapeStop();

  // Send PSD shape response to engine
  ret = VbEAPsdShapeCnfSend(VB_EA_PSD_SHAPE_STATUS_ERR);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMPsdShapeAlreadyRunningTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret;

  // Send PSD shape response to engine
  ret = VbEAPsdShapeCnfSend(VB_EA_PSD_SHAPE_STATUS_ERR);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMCdtaEndOkTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret;

  // Stop PSD shape thread
  VbCdtaStop();

  // Send PSD shape response to engine
  ret = VbEACdtaCnfSend(VB_EA_PSD_SHAPE_STATUS_OK);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMCdtaEndKoTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret;

  // Stop PSD shape thread
  VbCdtaStop();

  // Send PSD shape response to engine
  ret = VbEACdtaCnfSend(VB_EA_PSD_SHAPE_STATUS_ERR);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMCdtaAlreadyRunningTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret;

  // Send Cdta response to engine
  ret = VbEACdtaCnfSend(VB_EA_PSD_SHAPE_STATUS_ERR);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasureCollectStartTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbMeasureMeasuresCollectRun(msg->eaMsg->eaPayload.msg, msg->eaMsg->eaPayload.msgLen);
  }

  if (ret != VB_COM_ERROR_NONE)
  {
    VbMainQEvSend(DRIVER_EV_MEAS_COLLECT_KO, vbQueue, (void*)VB_EA_MEAS_RSP_ERR_NO_RESOURCES);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasureCollectNodeEndTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode  ret = VB_COM_ERROR_NONE;

  // Stop measure collect thread
  VbMeasureMeasuresCollectStop(data);

  // Send event collection done for this node
  if (ret == VB_COM_ERROR_NONE)
  {
    if (VbMeasurementCollectThreadsRunning() == FALSE)
    {
      t_vbEAMeasRspErrorCode collect_end_err;
      INT8U                  plan_id;

      // Get Measure Plan Id
      plan_id = VBMeasurementPlanIDGet();

      // Get collect end error
      collect_end_err = VBMeasurementCollectEndErrorGet();

      if (collect_end_err == VB_EA_MEAS_RSP_ERR_NONE)
      {
        ret = VbEAMeasCollectionEndSend(plan_id);
      }
      else
      {
        ret = VbEAMeasurePlanErrorSend(plan_id, collect_end_err);
      }

      VbMainQEvSend(DRIVER_EV_MEAS_COLLECT_ALL_END, vbQueue, (void *)NULL);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasureCollectAllNodesEndTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  VbMeasurementStop();

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMMeasureCancelTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Stop current measurement action
    VbMeasurementStop();

    // Send Cancel to FW and cnf to Engine
    VbMeasurementCancel(msg->eaMsg->eaPayload.msg);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMPsdShapingTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  VbPsdShapeWriteRun(msg->eaMsg->eaPayload.msg, msg->eaMsg->eaPayload.msgLen);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMCdtaTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  VbCdtaWriteRun(msg->eaMsg->eaPayload.msg, msg->eaMsg->eaPayload.msgLen);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMTrafficErrorTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  VbLogPrint(VB_LOG_ERROR, "Traffic awareness ERROR");
  VbTrafficAwarenessRun();

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMSyncLostMonitorErrorTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  BOOLEAN           run;

  VbLogPrint(VB_LOG_ERROR, "Sync Lost monitor ERROR");

  run = VbAlignmentSyncLostMonitorRun();

  if (run == FALSE)
  {
    ret = VB_COM_ERROR_THREAD;
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMDomainReqRxTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  // Check if first discovery attempt has finished
  if (VbDomainsMonitorFirstAttemptGet() == FALSE)
  {
    // Process request
    ret = FSMGenericRxFrameTransition(msg, currStt, data);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMGenericRxFrameTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Process frame
    VbEAFrameRxProcess(msg->eaMsg);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode FSMSNRProbeRequestTransition(t_driverMsg *msg, t_driverState currStt, void *data)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbMeasureSNRProbeReqRun(msg->eaMsg->eaPayload);
  }

  return ret;
}

/*******************************************************************/

static t_vb_counter_error VbMainDriverCountersInit( void )
{
  t_vb_counter_error err;
  INT8U i;

  err = VbCountersInit(NULL);

  if (err == VB_COUNTERS_ERROR_NONE)
  {
    for (i = 0 ; i < VB_DRIVER_COUNTERS_NUM ; i++)
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

static void VbMainLoop(void)
{
  t_VB_comErrorCode ret;
  ssize_t           num_read;
  t_driverMsg      *vb_msg;

  vbMainRunMAIN = TRUE;
  VbEAStateTrgSend(FSMSttToStrGet(driverState));

  // Send DRIVER_EV_START
  ret = VbMainStart();

  if (ret != VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d starting driver", ret);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    while (vbMainRunMAIN == TRUE)
    {
      ret = VB_COM_ERROR_NONE;
      num_read = mq_receive(vbQueue, driverMsgBuffer, DRIVER_MSG_SIZE, NULL);

      if (num_read < 0)
      {
        VbLogPrint( VB_LOG_ERROR, "Error reading from main queue %u. Finishing execution!", errno);

        // Fatal error, finish execution
        vbMainRunMAIN = FALSE;

        ret = VB_COM_ERROR_QUEUE;
      }

      if ((ret == VB_COM_ERROR_NONE) && (num_read != DRIVER_MSG_SIZE))
      {
        VbLogPrint( VB_LOG_ERROR, "Message size error, numbytes received %d", num_read);
        ret = VB_COM_ERROR_QUEUE;
      }

      if (ret == VB_COM_ERROR_NONE)
      {
        vb_msg = (t_driverMsg *)driverMsgBuffer;

        // Perform FSM transition
        ret = FSMEventDo(vb_msg, vb_msg->args);

        if (vb_msg->eaMsg != NULL)
        {
          // Release allocated memory
          VbEAMsgFree(&(vb_msg->eaMsg));
        }
      }
    }
  }
}

/*******************************************************************/

static void MainQueueFlush(void)
{
  t_VB_comErrorCode err = VB_COM_ERROR_NONE;
  struct mq_attr   attr;
  ssize_t           num_read = 0 ;
  t_driverMsg          *vb_msg;
  int               mq_err = 0;
  BOOLEAN           queue_is_empty = FALSE;

  VbLogPrint(VB_LOG_INFO, "Flushing vbQueue...");

  if (mq_getattr(vbQueue, &attr) == -1)
  {
    VbLogPrint(VB_LOG_ERROR, "Error mq_getattr. errno %s", strerror(errno));
    err = VB_COM_ERROR_QUEUE;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Clear queue
    // Config nonBlock queue
    attr.mq_flags = O_NONBLOCK;

    mq_err = mq_setattr(vbQueue, &attr, NULL);

    if (mq_err == -1)
    {
      VbLogPrint( VB_LOG_ERROR, "Error mq_setattr main (%u). Aborting flush operation...", errno);
      err = VB_COM_ERROR_QUEUE;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    while (queue_is_empty == FALSE)
    {
      num_read = mq_receive(vbQueue, driverMsgBuffer, DRIVER_MSG_SIZE, NULL);

      if (num_read < 0)
      {
        queue_is_empty = TRUE;
      }
      else if (num_read != DRIVER_MSG_SIZE)
      {
        VbLogPrint( VB_LOG_ERROR,
                   "Cleanning queue numbytes received mismatch %u != %u", num_read, sizeof(t_driverMsg));
      }
      else
      {
        vb_msg = (t_driverMsg *)driverMsgBuffer;

        if (vb_msg->eaMsg != NULL)
        {
          free(vb_msg->eaMsg);
          vb_msg->eaMsg = NULL;
        }
      }
    }
  }

  VbLogPrint(VB_LOG_INFO, "Flushed vbQueue...");
}

/*******************************************************************/

static void VbDriverSignalHandler(INT32U signum)
{
  if (signum == SIGINT)
  {
    VbMainKill();
  }
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/****************************************************************
 * MAIN                                                         *
 *****************************************************************/

int main(int argc,  char **argv)
{
  t_VB_comErrorCode    err = VB_COM_ERROR_NONE;

#if (_MEMORY_DEBUG_ == 1)
  mtrace();
#endif

  printf("VectorBoost driver version: %s\n", VERSION);

  // Take care of CTRL+C
  if (VbBlockSignals() == FALSE)
  {
    err = VB_COM_ERROR_THREAD;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Init FSM transitions
    FSMInit();

    // Configure max prio for main thread
    MainThreadPrioConf();

    // Destroy previous posix queue instances and create a new one
    err = MainQueueInit();
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Parse configuration
    err = VbDriverConfParse(argc, argv);
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Init components
    err = ComponentsInit();
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Starting components
    err = ComponentsStart();
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Entering main loop
    VbMainLoop();
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

const CHAR* VbMainStateStrGet(void)
{
  return FSMSttToStrGet(driverState);
}

/*******************************************************************/

t_VB_comErrorCode VbMainIdleModeSet(void)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  mqd_t             vb_queue;

  // Connect to main queue
  vb_queue = mq_open(VBQUEUENAME, O_WRONLY);

  if (vb_queue == -1)
  {
    ret = VB_COM_ERROR_QUEUE;
    VbLogPrint(VB_LOG_ERROR, "Error opening queue [%s]", strerror(errno));
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbMainQEvSend(DRIVER_EV_IDLE, vb_queue, NULL);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Wait for main thread to go IDLE
    while (driverState != DRIVER_STT_IDLE)
    {
      VbThreadSleep(DRIVER_WAIT_TO_IDLE);
    }
  }

  mq_close(vb_queue);

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbMainStart(void)
{
  t_VB_comErrorCode ret;
  mqd_t             vb_queue;

  // Connect to main queue
  vb_queue = mq_open(VBQUEUENAME, O_WRONLY);

  ret = VbMainQEvSend(DRIVER_EV_START, vb_queue, NULL);

  mq_close(vb_queue);

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbMainKill(void)
{
  t_VB_comErrorCode ret;
  mqd_t             vb_queue;

  // Connect to main queue
  vb_queue = mq_open(VBQUEUENAME, O_WRONLY);

  ret = VbMainQEvSend(DRIVER_EV_KILL, vb_queue, NULL);

  mq_close(vb_queue);

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbMainQEvSend(t_driverEvent event, mqd_t vbQueue, void *args)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_driverMsg       vb_msg;
  int               err;

  if (vbMainRunMAIN == FALSE)
  {
    // Thread is closing, do not queue more messages
    ret = VB_COM_ERROR_NOT_STARTED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    bzero(&vb_msg, sizeof(vb_msg));
    vb_msg.event = event;
    vb_msg.eaMsg = NULL;
    vb_msg.args = args;

    err = mq_send(vbQueue, ((const char *)(&vb_msg)), DRIVER_MSG_SIZE, VB_THREADMSG_PRIORITY);

    if (err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error posting msg to Main queue. errno %s", strerror(errno));
      ret = VB_COM_ERROR_QUEUE;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbMainQEAFrameEvSend(t_vbEAMsg *msg, mqd_t vbQueue, void *args)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_driverMsg       vb_msg;
  int               err;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if ( (msg->opcode >= VB_EA_OPCODE_LAST) || (eaLookupEvents[msg->opcode] >= DRIVER_EV_LAST) )
    {
      VbLogPrint(VB_LOG_ERROR, "Unexpected OPCODE 0x%02X", (INT32U)msg->opcode);

      ret = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (vbMainRunMAIN == FALSE)
    {
      // Thread is closing, do not queue more messages
      ret = VB_COM_ERROR_NOT_STARTED;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    bzero(&vb_msg, sizeof(vb_msg));
    vb_msg.event = eaLookupEvents[msg->opcode];
    vb_msg.eaMsg = msg;
    vb_msg.args = args;

    err = mq_send(vbQueue, ((const char *)(&vb_msg)), DRIVER_MSG_SIZE, VB_THREADMSG_PRIORITY);

    if (err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error posting msg to Main queue. errno %s", strerror(errno));
      ret = VB_COM_ERROR_QUEUE;
    }
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
