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
 * @file vb_engine_process.c
 * @brief Implements engine process
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
#include <errno.h>
#include <pthread.h>

#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_engine_datamodel.h"
#include "vb_engine_EA_interface.h"
#include "vb_log.h"
#include "vb_engine_communication.h"
#include "vb_engine_process.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_console.h"
#include "vb_engine_conf.h"
#include "vb_counters.h"
#include "vb_metrics.h"
#include "vb_counters.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_alignment.h"
#include "vb_engine_clock.h"
#include "vb_engine_socket_alive.h"
#include "vb_engine_measure.h"
#include "vb_engine_SNR_calculation.h"
#include "vb_engine_l2rPSD_calculation.h"
#include "vb_engine_psd_shape.h"
#include "vb_util.h"

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/// FSM transition functions
static t_VB_engineErrorCode VbeFSMAllDriversKillAllTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversKillSingleDriverTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMDisconnectTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMConnectTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignSyncLostTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMDiscoverVersionTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMDiscoverStateTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMClockRspWaitTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMClockRspTOTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMClockRspTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMStateRxTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMVersionRxAndDiscoverStateTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversClockReqSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversClockReqTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversClockForceReqTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignCheckRestartTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignCheckTOTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignCheckTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignChangeTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignChangeSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignDoneTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignDoneSkipMeasTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignPeriodicCheckTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignConfigSyncTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignRestartTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMGenericFrameRxTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentConfigTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentConfigSyncTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentCheckSyncTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentChangeKoTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentChangeTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentChangeSyncTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllMeasPlanBuildTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllCalculateSNRTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllComputationOkTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllBoostAlgRunTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllBoostUpdateTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllBoostTimerSetAndAlgRunTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllBoostUpdateEndKoTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllBoostUpdateEndTOTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllComputationKoTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllPSDForceUpdateTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMMeasPlanSendTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMMeasPlanFailTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMMeasPlanCnfSyncTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversMeasPlanCnfSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllMeasCollectMeasuresTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMMeasCollectMeasuresTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMMeasuresCollectDoneTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllMeasuresCollectDoneSyncTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMSNRProbeForcedTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentClusterAllStopSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignAllClustersTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignAllClustersTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMDriverAlignAllClustersTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignClustersAllStopSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignClusterITransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllEPChangeTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMMeasPlanCancelSendTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllPlanRestartTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMEmptyDomainTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMMeasuresPlanCancelSyncTransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversMeasPlanCancelSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllMeasPlanReBuildTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAlignmentClusterIStopSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAlignClusterIStopSyncTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMDriverAlignClusterITransition( t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMDMRemBestActionMeasTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMDMRemBestActionBoostTransition(t_VBProcessMsg *processMsg);
static t_VB_engineErrorCode VbeFSMAllDriversAliveSocketReqTransition(t_VBProcessMsg *processMsg);

/**
* @brief This function executes the FSM process
* param[in] arg Arguments to thread
**/
static void VbEngineProcess( void *arg);

/**
* @brief Check if all drivers are in the same given state
* @param[in] state FSM state to check
**/
static t_VB_engineErrorCode VbEngineProcessCheckDriversSync(t_vbEngineProcessFSMState state);

/* Log current state */
static void VbEngineProcessStateLogToFile(const t_VBDriver *thisDriver);

/**
* @brief Flush the engine_process message queue
*
**/
static t_VB_engineErrorCode VbEngineProcessQueueFlush(void);

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_ENGINE_SYNC_TIMEOUT                  (10000)
#define VB_ENGINE_ALIGN_CHECK_TIMEOUT           (5000)
#define VB_ENGINE_ALIGN_CHANGE_TIMEOUT          (2000)
#define VB_ENGINE_REQUEST_TIMEOUT               (1000)
#define VB_ENGINE_ALIGN_CONF_WAIT               (1000)
#define VB_ENGINE_MEASURE_COLLECT_TIMEOUT       (100000)
#define VB_ENGINE_MEASURE_CANCEL_TIMEOUT        (100000)
#define VB_ENGINE_ALIGN_CHANGE_MAX_ATTS         (3)
#define VB_ENGINE_PROCESS_THREAD_NAME           "vb_engine_process"
#define VB_ENGINE_PROCESS_MQ_MSG_SIZE           (sizeof(t_VBProcessMsg))
#define VB_ENGINE_PROCESS_MQ_MAX_MSGS           (200)
#define VB_ENGINE_PROCESS_MQ_NAME               "/VbEngineProcessQ"
#define VB_ENGINE_CHANGES_APPLY_MARGIN          (10) // In ms

#define VB_ENGINE_ALIVE_CHECK_TIMEOUT           (10000)


/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef t_VB_engineErrorCode (*t_stateHandler)(t_VBProcessMsg *vbProcessMsg);

typedef struct transition
{
  t_vbEngineProcessFSMState nextState;
  t_stateHandler            stateHandler;
} t_VbEngineFSMStep;

typedef struct s_VbEngineProcess
{
  BOOLEAN          running;
  pthread_t        threadId;
  t_vbQueueName    queueName;
  mqd_t            rdWrQueueId;
  mqd_t            wrOnlyQueueId;
} t_VbEngineProcess;

typedef struct
{
  t_VB_Comm_Event           event;
} t_VbEngineAllDriversArgs;


/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_VbEngineProcess    vbEngineProcess;
static t_VbEngineFSMStep    vbeFSMTransition[ENGINE_STT_LAST][ENGINE_EV_LAST];
static CHAR                 vbEngineProcessMQBuffer[VB_ENGINE_PROCESS_MQ_MSG_SIZE];
static t_VBDMsHistory       vbDMsHistory = { NULL, 0 };
static pthread_mutex_t      vbDMsHistoryMutex;

static const CHAR *FSMStateString[ENGINE_STT_LAST] =
  {
   "DISCONNECTED",
   "DISCOVERING_VERSION_REQUEST",
   "DISCOVERING_STATE_REQUEST",
   "DISCOVERING_DOMAIN_WAIT_EMPTY",
   "DISCOVERING_DOMAIN_WAIT",
   "DISCOVERING_DOMAIN_SYNC",
   "CLOCK_RSP_WAIT",
   "CLOCK_REQ_SYNC",
   "ALIGNMENT_PREPARE_ALL",
   "ALIGNMENT_PREPARE_I",
   "ALIGNMENT_CLUSTERS_ALL_STOP_SYNC",
   "ALIGNMENT_CLUSTER_I_STOP_SYNC",
   "ALIGNMENT_CHECK",
   "ALIGNMENT_CHECK_SYNC",
   "ALIGNMENT_CHANGE",
   "ALIGNMENT_CHANGE_SYNC",
   "ALIGNMENT_CONFIG",
   "ALIGNMENT_CONFIG_SYNC",
   "ALIGNMENT_CONFIG_DONE_SYNC",
   "ALIGNMENT_WAIT_SYNC",
   "ALIGNMENT_DONE_SYNC",
   "MEASURING_MEASPLAN_RSP_WAIT",
   "MEASURING_MEASPLAN_RSP_SYNC",
   "MEASURING_MEASPLAN_END_WAIT",
   "MEASURING_COLLECT_MEAS",
   "MEASURING_END_SYNC",
   "MEASURING_CANCEL_WAIT",
   "MEASURING_CANCEL_WAIT_SYNC",
   "BOOSTING_CALCULATE_SNR",
   "BOOSTING_WAIT_TRIGGERS",
   "BOOSTING_WAIT_UPDATE",
   "ENGINE_STT_UNDEF",
  };

static const CHAR *vbEngineEventString[ENGINE_EV_LAST] =
  {
      "KILL_ALL",
      "KILL",
      "CONNECT",
      "DISCONNECT",
      "BOOST_UPDATE",
      "BOOST_UPDATE_END_OK",
      "BOOST_UPDATE_END_KO",
      "ALIGN_CHECK_RESTART",
      "ALIGN_PERIODIC_CHECK",
      "ALIGN_CHECK",
      "ALIGN_ALL",
      "ALIGN_CLUSTER_I",
      "ALIGN_DONE",
      "ALIGN_DONE_SKIP_MEAS",
      "ALIGN_CHANGE",
      "ALIGN_CHANGE_SYNC",
      "ALIGN_WAIT",
      "ALIGN_RESTART",
      "ALIGN_CONF_SYNC",
      "CLOCK_REQ",
      "CLOCK_REQ_SYNC",
      "CLOCK_FORCE_REQ",
      "MEASPLAN_FAIL",
      "MEASPLAN_RSP_SYNC",
      "MEASURE_END_SYNC",
      "COMPUTATION_OK",
      "COMPUTATION_KO",
      "MEAS_FORCE",
      "SNR_PROBES_MEAS_FORCE",
      "PSD_UPDATE_FORCE",
      "MEAS_COLLECT_END_NO_LINES",
      "DISC_VERS_TO",
      "DISC_STATE_TO",
      "DOMAINS_SYNC_TO",
      "CLOCK_RSP_TO",
      "ALIGN_CHECK_TO",
      "ALIGN_CHANGE_TO",
      "ALIGN_SYNC_TO",
      "MEASPLAN_RSP_TO",
      "MEAS_COMPLETE_TO",
      "MEAS_COLLECT_TO",
      "MEASPLAN_CANCEL_TO",
      "MEASPLAN_RESTART",
      "MEASPLAN_BUILD",
      "MEASPLAN_CANCEL_END_SYNC",
      "BOOST_UPDATE_END_TO",
      "BOOST_ALG_RUN_TO",
      "ALIGN_CONFIG_TO",
      "RX_NETWORK_CHANGE_TRG",
      "RX_VERSION_RSP",
      "RX_STATE_RSP",
      "RX_DOMAIN_RSP",
      "RX_EMPTY_DOMAIN_RSP",
      "RX_NON_EMPTY_DOMAIN_RSP",
      "RX_CYCQUERY_RSP",
      "RX_CYCQUERY_RSP_KO",
      "RX_CYCQUERY_RSP_INV",
      "RX_CYCCHANGE_RSP",
      "RX_CYCCHANGE_RSP_KO",
      "RX_MEASPLAN_RSP",
      "RX_MEASPLAN_CANCEL_RSP",
      "RX_MEASPLAN_ERR_TRG",
      "RX_MEASURE_BGN_RSP",
      "RX_MEASURE_CFR_RSP",
      "RX_MEAS_COLLECT_ENG_TRG",
      "RX_TRAFFIC_AWARENESS_TRG",
      "RX_CLOCK_RSP",
      "RX_PSD_SHAPE_RSP",
      "RX_MEASURE_SNRPROBES_RSP",
      "RX_CDTA_RSP",
      "RX_ALIGN_MODE_RSP",
      "RX_ALIGN_MODE_RSP_KO",
      "RX_ALIGN_SYNC_LOST_TRG",
      "RX_ALIGN_CLUSTER_STOP_RSP",
      "RX_ALIGN_CLUSTER_STOP_TO",
      "RX_ALIGN_CLUSTERS_ALL_STOP_SYNC",
      "RX_ALIGN_CLUSTER_I_STOP_SYNC",
      "RX_ALIGN_CLUSTER_STOP_FAIL",
      "NETWORK_RX_REPORT_CHANGE",
      "NETWORK_DIFF_EP_CHANGE",
      "NETWORK_DIFF_DM_REM",
      "ALIVE_SOCK_CHECK",
      "ALIVE_SOCK_RSP"

  };

// Lookup array to convert frame opcodes -> FSM Events
static const t_VB_Comm_Event vb_EAI_lookup_events[VB_EA_OPCODE_LAST] =
{
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_MEASURE_PLAN_REQ        = 0x00,
    ENGINE_EV_RX_MEASPLAN_RSP,                   //VB_EA_OPCODE_MEASURE_PLAN_RESP       = 0x01,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_DOMAIN_REQ              = 0x02,
    ENGINE_EV_RX_DOMAIN_RSP,                     //VB_EA_OPCODE_DOMAIN_RESP             = 0x03,
    ENGINE_EV_RX_MEASURE_BGN_RSP,                //VB_EA_OPCODE_BGN_RESP                = 0x04,
    ENGINE_EV_RX_MEASURE_CFR_RSP,                //VB_EA_OPCODE_CFR_RESP                = 0x05,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_SNRPROBES_REQ           = 0x06,
    ENGINE_EV_RX_MEASURE_SNRPROBES_RSP,          //VB_EA_OPCODE_SNRPROBES_RESP          = 0x07,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_PSD_REQ                 = 0x08,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_PSD_RESP                = 0x09,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_PSD_SHAPE_CFG           = 0x0A,
    ENGINE_EV_RX_PSD_SHAPE_RSP,                  //VB_EA_OPCODE_PSD_SHAPE_CFM           = 0x0B,
    ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG,          //VB_EA_OPCODE_TRAFFIC_AWARENESS_TRG   = 0x0C,
    ENGINE_EV_RX_NETWORK_CHANGE_TRG,             //VB_EA_OPCODE_NETWORK_CHANGE_TRG      = 0x0D,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_VERSION_REQ             = 0x0E,
    ENGINE_EV_RX_VERSION_RSP,                    //VB_EA_OPCODE_VERSION_RESP            = 0x0F,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_VBDRIVER_STATE_REQ      = 0x10,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_VBDRIVER_STATE_RESP     = 0x11,
    ENGINE_EV_RX_MEASPLAN_ERR_TRG,               //VB_EA_OPCODE_MEASURE_END_TRG         = 0x12,
    ENGINE_EV_RX_STATE_RSP,                      //VB_EA_OPCODE_VBDRIVER_STATE_TRG      = 0x13,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_CLOCK_REQ               = 0x14,
    ENGINE_EV_RX_CLOCK_RSP,                      //VB_EA_OPCODE_CLOCK_RSP               = 0x15,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_CYCQUERY_REQ            = 0x16,
    ENGINE_EV_RX_CYCQUERY_RSP,                   //VB_EA_OPCODE_CYCQUERY_RSP            = 0x17,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_CYCCHANGE_REQ           = 0x18,
    ENGINE_EV_RX_CYCCHANGE_RSP,                  //VB_EA_OPCODE_CYCCHANGE_RSP           = 0x19,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_MEAS_COLLECT_REQ        = 0x1A,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_MEAS_COLLECT_RSP        = 0x1B,
    ENGINE_EV_RX_MEAS_COLLECT_END_TRG,           //VB_EA_OPCODE_MEAS_COLLECT_END        = 0x1C,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_MEASURE_PLAN_CANCEL_REQ = 0x1D,
    ENGINE_EV_RX_MEASPLAN_CANCEL_RSP,            //VB_EA_OPCODE_MEASURE_PLAN_CANCEL_RSP = 0x1E,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_CDTA_CFG                = 0x1F,
    ENGINE_EV_RX_CDTA_RSP,                       //VB_EA_OPCODE_CDTA_CFM                = 0x20,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_REDIRECT_REQ            = 0x21,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_ALIGN_MODE_REQ          = 0x22,
    ENGINE_EV_RX_ALIGNMODE_RSP,                  //VB_EA_OPCODE_ALIGN_MODE_RSP          = 0x23,
    ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG,            //VB_EA_OPCODE_ALIGN_SYNC_LOST_TRG     = 0x24,
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_ALIGN_STOP_CLUSTER_REQ  = 0x25,
    ENGINE_EV_RX_ALIGN_CLUSTER_STOP_RSP,         //VB_EA_OPCODE_ALIGN_STOP_CLUSTER_RESP = 0x26,
    ENGINE_EV_RX_NETWORK_CHANGE,                 //VB_EA_OPCODE_NETWORK_CHANGE_REPORT   = 0x27
    ENGINE_EV_LAST,                              //VB_EA_OPCODE_SOCKET_ALIVE_REQUEST    = 0x28
    ENGINE_EV_RX_ALIVE_SOCK_RSP                  //VB_EA_OPCODE_SOCKET_ALIVE_RESP       = 0x29
};

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VbEngineProcessInit()
{
  INT16U i,j;

  pthread_mutex_init(&vbDMsHistoryMutex, NULL);

  VbEngineMeasurePlanInit();

  VbEngineAlignCluster0Add();

  // Build queue name
  VbUtilQueueNameBuild(VB_ENGINE_PROCESS_MQ_NAME, VbEngineConfEngineIdGet(), vbEngineProcess.queueName);

  for(i=0; i<ENGINE_STT_LAST; i++)
  {
    for(j =0; j<ENGINE_EV_LAST; j++)
    {
      vbeFSMTransition[i][j] = (t_VbEngineFSMStep){ENGINE_STT_LAST, NULL};
    }
  }

  //        CURRENT_STATE                                              EVENT                                                        NEXT_STATE                                 TRANSITION_FUNCTION
  /*** ENGINE_STT_DISCONNECTED ***/
  /*
   * Driver is present in the system
   * but it is ignored.
   *
   * Expected events:
   * - ENGINE_EV_CONNECT: Driver is connected to engine. Move to next state.
   */
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_CONNECT]                    = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   VbeFSMConnectTransition};
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        NULL};
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        NULL};
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_CLOCK_REQ_SYNC]             = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        NULL};
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        NULL};
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        NULL};
  vbeFSMTransition[ENGINE_STT_DISCONNECTED]                       [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        NULL};

  /*** ENGINE_STT_DISCOVER_VERS_REQ ***/
  /*
   * DISCOVER
   * Version and Driver Id have been requested.
   * Wait response from driver.
   *
   * Expected events:
   * - ENGINE_EV_RX_VERSION_RSP: version and driver Id is received. Move to next state
   * - ENGINE_EV_DISC_VERS_TO: no response from driver. Try again.
   */
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_DOMAIN_RSP]              = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_DISC_VERS_TO]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   VbeFSMDiscoverVersionTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_VERSION_RSP]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  VbeFSMVersionRxAndDiscoverStateTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_VERS_REQ]                  [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_VERS_REQ,                   VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_DISCOVER_STATE_REQ ***/
  /*
   * DISCOVER
   * Driver state has been requested.
   * Wait response from driver.
   *
   * Expected events:
   * - ENGINE_EV_RX_STATE_RSP: state received from driver. Move to next state, Domain Wait, the driver shall now transmit a FULL doamin report so that the engine can start the alignment.
   * - ENGINE_EV_DISC_STATE_TO: no response from driver. Try again.
   */
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMStateRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_NETWORK_CHANGE_TRG]      = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_DOMAIN_RSP]              = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_DISC_STATE_TO]              = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  VbeFSMDiscoverStateTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_STATE_REQ]                 [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_STATE_REQ,                  VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_DISCOVER_DOMAIN_WAIT ***/
  /*
   * DISCOVER
   * Domains list is empty.
   * Wait until a network change is detected.
   *
   * Expected events:
   * - ENGINE_EV_NETWORK_CHANGE: network has changed. Start from scratch.
   */
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_CLOCK_REQ_SYNC]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_CHECK_RESTART]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_CHECK]                = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_DONE]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_DONE_SKIP_MEAS]       = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_PERIODIC_CHECK]       = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_ALIGNMODE_RSP_KO]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEASPLAN_RSP_SYNC]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEAS_COMPLETE_TO]           = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEAS_COLLECT_END_NO_LINES]  = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEASURE_END_SYNC]           = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_COMPUTATION_OK]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_COMPUTATION_KO]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_BOOST_UPDATE]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_BOOST_UPDATE_END_OK]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_BOOST_UPDATE_END_KO]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_CONF_SYNC]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_CYCQUERY_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_CLOCK_FORCE_REQ]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_WAIT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_ALIGN_CLUSTERS_ALL_STOP_SYNC] = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,     NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_CHANGE_SYNC]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_CHANGE]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_ALIGN_CLUSTER_I_STOP_SYNC]    = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,     NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEAS_FORCE]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_FAIL] = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO]   = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_CYCQUERY_RSP_KO]         = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEASPLAN_BUILD]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_MEASPLAN_CANCEL_END_SYNC]   = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY]         [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_DISCOVER_DOMAIN_WAIT ***/
  /*
   * DISCOVER
   * Domains list is empty.
   * Wait until a network change is detected.
   *
   * Expected events:
   * - ENGINE_EV_NETWORK_CHANGE: network has changed. Start from scratch.
   */
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_CLOCK_REQ_SYNC]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_CHECK_RESTART]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_CHECK]                = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_DONE]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_DONE_SKIP_MEAS]       = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_PERIODIC_CHECK]       = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_MEASPLAN_RSP_SYNC]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_MEAS_COMPLETE_TO]           = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_MEAS_COLLECT_END_NO_LINES]  = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_MEASURE_END_SYNC]           = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_COMPUTATION_OK]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_COMPUTATION_KO]             = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_BOOST_UPDATE]               = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_BOOST_UPDATE_END_OK]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_BOOST_UPDATE_END_KO]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_CONF_SYNC]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_CYCQUERY_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_CLOCK_FORCE_REQ]            = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_ALIGN_WAIT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                NULL};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_DISCOVER_DOMAIN_WAIT]               [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT,                VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_PREPARE_ALL ***/
  /*
   * ALIGNMENT
   * Stop clusters (the smallest one(s) will have their Tx stopped).
   * Wait to receive response from driver.
   *
   * Expected events:
   * - ENGINE_EV_RX_ALIGN_CLUSTER_STOP_RSP: response received from driver. Move to next step.
   * - ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO: no response from driver. Try again.
   * - ENGINE_EV_RX_ALIGN_CLUSTER_STOP_FAIL: error from driver. Try again.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_ALIGNMODE_RSP]           = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_ALIGNMODE_RSP_KO]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_CYCQUERY_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_CYCQUERY_RSP_KO]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_CYCQUERY_RSP_INV]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_RSP]  = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    VbeFSMAlignmentClusterAllStopSyncTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_FAIL] = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_MEASPLAN_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_MEASPLAN_CANCEL_RSP]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_ALL]              [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_PREPARE_I ***/
  /*
   * ALIGNMENT
   * Stop cluster (Tx is not stopped).
   * Wait to receive response from driver.
   *
   * Expected events:
   * - ENGINE_EV_RX_ALIGN_CLUSTER_STOP_RSP: response received from driver. Move to next step.
   * - ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO: no response from driver. Try again.
   * - ENGINE_EV_RX_ALIGN_CLUSTER_STOP_FAIL: error from driver. Try again.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_ALIGNMODE_RSP]           = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_ALIGNMODE_RSP_KO]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_CYCQUERY_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_CYCQUERY_RSP_KO]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_CYCQUERY_RSP_INV]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_MEASPLAN_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_MEASPLAN_CANCEL_RSP]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_CDTA_RSP]                = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_MEASURE_CFR_RSP]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_MEASURE_BGN_RSP]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_BOOST_UPDATE]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_RSP]  = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       VbeFSMAlignmentClusterIStopSyncTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_ALIGN_CLUSTER_STOP_FAIL] = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMAlignAllClustersTransition},
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_PREPARE_I]              [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC ***/
  /*
   * ALIGNMENT
   * Wait for clusters to be stopped.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_RX_ALIGN_CLUSTERS_ALL_STOP_SYNC: all drivers ready. Alignment set to G.hn mode. Go to next step (3rd).
   * - ENGINE_EV_CLOCK_FORCE_REQ: all drivers ready. Alignment set to Common clock mode. Jump to next step (CLOCK 1st step).
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_ALIGN_WAIT]                 = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_CLOCK_FORCE_REQ]            = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMClockRspWaitTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_ALIGN_CLUSTERS_ALL_STOP_SYNC] = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,               VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    VbeFSMAlignmentClusterAllStopSyncTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC] [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,    VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC ***/
  /*
   * ALIGNMENT
   * Wait for clusters to be stopped.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_RX_ALIGN_CLUSTER_I_STOP_SYNC: all drivers ready. Alignment set to G.hn mode. Go to next step (3rd).
   * - ENGINE_EV_CLOCK_FORCE_REQ: all drivers ready. Alignment set to Common clock mode. Jump to next step (CLOCK 1st step).
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_ALIGN_WAIT]                 = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_CLOCK_FORCE_REQ]            = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMClockRspWaitTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_ALIGN_CLUSTER_I_STOP_SYNC]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,               VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       VbeFSMAlignmentClusterIStopSyncTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC]    [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,       VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CONFIG ***/
  /*
   * ALIGNMENT
   * Alignment mode has been sent to driver.
   * Wait to receive response from driver.
   *
   * Expected events:
   * - ENGINE_EV_RX_ALIGNMODE_RSP: response received from driver. Move to next step.
   * - ENGINE_EV_ALIGN_CONF_TO: no response from driver. Try again.
   * - ENGINE_EV_RX_ALIGNMODE_RSP_KO: error from driver. Try again.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_ALIGN_CONF_TO]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_ALIGNMODE_RSP]           = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMAlignmentConfigSyncTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_ALIGNMODE_RSP_KO]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_CYCQUERY_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG]                   [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CONFIG_SYNC ***/
  /*
   * ALIGNMENT
   * Alignment response received from driver.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_ALIGN_WAIT: all drivers ready. Alignment set to G.hn mode. Go to next step (3rd).
   * - ENGINE_EV_CLOCK_FORCE_REQ: all drivers ready. Alignment set to Common clock mode. Jump to next step (CLOCK 1st step).
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_ALIGN_CONF_SYNC]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC,          NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_SYNC]              [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CONFIG_SYNC ***/
  /*
   * ALIGNMENT
   * Alignment response received from driver.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_ALIGN_WAIT: all drivers ready. Alignment set to G.hn mode. Go to next step (3rd).
   * - ENGINE_EV_CLOCK_FORCE_REQ: all drivers ready. Alignment set to Common clock mode. Jump to next step (CLOCK 1st step).
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_SYNC,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_ALIGN_WAIT]                 = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_CLOCK_FORCE_REQ]            = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMClockRspWaitTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC,          VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC]         [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC,          VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_WAIT_SYNC ***/
  /*
   * ALIGNMENT
   * Wait some time to apply G.hn alignment mode.
   *
   * Expected events:
   * - ENGINE_EV_CLOCK_FORCE_REQ: after some time, go to next step.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_CYCQUERY_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_ALIGNMODE_RSP]           = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_ALIGNMODE_RSP_KO]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_CLOCK_FORCE_REQ]            = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMClockRspWaitTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_WAIT_SYNC]                [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_WAIT_SYNC,                 VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_CLOCK_RSP_WAIT ***/
  /*
   * CLOCK
   * Clock has been requested.
   * Wait to receive the clock response from driver.
   *
   * Expected events:
   * - ENGINE_EV_RX_CLOCK_RSP: clock response received from driver. Move to next step.
   * - ENGINE_EV_CLOCK_RSP_TO: no response from driver. Try again.
   */
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      NULL};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      NULL};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_CLOCK_RSP_TO]               = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMClockRspTOTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_RSP_WAIT]                     [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_CLOCK_REQ_SYNC ***/
  /*
   * CLOCK
   * Clock response has been received from driver.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_CLOCK_REQ_SYNC: all drivers synchronised. Move to next step.
   */
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      VbeFSMClockRspTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      NULL};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      NULL};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_CLOCK_REQ_SYNC]             = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_CLOCK_REQ_SYNC]                     [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_REQ_SYNC,                      VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CHECK ***/
  /*
   * ALIGNMENT
   * CycQuery has been request to driver.
   * This frame is used to check alignment between G.hn domains.
   * Wait to receive response from driver.
   *
   * Expected events:
   * - ENGINE_EV_ALIGN_CHECK_RESTART: Restart alignment check process.
   * - ENGINE_EV_RX_CYCQUERY_RSP: expected response with alignment info. Move to next state.
   * - ENGINE_EV_RX_CYCQUERY_RSP_INV: unexpected response received. Ignore it.
   * - ENGINE_EV_RX_CYCQUERY_RSP_KO: error detected in EACycQuery request. Restart alignment check.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_ALIGN_CHECK_RESTART]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_CYCQUERY_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                VbeFSMAlignmentCheckSyncTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_CYCQUERY_RSP_INV]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_CYCQUERY_RSP_KO]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_ALIGN_CHECK_TO]             = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK]                    [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CHECK_SYNC ***/
  /*
   * ALIGNMENT
   * CycQuery has been received.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_ALIGN_CHECK: all drivers have received CycQuery response. Move to next step.
   * - ENGINE_EV_ALIGN_CHECK_RESTART: an error has been detected. Restart alignment check process.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_ALIGN_CHECK]                = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_ALIGN_CHECK_RESTART]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_ALIGN_CHECK_TO]             = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHECK_SYNC]               [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK_SYNC,                VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_DONE_SYNC ***/
  /*
   * ALIGNMENT
   * EACycQuery.rsp has been received from all drivers.
   * Wait for the alignment result.
   *
   * Expected events:
   * - ENGINE_EV_ALIGN_DONE: alignment is Ok. Move to measuring process.
   * - ENGINE_EV_ALIGN_DONE_SKIP_MEAS: alignment is Ok. Skip measuring and move to boosting process.
   * - ENGINE_EV_ALIGN_CHANGE: alignment is Ko. Perform alignment corrections and check again.
   * - ENGINE_EV_ALIGN_CHECK_RESTART: an error has been detected. Restart alignment check process.
   * - ENGINE_EV_ALIGN_RESTART: alignment mode is G.hn and domains are not aligned. Reconfigure domains to try again.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_ALIGN_DONE]                 = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_ALIGN_DONE_SKIP_MEAS]       = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_ALIGN_CHANGE]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMAlignmentChangeTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_ALIGN_CHECK_RESTART]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_ALIGN_RESTART]              = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CONFIG,                    VbeFSMAlignmentConfigTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_DONE_SYNC]                [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_DONE_SYNC,                 VbeFSMGenericFrameRxTransition};


  /*** ENGINE_STT_ALIGNMENT_CHANGE ***/
  /*
   * ALIGNMENT (Zero Cross Only, align mode = 0)
   * Alignment was incorrect.
   * Alignment changes requested to driver.
   * Wait for the response from driver.
   *
   * Expected events:
   * - ENGINE_EV_RX_CYCCHANGE_RSP: response received from driver. Move to next step.
   * - ENGINE_EV_RX_CYCCHANGE_RSP_KO: response received from driver but error detected. Try again.
   * - ENGINE_EV_ALIGN_CHANGE_TO: no response from driver. Try again.
   * - ENGINE_EV_ALIGN_CHECK_RESTART: an error has been detected. Restart alignment check process.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_CYCCHANGE_RSP]           = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               VbeFSMAlignmentChangeSyncTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_CYCCHANGE_RSP_KO]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMAlignmentChangeKoTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_ALIGN_CHANGE_TO]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMAlignmentChangeKoTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_ALIGN_CHECK_RESTART]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE]                   [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE,                    VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_ALIGNMENT_CHANGE_SYNC ***/
  /*
   * ALIGNMENT (Zero Cross Only, align mode = 0)
   * Alignment change response received from driver.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_ALIGN_CHANGE_SYNC: all drivers have received CycChange response.
   *   Move to next step and request the clock again, as it may be different after alignment change request.
   * - ENGINE_EV_ALIGN_CHECK_RESTART: an error has been detected. Restart alignment check process.
   */
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_ALIGN_CHANGE_SYNC]          = (t_VbEngineFSMStep){ENGINE_STT_CLOCK_RSP_WAIT,                      VbeFSMClockRspWaitTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_ALIGN_CHECK_RESTART]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMEmptyDomainTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_ALIGNMENT_CHANGE_SYNC]              [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHANGE_SYNC,               VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT ***/
  /*
   * MEASURING
   * Measure plan has been requested to driver.
   * Wait for driver response accepting the measure plan.
   *
   * Expected events:
   * - ENGINE_EV_RX_MEASPLAN_RSP: measure plan accepted by driver. Move to next step.
   * - ENGINE_EV_MEASPLAN_FAIL: error detected building the measure plan or reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASPLAN_ERR_TRG: error reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_MEASPLAN_RSP_TO: no response from driver. Request the same measure plan.
   */
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_MEASPLAN_FAIL]              = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_MEASPLAN_RSP_TO]            = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_MEASPLAN_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMMeasPlanCnfSyncTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMMeasPlanCancelSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT]        [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC ***/
  /*
   * MEASURING
   * Measure plan confirmation received.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_MEASPLAN_RSP_SYNC: measure plan confirmed by all drivers. Move to next step.
   * - ENGINE_EV_MEASPLAN_FAIL: error detected building the measure plan or reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASPLAN_ERR_TRG: error reported by driver. Signal this error to all drivers.
   */
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_MEASPLAN_FAIL]              = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_MEASPLAN_RSP_SYNC]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMMeasPlanCancelSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC]        [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,         VbeFSMGenericFrameRxTransition};


  /*** ENGINE_STT_MEASURING_MEASPLAN_END_WAIT ***/
  /*
   * MEASURING
   * Measure plan confirmed by all drivers.
   * Wait until measure plan finish.
   *
   * Expected events:
   * - ENGINE_EV_MEAS_COMPLETE_TO: measure plan has finished. Move to next step.
   * - ENGINE_EV_MEASPLAN_FAIL: error detected building the measure plan or reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASPLAN_ERR_TRG: error reported by driver. Signal this error to all drivers.
   */
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_MEASPLAN_FAIL]              = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_MEAS_COMPLETE_TO]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMMeasCollectMeasuresTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMMeasPlanCancelSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_MEASPLAN_END_WAIT]        [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,         VbeFSMGenericFrameRxTransition};


  /*** ENGINE_STT_MEASURING_COLLECT_MEAS ***/
  /*
   * MEASURING
   * Measure plan has finished.
   * Collect all measures from driver.
   *
   * Expected events:
   * - ENGINE_EV_MEAS_COLLECT_TO: timeout waiting for measures. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASURE_CFR_RSP: CFR measure received. Store it in datamodel.
   * - ENGINE_EV_RX_MEASURE_BGN_RSP: BGN measure received. Store it in datamodel.
   * - ENGINE_EV_RX_MEAS_COLLECT_END_TRG: measure collection process finished. Move to next step.
   * - ENGINE_EV_MEAS_COLLECT_END_NO_LINES: no lines to measure present in driver. Move to next step.
   * - ENGINE_EV_MEASPLAN_FAIL: error detected building the measure plan or reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASPLAN_ERR_TRG: error reported by driver. Signal this error to all drivers.
   */
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_MEASURE_CFR_RSP]         = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_MEASURE_BGN_RSP]         = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_MEAS_COLLECT_TO]            = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_MEASPLAN_FAIL]              = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_MEAS_COLLECT_END_TRG]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMMeasuresCollectDoneTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_MEAS_COLLECT_END_NO_LINES]  = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMMeasuresCollectDoneTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMMeasPlanCancelSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_COLLECT_MEAS]             [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_COLLECT_MEAS,              VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_MEASURING_END_SYNC ***/
  /*
   * MEASURING
   * Measures collected from driver.
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_MEASURE_END_SYNC: measure collection process finished in all drivers. Move to next step.
   * - ENGINE_EV_MEASPLAN_FAIL: error detected building the measure plan or reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASPLAN_ERR_TRG: error reported by driver. Signal this error to all drivers.
   */
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_MEASURE_END_SYNC]           = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_MEASPLAN_FAIL]              = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionMeasTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMMeasPlanCancelSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_END_SYNC]                 [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                   VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_MEASURING_CANCEL_WAIT ***/
  /*
   * MEASURING - Cancel step:
   * Likely reason is that an EP has appeared while doing Measure plan, cancel and do it again
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_MEASURE_END_SYNC: measure collection process finished in all drivers. Move to next step.
   * - ENGINE_EV_MEASPLAN_FAIL: error detected building the measure plan or reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASPLAN_ERR_TRG: error reported by driver. Signal this error to all drivers.
   */
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_MEASURE_END_SYNC]           = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_MEASPLAN_FAIL]              = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_MEASURE_CFR_RSP]         = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_MEASURE_BGN_RSP]         = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_MEASPLAN_CANCEL_RSP]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMMeasuresPlanCancelSyncTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_MEASPLAN_CANCEL_TO]         = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_MEASPLAN_RSP]            = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT]              [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT,               VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_MEASURING_CANCEL_WAIT ***/
  /*
   * MEASURING - Cancel step:
   * Likely reason is that an EP has appeared while doing Measure plan, cancel and do it again
   * Wait until all drivers reach this point.
   *
   * Expected events:
   * - ENGINE_EV_MEASURE_END_SYNC: measure collection process finished in all drivers. Move to next step.
   * - ENGINE_EV_MEASPLAN_FAIL: error detected building the measure plan or reported by driver. Signal this error to all drivers.
   * - ENGINE_EV_RX_MEASPLAN_ERR_TRG: error reported by driver. Signal this error to all drivers.
   */
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_MEASURE_END_SYNC]           = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_MEASPLAN_FAIL]              = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_MEASPLAN_ERR_TRG]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMMeasPlanFailTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_MEASPLAN_RESTART]           = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_MEASPLAN_CANCEL_RSP]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_END_SYNC,                  VbeFSMMeasuresPlanCancelSyncTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_MEASPLAN_CANCEL_END_SYNC]   = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          NULL};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_MEASPLAN_BUILD]             = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_MEASPLAN_CANCEL_TO]         = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC]         [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,          VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_BOOSTING_CALCULATE_SNR & Capacity ***/
  /*
   * BOOSTING
   * All expected measures collected.
   * Engine can compute the SNR and capacity for each G.hn node.
   *
   * Expected events:
   * - ENGINE_EV_COMPUTATION_OK: SNR and capacity calculation is OK. Move to next step.
   * - ENGINE_EV_COMPUTATION_KO: error calculating SNR or capacity. Start from scratch.
   */
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_COMPUTATION_OK]             = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  NULL};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_COMPUTATION_KO]             = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMDMRemBestActionBoostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionBoostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_MEAS_FORCE]                 = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_CALCULATE_SNR]             [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_CALCULATE_SNR,              VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_BOOSTING_WAIT_TRGS ***/
  /*
   * BOOSTING
   * SNR and capacity already calculated.
   * Wait for triggers to modify the resources allocation.
   *
   * Expected events:
   * - ENGINE_EV_MEAS_FORCE: launch a new measure plan.
   * - ENGINE_EV_SNR_PROBES_MEAS_FORCE: start a process to measure SNR with G.hn PROBE frames (debug purposes).
   * - ENGINE_EV_RX_MEASURE_SNRPROBES_RSP: SNR PROBE measure received.
   * - ENGINE_EV_ALIGN_PERIODIC_CHECK: start a new alignment check. Alignment shall be checked periodically.
   * - ENGINE_EV_BOOST_UPDATE: a new resources allocation is requested. Move to next step.
   */
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_MEAS_FORCE]                 = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_SNR_PROBES_MEAS_FORCE]      = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMSNRProbeForcedTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_MEASURE_SNRPROBES_RSP]   = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_BOOST_UPDATE]               = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                NULL};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_ALIGN_PERIODIC_CHECK]       = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_CHECK,                     NULL};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMDMRemBestActionBoostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionBoostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_TRGS]                 [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_BOOSTING_WAIT_UPDATE ***/
  /*
   * BOOSTING
   * A new resource allocation has been scheduled.
   * Wait for G.hn nodes to confirm the new allocation and
   * also wait until the new allocation applies.
   *
   * Expected events:
   * - ENGINE_EV_RX_PSD_SHAPE_RSP: new resources allocation confirmed by G.hn node.
   * - ENGINE_EV_BOOST_UPDATE_END_OK: new resources allocation has been applied successfully.
   *   Return to ENGINE_STT_BOOSTING_WAIT_TRGS state.
   * - ENGINE_EV_BOOST_UPDATE_END_KO: error detected while applying new resources allocation.
   *   Return to ENGINE_STT_BOOSTING_WAIT_TRGS state and wait to start from scratch.
   */
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_DISCONNECT]                 = (t_VbEngineFSMStep){ENGINE_STT_DISCONNECTED,                        VbeFSMDisconnectTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_STATE_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_CLOCK_RSP]               = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG]   = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG]     = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMAlignSyncLostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_PSD_SHAPE_RSP]           = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_CDTA_RSP]                = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_BOOST_UPDATE_END_OK]        = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  NULL};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_BOOST_UPDATE_END_KO]        = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_TRGS,                  NULL};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_ALIGN_ALL_CLUSTERS]         = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_ALL,               VbeFSMDriverAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_ALIGN_CLUSTER_I]            = (t_VbEngineFSMStep){ENGINE_STT_ALIGNMENT_PREPARE_I,                 VbeFSMDriverAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_NETWORK_DIFF_DM_REM]        = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMDMRemBestActionBoostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]     = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_NETWORK_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMGenericFrameRxTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_EMPTY_DOMAIN_RSP]        = (t_VbEngineFSMStep){ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,          VbeFSMDMRemBestActionBoostTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_MEAS_FORCE]                 = (t_VbEngineFSMStep){ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,         VbeFSMMeasPlanSendTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP]    = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_BOOSTING_WAIT_UPDATE]               [ENGINE_EV_RX_ALIVE_SOCK_RSP]          = (t_VbEngineFSMStep){ENGINE_STT_BOOSTING_WAIT_UPDATE,                VbeFSMGenericFrameRxTransition};

  /*** ENGINE_STT_UNDEFINED  - All drivers ***/
  /*
   * This is a special state useful to "broadcast" the received event to all FSM drivers.
   * Each driver will process the given event, running the transition actions related to their current state.
   * The stateHandler specified in this state will also be run once (if it is different than NULL).
   * The nextState of these transitions shall be set to ENGINE_STT_UNDEFINED.
   */
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_KILL_ALL]                        = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversKillAllTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_KILL]                            = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversKillSingleDriverTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_CLOCK_REQ]                       = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversClockReqTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_CLOCK_REQ_SYNC]                  = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversClockReqSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_CHECK_TO]                  = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignCheckTOTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_CHECK_RESTART]             = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignCheckRestartTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_CHECK]                     = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignCheckTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_CHANGE]                    = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignChangeTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_CHANGE_SYNC]               = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignChangeSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_DONE]                      = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignDoneTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_DONE_SKIP_MEAS]            = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignDoneSkipMeasTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_PERIODIC_CHECK]            = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignPeriodicCheckTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEASPLAN_RSP_SYNC]               = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversMeasPlanCnfSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEAS_COMPLETE_TO]                = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllMeasCollectMeasuresTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEASURE_END_SYNC]                = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllMeasuresCollectDoneSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_COMPUTATION_OK]                  = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllComputationOkTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_COMPUTATION_KO]                  = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllComputationKoTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_BOOST_ALG_RUN_TO]                = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllBoostAlgRunTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_BOOST_UPDATE]                    = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllBoostUpdateTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_BOOST_UPDATE_END_TO]             = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllBoostUpdateEndTOTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_BOOST_UPDATE_END_OK]             = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllBoostTimerSetAndAlgRunTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_BOOST_UPDATE_END_KO]             = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllBoostUpdateEndKoTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEAS_FORCE]                      = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllMeasPlanBuildTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_PSD_UPDATE_FORCE]                = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllPSDForceUpdateTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_WAIT]                      = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           NULL};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_CONF_SYNC]                 = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignConfigSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_CLOCK_FORCE_REQ]                 = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversClockForceReqTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_RESTART]                   = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignRestartTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_RX_ALIGN_CLUSTERS_ALL_STOP_SYNC] = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignClustersAllStopSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_RX_ALIGN_CLUSTER_I_STOP_SYNC]    = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignClusterIStopSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_ALL_CLUSTERS]              = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignAllClustersTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIGN_CLUSTER_I]                 = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAlignClusterITransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_NETWORK_DIFF_EP_CHANGE]          = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllEPChangeTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEASPLAN_RESTART]                = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllPlanRestartTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEASPLAN_BUILD]                  = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllMeasPlanReBuildTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEASPLAN_CANCEL_END_SYNC]        = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversMeasPlanCancelSyncTransition};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_MEASPLAN_CANCEL_TO]              = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           NULL};
  vbeFSMTransition[ENGINE_STT_UNDEFINED]                          [ENGINE_EV_ALIVE_SOCK_CHECK]                = (t_VbEngineFSMStep){ENGINE_STT_UNDEFINED,                           VbeFSMAllDriversAliveSocketReqTransition};

}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessDriversStartCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;

  if (driver == NULL)
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineEAProtocolDriverThreadStart(driver);

    if (result != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error creating External Agent Interface");
    }

    VbEngineProcessStateLogToFile(driver);
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessDriverCheckFSMState(t_VBDriver *thisDriver, void *args)
{
  t_vbEngineProcessFSMState  state_to_check;
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;

  if ((args == NULL) || (thisDriver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    state_to_check = (t_vbEngineProcessFSMState)args;

    if (state_to_check >= ENGINE_STT_LAST)
    {
      ret = VB_ENGINE_ERROR_INVALID_STATE;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ((thisDriver->FSMState != state_to_check) &&
        (thisDriver->FSMState != ENGINE_STT_DISCONNECTED) &&
        (thisDriver->FSMState != ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY))
    {
      ret = VB_ENGINE_ERROR_NOT_READY;

      VbLogPrintExt(VB_LOG_DEBUG, thisDriver->vbDriverID, "Driver (cluster %d) is not in desired state (%s -> %s)", thisDriver->clusterId,
          FSMSttToStrGet(thisDriver->FSMState), FSMSttToStrGet(state_to_check));
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessCheckDriversSync(t_vbEngineProcessFSMState state)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (state >= ENGINE_STT_LAST)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the same given state
    ret = VbEngineDatamodelDriversLoop(VbEngineProcessDriverCheckFSMState, (void *)state);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessCheckClustersSync(t_vbEngineProcessFSMState state, t_ClusterCast clusters)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (state >= ENGINE_STT_LAST)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U i;

    for(i=0; i<clusters.numCLuster; i++)
    {
      VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "CheckClusterSync id %d", clusters.list[i]);

      // Check if all drivers are in the same given state
      ret = VbEngineDatamodelClusterXDriversLoop(VbEngineProcessDriverCheckFSMState, clusters.list[i], (void *)state);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessQueueFlush(void)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  CHAR buffer[VB_ENGINE_PROCESS_MQ_MSG_SIZE];
  t_VBProcessMsg *vb_process_msg = NULL;
  BOOL flushing = TRUE;
  ssize_t numRead = 0;
  mqd_t queue_id;

  queue_id =  mq_open(vbEngineProcess.queueName, O_NONBLOCK | O_RDONLY);

  if (queue_id == (mqd_t) -1)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error opening process queue");
    result = VB_ENGINE_ERROR_QUEUE;
  }
  else
  {
    //Clear queue
    while(flushing)
    {
      numRead = mq_receive(queue_id, buffer, VB_ENGINE_PROCESS_MQ_MSG_SIZE, NULL);
      if (numRead <= 0)
      {
        flushing = FALSE;
      }
      else if (numRead != VB_ENGINE_PROCESS_MQ_MSG_SIZE)
      {
        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Cleanning queue numbytes received error");
      }
      else
      {
        vb_process_msg = (t_VBProcessMsg *)buffer;

        if (vb_process_msg->msg != NULL)
        {
          // Release attached message
          VbEAMsgFree(&(vb_process_msg->msg));
        }
      }
    }

    mq_close(queue_id);
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode DriverStateChange(t_VBDriver *driver, t_vbEngineProcessFSMState nextState, t_VB_Comm_Event event)
{
  t_VB_engineErrorCode      ret = VB_ENGINE_ERROR_NONE;
  t_vbEngineProcessFSMState current_driver_state;

  if ((driver == NULL) || (nextState >= ENGINE_STT_LAST) || (event >= ENGINE_EV_LAST))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    current_driver_state = driver->FSMState;
    driver->FSMState = nextState;

    if(current_driver_state != nextState)
    {
      // Update driver state in files
      VbEngineProcessStateLogToFile(driver);
    }

    // Filter logs for high frequency messages
    if ( ((event != ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG) && (event != ENGINE_EV_RX_MEASURE_SNRPROBES_RSP)) ||
        (current_driver_state != nextState))
    {
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "FSM Transition : STT_%-28s -> EV_%-23s -> STT_%-28s",
          FSMSttToStrGet(current_driver_state),
          FSMEvToStrGet(event),
          FSMSttToStrGet(nextState));
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AllDriversFSMEventDoCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode      ret = VB_ENGINE_ERROR_NONE;
  t_VbEngineAllDriversArgs *all_drivers_args = (t_VbEngineAllDriversArgs *)args;
  t_VB_Comm_Event           event;
  t_vbEngineProcessFSMState current_driver_state;
  t_VbEngineFSMStep         state_transition = {0, NULL};

  if ((driver == NULL) || (all_drivers_args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check valid event
    if (all_drivers_args->event >= ENGINE_EV_LAST)
    {
      ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Invalid event (%u)", all_drivers_args->event);
    }
    else
    {
      event = all_drivers_args->event;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check driver FSM state
    if (driver->FSMState >= ENGINE_STT_LAST)
    {
      ret = VB_ENGINE_ERROR_INVALID_STATE;
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Invalid driver state (%u)", driver->FSMState);
    }
    else
    {
      current_driver_state = driver->FSMState;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get state related transition
    state_transition = vbeFSMTransition[current_driver_state][event];

    if (state_transition.nextState == ENGINE_STT_LAST)
    {
      ret = VB_ENGINE_ERROR_INVALID_TRANSITION;
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Invalid transition (driverState %s; event %s)",
          FSMSttToStrGet(current_driver_state), FSMEvToStrGet(event));
    }
  }

  if ((ret == VB_ENGINE_ERROR_NONE) &&
      (state_transition.stateHandler != NULL))
  {
    t_VBProcessMsg msg;
    msg.msg = NULL;
    msg.vbCommEvent = event;
    msg.senderDriver = driver;
    msg.args = NULL;

    ret = state_transition.stateHandler(&msg);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Update current FSM state
    ret = DriverStateChange(driver, state_transition.nextState, event);
  }

  if ((ret == VB_ENGINE_ERROR_INVALID_TRANSITION) ||
      (ret == VB_ENGINE_ERROR_INVALID_STATE))
  {
    /*
     * Invalid state or transition for this driver.
     * However, we shall return ERROR_NONE to continue with next drivers.
     */
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/************************************************************************/

static t_VB_engineErrorCode VbEngineFSMEventDo(t_VBProcessMsg *msg)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VB_Comm_Event event;
  t_VbEngineFSMStep state_transition = {0, NULL};
  t_vbEngineProcessFSMState current_driver_state;

  // Check valid args
  if (msg == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Invalid args in VbEngineFSMEventDo");
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check valid event
    if (msg->vbCommEvent >= ENGINE_EV_LAST)
    {
      ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Invalid event (%u)", msg->vbCommEvent);
    }
    else
    {
      event = msg->vbCommEvent;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Increase related counter
    VbCounterIncrease(VbEngineDatamodelEvToCounter(msg->vbCommEvent));
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Event received from one specific driver?
    if (msg->senderDriver != NULL)
    {
      if (msg->senderDriver->FSMState >= ENGINE_STT_LAST)
      {
        ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
        VbLogPrintExt(VB_LOG_ERROR, msg->senderDriver->vbDriverID, "Invalid driver state (%u)", msg->senderDriver->FSMState);
      }
      else
      {
        current_driver_state = msg->senderDriver->FSMState;
      }
    }
    else
    {
      // If it was a event generated by process_engine (no driver associated)
      // we use Undefined state
      current_driver_state = ENGINE_STT_UNDEFINED;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get state related transition
    state_transition = vbeFSMTransition[current_driver_state][event];

    if (state_transition.nextState == ENGINE_STT_LAST)
    {
      ret = VB_ENGINE_ERROR_INVALID_TRANSITION;

      if (msg->senderDriver == NULL)
      {
        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Invalid transition (driverState %s; event %s)",
            FSMSttToStrGet(current_driver_state), FSMEvToStrGet(msg->vbCommEvent));
      }
      else
      {
        VbLogPrintExt(VB_LOG_ERROR, msg->senderDriver->vbDriverID, "Invalid transition (driverState %s; event %s)",
            FSMSttToStrGet(current_driver_state), FSMEvToStrGet(msg->vbCommEvent));
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (msg->senderDriver != NULL)
    {
      // Target is a single driver instance

      if (state_transition.stateHandler != NULL)
      {
        // Do transition actions for given driver
        ret = state_transition.stateHandler(msg);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Update current FSM state
        ret = DriverStateChange(msg->senderDriver, state_transition.nextState, msg->vbCommEvent);
      }
    }
    else
    {
      // Target are all drivers
      t_VbEngineAllDriversArgs all_drivers_args;

      // Run common handler for "ENGINE_STT_UNDEFINED" state (if != NULL)
      if (state_transition.stateHandler != NULL)
      {
        /*
         * Run the common handler.
         * It can return "VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP" to avoid
         * sending the event to all drivers.
         */
        ret = state_transition.stateHandler(msg);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Now run given event in all FSM drivers
        all_drivers_args.event = msg->vbCommEvent;

        if(msg->clusterCast.numCLuster == 0)
        {
          // Do transition actions for all drivers and change their FSM states
          ret = VbEngineDatamodelDriversLoop(AllDriversFSMEventDoCb, &all_drivers_args);
        }
        else
        {
          INT32U i;

          // Do transition actions for specific cluster and change their FSM states
          for(i = 0; i < msg->clusterCast.numCLuster ; i++)
          {
            VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "FSM Transition : STT_%-28s -> EV_%-23s (err %d) // cluster[%d] %d",
                FSMSttToStrGet(current_driver_state),
                FSMEvToStrGet(event),
                ret, i, msg->clusterCast.list[i]);
            ret = VbEngineDatamodelClusterXDriversLoop(AllDriversFSMEventDoCb, msg->clusterCast.list[i], &all_drivers_args);
          }
        }
      }

      if (event != ENGINE_EV_BOOST_ALG_RUN_TO)
      {
        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "FSM Transition : STT_%-28s -> EV_%-23s (err %d)",
            FSMSttToStrGet(current_driver_state),
            FSMEvToStrGet(event),
            ret);
      }

      if (ret == VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP)
      {
        /*
         * Expected error when the common handler does not want to
         * deliver the event to all drivers.
         */
        ret = VB_ENGINE_ERROR_NONE;
      }
    }

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      // Don't perform transition. Maybe we are in a sync state and we
      // wait until all drivers all in the same state

      if (msg->senderDriver == NULL)
      {
        VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Error %d running state transition (driverState %s; event %s)",
            ret, FSMSttToStrGet(current_driver_state), FSMEvToStrGet(msg->vbCommEvent));
      }
      else
      {
        VbLogPrintExt(VB_LOG_WARNING, msg->senderDriver->vbDriverID, "Error %d running state transition (driverState %s; event %s)",
            ret, FSMSttToStrGet(current_driver_state), FSMEvToStrGet(msg->vbCommEvent));
      }
    }
  }

  return ret;
}

/*******************************************************************/

static void VbEngineProcess( void *arg)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  struct mq_attr attr;
  ssize_t numRead = 0;
  t_VBProcessMsg *vb_process_msg = NULL;

  // Init data structures
  //
  VbEngineProcessInit();

  // Create and configure the message queue for this driver to receive messages
  // from the thread in charge of processing incomming TCP messages.
  //
  mq_unlink(vbEngineProcess.queueName);

  attr.mq_maxmsg  = VB_ENGINE_PROCESS_MQ_MAX_MSGS;
  attr.mq_msgsize = VB_ENGINE_PROCESS_MQ_MSG_SIZE;

  vbEngineProcess.rdWrQueueId =  mq_open(vbEngineProcess.queueName, O_CREAT | O_RDWR , 0666, &attr);

  if (vbEngineProcess.rdWrQueueId == (mqd_t) -1)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error to create mqueue. errno: [%s]", strerror(errno));
    result = VB_ENGINE_ERROR_QUEUE;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    vbEngineProcess.wrOnlyQueueId =  mq_open(vbEngineProcess.queueName, O_WRONLY);

    if (vbEngineProcess.wrOnlyQueueId == (mqd_t) -1)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error to create mqueue. errno: [%s]", strerror(errno));
      result = VB_ENGINE_ERROR_QUEUE;
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    VbEngineProcessQueueFlush();

    if (VbEngineConfServerConnModeGet())
    {
      // Start server thread
      VbEngineEAProtocolServerThreadStart();
    }
    else
    {
      // Start Drivers threads
      result = VbEngineDatamodelDriversLoop(VbEngineProcessDriversStartCb, NULL);
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    while (vbEngineProcess.running == TRUE)
    {
      numRead = mq_receive(vbEngineProcess.rdWrQueueId, vbEngineProcessMQBuffer, VB_ENGINE_PROCESS_MQ_MSG_SIZE, NULL);

      if (numRead < 0)
      {
        // Fatal error thread
        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Process mq_receive error. errno %s", strerror(errno));
      }
      else if (numRead != VB_ENGINE_PROCESS_MQ_MSG_SIZE)
      {
        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Process numbytes received error, NumRead %d", (int)numRead);
      }
      else
      {
        vb_process_msg = (t_VBProcessMsg *)vbEngineProcessMQBuffer;

        // Pass new event to FSM
        result = VbEngineFSMEventDo(vb_process_msg);

        if (vb_process_msg->msg != NULL)
        {
          // Release attached message
          VbEAMsgFree(&(vb_process_msg->msg));
        }
      }
    } // end while
  }

  // Clean up!

  VbCounterIncrease(VB_ENGINE_COUNTER_DISCONNECTED_STATUS);

  // Stop EA thread
  if (VbEngineConfServerConnModeGet())
  {
    // Stop server thread
    VbEngineEAProtocolServerThreadStop();
  }

  VbEngineProcessQueueFlush();
  mq_close(vbEngineProcess.wrOnlyQueueId);
  mq_close(vbEngineProcess.rdWrQueueId);
  mq_unlink(vbEngineProcess.queueName);
  pthread_mutex_destroy(&vbDMsHistoryMutex);
  free(vbDMsHistory.DMs);
  vbDMsHistory.DMs = NULL;
  vbDMsHistory.NumDMs = 0;

  return;
}

/************************************************************************/

static t_VB_engineErrorCode BoostAlgorithmRun(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  BOOLEAN              send_update = FALSE;
  BOOLEAN              qos_rate_change = FALSE;
  t_psdl2rArgs         psd_l2r_args;
  t_vbEngineQosRate    next_qos_rate;
  t_vbEngineQosRate    current_qos_rate;

  // Get current Rate set
  current_qos_rate = VbCdtaQosRateGet(clusterId);

  // Analyse CDTA info to extract the best Qos Rate / Bands per user to use
  ret = VbCdtaAnalyseRun(&next_qos_rate, clusterId);
  if (ret == VB_ENGINE_ERROR_NONE)
  {
    qos_rate_change = (current_qos_rate != next_qos_rate)? TRUE:FALSE;
    psd_l2r_args.qos = next_qos_rate;
    psd_l2r_args.psdBandsAllocation = VbEngineConfPSDBandAllocationGet();

    // Build PSD shapes as requested by CDTA algorithm
    ret = VbEngineDatamodelClusterXAllNodesLoop(VbEngineLeftToRightPSDShapeRun, clusterId, (void*)&psd_l2r_args);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    send_update = VbEnginePsdShapeSendCheck(clusterId);

    if ((send_update == TRUE) || (qos_rate_change == TRUE))
    {
      VbLogPrint(VB_LOG_INFO, "Algorithm send Update %u, qos rate change %u", send_update, next_qos_rate);

      // Send event to all to wait until boost update is finished
      ret = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_BOOST_UPDATE, NULL, clusterId);

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        BOOLEAN force_channel_estimation;

        // Force channel estimation only if some bands have changed, not in case of only a rate change
        force_channel_estimation = (send_update == TRUE)?TRUE:FALSE;

        // send PSD shape & Qos Rate packet (CDTA packet)
        ret = VbEngineCDTAConfigureAll(force_channel_estimation, next_qos_rate, clusterId);
      }
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    // Finish boost update with error
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_BOOST_UPDATE_END_KO, NULL,clusterId);
  }

  return ret;
}

/************************************************************************/

static t_VB_engineErrorCode VbEngineTimeToFinishGet(INT32U *timeToFinish, struct timespec *applyTs)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  struct timespec     current_ts;

  if ((timeToFinish == NULL) || (applyTs == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    int     err;

    // Get current time
    err = clock_gettime(CLOCK_REALTIME, &current_ts);

    if (err != 0)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d (%s) getting current time", err, strerror(errno));
      result = VB_ENGINE_ERROR_CLOCK;
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    INT64S  time_to_finish;

    // Calculate time to finish
    time_to_finish = VbUtilElapsetimeTimespecMs(current_ts, *applyTs);

    *timeToFinish = (INT32U)time_to_finish + VB_ENGINE_CHANGES_APPLY_MARGIN;
  }

  return result;
}

/*
 ************************************************************************
 ** FSM Transition handler functions
 ************************************************************************
 */

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversKillAllTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * The engine is going to be closed.
   * Remove all drivers from system.
   *
   * ACTIONS:
   * - Stop all EA threads.
   * - Remove all drivers from drivers list.
   * - Stop engine thread.
   */

  // Stop all EA threads
  VbEngineEAProtocolAllDriversThreadStop();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Releasing drivers list...");

  // Delete cluster list and associated memory
  VbEngineCltListClustersDestroy();

  // Release all drivers list
  VbEngineDrvListDriversDestroy();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Drivers list released!");

  // Stop Engine Process thread
  VbEngineProcessThreadRunningSet(FALSE);

  if (error == VB_ENGINE_ERROR_NONE)
  {
    /*
     * Abort delivering ENGINE_EV_KILL_ALL event to all drivers.
     * This event shall be processed only by this function.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

t_VB_engineErrorCode VbeUpdateDMsHistory(INT8U *MAC, INT32U clusterId, t_alignRole role, BOOL isAdding)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_VBDMsHistory    *history;

  pthread_mutex_lock(&(vbDMsHistoryMutex));

  history = &vbDMsHistory;

  if (MAC == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U i;

    for (i = 0; i < history->NumDMs; i++)
    {
      if (!memcmp(history->DMs[i].MAC, MAC, ETH_ALEN))
      {
        history->DMs[i].clusterId = clusterId;
        history->DMs[i].role = role;
        break;
      }
    }

    if (i == history->NumDMs)
    {
      history->DMs = realloc(history->DMs, (history->NumDMs + 1) * sizeof(*history->DMs));
      memcpy(history->DMs[i].MAC, MAC, ETH_ALEN);
      history->DMs[i].clusterId = clusterId;
      history->DMs[i].role = role;
      history->NumDMs++;
    }

    if (isAdding == FALSE && role == VB_ALIGN_ROLE_REF)
    {
      t_VBCluster *cluster;

      ret = VbEngineClusterByIdGet(clusterId, &cluster);

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        if (cluster->clusterInfo.numLines > 1)
          history->DMs[i].role = VB_ALIGN_ROLE_SLAVE;
      }
    }

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "NumDMs in History %d.", history->NumDMs);

  }

  pthread_mutex_unlock(&(vbDMsHistoryMutex));

  return ret;
}

/************************************************************************/

t_VB_engineErrorCode VbeFindDMsClusterIdByMAC(INT8U *DMsMAC, INT32U *clusterId, t_alignRole *role)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_VBDMsHistory    *history;
  INT32U i;

  pthread_mutex_lock(&(vbDMsHistoryMutex));

  history = &vbDMsHistory;

  if (DMsMAC == NULL || history == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VB_ENGINE_ERROR_NOT_FOUND;

    for (i = 0; i < history->NumDMs; i++)
    {
      if (!memcmp(history->DMs[i].MAC, DMsMAC, ETH_ALEN))
      {
         if (clusterId)
           *clusterId = history->DMs[i].clusterId;
         if (role)
           *role = history->DMs[i].role;
         ret = VB_ENGINE_ERROR_NONE;
         break;
      }
    }
  }

  pthread_mutex_unlock(&(vbDMsHistoryMutex));

  return ret;
}

/************************************************************************/

static t_VB_engineErrorCode UpdateDMsList(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_VBDMsHistory    *history = (t_VBDMsHistory *)args;

  if (driver == NULL || domain == NULL || history == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbeUpdateDMsHistory(domain->dm.MAC, driver->clusterId, domain->dm.nodeAlignInfo.role, FALSE);
  }

  return ret;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversKillSingleDriverTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBDriver          *driver;

  /*
   * Given driver has been disconnected and shall be removed from system.
   *
   * ACTIONS:
   * - Stop EA thread.
   * - Remove driver from drivers list.
   * - Release driver allocated memory (only in SERVER mode).
   * - Send an event to notify the network change.
   */

  if ((processMsg == NULL) || (processMsg->args == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error, wrong arguments");
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Get driver to kill from generic "args" pointer
    driver = (t_VBDriver *)processMsg->args;

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Killing driver %s",  driver->vbDriverID);

    VbEngineDatamodelDomainsLoop(driver, UpdateDMsList, &vbDMsHistory);

    // Stop related EA thread
    VbEngineEAProtocolDriverThreadStop(driver, NULL);

    // Remove driver from drivers list
    VbEngineDrvListRemoveDriver(driver);

    // If Engine is in SERVER mode, release its related memory
    VbEngineDatamodelDriverDel(&driver);

    /*
     * Do not access to driver pointer beyond this point, as driver
     * was released in DriverKill
     */

    if (VbEngineProcessThreadRunningGet())
    {
      INT16U num_drivers_in_cluster_x;

      // Send ALIGN CLUSTER I event if there are still drivers in this cluster
      // Remove cluster otherwise
      num_drivers_in_cluster_x = VbEngineDataModelNumDriversInCLusterXGet(processMsg->clusterCast.list[0]);
      if(num_drivers_in_cluster_x > 0)
      {
        error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, processMsg->clusterCast.list[0]);
      }
      else
      {
        t_VBCluster         *cluster;

        error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
        if(error == VB_ENGINE_ERROR_NONE)
        {
          // Stop and release timer
          VbEngineTimeoutStop(&(cluster->timeoutCnf));
        }

        if(error == VB_ENGINE_ERROR_NONE)
        {
          if(processMsg->clusterCast.list[0] != 0)
          {
            VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Remove cluster %d", processMsg->clusterCast.list[0]);
            error = VbEngineAlignClusterRemove(processMsg->clusterCast.list[0]);
          }
        }
      }
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    /*
     * Abort delivering ENGINE_EV_KILL event to all drivers.
     * This event shall be processed only by this function.
     * Driver being killed will go to the DISCONNECT state from here
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMDisconnectTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBDriver *driver;

  /*
   * The driver has been disconnected from EA interface and
   * communication with driver is lost.
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Write current status to files.
   * - If engine is in server mode:
   *     - Kill the driver and remove it from system.
   * - If engine is in client mode:
   *     - Inform all drivers that a driver has been removed
   *     from system and we shall start from scratch.
   *     - Clear the domains list of given driver.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error, wrong arguments");
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    driver = processMsg->senderDriver;
    // Stop previous timer (if any)
    VbEngineDriverTimeoutStop(driver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    VbCounterIncrease(VB_ENGINE_COUNTER_COMMUNICATION_DRIVER_LOST);

    // Set remote state as disconnected in files
    VbLogSaveStringToTextFile(processMsg->senderDriver->remoteStateFileName, "w+", "VBDriverState=DISCONNECTED\n");

    if (VbEngineProcessThreadRunningGet())
    {
      if (driver->vbEAConnDesc.type == VB_EA_TYPE_SERVER_CONN)
      {
        // Send kill event to completely remove this driver from engine
        error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_KILL, driver, driver->clusterId);
      }
      else
      {
        // Send kill event to completely remove this driver from engine
        error = VbEngineProcessAllDriversEvSend(ENGINE_EV_KILL, NULL);

        // Copy remote state to internal structure
        strncpy(driver->remoteState, (CHAR *)"DISCONNECTED", DRIVER_REMOTE_STATE_MAX_LEN);

        // Force '\0' character
        driver->remoteState[DRIVER_REMOTE_STATE_MAX_LEN - 1] = '\0';
      }
    }

    if (driver->vbEAConnDesc.type != VB_EA_TYPE_SERVER_CONN)
    {
      // If Engine is in CLIENT mode, remove domains list from driver
      VbEngineDatamodelListDomainsDestroy(driver);
    }

    VbLogPrintExt(VB_LOG_INFO, processMsg->senderDriver->vbDriverID, "Disconnected");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMConnectTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * A connection with driver has been established through
   * EA interface.
   *
   * ACTIONS:
   * - Inform all drivers that a new driver has joined the
   * system and we shall start from scratch.
   * - Request the Driver version and Driver Id.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error, wrong arguments");
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Send Discover Version request
    error = VbeFSMDiscoverVersionTransition(processMsg);

    VbLogPrintExt(VB_LOG_INFO, processMsg->senderDriver->vbDriverID, "Connected");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignSyncLostTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Alignment sync lost detected
   *
   * ACTIONS:
   * - Process message (for debug purposes)
   * - Do same actions than a network change
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error, wrong arguments");
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Process frame
    error = VbEngineFrameRxProcess(processMsg);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    VbEngineClusterSyncLostSet(processMsg->senderDriver->clusterId, TRUE);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Send ALIGN I to all drivers in cluster Id
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMDiscoverVersionTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Request version and driver Id from driver.
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Request the Driver version and Driver Id.
   * - Launch a timer
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop previous timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Send request
    error = VbEngineProcessVersionRequest(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Launch timer
    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_DISC_VERS_TO, VB_ENGINE_REQUEST_TIMEOUT, "DiscoverVersTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMVersionRxAndDiscoverStateTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Request current state from driver.
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Process EAVersion.rsp frame.
   * - Request the current driver state.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Process rx frame
    error = VbeFSMGenericFrameRxTransition(processMsg);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Launch discover domain request
    error = VbeFSMDiscoverStateTransition(processMsg);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMDiscoverStateTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Request current state from driver.
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Request the current driver state.
   * - Launch timer.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Send request
    error = VbEngineProcessDriverStateRequest(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Launch timer
    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_DISC_STATE_TO, VB_ENGINE_REQUEST_TIMEOUT, "DiscoverStateTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMClockRspWaitTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Clock has been requested.
   * Wait to receive EAClock.rsp frame from driver.
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Launch a timer
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Launch timer
    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_CLOCK_RSP_TO,
                                       MAX(processMsg->senderDriver->transactionMinTime, VB_ENGINE_REQUEST_TIMEOUT),
                                       "ClockRspTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMClockRspTOTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Timeout waiting for clock response from driver.
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Send an event to request the clock from all drivers.
   * - Launch a timer
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Send a global event to request clock
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_CLOCK_REQ, NULL, processMsg->senderDriver->clusterId);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Launch timer
    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_CLOCK_RSP_TO,
                                       MAX(processMsg->senderDriver->transactionMinTime, VB_ENGINE_REQUEST_TIMEOUT),
                                       "ClockRspTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMClockRspTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * EAClock.rsp frame received
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Process EAClock.rsp frame.
   * - Send an event to check all drivers states.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Process rx frame
    error = VbeFSMGenericFrameRxTransition(processMsg);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Send a global event to check drivers state
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_CLOCK_REQ_SYNC, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMStateRxTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * EAVbDriverState.rsp frame received
   *
   * ACTIONS:
   * - Clear pending timers.
   * - Process EAVbDriverState.rsp frame.
   * - Request the driver domains list.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Process rx frame
    error = VbeFSMGenericFrameRxTransition(processMsg);
  }

  return error;
}


/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversClockReqSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;
  t_VBCluster              *cluster;
  INT32U                    max_resp_time;

  /*
   * Check that all drivers are waiting in ENGINE_STT_CLOCK_REQ_SYNC state.
   * Start alignment check process.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Request a CycQuery from all drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_CLOCK_REQ_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_CLOCK_REQ_SYNC event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);

    if(error == VB_ENGINE_ERROR_NONE)
    {
      // The process was started from scratch, perform measure plan after alignment
      cluster->skipMeasPlan = FALSE;
    }
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClockAllDriversNTPDeviationCheck();
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // All drivers are in the desired state, request CycQuery to check alignment
    error = VbEngineAlignCycQueryRequest(processMsg->clusterCast.list[0]);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Get max resp time from drivers
    error = VbEngineClusterMinTimeDriverGet(processMsg->clusterCast.list[0], &max_resp_time);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    // Configure cluster Ids to trigger
    cluster->timeoutCnf.clusterCast.numCLuster = 1;
    cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

    // Configure a timer
    error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_ALIGN_CHECK_TO,
                                 MAX(max_resp_time, VB_ENGINE_ALIGN_CHECK_TIMEOUT), FALSE, "AlignCheckTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversClockReqTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;

  /*
   * Request a clock update from all drivers.
   *
   * ACTIONS:
   * - Request clock update.
   */

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // YYRR Review Force a clock update of all drivers
    error = VbEngineClockAllDriversUpdate();

    if (error == VB_ENGINE_ERROR_NONE)
    {
      /*
       * Abort delivering ENGINE_EV_CLOCK_REQ event to all drivers.
       * This event shall be processed only by this function.
       */
      error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversClockForceReqTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error;

  /*
   * Request a clock update from all drivers.
   *
   * ACTIONS:
   * - Request clock update.
   */

  // Force a clock update of all drivers
  error = VbEngineClockClusterUpdate(processMsg->clusterCast.list[0]);

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAliveSocketReqTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;

  /*
   * Request a socket check from all drivers.
   *
   * ACTIONS:
   * - Request socket check.
   */

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // YYRR Review Force a clock update of all drivers
    error = VbEngineSocketAliveCheckReq();
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignCheckTOTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;

  /*
   * Restart alignment process.
   *
   * ACTIONS:
   * - Request CycQuery from all drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbeFSMAllDriversAlignCheckRestartTransition(processMsg);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignCheckRestartTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;
  t_VBCluster              *cluster;
  INT32U                    max_resp_time;
  /*
   * Restart alignment process.
   *
   * ACTIONS:
   * - Request CycQuery from all drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    // Retry CycQuery to check alignment
    error = VbEngineAlignCycQueryRequest(processMsg->clusterCast.list[0]);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Get max resp time from drivers
    error = VbEngineClusterMinTimeDriverGet(processMsg->clusterCast.list[0], &max_resp_time);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Configure cluster Ids to trigger
    cluster->timeoutCnf.clusterCast.numCLuster = 1;
    cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

    // Configure a timer
    error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_ALIGN_CHECK_TO,
                                 MAX(max_resp_time, VB_ENGINE_ALIGN_CHECK_TIMEOUT), FALSE, "AlignCheckTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignCheckTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;
  t_vbEngineNumNodes        num_nodes;
  t_VBCluster              *cluster;

  /*
   * Check that all drivers are waiting in ENGINE_STT_ALIGNMENT_CHECK_SYNC state.
   * If there are not complete lines in the system, return an error and wait for a network change.
   * Check alignment between all G.hn nodes.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Stop Align check timer
   * - Check number of complete lines and return an error if 0 complete lines.
   * - Check alignment and send proper event depending on the result.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_ALIGNMENT_CHECK_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Get number of nodes
    error = VbEngineDataModelNumNodesInClusterXGet(processMsg->clusterCast.list[0], &num_nodes);
  }

#if 0
  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Ensure that there is at least 1 complete line (DM + EP) in the whole system
    if (num_nodes.numCompleteLines == 0)
    {
      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "No complete lines detected (number of DMs %u; number of EPs %u) in cluster %d, waiting...",
          num_nodes.numDms, num_nodes.numEps, processMsg->clusterCast.list[0]);
      error = VB_ENGINE_ERROR_NOT_READY;
    }
  }
#endif

  if (error == VB_ENGINE_ERROR_NONE)
  {
    BOOLEAN aligned;
    INT32U new_cluster_id = 0;
    t_vbAlignClusterBuildInfo  cluster_build_info;

    // Check alignment between all domains
    aligned = VbEngineAlignCheck(processMsg->clusterCast.list[0]);
    if (aligned == TRUE)
    {
      // All lines in cluster 0 or i are aligned
      // cluster 0 -> Create new cluster
      // cluster i -> Update existing one
      // Send Align Done o move to next step (Measure Plan or boosted if the align came from a periodic check

      // Ensure that there is at least 1 complete line (DM + EP) in the whole system
      if (num_nodes.numCompleteLines == 0)
      {
        // Alignment done
        VbEngineAlignDoneNotify(processMsg->clusterCast.list[0]);

        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "No complete lines detected (number of DMs %u; number of EPs %u) in cluster %d, waiting...",
            num_nodes.numDms, num_nodes.numEps, processMsg->clusterCast.list[0]);
        error = VB_ENGINE_ERROR_NOT_READY;
      }

      if(error == VB_ENGINE_ERROR_NONE)
      {
        if ( (cluster->skipMeasPlan == TRUE) && (cluster->clusterInfo.clusterId > 0) )
        {
          // Comes from a periodic align check

          // Alignment done
          VbEngineAlignDoneNotify(processMsg->clusterCast.list[0]);
          error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_DONE_SKIP_MEAS, NULL, processMsg->clusterCast.list[0]);
        }
        else
        {
          error = VbEngineAlignClusterAlignInfoGet(&cluster_build_info, processMsg->clusterCast.list[0]);
          if (error == VB_ENGINE_ERROR_NONE)
          {
            if(processMsg->clusterCast.list[0] == 0)
            {
              // Newly aligned cluster is cluster 0 -> Create "real" cluster

              // Create cluster and Tag lines
              error = VbEngineAlignClusterAdd(processMsg->clusterCast.list[0], &new_cluster_id, &cluster_build_info);
              if(error == VB_ENGINE_ERROR_NONE)
              {
                // Alignment done
                VbEngineAlignDoneNotify(new_cluster_id);

                // New Broadcast cluster
                processMsg->clusterCast.list[0] = new_cluster_id;
                error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_DONE, NULL, new_cluster_id);
              }
            }
            else
            {
              // Aligned cluster is an existing cluster -> reuse cluster Id
              error = VbEngineAlignClusterUpdate(processMsg->clusterCast.list[0], &cluster_build_info);
              if(error == VB_ENGINE_ERROR_NONE)
              {
                VbEngineAlignDoneNotify(processMsg->clusterCast.list[0]);
                error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_DONE, NULL, processMsg->clusterCast.list[0]);
              }
            }
          }
        }

        if (error == VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Cluster %d %d successfully aligned (skipMeasPlan %u)",
                        processMsg->clusterCast.list[0], new_cluster_id, cluster->skipMeasPlan);
        }
        else
        {
          VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Cluster %d %d unsuccessfully aligned, Error %l",
                        processMsg->clusterCast.list[0], new_cluster_id, error);

        }
      }
    }
    else
    {
      t_VBAlignmentMode alignment_mode;
      INT32U            current_cluster_id = processMsg->clusterCast.list[0];

      VbEngineConfAlignmentModeGet(&alignment_mode);

      if (alignment_mode == VB_ALIGN_MODE_GHN)
      {
        t_vbAlignNextStep next = VB_ENGINE_ALIGNMENT_NEXT_NONE;

        // Not all lines in cluster 0 or i are aligned
        // Call VbEngineAlignGhnNextTry
        // Result could be:
        //  1. Not aligned and candidate relay still to be tried
        //    -> Send RESTART
        //  2. Part of a cluster Id is aligned and nothing else can be done to try to align the rest
        //    -> Create new cluster
        //    -> Send DONE

        // Alignment "fail", check if cluster needs to be created and DONE and/or RESTART sent
        error = VbEngineAlignGhnNextTry(&next, &cluster_build_info, current_cluster_id);
        if(error == VB_ENGINE_ERROR_NONE)
        {
          INT32U num_clusters_to_cast = 0;

          if(next & (1 << VB_ENGINE_ALIGNMENT_NEXT_DONE_EVT))
          {
            error = VbEngineAlignClusterAlignInfoGet(&cluster_build_info, current_cluster_id);
            if(error == VB_ENGINE_ERROR_NONE)
            {
              // Create cluster and Tag lines
              error = VbEngineAlignClusterAdd(current_cluster_id, &new_cluster_id, &cluster_build_info);
            }

            if(error == VB_ENGINE_ERROR_NONE)
            {
              // Send done to newly formed cluster
              error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_DONE, NULL, new_cluster_id);
              if(error == VB_ENGINE_ERROR_NONE)
              {
                // Alignment done
                VbEngineAlignDoneNotify(new_cluster_id);

                // Build list of clusters to process the driver individual processing of this event
                processMsg->clusterCast.list[num_clusters_to_cast] = new_cluster_id;
                num_clusters_to_cast++;
                processMsg->clusterCast.numCLuster = num_clusters_to_cast;
              }
            }
          }

          if(next & (1 << VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT))
          {
            // Send Restart to remaining lines in cluster
            error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_RESTART, NULL, current_cluster_id);

            // Build list of clusters to process the driver individual processing of this event
            processMsg->clusterCast.list[num_clusters_to_cast] = current_cluster_id;
            num_clusters_to_cast++;
            processMsg->clusterCast.numCLuster = num_clusters_to_cast;
          }
        }
        else
        {
          VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Restart Error = %l", error);
          error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_RESTART, NULL, current_cluster_id);
        }
      }
      else
      {
        // Alignment fail, apply changes
        error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CHANGE, NULL, current_cluster_id);
      }

      VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Domains not yet aligned");
    }
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_ALIGN_CHECK event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignChangeTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;

  /*
   * Check that all drivers are waiting in ENGINE_STT_ALIGNMENT_DONE_SYNC state.
   * Start the alignment adjustment process.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Select an alignment reference.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_ALIGNMENT_DONE_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Do measure plan after alignment adjusts
    if (error == VB_ENGINE_ERROR_NONE)
    {
      t_VBCluster *cluster;

      error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
      if(error == VB_ENGINE_ERROR_NONE)
      {
        // The process was started from scratch, perform measure plan after alignment
        cluster->skipMeasPlan = FALSE;
      }
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Calculate reference
    error = VbEngineAlignRefCalc(processMsg->clusterCast.list[0]);

    if (error != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d calculating alignment reference", error);
    }
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_ALIGN_CHANGE event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignChangeSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;

  /*
   * Check that all drivers are waiting in ENGINE_STT_ALIGNMENT_CHANGE_SYNC state.
   * Alignment has been modified, so request a new clock update from all drivers.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Request a clock update from all drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckDriversSync(ENGINE_STT_ALIGNMENT_CHANGE_SYNC);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_ALIGN_CHANGE_SYNC event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Force a clock update of all drivers
    error = VbEngineClockAllDriversUpdate();
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignClustersAllStopSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;

  /*
   * Check that all drivers are waiting in ENGINE_STT_ALIGNMENT_CHANGE_SYNC state.
   * Alignment has been modified, so request a new clock update from all drivers.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Request a clock update from all drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all Drivers are in the desired state
    error = VbEngineProcessCheckDriversSync(ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_STT_ALIGNMENT_CLUSTERS_STOP_SYNC event individually to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Make sure SNR threads are stopped
    VbCalculateAllSNRAndCapacityStop();
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Set all drivers with cluster Id 0
    error = VbEngineAlignAllDriversClusterTag(0);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Remove all clusters but 0
    VbEngineCltListClustersNonZeroDestroy();
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Reset hasBeenCandidate flag
    error = VbEngineAlignmentHasBeenCandidateReset(0);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Build engine Conf to align all drivers in cluster Id 0
    error = VbEngineEngineConfReqBuild(0, TRUE);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignClusterIStopSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;

  /*
   * Check that all drivers are waiting in ENGINE_STT_ALIGNMENT_CHANGE_SYNC state.
   * Alignment has been modified, so request a new clock update from all drivers.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Request a clock update from all drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all Drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_STT_ALIGNMENT_CLUSTERS_STOP_SYNC event individually to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Make sure SNR threads are stopped
    VbCalculateSNRAndCapacityStop(processMsg->clusterCast.list[0]);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Reset hasBeenCandidate flag
    error = VbEngineAlignmentHasBeenCandidateReset(processMsg->clusterCast.list[0]);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Build engine Conf to align all drivers in cluster Id 0
    error = VbEngineEngineConfReqBuild(processMsg->clusterCast.list[0], TRUE);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignDoneTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Check that all drivers are waiting in ENGINE_STT_ALIGNMENT_DONE_SYNC state.
   * Alignment is OK.
   * Start the measuring process.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Build a new measure plan to execute.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_ALIGNMENT_DONE_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbeFSMAllMeasPlanBuildTransition(processMsg);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_ALIGN_DONE event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignDoneSkipMeasTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Check that all drivers are waiting in ENGINE_STT_ALIGNMENT_DONE_SYNC state.
   * Alignment is OK.
   * Skip the measuring process.
   * Start the boosting process.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Start boosting process.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_ALIGNMENT_DONE_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbeFSMAllBoostTimerSetAndAlgRunTransition(processMsg);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_ALIGN_DONE_SKIP_MEAS event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignPeriodicCheckTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;
  t_VBCluster              *cluster;
  /*
   * Start a new alignment check process.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Skip measure plan when alignment process finishes.
   * - Request CycQuery from all drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_BOOSTING_WAIT_TRGS, processMsg->clusterCast);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    if (error == VB_ENGINE_ERROR_NONE)
    {
      t_VBCluster *cluster;

      error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
      if(error == VB_ENGINE_ERROR_NONE)
      {
        // This is a periodic alignment check, avoid running a new measure plan
        cluster->skipMeasPlan = TRUE;
      }
    }

    if (error == VB_ENGINE_ERROR_NONE)
    {
      // Request CycQuery to check alignment
      error = VbEngineAlignCycQueryRequest(processMsg->clusterCast.list[0]);
    }

    if (error == VB_ENGINE_ERROR_NONE)
    {

      // Configure a timer
      cluster->timeoutCnf.clusterCast.numCLuster = 1;
      cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

      error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_ALIGN_CHECK_TO, VB_ENGINE_ALIGN_CHECK_TIMEOUT, FALSE, "AlignCheckTO");
    }
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;    
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMGenericFrameRxTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * A new EA frame has been received.
   *
   * ACTIONS:
   * - Process it.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineFrameRxProcess(processMsg);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentConfigTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Configure alignment mode in all DMs
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send AlignMode.req message to driver.
   * - Launch a timer.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Send EA_EngineConf.req message to align driver
    error = VbEngineEngineConfSend(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Launch timer to wait for AlignMode responses
    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_ALIGN_CONF_TO,
                                       MAX(processMsg->senderDriver->transactionMinTime,VB_ENGINE_REQUEST_TIMEOUT),
                                       "ALIGN_CONFIG_TO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignAllClustersTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  /*
   * Send ENGINE_EV_ALIGN_ALL_CLUSTERS to all drivers.
   */

  if (processMsg == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    ret = VbEngineProcessAllDriversEvSend(ENGINE_EV_ALIGN_ALL_CLUSTERS, NULL);
  }

  return ret;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMDriverAlignAllClustersTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  BOOL stop_tx = FALSE;

  /*
   * If this driver is part of the smallest cluster list, send Cluster Stop message to driver
   * Else send Cluster Stop Resp Evt
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send StopCluster.req message if needed to driver.
   * - Launch a timer.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
     VbEngineDriverTimeoutStop(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // if the driver is part of the smallest cluster list, send cluster Stop message with stop Tx indication
    error = VbEngineAlignClusterStopGet(processMsg->senderDriver->clusterId, &stop_tx);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    if(stop_tx == FALSE)
    {
      INT8U current_qos_rate;

      // Cluster maintained, get its current Qos Rate and apply it to Cluster 0
      current_qos_rate = VbCdtaQosRateGet(processMsg->senderDriver->clusterId);
      error = VbCdtaQosRateForce(0, current_qos_rate);
    }

    VbLogPrintExt(VB_LOG_INFO, processMsg->senderDriver->vbDriverID, "Send Stop cluster %d (stopTx %d)", processMsg->senderDriver->clusterId, stop_tx);

    error = VbEngineAlignStopClusterSend(processMsg->senderDriver, stop_tx);
    if (error == VB_ENGINE_ERROR_NONE)
    {
      // Launch timer to wait for cluster stop responses
      error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO,
                                         MAX(processMsg->senderDriver->transactionMinTime,VB_ENGINE_REQUEST_TIMEOUT),
                                         "ALIGN_CLUSTER_STOP_TO");
    }

    if(stop_tx == FALSE)
    {
      VbLogPrintExt(VB_LOG_INFO, processMsg->senderDriver->vbDriverID, "Cluster maintained %d", processMsg->senderDriver->clusterId);
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMDriverAlignClusterITransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  BOOL stop_tx = FALSE;

  /*
   * If this driver is part of the smallest cluster list, send Cluster Stop message to driver
   * Else send Cluster Stop Resp Evt
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send StopCluster.req message if needed to driver.
   * - Launch a timer.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
     VbEngineDriverTimeoutStop(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, processMsg->senderDriver->vbDriverID, "Send Stop cluster %d (stopTx %d)", processMsg->senderDriver->clusterId, stop_tx);

    error = VbEngineAlignStopClusterSend(processMsg->senderDriver, stop_tx);
    if (error == VB_ENGINE_ERROR_NONE)
    {
      // Launch timer to wait for cluster stop responses
      error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO,
                                         MAX(processMsg->senderDriver->transactionMinTime, VB_ENGINE_REQUEST_TIMEOUT),
                                         "ALIGN_CLUSTER_STOP_TO");
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentConfigSyncTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Alignment configuration response received.
   *
   * ACTIONS:
   * - Stop timer.
   * - Send an event to check all drivers states.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CONF_SYNC, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignConfigSyncTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;
  /*
   * Alignment configuration on-going.
   * Wait for all drivers to confirm new alignment parameters.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Check alignment mode:
   *      G.hn:
   *          - Move to ENGINE_STT_ALIGNMENT_WAIT_SYNC to wait until new alignment parameters are applied.
   *          - Configure a global timer to exit from wait state after VB_ENGINE_ALIGN_CONF_WAIT msecs.
   *      Common clock:
   *          - Move to next step: request clock to drivers.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers in cluster X are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_ALIGNMENT_CONFIG_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_MEASPLAN_RSP_SYNC event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    t_VBAlignmentMode alignmentMode;

    VbEngineConfAlignmentModeGet(&alignmentMode);
    if (alignmentMode == VB_ALIGN_MODE_GHN)
    {
      error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
      if(error == VB_ENGINE_ERROR_NONE)
      {
        // Send event to move to ENGINE_STT_ALIGNMENT_WAIT_SYNC state
        error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_WAIT, NULL, processMsg->clusterCast.list[0]);
        if (error == VB_ENGINE_ERROR_NONE)
        {
          VbEngineTimeoutStop(&(cluster->timeoutCnf));

          cluster->timeoutCnf.clusterCast.numCLuster = 1;
          cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

          // Start global timer to wait for new alignment parameters to apply
          error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_CLOCK_FORCE_REQ, VB_ENGINE_ALIGN_CONF_WAIT, FALSE, "AlignWaitTimer");
        }
      }
    }
    else
    {
      // Send event to move to clock_rsp_wait
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_CLOCK_FORCE_REQ, NULL, processMsg->clusterCast.list[0]);
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignRestartTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;
  t_VBCluster              *cluster;

  /*
   * Alignment check fail, restart alignment from scratch
   *
   * ACTIONS:
   * - Try next reference (only in G.hn alignment mode)
   * - Build AlignMode.req message to be sent
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);

    if (error == VB_ENGINE_ERROR_NONE)
    {
      // Stop and release timer
      VbEngineTimeoutStop(&(cluster->timeoutCnf));
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Build alignment mode message to send
    error = VbEngineEngineConfReqBuild(processMsg->clusterCast.list[0], FALSE);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMEmptyDomainTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Alignment check process has been started.
   * EACycQuery.req has been requested to driver.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Launch a timer.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentChangeTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Alignment is incorrect.
   * Build alignment changes request and send it to driver.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Clear attempts number.
   * - Build and send EACycChange.req to driver.
   * - Launch a timer to wait for EACycchange.rsp.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Init number of attempts
    processMsg->senderDriver->attempts = 0;

    // Build and send EA_CycChange.req message to align driver
    error = VbEngineAlignCycChangeSend(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Launch timer to wait for CycChange responses
    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_ALIGN_CHANGE_TO,
                                       MAX(processMsg->senderDriver->transactionMinTime,VB_ENGINE_ALIGN_CHANGE_TIMEOUT),
                                       "AlignChangeTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentCheckSyncTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * EACycQuery.rsp has been received from driver with alignment information.
   * Check if EACycQuery.rsp has been received from all drivers and process
   * alignment information.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Process EACycQuery.rsp frame.
   * - Send an event to check all drivers states and process all gathered information.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Process CycQuery.rsp frame
    error = VbeFSMGenericFrameRxTransition(processMsg);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Send a global event to check alignment of all drivers
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CHECK, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentChangeKoTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * EACycQuery.rsp has been received from driver conveying an error.
   * Try the request again or restart alignment process from scratch.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - If attempts remaining: try the request again.
   * - Max attempts reached: send an event to restart alignment process from scratch.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Check number of attempts
    if (processMsg->senderDriver->attempts < VB_ENGINE_ALIGN_CHANGE_MAX_ATTS)
    {
      processMsg->senderDriver->attempts++;

      // Retry CycChange request

      // Build and send EA_CycChange.req message to align driver
      error = VbEngineAlignCycChangeSend(processMsg->senderDriver);

      if (error == VB_ENGINE_ERROR_NONE)
      {
        // Launch timer to wait for CycChange responses
        error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_ALIGN_CHANGE_TO,
                                           MAX(processMsg->senderDriver->transactionMinTime, VB_ENGINE_ALIGN_CHANGE_TIMEOUT), "AlignChangeTO");
      }
    }
    else
    {
      // Send a global event to restart alignment process
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CHECK_RESTART, NULL, processMsg->senderDriver->clusterId);
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentChangeSyncTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * EACycChange.rsp has been received from driver.
   * Check if EACycChange.rsp has been received from all drivers.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send an event to check all drivers states.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Send a global event to check alignment of all drivers
    error = VbEngineProcessAllDriversEvSend(ENGINE_EV_ALIGN_CHANGE_SYNC, NULL);
  }

  return error;
}


/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllMeasPlanReBuildTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error;

  /*
   * A new measuring process is is going to be restarted (Comes fom a cancel measure plan)
   * Make sure all drivers in cluster are in state ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC and build.
   * Build a new measure plan.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Build a new measure plan to execute.
   * - Send event if error is detected.
   */

  // Check if all drivers are in the desired state
  error = VbEngineProcessCheckClustersSync(ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC, processMsg->clusterCast);

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbeFSMAllMeasPlanBuildTransition(processMsg);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}


/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllMeasPlanBuildTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error;
  t_VBCluster         *cluster;

  /*
   * A new measuring process is going to start.
   * Build a new measure plan.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Build a new measure plan to execute.
   * - Send event if error is detected.
   */

  error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  if(error == VB_ENGINE_ERROR_NONE)
  {
    // This is the entrance of the measure plan, clear epChange flag as the whole domain so far is going to be taken into account
    cluster->epChange = FALSE;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    error = VbEngineProcessMeasurePlanBuild(processMsg->clusterCast.list[0]);
  }

  if (error != VB_ENGINE_ERROR_NONE)
  {
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_FAIL, NULL, processMsg->clusterCast.list[0]);
  }


  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllEPChangeTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * A new measuring process is going to start.
   * Build a new measure plan.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Build a new measure plan to execute.
   * - Send event if error is detected.
   */

  if(processMsg->clusterCast.list[0] > 0)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
    if(error == VB_ENGINE_ERROR_NONE)
    {
      cluster->epChange = FALSE;

      // Stop and release timers
      VbEngineTimeoutStop(&(cluster->timeoutCnf));

      error =  VbEngineProcessMeasurePlanBuild(processMsg->clusterCast.list[0]);
      if (error != VB_ENGINE_ERROR_NONE)
      {
        VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_FAIL, NULL, processMsg->clusterCast.list[0]);
      }
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllPlanRestartTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * A new measuring process is going to start.
   * Build a new measure plan.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Build a new measure plan to execute.
   * - Send event if error is detected.
   */

  if(processMsg->clusterCast.list[0] > 0)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
    if(error == VB_ENGINE_ERROR_NONE)
    {
      cluster->epChange = FALSE;

      // Stop and release timers
      VbEngineTimeoutStop(&(cluster->timeoutCnf));

      cluster->timeoutCnf.clusterCast.numCLuster =1;
      cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

      error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_MEASPLAN_CANCEL_TO, VB_ENGINE_MEASURE_CANCEL_TIMEOUT, FALSE, "MeasPlanCancelEndTO");
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMMeasPlanSendTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Measure plan is already built. Send it to driver.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send Measure Plan request to driver.
   * - Launch timer to wait for measure plan confirmation from each drivers.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop previous timers
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    VbLogPrintExt(VB_LOG_INFO, processMsg->senderDriver->vbDriverID, "Measure Plan Send");

    // Send msg to connected drivers
    error = VbEngineProcessMeasurePlanRequestSend(processMsg->senderDriver);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Set timer to wait for confirmation
    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_MEASPLAN_RSP_TO,
                                       MAX(processMsg->senderDriver->transactionMinTime, VB_ENGINE_REQUEST_TIMEOUT), "MeasCnfTO");
  }

  return error;
}


/************************************************************************/

static t_VB_engineErrorCode VbeFSMDMRemBestActionMeasTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  BOOL                 ref_or_relay = (BOOL)(VOIDP2INT(processMsg->args));

  /*
   * Measure plan is already built. Send it to driver.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send Measure Plan request to driver.
   * - Launch timer to wait for measure plan confirmation from each drivers.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop previous timers
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    if(ref_or_relay == TRUE)
    {
      // Removed DM was Ref, Relay or Relay Candidate -> ALIGN_I
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, processMsg->senderDriver->clusterId);
    }
    else
    {
      // Removed DM was a SLAVE from the alignment point of view -> CANCEL CURRENT MEASURE PLAN & Build a new one
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_RESTART, NULL, processMsg->senderDriver->clusterId);
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMDMRemBestActionBoostTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  BOOL                 ref_or_relay = (BOOL)(VOIDP2INT(processMsg->args));

  /*
   * Measure plan is already built. Send it to driver.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send Measure Plan request to driver.
   * - Launch timer to wait for measure plan confirmation from each drivers.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop previous timers
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    if(ref_or_relay == TRUE)
    {
      // Removed DM was Ref, Relay or Relay Candidate -> ALIGN_I
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, processMsg->senderDriver->clusterId);
    }
    else
    {
      // Removed DM was a SLAVE from the alignment point of view -> BUILD MEASURE PLAN
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEAS_FORCE, NULL, processMsg->senderDriver->clusterId);
    }
  }

  return error;
}


/************************************************************************/

static t_VB_engineErrorCode VbeFSMMeasPlanCancelSendTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Measure plan is already built. Send it to driver.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send Measure Plan request to driver.
   * - Launch timer to wait for measure plan confirmation from each drivers.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop previous timers
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    VbLogPrintExt(VB_LOG_INFO, processMsg->senderDriver->vbDriverID, "Measure Plan Cancel Send");

    error = VbEngineProcessMeasurePlanCancelRequestSend(processMsg->senderDriver);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMMeasPlanCnfSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Measure plan confirmed by driver.
   * Then send an event to check if measure plan is confirmed by all drivers.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send event to all drivers to check if measure plan is confirmed by all drivers.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Send a global event to check alignment of all drivers
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_RSP_SYNC, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversMeasPlanCnfSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;
  INT32U                    time_to_completion;
  INT32U                    max_resp_time;
  t_VBCluster              *cluster;

  /*
   * Check that all drivers are waiting in ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC state.
   * Measure plan was confirmed by all drivers, wait until it finishes.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Relase measure plan resources.
   * - Calculate measure plan end time.
   * - Launch a timer to wait for measure plan completion.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_MEASPLAN_RSP_SYNC event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Calculate the measurement time in ms
    error = VbEngineProcessMeasurePlanTimeToFinishGet(processMsg->clusterCast.list[0], &time_to_completion);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    // Get max resp time from drivers
    error = VbEngineClusterMinTimeDriverGet(processMsg->clusterCast.list[0], &max_resp_time);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    cluster->timeoutCnf.clusterCast.numCLuster =1;
    cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

    error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_MEAS_COMPLETE_TO,
                                 MAX(max_resp_time, time_to_completion), FALSE, "MeasPlanEndTO");
  }

  return error;
}


/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversMeasPlanCancelSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode      error = VB_ENGINE_ERROR_NONE;
  t_VBCluster              *cluster;

  /*
   * Check that all drivers are waiting in ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC state.
   * Measure plan was confirmed by all drivers, wait until it finishes.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Relase measure plan resources.
   * - Calculate measure plan end time.
   * - Launch a timer to wait for measure plan completion.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check if all drivers are in the desired state
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_MEASPLAN_RSP_SYNC event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if (error == VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP)
  {
    // Stop and restart cluster timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    cluster->timeoutCnf.clusterCast.numCLuster =1;
    cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

    VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_MEASPLAN_CANCEL_TO, VB_ENGINE_MEASURE_CANCEL_TIMEOUT, FALSE, "MeasPlanCancelEndTO");
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_BUILD, NULL, processMsg->clusterCast.list[0]);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMMeasuresPlanCancelSyncTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * All CFRs (direct & Crosstalk) and BGN collected from this driver.
   * Send an event to check if measure collection process has finished in all drivers.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send an event to check measure collection completion in all drivers.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Collection completed from one driver , now check collection completion from all drivers
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_CANCEL_END_SYNC, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllMeasCollectMeasuresTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * Measure plan completed.
   * Start measures collection.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Start measures collect process.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    // Start collecting all measures
    error = VbEngineProcessCollectMeasuresStart(processMsg->clusterCast.list[0]);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMMeasCollectMeasuresTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Measure plan has finished.
   * Measure collect request has been sent to driver, wait until all measures are received.
   * VB_ENGINE_MEASURE_COLLECT_TIMEOUT shall be big enough to ensure the measure
   * collection process can be completed also in big networks.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Launch a new timer.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    error = VbEngineDriverTimeoutStart(processMsg->senderDriver, ENGINE_EV_MEAS_COLLECT_TO,
                                       MAX(processMsg->senderDriver->transactionMinTime, VB_ENGINE_MEASURE_COLLECT_TIMEOUT), "CollectMeasTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMMeasuresCollectDoneTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * All CFRs (direct & Crosstalk) and BGN collected from this driver.
   * Send an event to check if measure collection process has finished in all drivers.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send an event to check measure collection completion in all drivers.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Collection completed from one driver , now check collection completion from all drivers
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASURE_END_SYNC, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllMeasuresCollectDoneSyncTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Check that all drivers are waiting in ENGINE_STT_MEASURING_END_SYNC state.
   * All drivers have received CFRs and BGN measures.
   * Start SNR calculation process.
   *
   * ACTIONS:
   * - Check all drivers in expected state.
   * - Start SNR and capacity calculation process.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_MEASURING_END_SYNC, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NOT_READY)
  {
    /*
     * Drivers not in desired state.
     * Abort delivering ENGINE_EV_MEASURE_END_SYNC event to all drivers
     * and wait until all drivers reach the desired state.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Move to next step and calculate SNR for each node
    error = VbeFSMAllCalculateSNRTransition(processMsg);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMMeasPlanFailTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Error was detected while performing measuring process.
   *
   * ACTIONS:
   * - Clear pending timer.
   * - Send event to all drivers to cancel current measuring process.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Something went wrong, Stop pending measure plan in all drivers and go back to align phase
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllCalculateSNRTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Start SNR and capacity calculation process.
   *
   * ACTIONS:
   * - Lauch SNR and capacity calculation thread.
   * - Send event if error is detected.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Loop through all domains and calculate SNR (low band & Full) & capacities
    error = VbCalculateSNRAndCapacityRun(processMsg->clusterCast.list[0]);
  }

  if (error != VB_ENGINE_ERROR_NONE)
  {
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_COMPUTATION_KO, NULL, processMsg->clusterCast.list[0]);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllComputationOkTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  INT16U               boosting_alg_period;
  t_VBCluster         *cluster;
  /*
   * SNR and capacity calculated successfully.
   * Start boosting process.
   *
   * ACTIONS:
   * - Stop SNR and capacity thread.
   * - Save measures to disk, if required.
   * - Start a timer to run boosting algorithm each VbEngineConfBoostAlgPeriodGet ms.
   * - Run boosting algorithm.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);

    // Stop SNR & Capacity calculation thread
    VbCalculateSNRAndCapacityStop(processMsg->clusterCast.list[0]);

    // Save measure to Disk if requested
    if(VbEngineConfSaveMeasuresGet() == TRUE)
    {
      VbEngineMeasureSave(processMsg->clusterCast.list[0]);
    }

    // Get algorithm period
    boosting_alg_period = VbEngineConfBoostAlgPeriodGet();

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Configuring boosting algorithm to run each %u ms", boosting_alg_period);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Configure a periodic timer to launch event to run boost algorithm
    cluster->timeoutCnf.clusterCast.numCLuster =1;
    cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

    error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_BOOST_ALG_RUN_TO, boosting_alg_period, TRUE, "BoostAlgTO");
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = BoostAlgorithmRun(processMsg->clusterCast.list[0]);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllBoostAlgRunTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * VbEngineConfBoostAlgPeriodGet timeout.
   * Run boosting algorithm or run a periodic alignment check.
   *
   * ACTIONS:
   * - Check if a new alignment check is required and launch it.
   * - Or, run boosting algorithm.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    if(cluster->epChange == TRUE)
    {
      // an EP has appeared or has gone -> New Measure plan in current cluster
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_NETWORK_DIFF_EP_CHANGE, NULL, processMsg->clusterCast.list[0]);
    }
    else if (VbEngineAlignPeriodicCheckNeeded(&cluster->clusterInfo.lastAlignCheck, processMsg->clusterCast.list[0]) == TRUE)
    {
      // Check if a periodic alignment check is needed at this moment
      // Start a new alignment check
      error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_PERIODIC_CHECK, NULL, processMsg->clusterCast.list[0]);
    }
    else
    {
      error = BoostAlgorithmRun(processMsg->clusterCast.list[0]);
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllBoostUpdateTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  INT32U               time_to_finish;
  INT32U               max_resp_time;
  t_VBCluster         *cluster;

  /*
   * Resources have been reallocated by engine.
   * Wait until new allocation has been applied by all G.hn nodes.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Calculate time to apply new resources allocation.
   * - Launch a timer to wait.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Get time to apply PSD shape
    error = VbEngineTimeToFinishGet(&time_to_finish, &cluster->applyOwnTs);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Get max resp time from drivers
    error = VbEngineClusterMinTimeDriverGet(processMsg->clusterCast.list[0], &max_resp_time);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Configure a timer to wait for Boost update finish
    cluster->timeoutCnf.clusterCast.numCLuster =1;
    cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

    error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_BOOST_UPDATE_END_TO,
                                   MAX(max_resp_time, time_to_finish), FALSE, "BoostUpdateTO");
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllBoostTimerSetAndAlgRunTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  INT16U               boosting_alg_period;
  t_VBCluster         *cluster;

  /*
   * System ready to perform boosting.
   * Configure timer to run boosting algorithm periodically.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Start timer to run boosting algorithm each VbEngineConfBoostAlgPeriodGet ms.
   * - Run boosting algorithm.
   */
  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));

    // Get algorithm period
    boosting_alg_period = VbEngineConfBoostAlgPeriodGet();

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Configuring boosting algorithm to run each %u ms", boosting_alg_period);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Configure a periodic timer to launch event to run boost algorithm
    cluster->timeoutCnf.clusterCast.numCLuster =1;
    cluster->timeoutCnf.clusterCast.list[0] = processMsg->clusterCast.list[0];

    error = VbEngineTimeoutStart(&(cluster->timeoutCnf), ENGINE_EV_BOOST_ALG_RUN_TO, boosting_alg_period, TRUE, "BoostAlgTO");
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Run boost algorithm to check last traffic reports received
    error = BoostAlgorithmRun(processMsg->clusterCast.list[0]);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllBoostUpdateEndKoTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * Error detected while reallocating resources.
   * Start from scratch.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Send network change event to start from scratch.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Send network change to all to start from scratch
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, processMsg->clusterCast.list[0]);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllBoostUpdateEndTOTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * Timeout waiting to apply new resources allocation.
   * Check confirmation received from all G.hn nodes.
   *
   * ACTIONS:
   * - Stop pending timer.
   * - Check confirmation from all nodes.
   * - Send event to all drivers depending on result.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineProcessCheckClustersSync(ENGINE_STT_BOOSTING_WAIT_UPDATE, processMsg->clusterCast);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop and release timer
    VbEngineTimeoutStop(&(cluster->timeoutCnf));
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Check that PSD shape has been confirmed
    error = VbEnginePsdShapeCnfCheck(processMsg->clusterCast.list[0]);
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // PSD shape confirmed for all nodes
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_BOOST_UPDATE_END_OK, NULL, processMsg->clusterCast.list[0]);
  }
  else
  {
    // PSD shape not confirmed for all nodes
    error = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_BOOST_UPDATE_END_KO, NULL, processMsg->clusterCast.list[0]);
  }

  if ((error == VB_ENGINE_ERROR_NONE)|| (error == VB_ENGINE_ERROR_NOT_READY))
  {
    /*
     * Abort delivering ENGINE_EV_BOOST_UPDATE_END_TO event to all drivers.
     * This event shall be processed only by this function.
     */
    error = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMSNRProbeForcedTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Request a new SNR with PROBEs measure from driver.
   *
   * ACTIONS:
   * - Request SNR with PROBEs measures.
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineSNRProbeForceRequest(processMsg->senderDriver, processMsg->args);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllComputationKoTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * Error detected while calculating SNR and capacity.
   * Broadcast this event to all drivers to start from scratch.
   *
   * ACTIONS:
   * - Stop SNR and capacity thread.
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop SNR & Capacity calculation thread
    VbCalculateSNRAndCapacityStop(processMsg->clusterCast.list[0]);

    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
    if (error == VB_ENGINE_ERROR_NONE)
    {
      // Stop and release timer
      VbEngineTimeoutStop(&(cluster->timeoutCnf));
    }
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllPSDForceUpdateTransition( t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error;

  /*
   * Force to apply the last resource allocation calculated by boosting algorithm.
   *
   * ACTIONS:
   * - Send a PSD shape request to all drivers.
   */

  error = VbEnginePSDShapeConfigureAll(processMsg->clusterCast.list[0]);

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentClusterAllStopSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Stop timers
   * Clear Role of nodes in stopped clusters
   * Send ENGINE_EV_RX_ALIGN_CLUSTERS_ALL_STOP_SYNC to all drivers to check sync
   *
   * ACTIONS:
   * - .
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    BOOL stop_tx;

    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // if the driver is part of the smallest cluster list, remove Ref and relays
    error = VbEngineAlignClusterStopGet(processMsg->senderDriver->clusterId, &stop_tx);

    if (error == VB_ENGINE_ERROR_NONE)
    {
      if(stop_tx == TRUE)
      {
        //  Remove Ref and Relays in this cluster
        VbEngineAlignClusterRoleReset(processMsg->senderDriver->clusterId);
      }
    }

    // Cluster Stop completed from one driver, now check Cluster Stop completion from all drivers
    VbEngineProcessAllDriversEvSend(ENGINE_EV_RX_ALIGN_CLUSTERS_ALL_STOP_SYNC, NULL);
  }

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAlignmentClusterIStopSyncTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Stop timers
   * Send ENGINE_EV_RX_ALIGN_CLUSTER_I_STOP_SYNC to all drivers in cluster X to check sync
   *
   * ACTIONS:
   * - .
   */

  if ((processMsg == NULL) || (processMsg->senderDriver == NULL))
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Stop timer
    VbEngineDriverTimeoutStop(processMsg->senderDriver);

    // Cluster Stop completed from one driver, now check Cluster Stop completion from all drivers in cluster X
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_RX_ALIGN_CLUSTER_I_STOP_SYNC, NULL, processMsg->senderDriver->clusterId);
  }

  return error;
}


/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignAllClustersTransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;

  /*
   * Realign all clusters starting point (stop smallest one, keep biggest one, ...)
   *
   * ACTIONS:
   * - .
   */

  VbEngineClusterStopTimers();

  return error;
}

/************************************************************************/

static t_VB_engineErrorCode VbeFSMAllDriversAlignClusterITransition(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  /*
   * Re ALign cluster x only (leave other clusters as they are)
   *
   * ACTIONS:
   * - .
   */

  if (processMsg == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    error = VbEngineClusterByIdGet(processMsg->clusterCast.list[0], &cluster);
    if (error == VB_ENGINE_ERROR_NONE)
    {
      // Stop and release timer
      VbEngineTimeoutStop(&(cluster->timeoutCnf));
    }
  }

  return error;
}

/*******************************************************************/

static void VbEngineProcessStateLogToFile(const t_VBDriver *thisDriver)
{
  if ((thisDriver != NULL) && (thisDriver->definitiveDriverId == TRUE))
  {
    // Only dump to file when DriverId is known
    VbLogSaveStringToTextFile(thisDriver->fsmStateFileName, "w+", "VBEngineState=%s\n", FSMSttToStrGet(thisDriver->FSMState));
  }
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessFrameRxToEventTranslate(t_vbEAMsg *msg, t_VB_Comm_Event *event, t_VBDriver *driver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((msg == NULL) || (event == NULL) || (driver == NULL) || (msg->opcode >= VB_EA_OPCODE_LAST))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *event = vb_EAI_lookup_events[msg->opcode];

    switch (*event)
    {
      case (ENGINE_EV_RX_DOMAIN_RSP):
      {
        // Check number of domains discovered
        INT32U num_domains = VbEngineDomainsRespNumDomainsGet(msg->eaPayload.msg);

        if (num_domains == 0)
        {
          // Update event to wait for new domains
          *event = ENGINE_EV_RX_EMPTY_DOMAIN_RSP;
        }

        break;
      }

      case (ENGINE_EV_RX_CYCQUERY_RSP):
      {
        t_vbEACycQueryRspErrorCode err_cod;

        // Get error
        ret = VbEngineAlignCycQueryRspErrGet(msg->eaPayload.msg, &err_cod, driver->clusterId);

        if ((ret != VB_ENGINE_ERROR_NONE) || (err_cod != VB_EA_CYCQUERY_RSP_ERR_NONE))
        {
          // Error while processing CycQuery

          if (ret == VB_ENGINE_ERROR_PARAMS)
          {
            // Unexpected response, wait for the correct one
            *event = ENGINE_EV_RX_CYCQUERY_RSP_INV;
          }
          else
          {
            *event = ENGINE_EV_RX_CYCQUERY_RSP_KO;
          }
          ret = VB_ENGINE_ERROR_NONE;
        }

        break;
      }

      case (ENGINE_EV_RX_CYCCHANGE_RSP):
      {
        t_vbEACycChangeRspErrorCode err_cod;

        // Get error
        ret = VbEngineAlignCycChangeRspErrGet(msg->eaPayload.msg, &err_cod);

        if ((ret != VB_ENGINE_ERROR_NONE) || (err_cod != VB_EA_CYCCHANGE_RSP_ERR_NONE))
        {
          // Error while processing CycChange
          *event = ENGINE_EV_RX_CYCCHANGE_RSP_KO;
          ret = VB_ENGINE_ERROR_NONE;
        }

        break;
      }

      case (ENGINE_EV_RX_ALIGNMODE_RSP):
      {
        t_vbEAAlignModeRspErrorCode err_cod;

        // Get error
        ret = VbEngineAlignModeRspErrGet(msg->eaPayload.msg, &err_cod, driver);

        if ((ret != VB_ENGINE_ERROR_NONE) || (err_cod == VB_EA_ALIGNMODE_RSP_ERR_DRV))
        {
          // Error while processing AlignMode
          // Some error detected, Stop cluster rejected by driver
          *event = ENGINE_EV_RX_ALIGNMODE_RSP_KO;
          ret = VB_ENGINE_ERROR_NONE;
        }
        else if(err_cod == VB_EA_ALIGNMODE_RSP_ERR_ID)
        {
          // Not for this align Id, ignore packet
          ret = VB_ENGINE_ERROR_SKIP;
        }

        break;
      }

      case (ENGINE_EV_RX_MEAS_COLLECT_END_TRG):
      {
        // Check response
        ret = VbEngineMeasCollectionEndProcess(msg->eaPayload.msg, msg->eaPayload.msgLen, driver);

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          // Collection End error
          *event = ENGINE_EV_MEASPLAN_FAIL;
          ret = VB_ENGINE_ERROR_NONE;
        }

        break;
      }

      case (ENGINE_EV_RX_MEASPLAN_RSP):
      {
        // Check response
        ret = VbEngineMeasureRespProcess(msg->eaPayload.msg, msg->eaPayload.msgLen, driver);

        if (ret == VB_ENGINE_ERROR_INVALID_PLANID)
        {
          // Ignore message, return this error to upper layers to avoid sending an event
        }
        else if (ret != VB_ENGINE_ERROR_NONE)
        {
          // Measure Plan error
          *event = ENGINE_EV_MEASPLAN_FAIL;
          ret = VB_ENGINE_ERROR_NONE;
        }

        break;
      }

      case (ENGINE_EV_RX_MEASPLAN_CANCEL_RSP):
      {
        // Check response
        ret = VbEngineMeasureCancelRespProcess(msg->eaPayload.msg, msg->eaPayload.msgLen, driver);

        if (ret == VB_ENGINE_ERROR_INVALID_PLANID)
        {
          // Ignore message, return this error to upper layers to avoid sending an event
        }
        else if (ret != VB_ENGINE_ERROR_NONE)
        {
          // Ignore the rest of errors and do not change event, measure plan was cancelled
          ret = VB_ENGINE_ERROR_NONE;
        }

        break;
      }

      case (ENGINE_EV_RX_MEASPLAN_ERR_TRG):
      {
        // Check response
        ret = VbEngineMeasureErrorNotifyProcess(msg->eaPayload.msg, msg->eaPayload.msgLen, driver);

        if (ret == VB_ENGINE_ERROR_INVALID_PLANID)
        {
          // Ignore message, return this error to upper layers to avoid sending an event
        }
        else if (ret != VB_ENGINE_ERROR_NONE)
        {
          // Ignore the rest of errors and do not change event, measure plan was cancelled
          ret = VB_ENGINE_ERROR_NONE;
        }

        break;
      }

      case (ENGINE_EV_RX_ALIGN_CLUSTER_STOP_RSP):
      {
        // Check response
        ret = VbEngineAlignClusterStopRspProcess(msg->eaPayload.msg, msg->eaPayload.msgLen, driver);

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          // Cluster Stop End error
          *event = ENGINE_EV_RX_ALIGN_CLUSTER_STOP_FAIL;
          ret = VB_ENGINE_ERROR_NONE;
        }

        break;
      }

      default:
      {
        // Do not change event
        break;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineGenericEvSend(mqd_t queue, t_VBProcessMsg *msg)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    unsigned int msg_prio;
    int          err;

    if (msg->senderDriver == NULL)
    {
      msg_prio = VB_THREADMSG_HIGH_PRIORITY;
    }
    else
    {
      msg_prio = VB_THREADMSG_PRIORITY;
    }

    err = mq_send(queue, ((const char *)(msg)), VB_ENGINE_PROCESS_MQ_MSG_SIZE, msg_prio);

    if (err != 0)
    {
      ret = VB_ENGINE_ERROR_QUEUE;
    }
  }

  return ret;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

BOOL VbEngineProcessThreadRunningGet(void)
{
  return vbEngineProcess.running;
}

/*******************************************************************/

void VbEngineProcessThreadRunningSet(BOOL value)
{
  vbEngineProcess.running = value;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessEvSend(t_VBDriver *driver, t_VB_Comm_Event event, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((event >= ENGINE_EV_LAST) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineProcessThreadRunningGet() == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBProcessMsg vb_process_msg = { 0 };

    vb_process_msg.vbCommEvent = event;
    vb_process_msg.msg = NULL;
    vb_process_msg.senderDriver = driver;
    vb_process_msg.args = args;
    vb_process_msg.clusterCast.numCLuster = 0; // Not needed here

    ret = VbEngineGenericEvSend(vbEngineProcess.wrOnlyQueueId, &vb_process_msg);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error sending message to engine process Queue. errno: %s", strerror(errno));
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessEAFrameRx(t_vbEAMsg *msg, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode res = VB_ENGINE_ERROR_NONE;
  t_VB_Comm_Event      event;

  if ((thisDriver == NULL) || (msg == NULL))
  {
    res = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (res == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineProcessThreadRunningGet() == FALSE)
    {
      res = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if (res == VB_ENGINE_ERROR_NONE)
  {
    // Translate opcode to event
    res = VbEngineProcessFrameRxToEventTranslate(msg, &event, thisDriver);
  }

  if (res == VB_ENGINE_ERROR_NONE)
  {
    if (event >= ENGINE_EV_LAST)
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Unexpected OPCODE 0x%02X", (unsigned int)msg->opcode);

      res = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  if (res == VB_ENGINE_ERROR_NONE)
  {
    t_VBProcessMsg vb_process_msg = { 0 };

    // Convert the opcode into a event id
    vb_process_msg.vbCommEvent = event;
    vb_process_msg.msg = msg;
    vb_process_msg.senderDriver = thisDriver;
    vb_process_msg.args = NULL;
    vb_process_msg.clusterCast.numCLuster = 0; // Not needed here

    res = VbEngineGenericEvSend(thisDriver->vbEAConnDesc.queueId, &vb_process_msg);

    if (res != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Error sending message to engine process Queue. errno: %s", strerror(errno));
    }
  }

  if (res == VB_ENGINE_ERROR_SKIP)
  {
    res = VB_ENGINE_ERROR_NONE;
  }

  return res;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessAllDriversEvSend(t_VB_Comm_Event event, void *args)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (event >= ENGINE_EV_LAST)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineProcessThreadRunningGet() == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBProcessMsg vb_process_msg = { 0 };

    vb_process_msg.vbCommEvent = event;
    vb_process_msg.msg = NULL;
    vb_process_msg.senderDriver = NULL; // Set driver ptr to NULL to target all drivers
    vb_process_msg.args = args;
    vb_process_msg.clusterCast.numCLuster = 0; // 0 and senderDriver == NULL -> TO ALL

    ret = VbEngineGenericEvSend(vbEngineProcess.wrOnlyQueueId, &vb_process_msg);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error sending message (EV_%s) to engine_process Queue. errno: %s", FSMEvToStrGet(event), strerror(errno));
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessClusterXDriversEvSend(t_VB_Comm_Event event, void *args, INT32U clusterId)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (event >= ENGINE_EV_LAST)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineProcessThreadRunningGet() == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBProcessMsg vb_process_msg = { 0 };

    vb_process_msg.vbCommEvent = event;
    vb_process_msg.msg = NULL;
    vb_process_msg.senderDriver = NULL; // Set driver ptr to NULL to target all drivers
    vb_process_msg.args = args;
    vb_process_msg.clusterCast.numCLuster = 1;
    vb_process_msg.clusterCast.list[0] = clusterId;

    ret = VbEngineGenericEvSend(vbEngineProcess.wrOnlyQueueId, &vb_process_msg);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error sending message (EV_%s) to engine_process Queue. errno: %s", FSMEvToStrGet(event), strerror(errno));
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessClustersListDriversEvSend(t_VB_Comm_Event event, void *args, t_ClusterCast clusters)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (event >= ENGINE_EV_LAST)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineProcessThreadRunningGet() == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBProcessMsg vb_process_msg = { 0 };

    vb_process_msg.vbCommEvent = event;
    vb_process_msg.msg = NULL;
    vb_process_msg.senderDriver = NULL; // Set driver ptr to NULL to target all drivers
    vb_process_msg.args = args;
    vb_process_msg.clusterCast = clusters;

    ret = VbEngineGenericEvSend(vbEngineProcess.wrOnlyQueueId, &vb_process_msg);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error sending message (EV_%s) to engine_process Queue. errno: %s", FSMEvToStrGet(event), strerror(errno));
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessThreadInit(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  vbEngineProcess.threadId = 0;
  vbEngineProcess.running = FALSE;

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessThreadExecute(void)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;

  VbEngineProcessProtocolThreadStop();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Starting %s thread", VB_ENGINE_PROCESS_THREAD_NAME);

  vbEngineProcess.running = TRUE;

  if (FALSE == VbThreadCreate(VB_ENGINE_PROCESS_THREAD_NAME,
                              (void *)VbEngineProcess, NULL,
                              VB_ENGINE_PROCESS_THREAD_PRIORITY,
                              &(vbEngineProcess.threadId)))
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Can't create %s thread", VB_ENGINE_PROCESS_THREAD_NAME);
    result = VB_ENGINE_ERROR_EA_THREAD_CREATE;
    vbEngineProcess.running = FALSE;
  }

  return result;
}

/*******************************************************************/

void VbEngineProcessProtocolThreadStop(void)
{
  if (vbEngineProcess.running)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopping engine process thread...");

    VbEngineProcessAllDriversEvSend(ENGINE_EV_KILL_ALL, NULL);
    VbThreadJoin(vbEngineProcess.threadId, NULL);
  }
}

/************************************************************************/

const CHAR *FSMSttToStrGet(t_vbEngineProcessFSMState state)
{
  const CHAR *state_str = "UNKNOWN";

  if (state < ENGINE_STT_LAST)
  {
    state_str = FSMStateString[state];
  }

  return state_str;
}

/************************************************************************/

const CHAR *FSMEvToStrGet(t_VB_Comm_Event event)
{
  const CHAR *event_str = "UNKNOWN";

  if (event < ENGINE_EV_LAST)
  {
    event_str = vbEngineEventString[event];
  }

  return event_str;
}

/************************************************************************/

CHAR *VbEngineProcessQueueNameGet(void)
{
  return vbEngineProcess.queueName;
}

/************************************************************************/


/**
 * @}
 **/
