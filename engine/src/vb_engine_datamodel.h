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
 * @file vb_engine_dataModel.h
 * @brief Engine data model interface
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
 *
 **/

#ifndef VB_ENGINE_DATAMODEL_H_
#define VB_ENGINE_DATAMODEL_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_ea_communication.h"
#include "vb_mac_utils.h"
#include "vb_linked_list.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define VB_ENGINE_VERSION                                "3.0 r067"
#define VB_ENGINE_ALL_DRIVERS_STR                        "ALL"
#define DRIVER_REMOTE_STATE_MAX_LEN                      (30)
#define VB_ENGINE_LAST_LOW_BAND_28MHZ_NUM_CARRIER_IDX    (504)
#define VB_ENGINE_LAST_LOW_BAND_50MHZ_NUM_CARRIER_IDX    (952)
#define VB_ENGINE_MAX_FILE_NAME_SIZE                     (50)

#ifndef ENGINE_DISABLE_METRICS
#  define VB_ENGINE_METRICS_ENABLED                      (1)
#else
#  define VB_ENGINE_METRICS_ENABLED                      (0)
#endif

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */


typedef enum
{
  VB_ENGINE_QOS_RATE_0_100 = 0,
  VB_ENGINE_QOS_RATE_10_90,
  VB_ENGINE_QOS_RATE_20_80,
  VB_ENGINE_QOS_RATE_30_70,
  VB_ENGINE_QOS_RATE_40_60,
  VB_ENGINE_QOS_RATE_50_50,
  VB_ENGINE_QOS_RATE_60_40,
  VB_ENGINE_QOS_RATE_70_30,
  VB_ENGINE_QOS_RATE_80_20,
  VB_ENGINE_QOS_RATE_90_10,
  VB_ENGINE_QOS_RATE_LAST,
}t_vbEngineQosRate;

typedef enum
{
  VB_ENGINE_COUNTER_EV_KILL_ALL = 0,
  VB_ENGINE_COUNTER_EV_KILL,
  VB_ENGINE_COUNTER_EV_CONNECT,
  VB_ENGINE_COUNTER_EV_DISCONNECT,
  VB_ENGINE_COUNTER_EV_BOOST_UPDATE,
  VB_ENGINE_COUNTER_EV_BOOST_UPDATE_END_OK,
  VB_ENGINE_COUNTER_EV_BOOST_UPDATE_END_KO,
  VB_ENGINE_COUNTER_EV_ALIGN_CHECK_RESTART,
  VB_ENGINE_COUNTER_EV_ALIGN_PERIODIC_CHECK,
  VB_ENGINE_COUNTER_EV_ALIGN_CHECK,
  VB_ENGINE_COUNTER_EV_ALIGN_DONE,
  VB_ENGINE_COUNTER_EV_ALIGN_DONE_SKIP_MEAS,
  VB_ENGINE_COUNTER_EV_ALIGN_CHANGE,
  VB_ENGINE_COUNTER_EV_ALIGN_CHANGE_SYNC,
  VB_ENGINE_COUNTER_EV_ALIGN_WAIT,
  VB_ENGINE_COUNTER_EV_ALIGN_RESTART,
  VB_ENGINE_COUNTER_EV_ALIGN_CONF_SYNC,
  VB_ENGINE_COUNTER_EV_DISC_DOMAIN_SYNC,
  VB_ENGINE_COUNTER_EV_CLOCK_REQ,
  VB_ENGINE_COUNTER_EV_CLOCK_REQ_SYNC,
  VB_ENGINE_COUNTER_EV_CLOCK_FORCE_REQ,
  VB_ENGINE_COUNTER_EV_MEASPLAN_FAIL,
  VB_ENGINE_COUNTER_EV_MEASPLAN_RSP_SYNC,
  VB_ENGINE_COUNTER_EV_MEASURE_END_SYNC,
  VB_ENGINE_COUNTER_EV_MEAS_GENERAL_FAILURE,
  VB_ENGINE_COUNTER_EV_COMPUTATION_OK,
  VB_ENGINE_COUNTER_EV_COMPUTATION_KO,
  VB_ENGINE_COUNTER_EV_MEAS_FORCE,
  VB_ENGINE_COUNTER_EV_SNR_PROBES_MEAS_FORCE,
  VB_ENGINE_COUNTER_EV_PSD_UPDATE_FORCE,
  VB_ENGINE_COUNTER_EV_NETWORK_CHANGE,
  VB_ENGINE_COUNTER_EV_MEAS_COLLECT_END_NO_LINES,
  VB_ENGINE_COUNTER_EV_DISC_VERS_TO,
  VB_ENGINE_COUNTER_EV_DISC_STATE_TO,
  VB_ENGINE_COUNTER_EV_DOMAINS_SYNC_TO,
  VB_ENGINE_COUNTER_EV_CLOCK_RSP_TO,
  VB_ENGINE_COUNTER_EV_ALIGN_CHECK_TO,
  VB_ENGINE_COUNTER_EV_ALIGN_CHANGE_TO,
  VB_ENGINE_COUNTER_EV_ALIGN_SYNC_TO,
  VB_ENGINE_COUNTER_EV_MEASPLAN_RSP_TO,
  VB_ENGINE_COUNTER_EV_MEAS_COMPLETE_TO,
  VB_ENGINE_COUNTER_EV_MEAS_COLLECT_TO,
  VB_ENGINE_COUNTER_EV_MEASPLAN_CANCEL_TO,
  VB_ENGINE_COUNTER_EV_BOOST_UPDATE_END_TO,
  VB_ENGINE_COUNTER_EV_BOOST_ALG_RUN_TO,
  VB_ENGINE_COUNTER_EV_ALIGN_CONF_TO,
  VB_ENGINE_COUNTER_EV_RX_NETWORK_CHANGE_TRG,
  VB_ENGINE_COUNTER_EV_RX_VERSION_RSP,
  VB_ENGINE_COUNTER_EV_RX_STATE_RSP,
  VB_ENGINE_COUNTER_EV_RX_DOMAIN_RSP,
  VB_ENGINE_COUNTER_EV_RX_EMPTY_DOMAIN_RSP,
  VB_ENGINE_COUNTER_EV_RX_CYCQUERY_RSP,
  VB_ENGINE_COUNTER_EV_RX_CYCQUERY_RSP_KO,
  VB_ENGINE_COUNTER_EV_RX_CYCQUERY_RSP_INV,
  VB_ENGINE_COUNTER_EV_RX_CYCCHANGE_RSP,
  VB_ENGINE_COUNTER_EV_RX_CYCCHANGE_RSP_KO,
  VB_ENGINE_COUNTER_EV_MEASPLAN_RSP,
  VB_ENGINE_COUNTER_EV_MEASPLAN_CANCEL_RSP,
  VB_ENGINE_COUNTER_EV_MEASPLAN_ERR_TRG,
  VB_ENGINE_COUNTER_EV_MEASURE_BGN_RSP,
  VB_ENGINE_COUNTER_EV_RX_MEASURE_CFR_RSP,
  VB_ENGINE_COUNTER_EV_RX_MEAS_COLLECT_END_TRG,
  VB_ENGINE_COUNTER_EV_RX_TRAFFIC_AWARENESS_TRG,
  VB_ENGINE_COUNTER_EV_RX_CLOCK_RSP,
  VB_ENGINE_COUNTER_EV_RX_PSD_SHAPE_RSP,
  VB_ENGINE_COUNTER_EV_RX_MEASURE_SNRPROBES_RSP,
  VB_ENGINE_COUNTER_EV_RX_CDTA_RSP,
  VB_ENGINE_COUNTER_EV_RX_ALIGNMODE_RSP,
  VB_ENGINE_COUNTER_EV_RX_ALIGNMODE_RSP_KO,
  VB_ENGINE_COUNTER_EV_RX_CLUSTER_STOP_RSP,
  VB_ENGINE_COUNTER_EV_RX_CLUSTER_STOP_RSP_KO,
  VB_ENGINE_COUNTER_EV_RX_ALIGN_SYNC_LOST_TRG,
  VB_ENGINE_COUNTER_DISCONNECTED_STATUS,
  VB_ENGINE_COUNTER_BOOSTING_SEND_PSD_SHAPE,
  VB_ENGINE_COUNTER_LINE_LOST,
  VB_ENGINE_COUNTER_NEW_LINE,
  VB_ENGINE_COUNTER_COMMUNICATION_DRIVER_LOST,
  VB_ENGINE_COUNTER_MEASURE_PLAN_REQUESTED,
  VB_ENGINE_COUNTER_MEASURE_PLAN_FAILED,
  VB_ENGINE_COUNTER_MEASURE_PLAN_REJECTED,
  VB_ENGINE_COUNTER_MEASURE_PLAN_SUCCESS,
  VB_ENGINE_COUNTER_MEASURE_PLAN_CANCELLED,
  VB_ENGINE_COUNTER_CDTA_CFG,
  VB_ENGINE_COUNTER_CDTA_FORCE_NO_CHANGE,
  VB_ENGINE_COUNTER_ERR_CREATING_NEW_DRIVER,
  VB_ENGINE_COUNTERS_NUM
}t_vbEngineCountersIndex;

/*
 * Engine FSM events:
 * Modifying this enum implies updating:
 * - FSMStateString
 */
typedef enum
{
  ENGINE_STT_DISCONNECTED = 0,
  ENGINE_STT_DISCOVER_VERS_REQ,
  ENGINE_STT_DISCOVER_STATE_REQ,
  ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY,
  ENGINE_STT_DISCOVER_DOMAIN_WAIT,
  ENGINE_STT_DISCOVER_DOMAIN_SYNC,
  ENGINE_STT_CLOCK_RSP_WAIT,
  ENGINE_STT_CLOCK_REQ_SYNC,
  ENGINE_STT_ALIGNMENT_PREPARE_ALL,
  ENGINE_STT_ALIGNMENT_PREPARE_I,
  ENGINE_STT_ALIGNMENT_CLUSTERS_ALL_STOP_SYNC,
  ENGINE_STT_ALIGNMENT_CLUSTER_I_STOP_SYNC,
  ENGINE_STT_ALIGNMENT_CHECK,
  ENGINE_STT_ALIGNMENT_CHECK_SYNC,
  ENGINE_STT_ALIGNMENT_CHANGE,
  ENGINE_STT_ALIGNMENT_CHANGE_SYNC,
  ENGINE_STT_ALIGNMENT_CONFIG,
  ENGINE_STT_ALIGNMENT_CONFIG_SYNC,
  ENGINE_STT_ALIGNMENT_CONFIG_DONE_SYNC,
  ENGINE_STT_ALIGNMENT_WAIT_SYNC,
  ENGINE_STT_ALIGNMENT_DONE_SYNC,
  ENGINE_STT_MEASURING_MEASPLAN_RSP_WAIT,
  ENGINE_STT_MEASURING_MEASPLAN_RSP_SYNC,
  ENGINE_STT_MEASURING_MEASPLAN_END_WAIT,
  ENGINE_STT_MEASURING_COLLECT_MEAS,
  ENGINE_STT_MEASURING_END_SYNC,
  ENGINE_STT_MEASURING_CANCEL_WAIT,
  ENGINE_STT_MEASURING_CANCEL_WAIT_SYNC,
  ENGINE_STT_BOOSTING_CALCULATE_SNR,
  ENGINE_STT_BOOSTING_WAIT_TRGS,
  ENGINE_STT_BOOSTING_WAIT_UPDATE,
  ENGINE_STT_UNDEFINED,
  ENGINE_STT_LAST,
}t_vbEngineProcessFSMState;

typedef enum {
  VB_ENGINE_ERROR_NONE = 0,
  VB_ENGINE_ERROR_MALLOC = -1,
  VB_ENGINE_ERROR_PARAMS = -2,
  VB_ENGINE_ERROR_EA_THREAD_CREATE = -3,
  VB_ENGINE_ERROR_DATA_MODEL = -4,
  VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES = -5,
  VB_ENGINE_ERROR_SNRCALCERROR_DATA_LINEAR = -6,
  VB_ENGINE_ERROR_SEND_FRAME = -7,
  VB_ENGINE_ERROR_BAD_ARGUMENTS = -8,
  VB_ENGINE_ERROR_NOT_FOUND = -9,
  VB_ENGINE_ERROR_RECEIVE_UNEXPECTED = -10,
  VB_ENGINE_ERROR_MEASURE_PLAN = -11,
  VB_ENGINE_ERROR_DOMAIN_WITHOUT_LINE = -12,
  VB_ENGINE_ERROR_MEASURE_NODE = -13,
  VB_ENGINE_ERROR_MEASURING_SEQUENCE = -14,
  VB_ENGINE_ERROR_QUEUE = -15,
  VB_ENGINE_ERROR_INI_FILE = -16,
  VB_ENGINE_ERROR_FRAME = -17,
  VB_ENGINE_ERROR_TRAFFIC_AWARENESS_EMPTY = -18,
  VB_ENGINE_ERROR_UNKNOWN = -19,
  VB_ENGINE_ERROR_NOT_READY = -20,
  VB_ENGINE_ERROR_EXIT_LOOP_OK = -21,
  VB_ENGINE_ERROR_SKIP = -22,
  VB_ENGINE_ERROR_INVALID_TRANSITION = -23,
  VB_ENGINE_ERROR_INVALID_STATE = -24,
  VB_ENGINE_ERROR_NOT_STARTED = -25,
  VB_ENGINE_ERROR_TIMERS = -26,
  VB_ENGINE_ERROR_ALREADY_STARTED = -27,
  VB_ENGINE_ERROR_METRICS = -28,
  VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP = -29,
  VB_ENGINE_ERROR_ALIGN = -30,
  VB_ENGINE_ERROR_CLOCK = -31,
  VB_ENGINE_ERROR_INVALID_PLANID = -32,
  VB_ENGINE_ERROR_EA_THREAD_STOP = -33,
  VB_ENGINE_ERROR_BAD_CLUSTER_ID = -34,
  VB_ENGINE_ERROR_NOT_SYNCED = -35,
  VB_ENGINE_ERROR_NO_MEMORY = -36,
  VB_ENGINE_ERROR_NO_AVAILABLE_SEED = -37,
  VB_ENGINE_ERROR_NO_AVAILABLE_DID = -38,

} t_VB_engineErrorCode;

typedef enum
{
  VB_ENGINE_BOOST_MODE_AUTO = 0,
  VB_ENGINE_BOOST_MODE_FORCED_FULL,
  VB_ENGINE_BOOST_MODE_FORCED_LOW,
  VB_ENGINE_BOOST_MODE_LAST
} t_vbEngineBoostMode;

typedef enum
{
  VB_MAIN_EVENT_MAIN_END,
} t_VB_Process_Event;

/*
 * Engine FSM events:
 * Modifying this enum implies updating:
 * - t_vbEngineCountersIndex
 * - VB_COUNTERS_NAME
 * - vbEngineEventString
 */
typedef enum {
  ENGINE_EV_KILL_ALL = 0,
  ENGINE_EV_KILL,
  ENGINE_EV_CONNECT,
  ENGINE_EV_DISCONNECT,
  ENGINE_EV_BOOST_UPDATE,
  ENGINE_EV_BOOST_UPDATE_END_OK,
  ENGINE_EV_BOOST_UPDATE_END_KO,
  ENGINE_EV_ALIGN_CHECK_RESTART,
  ENGINE_EV_ALIGN_PERIODIC_CHECK,
  ENGINE_EV_ALIGN_CHECK,
  ENGINE_EV_ALIGN_ALL_CLUSTERS,
  ENGINE_EV_ALIGN_CLUSTER_I,
  ENGINE_EV_ALIGN_DONE,
  ENGINE_EV_ALIGN_DONE_SKIP_MEAS,
  ENGINE_EV_ALIGN_CHANGE,
  ENGINE_EV_ALIGN_CHANGE_SYNC,
  ENGINE_EV_ALIGN_WAIT,
  ENGINE_EV_ALIGN_RESTART,
  ENGINE_EV_ALIGN_CONF_SYNC,
  ENGINE_EV_CLOCK_REQ,
  ENGINE_EV_CLOCK_REQ_SYNC,
  ENGINE_EV_CLOCK_FORCE_REQ,
  ENGINE_EV_MEASPLAN_FAIL,
  ENGINE_EV_MEASPLAN_RSP_SYNC,
  ENGINE_EV_MEASURE_END_SYNC,
  ENGINE_EV_COMPUTATION_OK,
  ENGINE_EV_COMPUTATION_KO,
  ENGINE_EV_MEAS_FORCE,
  ENGINE_EV_SNR_PROBES_MEAS_FORCE,
  ENGINE_EV_PSD_UPDATE_FORCE,
  ENGINE_EV_MEAS_COLLECT_END_NO_LINES,
  ENGINE_EV_DISC_VERS_TO,
  ENGINE_EV_DISC_STATE_TO,
  ENGINE_EV_DOMAINS_SYNC_TO,
  ENGINE_EV_CLOCK_RSP_TO,
  ENGINE_EV_ALIGN_CHECK_TO,
  ENGINE_EV_ALIGN_CHANGE_TO,
  ENGINE_EV_ALIGN_SYNC_TO,
  ENGINE_EV_MEASPLAN_RSP_TO,
  ENGINE_EV_MEAS_COMPLETE_TO,
  ENGINE_EV_MEAS_COLLECT_TO,
  ENGINE_EV_MEASPLAN_CANCEL_TO,
  ENGINE_EV_MEASPLAN_RESTART,
  ENGINE_EV_MEASPLAN_BUILD,
  ENGINE_EV_MEASPLAN_CANCEL_END_SYNC,
  ENGINE_EV_BOOST_UPDATE_END_TO,
  ENGINE_EV_BOOST_ALG_RUN_TO,
  ENGINE_EV_ALIGN_CONF_TO,
  ENGINE_EV_RX_NETWORK_CHANGE_TRG,
  ENGINE_EV_RX_VERSION_RSP,
  ENGINE_EV_RX_STATE_RSP,
  ENGINE_EV_RX_DOMAIN_RSP,
  ENGINE_EV_RX_EMPTY_DOMAIN_RSP,
  ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP,
  ENGINE_EV_RX_CYCQUERY_RSP,
  ENGINE_EV_RX_CYCQUERY_RSP_KO,
  ENGINE_EV_RX_CYCQUERY_RSP_INV,
  ENGINE_EV_RX_CYCCHANGE_RSP,
  ENGINE_EV_RX_CYCCHANGE_RSP_KO,
  ENGINE_EV_RX_MEASPLAN_RSP,
  ENGINE_EV_RX_MEASPLAN_CANCEL_RSP,
  ENGINE_EV_RX_MEASPLAN_ERR_TRG,
  ENGINE_EV_RX_MEASURE_BGN_RSP,
  ENGINE_EV_RX_MEASURE_CFR_RSP,
  ENGINE_EV_RX_MEAS_COLLECT_END_TRG,
  ENGINE_EV_RX_TRAFFIC_AWARENESS_TRG,
  ENGINE_EV_RX_CLOCK_RSP,
  ENGINE_EV_RX_PSD_SHAPE_RSP,
  ENGINE_EV_RX_MEASURE_SNRPROBES_RSP,
  ENGINE_EV_RX_CDTA_RSP,
  ENGINE_EV_RX_ALIGNMODE_RSP,
  ENGINE_EV_RX_ALIGNMODE_RSP_KO,
  ENGINE_EV_RX_ALIGN_SYNC_LOST_TRG,
  ENGINE_EV_RX_ALIGN_CLUSTER_STOP_RSP,
  ENGINE_EV_RX_ALIGN_CLUSTER_STOP_TO,
  ENGINE_EV_RX_ALIGN_CLUSTERS_ALL_STOP_SYNC,
  ENGINE_EV_RX_ALIGN_CLUSTER_I_STOP_SYNC,
  ENGINE_EV_RX_ALIGN_CLUSTER_STOP_FAIL,
  ENGINE_EV_RX_NETWORK_CHANGE,
  ENGINE_EV_NETWORK_DIFF_EP_CHANGE,
  ENGINE_EV_NETWORK_DIFF_DM_REM,
  ENGINE_EV_ALIVE_SOCK_CHECK,
  ENGINE_EV_RX_ALIVE_SOCK_RSP,
  ENGINE_EV_LAST,

} t_VB_Comm_Event;

/// Alignment node role (G.hn alignment mode)
typedef enum
{
  VB_ALIGN_ROLE_NOT_INIT = 0,
  VB_ALIGN_ROLE_SLAVE,
  VB_ALIGN_ROLE_REF,
  VB_ALIGN_ROLE_RELAY,
  VB_ALIGN_ROLE_RELAY_CANDIDATE,
  VB_ALIGN_ROLE_LAST,
} t_alignRole;

typedef struct s_nodeAlignInfo
{
  t_alignRole role;
  INT32U      numExclusiveHit;
  BOOLEAN     visible;
  BOOLEAN     synced;
  BOOLEAN     hasBeenCandidate;
  INT8U       hops;
} t_nodeAlignInfo;

typedef struct s_psdBandAllocation
{
  INT16U        numBands100Mhz;
  INT16U        numBands200Mhz;
  INT16U        lastCarrier[VB_PSD_NUM_BANDS];
}t_psdBandAllocation;

typedef struct s_bandBoostCapacities
{
  INT32U       capacity1Boosted;
  INT32U       capacityAllBoosted;
}t_bandBoostCapacities;

typedef struct s_bandSettings
{
  INT8U                   numActiveBands[VB_ENGINE_QOS_RATE_LAST];
  INT8U                   forceNextTwoBandsBitmap[VB_ENGINE_QOS_RATE_LAST]; ///< bitmap of bands forced (the idea here is to force a n bands increment)
  INT8S                   tendency;
  BOOLEAN                 skipBand;
}t_bandSettings;


typedef struct s_cdtaUserProfile
{
  INT32U                  userWeight;
  INT32U                  slaWeight;
  INT32U                  userSLA;
}t_cdtaUserProfile;

typedef struct s_cdtaInfo
{
  INT8U                   hysteresisCounter;
  INT16U                  bpsReportedNBands;
  INT32S                  rateAdequationValue[VB_ENGINE_QOS_RATE_LAST];
  INT32U                  capacityPerRate[VB_ENGINE_QOS_RATE_LAST];
  INT32U                  bpsReportedCapacities[VB_PSD_NUM_BANDS][VB_ENGINE_QOS_RATE_LAST];
  t_bandSettings          bandSet;
  t_bandBoostCapacities   bandCapacities[VB_PSD_NUM_BANDS][VB_ENGINE_QOS_RATE_LAST];
  t_cdtaUserProfile       profile;
}t_nodeCdtaInfo;

typedef struct s_trafficReport
{
  INT16U  ingressTrafficP0;
  INT16U  ingressTrafficP1;
  INT16U  ingressTrafficP2;
  INT16U  ingressTrafficP3;
  INT8U   usageBuffP0;
  INT8U   usageBuffP1;
  INT8U   usageBuffP2;
  INT8U   usageBuffP3;
  INT16U  bpsCapacity;
  INT16U  realCapacity;
  INT16U  neededTheoricCapacity;
  INT16U  neededL2Xput;
  INT8U   macEfficiency;
  INT8U   debugCounter;
  INT16U  toLowBandHyst;
  INT16U  nBandsBps;
  INT16U  bpsBand[VB_PSD_NUM_BANDS];
  INT32U  rxReports;
  INT32U  consecL2XputSmaller;
  BOOLEAN reportsReceived;
}t_trafficReport;

typedef struct s_bandInfo
{
  INT16U       firstCarrier;
  INT16U       lastCarrier;
  INT32U       capacity;
}t_bandInfo;

typedef struct s_macsInfo
{
  INT8U *ptr;
  INT32U numNodes;
} t_macsInfo;

typedef struct s_nodesMacList
{
  t_macsInfo dmsMacs;
  t_macsInfo epsMacs;
} t_nodesMacList;

typedef struct
{
  INT32U numDms;
  INT32U numEps;
  INT32U numCompleteLines;
} t_vbEngineNumNodes;

typedef struct s_boostInfo
{
  t_vbEngineBoostMode  mode;
  INT8U                maxNumBands; ///< max number of bands (depends on SISO or MIMO mode of node and remote node)
  INT8U                forcedBandsBitmap;     ///< bitmap of bands forced to be present bit 0 -> band 0, bit 1 -> band 1 ...
  INT16U               level;     ///< Amount of bands in use (Bands as defined in engineConf)
  INT16U               levelCnf;  ///< Amount of bands in use (in the CNF Message)
  INT16U               lastLevel; ///< Amount of bands previously in use
  INT32U               bandsAge[VB_PSD_NUM_BANDS];
  INT16U               perc;      ///< Percentage of spectrum in use
  INT32U               lowBandCapacity;
  INT32U               maxCapacity;
  BOOLEAN              valid;
} t_boostInfo;

typedef struct s_nodeChannelSettings
{
  t_psdShape           psdShape;
  t_boostInfo          boostInfo;
  INT16U               firstValidCarrier;
  INT8U                interferenceDetectionCounter;
} t_nodeChannelSettings;

typedef struct s_node
{
  INT8U                 MAC[ETH_ALEN];
  CHAR                  MACStr[MAC_STR_LEN];
  INT8U                 devID;
  t_nodeType            type;
  t_nodeAlignInfo       nodeAlignInfo;
  t_additionalInfo1     addInfo1;
  t_nodeMeasures        measures;
  t_nodeChannelSettings channelSettings;
  t_trafficReport       trafficReports;
  t_vb_DevState         state;
  t_nodeCdtaInfo        cdtaInfo;
  CHAR                  stateFileName[VB_ENGINE_MAX_FILE_NAME_SIZE];
  struct s_node        *linkedNode;
} t_node;

typedef struct s_endPointList{
  INT16U  numEPs;
  t_node *epsArray;
} t_endPointList;

typedef struct s_Domain
{
  t_node          dm;
  t_alignInfo     alignInfo;
  t_endPointList  eps;
} t_domain;

typedef struct s_DomainsList
{
  INT16U numDomains;
  t_domain *domainsArray;
} t_domainsList;

typedef struct s_DriverTime
{
  pthread_mutex_t  mutex;
  struct timespec  lastClockReqTS;
  struct timespec  lastClockRx;
  struct timespec  adjClock;
  struct timespec  ownClock;
  INT16U           lastSeqNumberRx;
  BOOLEAN          lastSeqNumberValid;
  INT64S           rttNs;
  INT64S           deviationAbs;
  INT64S           deviationRel;
} t_DriverTime;


typedef struct
{
  INT32U numCLuster;
  INT32U list[10];
}t_ClusterCast;

typedef struct s_VBDriver t_VBDriver;

typedef struct s_TimeoutCnf
{
  timer_t         id;
  t_VB_Comm_Event event;
  t_VBDriver     *driver;
  t_ClusterCast   clusterCast;
  CHAR           *name;
  BOOLEAN         periodic;
  BOOLEAN         running;
} t_TimeoutCnf;

struct s_VBDriver
{
  t_linkedElement            l;
  t_domainsList              domainsList;
  pthread_mutex_t            domainsMutex;
  CHAR                       vbDriverID[VB_EA_DRIVER_ID_MAX_SIZE];
  t_vbEADesc                 vbEAConnDesc;
  t_vbEngineProcessFSMState  FSMState;
  CHAR                       remoteState[DRIVER_REMOTE_STATE_MAX_LEN];
  CHAR                       remoteVersion[VB_EA_VERSION_MAX_SIZE];
  CHAR                       versionFileName[VB_ENGINE_MAX_FILE_NAME_SIZE];
  CHAR                       remoteStateFileName[VB_ENGINE_MAX_FILE_NAME_SIZE];
  CHAR                       fsmStateFileName[VB_ENGINE_MAX_FILE_NAME_SIZE];
  t_DriverTime               time;
  t_TimeoutCnf               timeoutCnf;
  INT32U                     attempts;
  INT32U                     transactionMinTime;
  INT32S                     clusterId;
  BOOLEAN                    definitiveDriverId;
};

typedef struct
{
  t_VB_Process_Event vbEvent;
  t_VBDriver *thisDriver;
}t_VBMainMsg;

typedef struct
{
  t_VB_Comm_Event vbCommEvent;
  t_vbEAMsg      *msg;
  t_VBDriver     *senderDriver;
  void           *args;
  t_ClusterCast  clusterCast;
}t_VBProcessMsg;

typedef struct
{
  INT32U          clusterId;
  INT32U          numLines;
  INT8U           alignRef;
  INT8U           numRelays;
  INT8U           relays[VB_EA_ALIGN_GHN_MAX_RELAYS];
  INT8U           numRelaysCandidate;
  INT8U           relaysCandidate[VB_EA_ALIGN_GHN_MAX_RELAYS];
  struct timespec lastAlignCheck;
}t_vbEngineClusterInfo;

typedef struct s_VBCluster t_VBCluster;

struct s_VBCluster
{
  t_linkedElement            l;
  void                       *measurePlanData;
  void                       *cdtaData;
  INT8U                      *confReqBuffer;
  INT32U                     confReqBufferLen;
  struct timespec            applyOwnTs;
  struct timespec            applyDrvTs;
  INT16U                     applySeqNum;
  t_TimeoutCnf               timeoutCnf;
  BOOL                       snrComputationThreadRunning;
  pthread_t                  snrComputationThread;
  BOOLEAN                    epChange;
  BOOLEAN                    skipMeasPlan;
  BOOLEAN                    syncLostFlag;
  t_vbEngineClusterInfo      clusterInfo;
};

typedef struct s_VBDMsHistoryItem t_VBDMsHistoryItem;

struct s_VBDMsHistoryItem
{
  INT8U       MAC[ETH_ALEN];
  INT8U       status;
  INT32S      clusterId;
  BOOL        inactive;
  t_alignRole role;
};

typedef struct s_VBDMsHistory t_VBDMsHistory;

struct s_VBDMsHistory
{
  t_VBDMsHistoryItem *DMs;
  INT32U              NumDMs;
};

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Creates a new t_VBDriver struct
 * @param[in] driverId string with the id of the driver
 * @param[in] vbDriver pointer to a vbDriver pointer. It will point the new driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelCreateDriver(CHAR *driverId, t_VBDriver **vbDriver);

/**
 * @brief This function destroys all data structures contained inside t_VBDriver
 * @param[in] vbDriver Pointer to driver.
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineDatamodelDriverMemRelease(t_VBDriver *vbDriver);

/**
 * @brief This function destroys all data structures contained inside t_VBDriver and releases the driver itself.
 * @param[in,out] vbDriver Pointer to driver. It will be set to NULL when release is done.
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineDatamodelDriverDel(t_VBDriver **vbDriver);

/**
 * @brief Set driver Id
 * @param[in] driverId String contaning the driverId to be set
 * @param[in] driver Pointer to a t_VBDriver
 * @param[in] definitiveDriverId TRUE: given driver Id is the definitive value; FALSE: given driver Id is temporal, it will be retrieved later
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineDatamodelDriverIdSet(CHAR *driverId, t_VBDriver *driver, BOOLEAN definitiveDriverId);

/**
 * @brief Shows if given node is ready to boost
 * @param[in] node Pointer to node to check
 * @return TRUE: Node is ready; FALSE: otherwise
 **/
BOOLEAN VbEngineDatamodelNodeIsReadyToBoost(t_node *node);

/**
 * @brief Gets current boostStateCnf string
 * @param[in] node Pointer to node
 * @return String with boostState
 **/
CHAR *VbEngineDatamodelBoostedCnfStrGet(t_node *node);

/**
 * @brief Destroy t_VBProcessMsg and free memory
 * @param[in] processMsg message to be destroyed
 **/
/**
 * @brief Sets the first valid carrier of given node
 * @param[in] node Pointer to node to update
 * @param[in] firstCarrier First valid carrier value to set
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains list mutex shall be grabbed before calling this function
 **/
t_VB_engineErrorCode VbEngineDataModelChannelCapacityFirstCarrierSet(t_node *node, INT16U firstCarrier);

/**
 * @brief Sends an event to main thread
 * @param[in] event Event to send
 * @param[in] vbQueue Queue file descriptor
 * @param[in] thisDriver Pointer to driver structure
 **/
void VbEngineMainQEvSend(t_VB_Process_Event event, mqd_t vbQueue, t_VBDriver *thisDriver);

/**
 * @brief Destroy all measure structs and frees memory
 * @param[in] measure Pointer to measure structure
 **/
void VbDatamodelNodeProcessMeasureDestroy( t_processMeasure *measure );

/**
 * @brief Parses new domains list and check lost lines
 * @param[in] driver Pointer to driver structure
 * @param[in] newDomainsList Pointer to new domains list
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelListDomainsLostCheck(t_VBDriver *driver, t_domainsList *newDomainsList);

/**
 * @brief Recovers data from last domains list
 * @param[in] driver Pointer to driver structure
 * @param[in,out] newDomainsList Pointer to new domains list to update
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelListDomainsRecoverData( t_VBDriver *driver,  t_domainsList *newDomainsList);

/**
 * @brief Releases all previously allocated memory for given domains list
 * @param[in] driver Pointer to related driver
 * @param[in] domainsList Domains list to remove
 * @remarks Driver's mutex is not grabbed inside this function.
 * It shall be grabbed before calling this function.
 **/
void VbEngineDatamodelListDmDestroy(t_VBDriver *driver, t_domainsList *domainsList);

/**
 * @brief Releases all previously allocated memory for domains list of given driver
 * @param[in] thisDriver Pointer to driver structure
 * @remarks Driver's mutex is grabbed inside this function.
 **/
void VbEngineDatamodelListDomainsDestroy(t_VBDriver *thisDriver);

/**
 * @brief Finds a node by MAC in given VB driver
 * @param[in] driver Pointer to driver to search
 * @param[in] mac Pointer to MAC address to search
 * @param[out] node Pointer to node
 * @return @ref t_VB_engineErrorCode
 * @pre Domains list mutex shall be grabbed before calling this function
 **/
t_VB_engineErrorCode VbEngineDatamodelNodeFind(t_VBDriver *driver, const INT8U *mac, t_node **node);

/**
 * @brief Sets the boost mode of a given node
 * @param[in] nodeMac MAC to search
 * @param[in] boostMode Boost mode to set
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains mutex is grabbed inside this function
 **/
t_VB_engineErrorCode VbEngineBoostModeByMacSet(INT8U *nodeMac, t_vbEngineBoostMode boostMode);

/**
 * @brief Calculates the required capacity ratio of a given node
 * @param[in] node Pointer to node
 * @param[out] reqCap Required capacity in percentage
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineReqCapacityRatioCalc( INT16U needed, INT16U reference, INT32U *reqCap);

/**
 * @brief Configures a timer to send the given eventId after timeoutMs
 * @param[in] driver Pointer to driver to send the event
 * @param[in] eventId Event to send
 * @param[in] timeoutMs Timeout in msecs
 * @param[in] name Timeout name (for debug purposes)
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDriverTimeoutStart(t_VBDriver *driver, t_VB_Comm_Event eventId, INT32U timeoutMs, CHAR *name);

/**
 * @brief Stops and releases the timer related to given driver
 * @param[in] driver Pointer to driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDriverTimeoutStop(t_VBDriver *driver);

/**
 * @brief Configures a timer to send the given eventId after timeoutMs
 * @param[in] driver Pointer to driver to send the event
 * @param[in] eventId Event to send
 * @param[in] timeoutMs Timeout in msecs
 * @param[in] periodic TRUE: configures the timer as a periodic task; FALSE: the timer is one-shot
 * @param[in] name Timeout name (for debug purposes)
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineTimeoutStart(t_TimeoutCnf *timer, t_VB_Comm_Event eventId, INT32U timeoutMs, BOOLEAN periodic, CHAR *name);

/**
 * @brief Stops and releases the timer related to given driver
 * @param[in] driver Pointer to driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineTimeoutStop(t_TimeoutCnf *timer);

/**
 * @brief Counts the total number of DMs, EPs and complete lines present in the system
 * @param[out] numNodes Structure with number of DMs, EPs and complete lines.
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDataModelNumNodesGet(t_vbEngineNumNodes *numNodes);

t_VB_engineErrorCode VbEngineDataModelNumNodesInClusterXGet(INT32U clusterId, t_vbEngineNumNodes *numNodes);

/**
 * @brief Counts the total number of DMs, EPs and complete lines present in given driver
 * @param[in] driver Pointer to driver.
 * @param[out] numNodes Structure with number of DMs, EPs and complete lines.
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelNumNodesInDriverGet(t_VBDriver *driver, t_vbEngineNumNodes *numNodes);

/**
 * @brief Sets alignment info to a given domain
 * @param[in] driver Pointer to driver structure
 * @param[in] mac MAC address of domain master
 * @param[in] alignInfo Alignment info structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelAlignInfoSet(t_VBDriver *driver, INT8U *mac, t_alignInfo *alignInfo);

/**
 * @brief Shows if given domain has an EP
 * @param[in] domain Pointer to domain
 * @return TRUE: domain has an EP; FALSE: otherwise
 **/
BOOLEAN VbEngineDatamodelDomainIsComplete(t_domain *domain);

/**
 * @brief Shows if given domain is new
 * @param[in] domain Pointer to domain
 * @return TRUE: domain is new; FALSE: otherwise
 **/
BOOLEAN VbEngineDatamodelDomainIsNew(t_domain *domain);

/**
 * @brief Gets number of domains in given driver
 * @param[in] driver Pointer to driver
 * @return Number of domains in driver
 * @remarks Domains mutex of given driver is grabbed.
 **/
INT32U VbEngineDatamodelNumDomainsByDriverGet(t_VBDriver *driver);

/**
 * @brief Gets DMs and EPs MAC list for a given driver
 * @param[in] driver Pointer to driver
 * @param[out] macList MAC list of DMs and EPs present in driver
 * @param[in] onlyCompleteLines Get MACs only from complete lines (where DM and EP are present)
 * @return @ref t_VB_engineErrorCode
 * @remarks Both lists (DM and EP) are allocated inside this function and
 * shall be released by the caller when no longer needed.
 **/
t_VB_engineErrorCode VbEngineDatamodelNodesMacGet(t_VBDriver *driver, t_nodesMacList *macList, BOOLEAN onlyCompleteLines);

/**
 * @brief Gets DMs and EPs MAC list on a cluster
 * @param[out] macList MAC list of all DMs and EPs
 * @param[in] onlyCompleteLines Get MACs only from complete lines (where DM and EP are present)
 * @param[in] clusterId Cluster ID
 * @return @ref t_VB_engineErrorCode
 * @remarks Both lists (DM and EP) are allocated inside this function and
 * shall be released by the caller when no longer needed.
 **/
t_VB_engineErrorCode VbEngineDatamodelClusterXAllNodesMacGet(t_nodesMacList *macList, BOOLEAN onlyCompleteLines, INT32U clusterId);

/**
 * @brief Gets DMs and EPs MAC list
 * @param[out] macList MAC list of all DMs and EPs
 * @param[in] onlyCompleteLines Get MACs only from complete lines (where DM and EP are present)
 * @return @ref t_VB_engineErrorCode
 * @remarks Both lists (DM and EP) are allocated inside this function and
 * shall be released by the caller when no longer needed.
 **/
t_VB_engineErrorCode VbEngineDatamodelAllNodesMacGet(t_nodesMacList *macList, BOOLEAN onlyCompleteLines);

/**
 * @brief Releases allocated memory for given MAC list
 * @param[in] macList MAC list to release
 **/
void VbEngineDatamodelMacListRelease(t_nodesMacList *macList);

/**
 * @brief Releases allocated memory for given cross measure list
 * @param[in] crossMeasureList Pointer to cross measure list to release
 **/
void VbEngineDatamodelNodeCrossMeasureListDestroy( t_crossMeasureList *crossMeasureList );

/**
 * @brief Count engine event into counters variable
 * @param[in] event event to be counted
 * @return @ref t_vbEngineCountersIndex
 **/
t_vbEngineCountersIndex VbEngineDatamodelEvToCounter(t_VB_Comm_Event event);

/**
 * @brief Add a list of DMs in the nodes data base
 * @param[in] driver Pointer to driver
 * @param[in] numAddedDms number of DMs to be added
 * @param[in] dmsInfo nodes information
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelListDomainsDMsAdd(t_VBDriver *driver, INT32U numAddedDms, t_vbEADomainDiffRspDMAdded *dmsInfo);

/**
 * @brief Add a list of EPs in the nodes data base
 * @param[in] driver Pointer to driver
 * @param[in] numAddedEps number of EPs to be added
 * @param[in] epsInfo nodes information
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelListDomainsEPsAdd(t_VBDriver *driver, INT32U numAddedEps, t_vbEADomainDiffRspEPAdded *epsInfo);

/**
 * @brief Remove a list of DMs in the nodes data base
 * @param[in] driver Pointer to driver
 * @param[in] numRemDms number of DMs to be removed
 * @param[in] remMacdms DMs MACs to be removed
 * @param[out] refOrRelay indicate if the removed DM was Reference or Relay
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelListDomainsDMsRem(t_VBDriver *driver, INT32U numRemDms, INT8U *remMacdms, BOOL *refOrRelay);

/**
 * @brief Remove a list of EPs in the nodes data base
 * @param[in] driver Pointer to driver
 * @param[in] numRemEps number of EPss to be added
 * @param[in] remMacEps EPS MACs to be removed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelListDomainsEPsRem(t_VBDriver *driver, INT32U numRemEps, INT8U *remMacEps);

/**
 * @brief Gets the name of given QoS rate
 * @param[in] rate QoS DS/US rate
 * @return String of given DS/US rate
 **/
const CHAR *VbEngineQosRateToStrGet(t_vbEngineQosRate rate);

/**
 * @brief Free cluster memory
 * @param[in] vbCluster pointer to cluster
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelClusterMemFree(t_VBCluster *vbCluster);

/**
 * @brief Create cluster
 * @param[in] vbCluster Pointer to cluster
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelCreateCluster(t_VBCluster **vbCluster);

/**
 * @brief Delete cluster
 * @param[in] vbCluster Pointer to cluster
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelClusterDel(t_VBCluster **vbCluster);


/**
 * @brief Set the seed index that is currently used by the node passed as parameter
 * @param[in] nodeMac   MAC to search to update seed index value
 * @param[in] seedIndex Seed index value
 * @param[in] did       Domain index
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineSeedIndexByMacSet(INT8U *nodeMac, INT16U seedIndex, INT16U did);

/**
 * @brief Searches for a given domain by MAC address
 * @param[in] clusterId Cluster Id
 * @param[in] mac MAC address
 * @param[out] domain Target domain pointer
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelClusterXDomainByMacGet(INT32U clusterId, INT8U *mac, t_domain **domain);

/**
 * @brief Update line status (form NEW to PRESENT
 * @param[in] clusterId Cluster Id
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineLineStatusUpdate(INT32U clusterId, t_vb_DevState state);

/**
 * @brief Starts VB Engine datamodel
 **/
void VbEngineDatamodelStart(void);

/**
 * @brief Stops VB Engine datamodel
 **/
void VbEngineDatamodelStop(void);

#endif /* VB_ENGINE_DATAMODEL_H_ */

/**
 * @}
**/
