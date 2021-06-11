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
 * @file vb_ea_communication.h
 * @brief External agent (VB driver <-> VB engine) communication interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016/12/05
 *
 **/

#ifndef VB_EA_COMMUNICATION_H_
#define VB_EA_COMMUNICATION_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <arpa/inet.h>
#include <linux/if.h>
#include <netinet/in.h>
#include <mqueue.h>
#include <netdb.h>

#include "compiler.h"
#include "vb_types.h"
#include "vb_console.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define VB_EA_PSD_SHAPE_STATUS_ERR             (0)
#define VB_EA_PSD_SHAPE_STATUS_OK              (1)

#define VB_EA_CODE_FLAG                        (0xA5)
#define VB_EA_HEADER_SIZE                      (sizeof(t_vbEAFrameHeader))
#define VB_EA_PAYLOAD_OFFSET                   (VB_EA_HEADER_SIZE)
#define VB_EA_MEASURE_PLAN_RSP_SIZE            (sizeof(t_vbEAMeasurePlanRsp))
#define VB_EA_MEASURE_PLAN_CANCEL_REQ_SIZE     (sizeof(t_vbEAMeasurePlanCancelReq))
#define VB_EA_MEASURE_END_TRG_SIZE             (sizeof(t_vbEAMeasureEndTrg))
#define VB_EA_DOMAINS_FRAME_DM_SIZE            (sizeof(t_vbEADomainRspDM))
#define VB_EA_DOMAINS_FRAME_EP_SIZE            (sizeof(t_vbEADomainRspEP))
#define VB_EA_DOMAINSFRAME_DOMAINS_OFFSET      (2)
#define VB_EA_BGNFRAME_MEASURE_HEADER_SIZE     (sizeof(t_vbEABGNMeasure))
#define VB_EA_BGNFRAME_MEASURE_HEADER_SIZE     (sizeof(t_vbEABGNMeasure))
#define VB_EA_CFRFRAME_MEASURE_HEADER_SIZE     (sizeof(t_vbEACFRMeasure))
#define VB_EA_SNRPROBE_MEASURE_HEADER_SIZE     (sizeof(t_vbEASNRMeasure))
#define VB_EA_CLOCK_RSP_SIZE                   (sizeof(t_vbEAClockRsp))
#define VB_EA_CYCQUERY_REQ_SIZE                (sizeof(t_vbEACycQueryReq))
#define VB_EA_CYCQUERY_RSP_COMMON_SIZE         (sizeof(t_vbEACycQueryRspCommon))
#define VB_EA_CYCQUERY_RSP_NODE_SIZE           (sizeof(t_vbEACycQueryRspNode))
#define VB_EA_TRAFFIC_REPORT_HDR_SIZE          (sizeof(t_vbEATrafficReportHdr))
#define VB_EA_TRAFFIC_REPORT_RSP_SIZE          (sizeof(t_vbEATrafficReportRsp))
#define VB_EA_PSD_SHAPING_CNF_SIZE             (sizeof(t_vbEAPsdShapingCnf))
#define VB_EA_CDTA_CNF_SIZE                    (sizeof(t_vbEACdtaCnf))
#define VB_EA_MEASURE_PLAN_CANCEL_CNF_SIZE     (sizeof(t_vbEAMeasurePlanCancelCnf))
#define VB_EA_CYCCHANGE_REQ_COMMON_SIZE        (sizeof(t_vbEACycChangeReqCommon))
#define VB_EA_THREAD_NAME_LEN                  (50)
#define VB_EA_CYCCHANGE_REQ_NODE_SIZE          (sizeof(t_vbEACycChangeReqNode))
#define VB_EA_CYCCHANGE_RSP_SIZE               (sizeof(t_vbEACycChangeRsp))
#define VB_EA_MEAS_COLLECT_REQ_HDR_SIZE        (sizeof(t_vbEAMeasCollectReqHdr))
#define VB_EA_MEAS_COLLECT_REQ_NODE_SIZE       (sizeof(t_vbEAMeasCollectReqNode))
#define VB_EA_MEAS_COLLECT_END_TRG_SIZE        (sizeof(t_vbEAMeasureCollectEnd))
#define VB_EA_PSD_SHAPE_REQ_HDR_SIZE           (sizeof(t_vbEAPSDShapeHdr))
#define VB_EA_CDTA_REQ_HDR_SIZE                (sizeof(t_vbEACDTAHdr))
#define VB_EA_PSD_SHAPE_REQ_STEP_HDR_SIZE      (sizeof(t_vbEAPSDShapeStepHdr))
#define VB_EA_PSD_SHAPE_REQ_STEP_SIZE          (sizeof(t_vbEAPSDShapeStep))
#define VB_EA_ALIGN_SYNC_LOST_TRG_SIZE         (sizeof(t_vbEAAlignSyncLost))
#define VB_EA_CLUSTER_STOP_REQ_SIZE            (sizeof(t_vbEAAlignClusterStopReq))
#define VB_EA_SOCKET_ALIVE_REQ_SIZE            (sizeof(t_vbEASocketAliveReq))
#define VB_EA_VERSION_MAX_SIZE                 (31) // "xx.xx ryyyy" + null byte
#define VB_EA_DRIVER_ID_MAX_SIZE               (21) // 20 digits + null byte

#define VB_EA_ALIGN_GHN_MAX_RELAYS             (VB_ALIGN_GHN_MAX_RELAYS)
#define VB_EA_ALIGN_GHN_MAX_TX_NODES           (VB_ALIGN_GHN_MAX_TX_NODES)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef enum
{
  VB_EA_ERR_NONE = 0,
  VB_EA_ERR_NOT_SUPPORTED = -1,               ///< Requested action not supported (ex.: dummy package, ...)
  VB_EA_ERR_NOT_INITIALIZED = -2,             ///< Component not initialized
  VB_EA_ERR_NOT_STARTED = -3,                 ///< Component not started
  VB_EA_ERR_BAD_ARGS = -4,                    ///< Wrong parameter(s)
  VB_EA_ERR_NO_MEMORY = -5,                   ///< Not enough memory
  VB_EA_ERR_NOT_FOUND = -6,                   ///< Param not found
  // Error codes from 0 to -31 inclusive are reserved to be common errors to all components
  VB_EA_ERR_OTHER = -31,                      ///< Unknown error
  VB_EA_ERR_QUEUE = -32,                      ///< Queue related error
  VB_EA_ERR_SOCKET = -33,                     ///< Socket related error
  // Errors specific to the component are defined here starting at -32

} t_vbEAError;

/*
 * EA OPCODEs:
 * Modifying this enum implies updating:
 * - eaLookupEvents
 * - vb_EAI_lookup_events
 * - eaOpcodeString
 */
typedef enum
{
  VB_EA_OPCODE_MEASURE_PLAN_REQ        = 0x00,
  VB_EA_OPCODE_MEASURE_PLAN_RESP       = 0x01,
  VB_EA_OPCODE_DOMAIN_REQ              = 0x02,
  VB_EA_OPCODE_DOMAIN_RESP             = 0x03,
  VB_EA_OPCODE_BGN_RESP                = 0x04,
  VB_EA_OPCODE_CFR_RESP                = 0x05,
  VB_EA_OPCODE_SNRPROBES_REQ           = 0x06,
  VB_EA_OPCODE_SNRPROBES_RESP          = 0x07,
  VB_EA_OPCODE_PSD_REQ                 = 0x08,
  VB_EA_OPCODE_PSD_RESP                = 0x09,
  VB_EA_OPCODE_PSD_SHAPE_CFG           = 0x0A,
  VB_EA_OPCODE_PSD_SHAPE_CFM           = 0x0B,
  VB_EA_OPCODE_TRAFFIC_AWARENESS_TRG   = 0x0C,
  VB_EA_OPCODE_NETWORK_CHANGE_TRG      = 0x0D,
  VB_EA_OPCODE_VERSION_REQ             = 0x0E,
  VB_EA_OPCODE_VERSION_RESP            = 0x0F,
  VB_EA_OPCODE_VBDRIVER_STATE_REQ      = 0x10,
  VB_EA_OPCODE_VBDRIVER_STATE_RESP     = 0x11,
  VB_EA_OPCODE_MEASURE_ERR_NOTIFY      = 0x12,
  VB_EA_OPCODE_VBDRIVER_STATE_TRG      = 0x13,
  VB_EA_OPCODE_CLOCK_REQ               = 0x14,
  VB_EA_OPCODE_CLOCK_RSP               = 0x15,
  VB_EA_OPCODE_CYCQUERY_REQ            = 0x16,
  VB_EA_OPCODE_CYCQUERY_RSP            = 0x17,
  VB_EA_OPCODE_CYCCHANGE_REQ           = 0x18,
  VB_EA_OPCODE_CYCCHANGE_RSP           = 0x19,
  VB_EA_OPCODE_MEAS_COLLECT_REQ        = 0x1A,
  VB_EA_OPCODE_MEAS_COLLECT_RSP        = 0x1B,
  VB_EA_OPCODE_MEAS_COLLECT_END        = 0x1C,
  VB_EA_OPCODE_MEASURE_PLAN_CANCEL_REQ = 0x1D,
  VB_EA_OPCODE_MEASURE_PLAN_CANCEL_RSP = 0x1E,
  VB_EA_OPCODE_CDTA_CFG                = 0x1F,
  VB_EA_OPCODE_CDTA_CFM                = 0x20,
  VB_EA_OPCODE_REDIRECT_REQ            = 0x21,
  VB_EA_OPCODE_ENGINE_CONF_REQ         = 0x22,
  VB_EA_OPCODE_ENGINE_CONF_RSP         = 0x23,
  VB_EA_OPCODE_ALIGN_SYNC_LOST_TRG     = 0x24,
  VB_EA_OPCODE_ALIGN_STOP_CLUSTER_REQ  = 0x25,
  VB_EA_OPCODE_ALIGN_STOP_CLUSTER_RSP  = 0x26,
  VB_EA_OPCODE_NETWORK_CHANGE_REPORT   = 0x27,
  VB_EA_OPCODE_SOCKET_ALIVE_REQUEST    = 0x28,
  VB_EA_OPCODE_SOCKET_ALIVE_RESP       = 0x29,
  VB_EA_OPCODE_LAST,
} t_vbEAOpcode;

typedef enum
{
  VB_EA_MEAS_RSP_ERR_NONE             = 0x00,
  VB_EA_MEAS_RSP_ERR_PROCESS          = 0x01,
  VB_EA_MEAS_RSP_ERR_PLANID           = 0x02,
  VB_EA_MEAS_RSP_ERR_MAC              = 0x03,
  VB_EA_MEAS_RSP_ERR_MEASURE          = 0x04,
  VB_EA_MEAS_RSP_ERR_CROSSMAC         = 0x05,
  VB_EA_MEAS_RSP_ERR_ALREADY_RUNNING  = 0x06,
  VB_EA_MEAS_RSP_ERR_UNKNOWN          = 0x07,
  VB_EA_MEAS_RSP_ERR_NO_RESOURCES     = 0x08,
} t_vbEAMeasRspErrorCode;

typedef enum
{
  VB_EA_CYCQUERY_RSP_ERR_NONE               = 0x00,
  VB_EA_CYCQUERY_RSP_ERR_INVALID_TIME       = 0x01,
  VB_EA_CYCQUERY_RSP_ERR_INVALID_NOTIF      = 0x02,
  VB_EA_CYCQUERY_RSP_ERR_NO_NOTIF           = 0x03,
  VB_EA_CYCQUERY_RSP_ERR_OTHER              = 0x04,
} t_vbEACycQueryRspErrorCode;

typedef enum
{
  VB_EA_CYCCHANGE_RSP_ERR_NONE               = 0x00,
  VB_EA_CYCCHANGE_RSP_ERR_OTHER              = 0x01,
} t_vbEACycChangeRspErrorCode;

typedef enum
{
  VB_EA_ALIGNMODE_RSP_ERR_NONE               = 0x00,
  VB_EA_ALIGNMODE_RSP_ERR_DRV                = 0x01,
  VB_EA_ALIGNMODE_RSP_ERR_ID                 = 0x02,
} t_vbEAAlignModeRspErrorCode;

typedef enum
{
  VB_EA_ALIGNCLUSTERSTOP_RSP_ERR_NONE        = 0x00,
  VB_EA_ALIGNCLUSTERSTOP_RSP_ERR_OTHER       = 0x01,
} t_vbEAAlignClusterStopRspErrorCode;

typedef enum
{
  VB_EA_DOMAIN_REPORT_FULL        = 0x00,
  VB_EA_DOMAIN_REPORT_DIFF       = 0x01,
} t_vbEADomainReportType;

/// EA thread types
typedef enum
{
  VB_EA_TYPE_SERVER = 0,      ///< Server thread. It accepts new connections and calls to connectCb when new connection arrives
  VB_EA_TYPE_SERVER_CONN,     ///< Connection thread. It shall be created after server thread accepts a new connection. Receives EA messages and calls processRxMsgCb. It calls disconnectCb when the connection is closed.
  VB_EA_TYPE_CLIENT,          ///< Client thread. It connects to specified server and calls to connectCb. Receives EA messages and calls processRxMsgCb. It calls disconnectCb when the connection is closed.
  VB_EA_TYPE_LAST,
} t_vbEAType;

typedef struct
{
  INT8U       *msg;
  INT32U       msgLen;
} t_vbMsg;

typedef struct
{
  t_vbMsg      eaFullMsg; // The whole EA message
  t_vbMsg      eaPayload; // The payload of the EA message
  t_vbEAOpcode opcode;
} t_vbEAMsg;

typedef struct
{
  INT32U          cnt;
  struct timespec timeStamp;
} t_vbEADbgEntry;

typedef struct
{
  t_vbEADbgEntry  txTable[VB_EA_OPCODE_LAST];
  t_vbEADbgEntry  rxTable[VB_EA_OPCODE_LAST];
} t_vbEADbgTable;

typedef struct s_vbEADesc t_vbEADesc;
typedef void (*t_vbEACloseCb)(t_vbEADesc *desc);
typedef void (*t_vbEADisconnectCb)(t_vbEADesc *desc);
typedef t_vbEAError (*t_vbEAConnectCb)(t_vbEADesc *desc, struct sockaddr_in6 clientAddr, INT32S sockFd);
typedef void (*t_vbEAProcessRxMsgCb)(t_vbEADesc *desc, INT8U *msg, INT32U len);
typedef void (*t_vbEAThreadStartCb)(t_vbEADesc *desc);

/// Server/Client connection descriptor
struct s_vbEADesc
{
  t_vbEAType            type;                  ///< INPUT  param: EA thread type
  CHAR                  thrName[VB_EA_THREAD_NAME_LEN];///< INPUT  param: Thread name
  pthread_t             threadId;              ///< OUTPUT param: Thread Id
  CHAR                 *queueName;             ///< INPUT  param: Posix queue to open to send EA messages
  mqd_t                 queueId;               ///< OUTPUT param: Posix queue Id
  pthread_mutex_t       mutex;                 ///< OUTPUT param: Mutex to protect the socket descriptor
  struct sockaddr_in6   serverAddr;            ///< INPUT  param: Server IP address and port
  struct sockaddr_in6   clientAddr;            ///< OUTPUT param: Client IP address and port
  struct addrinfo      *clientInfo;            ///< OUTPUT param: Client info returned by 'getaddrinfo'
  INT32S                sockFd;                ///< OUTPUT param: Socket descriptor
  INT8U                 iface[IFNAMSIZ];       ///< INPUT  param: Interface name (only used in server side)
  INT8U                *buffer;                ///< OUTPUT param: Allocated buffer to store received messages
  t_vbEACloseCb         closeCb;               ///< INPUT  param: Callback called when thread finishes
  t_vbEADisconnectCb    disconnectCb;          ///< INPUT  param: Callback called when connection is closed
  t_vbEAConnectCb       connectCb;             ///< INPUT  param: Callback called when a new connection is opened
  t_vbEAProcessRxMsgCb  processRxMsgCb;        ///< INPUT  param: Callback called when a new message is received
  t_vbEAThreadStartCb   threadStartCb;         ///< INPUT  param: Callback called when a new thread is in ready state
  BOOLEAN               running;               ///< OUTPUT param: TRUE: thread is running; FALSE: otherwise
  BOOLEAN               connected;             ///< OUTPUT param: TRUE: connection is opened; FALSE: otherwise
  INT32U                socketAliveCounter;    ///<
  void                 *args;                  ///< INPUT  param: Generic arguments pointer
  t_vbEADbgTable        debugTable;            ///< OUTPUT param: Debug counters for this interface
};

struct PACKMEMBER _vbEAFrameHeader
{
  INT8U VBCode;
  INT16U length;
  INT8U opcode;
};
typedef struct _vbEAFrameHeader TYPE_ALIGNED32(t_vbEAFrameHeader);

struct PACKMEMBER _vbEAMeasurePlanRsp
{
  INT8U planID;
  INT8U errorCode;
};
typedef struct _vbEAMeasurePlanRsp TYPE_ALIGNED32(t_vbEAMeasurePlanRsp);

struct PACKMEMBER _vbEAPsdShapingCnf
{
  INT8U status;
};
typedef struct _vbEAPsdShapingCnf TYPE_ALIGNED32(t_vbEAPsdShapingCnf);

struct PACKMEMBER _vbEACdtaCnf
{
  INT8U status;
};
typedef struct _vbEACdtaCnf TYPE_ALIGNED32(t_vbEACdtaCnf);

struct PACKMEMBER _vbEAMeasurePlanCancelReq
{
  INT8U planID;
};
typedef struct _vbEAMeasurePlanCancelReq TYPE_ALIGNED32(t_vbEAMeasurePlanCancelReq);

struct PACKMEMBER _vbEAMeasurePlanCancelCnf
{
  INT8U planID;
  INT8U status;
};
typedef struct _vbEAMeasurePlanCancelCnf TYPE_ALIGNED32(t_vbEAMeasurePlanCancelCnf);

struct PACKMEMBER _vbEAMeasureEndTrg
{
  INT8U planID;
  INT8U errorCode;
};
typedef struct _vbEAMeasureEndTrg TYPE_ALIGNED32(t_vbEAMeasureEndTrg);

struct PACKMEMBER _vbEADomainRspDM
{
  INT8U  DM_MAC[ETH_ALEN];
  INT8U  fwVersion[VB_FW_VERSION_LENGTH];
  INT8U  DM_ID;
  INT16U qosRate;
  INT16U maxLengthTxop;
  INT16U DM_Extseed;
  INT16U NumEps;
};
typedef struct _vbEADomainRspDM TYPE_ALIGNED32(t_vbEADomainRspDM);

struct PACKMEMBER _vbEADomainRspEP
{
  INT8U EP_MAC[ETH_ALEN];
  INT8U fwVersion[VB_FW_VERSION_LENGTH];
  INT8U EP_ID;
};
typedef struct _vbEADomainRspEP TYPE_ALIGNED32(t_vbEADomainRspEP);

struct PACKMEMBER _vbEAMeasurePlanReq
{
  INT8U MAC[ETH_ALEN];
  INT8U PlanID;
};
typedef struct _vbEAMeasurePlanReq TYPE_ALIGNED32(t_vbEAMeasurePlanReq);

struct PACKMEMBER _vbEAMeasureCollectEnd
{
  INT8U PlanID;
};
typedef struct _vbEAMeasureCollectEnd TYPE_ALIGNED32(t_vbEAMeasureCollectEnd);

struct PACKMEMBER _vbEABGNMeasure
{
  INT8U   MAC[ETH_ALEN];
  INT8U   ErrorCode;
  INT16U  numCarriers;
  INT16U  firstCarrier;
  INT8U   spacing;
  INT8U   flags;
  INT8S   rxg1Compensation;
  INT8S   rxg2Compensation;
  BOOLEAN mimoInd;
  BOOLEAN mimoMeas;
  INT8U   planId;
};
typedef struct _vbEABGNMeasure TYPE_ALIGNED32(t_vbEABGNMeasure);

struct PACKMEMBER _vbEASNRMeasure
{
  INT8U   MAC[ETH_ALEN];
  INT8U   ErrorCode;
  INT16U  numCarriers;
  INT16U  firstCarrier;
  INT8U   spacing;
  INT8U   flags;
  INT8S   rxg1Compensation;
  INT8S   rxg2Compensation;
  BOOLEAN mimoInd;
  BOOLEAN mimoMeas;
};
typedef struct _vbEASNRMeasure TYPE_ALIGNED32(t_vbEASNRMeasure);

struct PACKMEMBER _vbEACFRMeasure
{
  INT8U   MACMeasurer[ETH_ALEN];
  INT8U   MACMeasured[ETH_ALEN];
  INT8U   ErrorCode;
  INT16U  numCarriers;
  INT16U  firstCarrier;
  INT8U   spacing;
  INT8U   flags;
  INT8S   rxg1Compensation;
  INT8S   rxg2Compensation;
  BOOLEAN mimoInd;
  BOOLEAN mimoMeas;
  INT8U   planId;
};
typedef struct _vbEACFRMeasure TYPE_ALIGNED32(t_vbEACFRMeasure);

struct PACKMEMBER _vbEAMeasCollectReqHdr
{
  INT8U  planID;
  INT8U  dataType;
  INT8U  formatType;
  INT16U numMACsMeasurer;
};
typedef struct _vbEAMeasCollectReqHdr TYPE_ALIGNED32(t_vbEAMeasCollectReqHdr);

struct PACKMEMBER _vbEAMeasCollectReqNode
{
  INT8U  macMeasurer[ETH_ALEN];
  INT16U numMACsMeasured;
  /// List of measured MACs
};
typedef struct _vbEAMeasCollectReqNode TYPE_ALIGNED32(t_vbEAMeasCollectReqNode);

/// trafficAwareness.trg header
struct PACKMEMBER _vbEATrafficReportHdr
{
  INT32U          numReports;                  ///< Number of traffic reports present in message
};
typedef struct _vbEATrafficReportHdr TYPE_ALIGNED32(t_vbEATrafficReportHdr);

/// trafficAwareness.trg
struct PACKMEMBER _vbEATrafficReportRsp
{
  INT8U           MAC[ETH_ALEN];               ///< Node MAC address
  INT32U          trafficPrio0:16;             ///< Ingress traffic estimation of priority 0 (in Mbps)
  INT32U          trafficPrio1:16;             ///< Ingress traffic estimation of priority 1 (in Mbps)
  INT32U          trafficPrio2:16;             ///< Ingress traffic estimation of priority 2 (in Mbps)
  INT32U          trafficPrio3:16;             ///< Ingress traffic estimation of priority 3 (in Mbps)
  INT32U          maxBuffPrio0:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          maxBuffPrio1:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          maxBuffPrio2:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          maxBuffPrio3:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          bpsCapacity:16;              ///< Channel capacity (in Mbps)
  INT32U          realCapacity:16;             ///< Effective channel capacity (taking into account overheads and tx time) (in Mbps)
  INT32U          neededTheoricCapacity:16;    ///< Desired channel capacity (in Mbps)
  INT32U          macEfficiency:8;             ///< MAC efficiency (in %)
  INT32U          padding:24;                  ///< Fill to reach 28 bytes size (Aligned 32)
};
typedef struct _vbEATrafficReportRsp TYPE_ALIGNED32(t_vbEATrafficReportRsp);

/// trafficAwareness.trg
struct PACKMEMBER _vbEATrafficBpsReport
{
  INT16U          nBands;                    ///< Node MAC address
  INT16U          bpsBand[VB_PSD_NUM_BANDS]; ///< Ingress traffic estimation of priority 0 (in Mbps)
};
typedef struct _vbEATrafficBpsReport TYPE_ALIGNED32(t_vbEATrafficBpsReport);


struct PACKMEMBER _vbEACDTAHdr
{
  INT16U   seqNumber;
  INT8U    numNodes;
  INT8U    qosRate;
  INT8U    defaultQosRate;
  INT8U    forceChannelEstimation;
  INT8U    numBandsFirstPSD; ///< PSD to apply first when waiting for engine info before transmitting
  INT8U    nBoostBands;      ///< number of potential boosted bands
  INT16U   bandCarrierDef[VB_PSD_NUM_BANDS];      ///< number of potential boosted bands

};
typedef struct _vbEACDTAHdr TYPE_ALIGNED32(t_vbEACDTAHdr);

struct PACKMEMBER _vbEAPSDShapeHdr
{
  INT16U seqNumber;
  INT8U  numNodes;
};
typedef struct _vbEAPSDShapeHdr TYPE_ALIGNED32(t_vbEAPSDShapeHdr);

struct PACKMEMBER _vbEAPSDShapeStepHdr
{
  INT8U  MAC[ETH_ALEN];
  INT16U numPSDSteps;
  INT32U maxPhyRateUp;
  INT32U maxPhyRateDown;
};
typedef struct _vbEAPSDShapeStepHdr TYPE_ALIGNED32(t_vbEAPSDShapeStepHdr);

struct PACKMEMBER _vbEAPSDShapeStep
{
  INT16U stopCarrier;
  INT8U  attPSD;
};
typedef struct _vbEAPSDShapeStep TYPE_ALIGNED32(t_vbEAPSDShapeStep);

// EA_Clock.rsp
struct PACKMEMBER _vbEAClockRsp
{
  INT32U  tvSec;
  INT32U  tvNsec;
  INT16U  seqNumber;
  BOOLEAN validSeqNum;
};
typedef struct _vbEAClockRsp TYPE_ALIGNED32(t_vbEAClockRsp);

// EA_CycQuery.req
struct PACKMEMBER _vbEACycQueryReq
{
  INT32U tvSec;
  INT32U tvNsec;
};
typedef struct _vbEACycQueryReq TYPE_ALIGNED32(t_vbEACycQueryReq);

// EA_CycQuery.rsp - common fields
struct PACKMEMBER _vbEACycQueryRspCommon
{
  INT32U tvSec;
  INT32U tvNsec;
  INT16U errorCode;
  INT16U numNodes;
};
typedef struct _vbEACycQueryRspCommon TYPE_ALIGNED32(t_vbEACycQueryRspCommon);

struct PACKMEMBER _vbEASyncDetInfo
{
  INT32U syncDetDid:8;
  INT32U res1:24;
  INT32U hitCount:32;
  INT32U reliability:32;
  INT32U adcOutRms:32;
};
typedef struct _vbEASyncDetInfo TYPE_ALIGNED32(t_vbEASyncDetInfo);

// EA_CycQuery.rsp - node fields
struct PACKMEMBER _vbEACycQueryRspNode
{
  INT8U  MAC[ETH_ALEN];
  INT16U seqNum;
  INT32U macClock;
  t_vbEASyncDetInfo  syncDetsInfo[VB_EA_ALIGN_GHN_MAX_TX_NODES];
};
typedef struct _vbEACycQueryRspNode TYPE_ALIGNED32(t_vbEACycQueryRspNode);

// EA_CycChange.req - common fields
struct PACKMEMBER _vbEACycChangeReqCommon
{
  INT32U numNodes;
};
typedef struct _vbEACycChangeReqCommon TYPE_ALIGNED32(t_vbEACycChangeReqCommon);

// EA_CycChange.req - node fields
struct PACKMEMBER _vbEACycChangeReqNode
{
  INT8U  MAC[ETH_ALEN];
  INT16U seqNum;
  INT8U  changeEdge;
};
typedef struct _vbEACycChangeReqNode TYPE_ALIGNED32(t_vbEACycChangeReqNode);

// EA_CycChange.rsp - common fields
struct PACKMEMBER _vbEACycChangeRsp
{
  INT8U errCode;
};
typedef struct _vbEACycChangeRsp TYPE_ALIGNED32(t_vbEACycChangeRsp);

struct PACKMEMBER _vbEAVersionRsp
{
  CHAR version[VB_EA_VERSION_MAX_SIZE];
  CHAR driverId[VB_EA_DRIVER_ID_MAX_SIZE];
  INT32U lcmpMcastTimeOut;
  INT32U lcmpMcastNAttempt;
};
typedef struct _vbEAVersionRsp TYPE_ALIGNED32(t_vbEAVersionRsp);

// EA_EngineConf.req
struct PACKMEMBER _vbAlignModeParam
{
  INT32U alignMode:8;                ///< Alignment mode: 0 - common clock; 1 - g.hn; 2 - ptp (not implemented)
  INT32U alignId:8;                  ///< Alignment message transaction Id
  INT32U refDID:8;                   ///< Reference node device Id
  INT32U absOffset:8;                ///< Shows if IDSync frames location is related to a downstream slot or it is an absolute value
  INT32U numRelays:8;                ///< Number of relay nodes
  INT8U  DMMAC[ETH_ALEN];            ///< Reference node MAC address
  INT32U numCyclesApplyExtraTime:8;  ///< Number of cycles the extra time shall be applied to the MAC cycle duration
  INT32U macCycleDur:32;             ///< MAC Cycle duration (in 10 ns unit)
  INT32U syncFrmOffset:32;           ///< MAC Clock to transmit synchronization frame (in 10ns units)
  INT32U maxDurSyncFrm:32;           ///< Duration of the sync TXOP (in 10ns units)
  INT32U clusterId:32;               ///< Current cluster Id
  INT32U tmpMacCycleDurExtraTime:32; ///< Extra time in 10 ns unit to be added to the current Mac cycle duration during numCyclesApplyExtraTime cycles
  INT32U numSeeds:16;                ///< Number of seeds to be assigned
};
typedef struct _vbAlignModeParam TYPE_ALIGNED32(t_vbAlignModeParam);

// EA_EngineConf.req
struct PACKMEMBER _vbCdtaEngineConf ///< Info embedded in the ENGINE CONF message sent at engine starts up
{
  INT8U   defaultRate;      ///< Default Qos Rate to use first when waiting for engine info before transmitting
  INT8U   currentRate;      ///< current Qos Rate to use upon reception of the Engine Conf Msg
  INT8U   numBandsFirstPSD; ///< PSD to apply first when waiting for engine info before transmitting
  INT8U   nBoostBands;      ///< number of potential boosted bands
};
typedef struct _vbCdtaEngineConf TYPE_ALIGNED32(t_vbCdtaEngineConf);

// EA_EngineConf.req
struct PACKMEMBER _vbBoostBandLastCarrier ///< Info embedded in the ENGINE CONF message sent at engine starts up
{
  INT16U   boostBandLastCarrier;      ///< number of potential boosted bands
};
typedef struct _vbBoostBandLastCarrier TYPE_ALIGNED32(t_vbBoostBandLastCarrier);


// EA_EngineConf.req (relay part)
struct PACKMEMBER _vbAlignModeRelayParam
{
  INT8U  relayMAC[ETH_ALEN]; ///< Relay MAC address
  INT32U deviceId:8;         ///< Relay device Id
  INT32U syncFrmOffset:32;   ///< MAC Clock to transmit synchronization frame (in 10ns units)
  INT32U duration:32;        ///< Duration of the sync TXOP (in 10ns units)
};
typedef struct _vbAlignModeRelayParam TYPE_ALIGNED32(t_vbAlignModeRelayParam);

// EA_EngineConf.req (seed part)
struct PACKMEMBER _vbAlignModeSeedParam
{
  INT8U  dmMAC[ETH_ALEN]; ///< DM MAC address
  INT32U dmSeed:16;       ///< Seed to be used by DM
  INT32U did:16;          ///< Device id to be used by DM
};
typedef struct _vbAlignModeSeedParam TYPE_ALIGNED32(t_vbAlignModeSeedParam);

// EA_AlignMode.rsp
struct PACKMEMBER _vbEAAlignModeRsp
{
  INT8U errCode;
  INT8U alignId;
};
typedef struct _vbEAAlignModeRsp TYPE_ALIGNED32(t_vbEAAlignModeRsp);

// EA_AlignClusterStop.rsp
struct PACKMEMBER _vbEAAlignClusterRsp
{
  INT8U errCode;
};
typedef struct _vbEAAlignClusterRsp TYPE_ALIGNED32(t_vbEAAlignClusterStopRsp);

// EA_AlignSyncLost.trg
struct PACKMEMBER _vbEAAlignSyncLost
{
  INT8U  reporterMac[ETH_ALEN];     ///< MAC address of reporter node
  INT32U syncDid:8;                 ///< Device Id lost
};
typedef struct _vbEAAlignSyncLost TYPE_ALIGNED32(t_vbEAAlignSyncLost);

// EA ClusterStop.req
struct PACKMEMBER _vbEAAlignClusterStopReq
{
  INT32U stopTxFlag;
};
typedef struct _vbEAAlignClusterStopReq TYPE_ALIGNED32(t_vbEAAlignClusterStopReq);

struct PACKMEMBER _vbEADomainDiffHdrRsp
{
  INT8U  reportType;
};
typedef struct _vbEADomainDiffHdrRsp TYPE_ALIGNED32(t_vbEADomainDiffHdrRsp);

struct PACKMEMBER _vbEADomainDiffRspDMAdded
{
  INT8U  dmMAC[ETH_ALEN];
  INT8U  fwVersion[VB_FW_VERSION_LENGTH];
  INT8U  dmDevId;
  INT16U qosRate;
  INT16U maxLengthTxop;
  INT16U extSeed;
  INT16U numEps;
};
typedef struct _vbEADomainDiffRspDMAdded TYPE_ALIGNED32(t_vbEADomainDiffRspDMAdded);

struct PACKMEMBER _vbEADomainDiffRspEPAdded
{
  INT8U  dmMAC[ETH_ALEN];
  INT8U  epMAC[ETH_ALEN];
  INT8U  fwVersion[VB_FW_VERSION_LENGTH];
  INT8U  epDevId;
};
typedef struct _vbEADomainDiffRspEPAdded TYPE_ALIGNED32(t_vbEADomainDiffRspEPAdded);

struct PACKMEMBER _vbEADomainDiffRspNodeRem
{
  INT8U  mac[ETH_ALEN];
};
typedef struct _vbEADomainDiffRspNodeRem TYPE_ALIGNED32(t_vbEADomainDiffRspNodeRem);

// EA SocketAlive.req
struct PACKMEMBER _vbEASocketAliveReq
{
  BOOL   enable;
  INT32U period;
  INT32U nLostMsg;
};
typedef struct _vbEASocketAliveReq TYPE_ALIGNED32(t_vbEASocketAliveReq);

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Starts the EA thread specified in given connection descriptor.
 * It also initializes the descriptor mutex and allocates a buffer to receive EA messages.
 * These mutex and buffer will be released when the thread finishes.
 * @param[in] desc Connection descriptor
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAThreadStart(t_vbEADesc *desc);

/**
 * @brief Stops the EA thread specified in given connection descriptor.
 * @param[in] desc Connection descriptor
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAThreadStop(t_vbEADesc *desc);

/**
 * @brief Allocates message structure (including buffer for given payload length) and initializes the header
 * @param[out] msg Pointer to EA msg structure with allocated buffer inside. It shall be released calling @ref VbEAMsgFree
 * @param[in] payloadLen Payload length in bytes
 * @param[in] opCode Message opcode
 * @remarks It shall be released calling @ref VbEAMsgFree
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAMsgAlloc(t_vbEAMsg **msg, INT32U payloadLen, t_vbEAOpcode opCode);

/**
 * @brief Releases allocated memory inside message structure
 * @param[in] msg Pointer to EA msg structure.
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAMsgFree(t_vbEAMsg **msg);

/**
 * @brief Parses the given buffer allocating a new msg structure
 * @param[in] msg Pointer to EA msg structure with allocated buffer inside. It shall be released calling @ref VbEAMsgFree
 * @param[in] rxBuffer Buffer containing the frame bytes
 * @remarks It shall be released calling @ref VbEAMsgFree
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAMsgParse(t_vbEAMsg **msg, INT8U *rxBuffer);

/**
 * @brief Sends the given message through the socket descriptor specified in "desc"
 * @param[in] desc Connection descriptor
 * @param[in] msg Management message to send
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAMsgSend(t_vbEADesc *desc, t_vbEAMsg *msg);

/**
 * @brief Sets the given server IP address and port in the connection descriptor
 * @param[in] desc Connection descriptor
 * @param[in] ipAddr Server IP Address
 * @param[in] port Server IP Port
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAServerAddrSet(t_vbEADesc *desc, struct in6_addr ipAddr, INT16U port);

/**
 * @brief Sets the given Client IP address and port in the connection descriptor
 * @param[in] desc Connection descriptor
 * @param[in] ipAddr Server IP Address
 * @param[in] port Server IP Port
 * @param[in] family Address family type (IPv4 or IPv6)
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEAClientAddrSet(t_vbEADesc *desc, CHAR *ipAddr, INT16U port, INT16U family);

/**
 * @brief Initializes given connection structure
 * @param[in] desc Connection descriptor to initialize
 * @return @ref t_vbEAError
 **/
t_vbEAError VbEADescInit(t_vbEADesc *desc);

/**
 * @brief Gets OPCODE name
 * @param[in] opcode EA Opcode
 * @return OPCODE name
 **/
const CHAR *VbEAOpcodeToStrGet(t_vbEAOpcode opcode);

/**
 * @brief Dump all Messages counters
 * @param[in] writeFun Function to write to
 * @param[in] tables t_vbEADbgTable struct containing the debugTables
 **/
void VbEADbgMsgDump(t_vbEADbgTable *tables, t_writeFun writeFun);

/**
 * @brief Reset all counters
 * @param[in] table Debug table to reset
 **/
void VbEADbgMsgReset(t_vbEADbgTable *table);

/**
 * @brief Console command related to EA messages module
 * @param[in] arg Generic argument
 * @param[in] writeFun Function to write to
 * @param[in] cmd Command arguments
 * @return TRUE if OK; FALSE otherwise
 **/
BOOL VbEAConsoleCmd(void *arg, t_writeFun writeFun, char **cmd);

/**
 * @brief Free socket descriptor memory
 * @param[in] Socket descriptor
 **/
t_vbEAError VbEADescDestroy(t_vbEADesc *desc);

#endif /* VB_EA_COMMUNICATION_H_ */

/**
 * @}
**/
