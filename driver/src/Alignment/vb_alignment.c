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
 * @file vb_alignment.c
 * @brief Alignment of DMs MAC cycle in Gnow
 *
 * @internal
 *
 * @author 
 * @date 15/01/2014
 *
 **/

/*
 ************************************************************************ 
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <strings.h>
#include <string.h>
#include <errno.h>

#include "vb_driver_conf.h"
#include "vb_alignment.h"
#include "vb_LCMP_com.h"
#include "vb_LCMP_socket.h"
#include "vb_log.h"
#include "vb_EA_interface.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_ea_communication.h"
#include "vb_main.h"
#include "vb_timer.h"
#include "vb_types.h"
#include "vb_domainsMonitor.h"

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

#define ALIGN_CHECK_THREAD_NAME                   ("AlignmentCheck")
#define ALIGN_CHANGE_THREAD_NAME                  ("AlignmentChange")
#define ALIGN_SYNC_LOST_THREAD_NAME               ("AlignmentSyncLost")
#define VB_CYCQUERYNOTIFEXT_SIZE                  (sizeof(t_lcmpCycQueryNotifExtInd))
#define VB_CYCQUERYEXT_SIZE                       (sizeof(t_lcmpCycQueryExtInd))
#define VB_MACSEQNUM_SIZE                         (sizeof(t_lcmpMacSeqNum))
#define TIMEOUT_CYCQUERY                          (VbDriverConfLcmpDefaultTimeoutGet()) //ms
#define ALIGN_CHANGE_NODES_WAIT                   (MAC_CYCLE_DURATION * 2) // Time required by G.hn nodes to apply new alignment parameters (in ms)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

///////////////////////////
///    LCMP Messages    ///
///////////////////////////

// CycChange.req
struct PACKMEMBER _lcmpCycChangeReq
{
  INT32U ctrlType:8;
  INT32U clockEdge:8;
  INT32U seqNumOffset:16;
};
typedef struct _lcmpCycChangeReq TYPE_ALIGNED32(t_lcmpCycChangeReq);

// CycQueryExt.ind
struct PACKMEMBER _lcmpCycQueryExtInd
{
  INT32U notifyType:8;
  INT8U  aeMac[ETH_ALEN];
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT32U resetStats:1;
  INT32U :7;
#else
  INT32U :7;
  INT32U resetStats:1;
#endif
};
typedef struct _lcmpCycQueryExtInd TYPE_ALIGNED32(t_lcmpCycQueryExtInd);


struct PACKMEMBER _vectorboostTlvSyncDetInfo
{
  INT32U syncDetDid:8;
  INT32U res1:24;
  INT32U hitCount:32;
  INT32U reliability:32;
  INT32U adcOutRms:32;
};
typedef struct _vectorboostTlvSyncDetInfo TYPE_ALIGNED32(t_vectorboostTlvSyncDetInfo);

/// CycQueryNotifExt.ind
struct PACKMEMBER _lcmpCycQueryNotifExtInd
{
  INT32U notifyType:8;
  INT32U seqNum:16;
  INT32U macClock:32;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT32U refUnitFlag:1;
  INT32U :7;
#else
  INT32U :7;
  INT32U refUnitFlag:1;
#endif
  INT8U  nodeMac[ETH_ALEN];
  t_vectorboostTlvSyncDetInfo  syncDetsInfo[VB_ALIGN_GHN_MAX_TX_NODES];
};
typedef struct _lcmpCycQueryNotifExtInd TYPE_ALIGNED32(t_lcmpCycQueryNotifExtInd);

/// AlignMode.req
struct PACKMEMBER _lcmpEngineConfReq
{
  INT32U ctrlType:8;         ///< Control type: set to VB_ALIGNMODE
  INT32U alignMode:8;        ///< Alignment mode: 0 - common clock; 1 - g.hn; 2 - ptp (not implemented)
  INT8U  DMMAC[ETH_ALEN];    ///< Reference node MAC address
  INT32U refDid:8;           ///< Reference node device Id
  INT32U macCycle:32;        ///< MAC Cycle duration (in 10 ns)
  INT32U absOffset:8;        ///< Shows if IDSync frames location is related to a downstream slot or it is an absolute value
  INT32U syncFrmOffset:32;   ///< Offset of where to send the sync frame in the mac cycle
  INT32U maxDurSyncFrm:32;   ///< MAC Clock to transmit synchronization frame (in 10ns units)
  INT32U clusterId:32;       ///< Cluster Id
  INT32U tmpMacCycleDurExtraTime:32; ///< Number of extra Mac cycle ticks (in 10 ns unit) to be temporarily added to the MAC cycle
  INT32U numCyclesApplyExtraTime:8; ///< number of cycles to apply the tmpMacCycleDurExtraTime
  INT32U numRelays:8;
  INT32U numSeeds:16;         ///< Number of seeds to be assigned
  INT32U defaultRate:8;      ///< Default Qos Rate to use first when connection with engine is lost
  INT32U currentRate:8;      ///< Current Qos Rate to use upon reception of this message
  INT32U numBandsFirstPSD:8; ///< PSD to apply first when waiting for engine info before transmitting
  INT32U nBoostBands:8;      ///< number of potential boosted bands
};
typedef struct _lcmpEngineConfReq TYPE_ALIGNED32(t_VB_LCMP_EngineConf_Param);

struct PACKMEMBER _lcmpBoostBandLastCarrier ///< Info embedded in the ENGINE CONF message sent at engine starts up
{
  INT16U   boostBandLastCarrier;      ///< number of potential boosted bands
};
typedef struct _lcmpBoostBandLastCarrier TYPE_ALIGNED32(t_VB_LCMP_BoostBandsLastCarrier_Param);

/// AlignMode.req (relay part)
struct PACKMEMBER _lcmpAlignModeRelay
{
  INT8U   relayMAC[ETH_ALEN];
  INT32U  deviceId:8;         ///< Relay device Id
  INT32U  syncFrmOffset:32;
  INT32U  duration:32;        ///< Duration of the sync TXOP (in 10ns units)
};
typedef struct _lcmpAlignModeRelay TYPE_ALIGNED32(t_VB_LCMP_AlignModeRelay_Param);

/// AlignMode.req (seed part)
struct PACKMEMBER _lcmpAlignModeSeed
{
  INT8U  dmMAC[ETH_ALEN];
  INT32U dmSeed:16;      ///< Seed to be used by the DM
  INT32U did:16;         ///< Device id to be used by the DM
};
typedef struct _lcmpAlignModeSeed TYPE_ALIGNED32(t_VB_LCMP_AlignModeSeed_Param);

/// AlignSyncLost.ind
struct PACKMEMBER _lcmpAlignSyncLostInd
{
  INT32U notifyType:8;       ///< Notify type: set to VB_ALIGN_SYNCLOST
  INT32U syncDid:8;          ///< Device Id lost
};
typedef struct _lcmpAlignSyncLostInd TYPE_ALIGNED32(t_lcmpAlignSyncLostInd);

// ClusterStop.req
struct PACKMEMBER _lcmpClusterStopReq
{
  INT32U ctrlType:8;
  INT32U stopTxFlag:8;
};
typedef struct _lcmpClusterStopReq TYPE_ALIGNED32(t_lcmpClusterStopReq);

/// MacSeqNum.req
struct PACKMEMBER _lcmpMacSeqNum
{
  INT32U paramType:8;      ///< Param type (VB_MACSEQNUM)
  INT32U macSeqNum:16;     ///< MAC sequence number
};
typedef struct _lcmpMacSeqNum TYPE_ALIGNED32(t_lcmpMacSeqNum);

/*******************************************************************/

typedef struct
{
  INT8U          *buffer;
  INT32U          idx;
  INT32U          numDms;
} t_VBAlignCycQueryRspMsg;

typedef struct
{
  struct timespec schedTime;
} t_VBAlignCycQueryParams;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_t               vbAlignmentCheckThread = 0;
static pthread_t               vbAlignmentChangeThread = 0;
static pthread_t               vbAlignmentSyncLostThread = 0;
static BOOLEAN                 vbAlignmentCycQueryThreadRunning = FALSE;
static BOOLEAN                 vbAlignmentCycChangeThreadRunning = FALSE;
static t_VBAlignCycQueryParams vbAlignmentCheckParams;
static BOOLEAN                 vbAlignmentSyncLostThreadRunning = FALSE;
static t_Callbacks            *vbAlignmentSyncLostLcmpCb = NULL;
static INT8U                   vbAlignmentId;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/**
 * @brief Implement the wait CycResp receive process and show the time stamps
 * return error code
**/
static t_VB_comErrorCode VBCycRespWait( t_Callbacks *callbackInstalled, struct timespec tsTx );

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VBAlignmentCheckStateSet(BOOLEAN running)
{
  vbAlignmentCycQueryThreadRunning = running;
}

/*******************************************************************/

static BOOL VBAlignmentCheckStateGet(void)
{
  return vbAlignmentCycQueryThreadRunning;
}

/*******************************************************************/

static void VBAlignmentChangeStateSet(BOOLEAN running)
{
  vbAlignmentCycChangeThreadRunning = running;
}

/*******************************************************************/

static BOOL VBAlignmentChangeStateGet(void)
{
  return vbAlignmentCycChangeThreadRunning;
}

/*******************************************************************/

static void VBAlignmentSyncLostMonitorStateSet(BOOLEAN running)
{
  vbAlignmentSyncLostThreadRunning = running;
}

/*******************************************************************/

static BOOLEAN VBAlignmentSyncLostMonitorStateGet(void)
{
  return vbAlignmentSyncLostThreadRunning;
}

/*******************************************************************/

static t_VB_comErrorCode LCMPCycChangeSend(const INT8U *mac, INT8U clockEdge, INT16U seqNumOffset)
{
  t_VB_comErrorCode        ret = VB_COM_ERROR_NONE;
  t_ValuesArray           *control_values_array = NULL;
  t_HGF_LCMP_ErrorCode     lcmp_err;
  t_lcmpComParams          lcmp_params;

  if (mac == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_lcmpCycChangeReq       cyc_change;

    // Build message
    bzero(&cyc_change, sizeof(cyc_change));
    cyc_change.ctrlType = VB_CYCCHANGE;
    cyc_change.clockEdge = clockEdge;
    cyc_change.seqNumOffset = seqNumOffset;

    ret = VbDatamodelValueToArrayAdd(sizeof(cyc_change), (INT8U *)(&cyc_change), &control_values_array);

    if (ret != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d building CycChange.req LCMP message", ret);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      ret = VB_COM_ERROR_LCMP;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = UNICAST;
    lcmp_params.dstMac          = mac;
    lcmp_params.paramIdReq      = VB_CYCCHANGE;
    lcmp_params.reqValues       = control_values_array;

    // Send control frame to modigy alignment
    lcmp_err = VbLcmpControl(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      CHAR mac_str[MAC_STR_LEN];

      MACAddrMem2str(mac_str, mac);
      VbLogPrint(VB_LOG_ERROR, "Error %d sending CycChange.req to node %s", lcmp_err, mac_str);

      ret = VB_COM_ERROR_LCMP_CONTROL;
    }
  }

  // Release req memory
  VbDatamodelHTLVsArrayDestroy(&control_values_array);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode LCMPAlignmentModeSend(t_vbAlignModeParam *engAlignMode, t_vbCdtaEngineConf *cdtaConf, t_vbBoostBandLastCarrier *boostBandLastCarrier, t_vbAlignModeRelayParam *relays, t_vbAlignModeSeedParam *seeds)
{
  t_VB_comErrorCode    ret = VB_COM_ERROR_NONE;
  t_ValuesArray       *control_values_array = NULL;
  INT8U               *payload = NULL;
  INT32U               payload_len;
  t_HGF_LCMP_ErrorCode lcmp_err;
  t_lcmpComParams      lcmp_params;

  if ((engAlignMode == NULL) || (cdtaConf == NULL) || (boostBandLastCarrier == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    payload_len = sizeof(t_VB_LCMP_EngineConf_Param) +
                  (cdtaConf->nBoostBands*sizeof(t_VB_LCMP_BoostBandsLastCarrier_Param)) +
                  (engAlignMode->numRelays*sizeof(t_VB_LCMP_AlignModeRelay_Param)) +
                  (engAlignMode->numSeeds*sizeof(t_VB_LCMP_AlignModeSeed_Param));

    if((cdtaConf->nBoostBands > 5) || (engAlignMode->numRelays > 5))
    {
      VbLogPrint(VB_LOG_ERROR, "Alignment Error nBands %lu Num Relays %lu", cdtaConf->nBoostBands, engAlignMode->numRelays);
    }

    payload = calloc(1, payload_len);

    if (payload == NULL)
    {
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_VB_LCMP_EngineConf_Param *conf_params;
    t_VB_LCMP_AlignModeRelay_Param *relays_ptr;
    t_VB_LCMP_AlignModeSeed_Param *seeds_ptr;
    INT8U i;

    conf_params = (t_VB_LCMP_EngineConf_Param *)payload;

    // Build message
    conf_params->ctrlType      = VB_ENGINE_CONF;
    conf_params->alignMode     = engAlignMode->alignMode;

    conf_params->defaultRate      = cdtaConf->defaultRate;
    conf_params->currentRate      = cdtaConf->currentRate;
    conf_params->numBandsFirstPSD = cdtaConf->numBandsFirstPSD;
    conf_params->nBoostBands      = cdtaConf->nBoostBands;

    if(cdtaConf->nBoostBands)
    {
      t_VB_LCMP_BoostBandsLastCarrier_Param *bands_ptr;

      bands_ptr = (t_VB_LCMP_BoostBandsLastCarrier_Param *)(payload + sizeof(t_VB_LCMP_EngineConf_Param));
      for (i = 0; i< cdtaConf->nBoostBands; i++, bands_ptr++)
      {
        bands_ptr->boostBandLastCarrier = _htons_ghn(_ntohs(((INT16U)(boostBandLastCarrier[i].boostBandLastCarrier))));
      }
    }

    if( conf_params->alignMode == VB_ALIGN_MODE_GHN)
    {
      MACAddrClone((INT8U *)conf_params->DMMAC, (INT8U *)engAlignMode->DMMAC);
      conf_params->macCycle                = _htonl_ghn(_ntohl(engAlignMode->macCycleDur));
      conf_params->maxDurSyncFrm           = _htonl_ghn(_ntohl(engAlignMode->maxDurSyncFrm));
      conf_params->absOffset               = engAlignMode->absOffset;
      conf_params->refDid                  = engAlignMode->refDID;
      conf_params->syncFrmOffset           = _htonl_ghn(_ntohl(engAlignMode->syncFrmOffset));
      conf_params->numRelays               = engAlignMode->numRelays;
      conf_params->clusterId               = _htonl_ghn(_ntohl(engAlignMode->clusterId));
      conf_params->tmpMacCycleDurExtraTime = _htonl_ghn(_ntohl(engAlignMode->tmpMacCycleDurExtraTime));
      conf_params->numCyclesApplyExtraTime = engAlignMode->numCyclesApplyExtraTime;

      // Handle relays
      if(conf_params->numRelays > 0)
      {
        relays_ptr = (t_VB_LCMP_AlignModeRelay_Param *)(payload + sizeof(t_VB_LCMP_EngineConf_Param) +
                                                        cdtaConf->nBoostBands*sizeof(t_VB_LCMP_BoostBandsLastCarrier_Param));

        for (i = 0; i< conf_params->numRelays; i++)
        {
          MACAddrClone((INT8U *)relays_ptr[i].relayMAC, (INT8U *)relays[i].relayMAC);
          relays_ptr[i].deviceId = relays[i].deviceId;
          relays_ptr[i].duration = _htonl_ghn(_ntohl(relays[i].duration));
          relays_ptr[i].syncFrmOffset = _htonl_ghn(_ntohl(relays[i].syncFrmOffset));
        }
      }
    }

    // Handle seeds
    conf_params->numSeeds = engAlignMode->numSeeds;
    if(conf_params->numSeeds > 0)
    {
      seeds_ptr = (t_VB_LCMP_AlignModeSeed_Param *)(payload + sizeof(t_VB_LCMP_EngineConf_Param) +
                                                      cdtaConf->nBoostBands*sizeof(t_VB_LCMP_BoostBandsLastCarrier_Param) +
                                                      (engAlignMode->numRelays*sizeof(t_VB_LCMP_AlignModeRelay_Param)));

      for (i = 0; i< conf_params->numSeeds; i++)
      {
        MACAddrClone((INT8U *)seeds_ptr[i].dmMAC, (INT8U *)seeds[i].dmMAC);
        seeds_ptr[i].dmSeed = _htons_ghn(_ntohs(seeds[i].dmSeed));
        seeds_ptr[i].did    = _htons_ghn(_ntohs(seeds[i].did));
        VbDatamodelNodeSeedIndexSet(seeds_ptr[i].dmMAC, seeds_ptr[i].dmSeed,seeds_ptr[i].did);
      }
    }

    ret = VbDatamodelValueToArrayAdd(payload_len, payload, &control_values_array);

    if (ret != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d building AlignMode.req LCMP message", ret);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      ret = VB_COM_ERROR_LCMP;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = MULTICAST_DMS;
    lcmp_params.paramIdReq      = VB_ENGINE_CONF;
    lcmp_params.reqValues       = control_values_array;

    lcmp_err = VbLcmpControl(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending AlignMode.req", lcmp_err);

      ret = VB_COM_ERROR_LCMP_CONTROL;
    }
  }

  // Release req memory
  VbDatamodelHTLVsArrayDestroy(&control_values_array);

  // Release allocated memory
  if (payload != NULL)
  {
    free(payload);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode CycChangeLoopCb(t_Domains *domain, void *args)
{
  t_VB_comErrorCode        ret = VB_COM_ERROR_NONE;

  if (domain == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if ((ret == VB_COM_ERROR_NONE) && (domain->alignChange.updated == TRUE))
  {
    ret = LCMPCycChangeSend(domain->dm.MAC, domain->alignChange.clockEdge, domain->alignChange.seqNumOffset);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Clear flag
    domain->alignChange.updated = FALSE;
  }

  if (VBAlignmentChangeStateGet() == FALSE)
  {
    // Thread was aborted, stop loop
    ret = VB_COM_ERROR_EXIT_LOOP_OK;
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode CycChangeProcess(void)
{
  t_VB_comErrorCode        ret;

  // Loop through domains to build the response
  ret = VbDatamodelDomainsLoop(CycChangeLoopCb, NULL);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode EACycQueryRspLoopCb(t_Domains *domain, void *args)
{
  t_VB_comErrorCode        ret = VB_COM_ERROR_NONE;
  t_VBAlignCycQueryRspMsg *msg = (t_VBAlignCycQueryRspMsg *)args;
  t_vbEACycQueryRspNode   *node_fields;
  INT32U i;

  if ((domain == NULL) || (msg == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    INT32U idx_to_write = (msg->idx * VB_EA_CYCQUERY_RSP_NODE_SIZE) + VB_EA_CYCQUERY_RSP_COMMON_SIZE;

    // update ptr to write to
    node_fields = (t_vbEACycQueryRspNode *)&(msg->buffer[idx_to_write]);

    // Fill message
    MACAddrClone(node_fields->MAC, domain->dm.MAC);
    node_fields->macClock = _htonl(domain->alignInfo.macClock);
    node_fields->seqNum = _htons(domain->alignInfo.seqNum);

    for (i = 0; i < VB_EA_ALIGN_GHN_MAX_TX_NODES; i++)
    {
      node_fields->syncDetsInfo[i].syncDetDid  = domain->alignInfo.syncDetsInfo[i].detDid;
      node_fields->syncDetsInfo[i].reliability = _htonl(domain->alignInfo.syncDetsInfo[i].reliability);
      node_fields->syncDetsInfo[i].hitCount    = _htonl(domain->alignInfo.syncDetsInfo[i].hitCount);
      node_fields->syncDetsInfo[i].adcOutRms   = _htonl(domain->alignInfo.syncDetsInfo[i].adcOutRms);
    }

    // Increase index to write
    msg->idx++;
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode EACycQueryRspSend(t_vbEACycQueryRspErrorCode errCode)
{
  t_VB_comErrorCode        ret = VB_COM_ERROR_NONE;
  t_vbEAError              ea_err;
  t_VBAlignCycQueryRspMsg  msg;
  t_vbEAMsg               *ea_msg = NULL;
  t_vbEACycQueryRspCommon *common_fields;

  // Get number of DMs
  msg.numDms = VbDatamodelNmDomainsAliveGet();

  // Allocate message
  ea_err = VbEAMsgAlloc(&ea_msg, VB_EA_CYCQUERY_RSP_COMMON_SIZE + (msg.numDms * VB_EA_CYCQUERY_RSP_NODE_SIZE), VB_EA_OPCODE_CYCQUERY_RSP);

  if (ea_err != VB_EA_ERR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
    ret = VB_COM_ERROR_EA;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Update pointer to EA msg payload
    msg.buffer = ea_msg->eaPayload.msg;

    // Fill common fields
    common_fields = (t_vbEACycQueryRspCommon *)msg.buffer;
    common_fields->tvSec = _htonl(vbAlignmentCheckParams.schedTime.tv_sec);
    common_fields->tvNsec = _htonl(vbAlignmentCheckParams.schedTime.tv_nsec);
    common_fields->numNodes = _htons(msg.numDms);
    common_fields->errorCode = _htons(errCode);
  }

  if ((ret == VB_COM_ERROR_NONE) && (msg.numDms > 0))
  {
    // Init loop index
    msg.idx = 0;

    // Loop through domains to build the response
    ret = VbDatamodelDomainsLoop(EACycQueryRspLoopCb, &msg);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Send response to engine
    ret = VbEADriverFrameSend(ea_msg);
  }

  // Always release memory
  VbEAMsgFree(&ea_msg);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode EACycChangeRspSend(t_vbEACycChangeRspErrorCode errCode)
{
  t_VB_comErrorCode        ret = VB_COM_ERROR_NONE;
  t_vbEAError              ea_err;
  t_vbEAMsg               *ea_msg = NULL;
  t_vbEACycChangeRsp      *cyc_change_ptr;

  // Allocate message
  ea_err = VbEAMsgAlloc(&ea_msg, VB_EA_CYCCHANGE_RSP_SIZE, VB_EA_OPCODE_CYCCHANGE_RSP);

  if (ea_err != VB_EA_ERR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
    ret = VB_COM_ERROR_EA;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Build the message
    cyc_change_ptr = (t_vbEACycChangeRsp *)ea_msg->eaPayload.msg;
    cyc_change_ptr->errCode = errCode;

    // Send response to engine
    ret = VbEADriverFrameSend(ea_msg);
  }

  // Always release memory
  VbEAMsgFree(&ea_msg);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode EAAlignModeRspSend(t_vbEAAlignModeRspErrorCode errCode, INT8U alignId)
{
  t_VB_comErrorCode        ret = VB_COM_ERROR_NONE;
  t_vbEAError              ea_err;
  t_vbEAMsg               *ea_msg = NULL;
  t_vbEAAlignModeRsp      *align_mode_ptr;

  // Allocate message
  ea_err = VbEAMsgAlloc(&ea_msg, sizeof(t_vbEAAlignModeRsp), VB_EA_OPCODE_ENGINE_CONF_RSP);

  if (ea_err != VB_EA_ERR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
    ret = VB_COM_ERROR_EA;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Build the message
    align_mode_ptr = (t_vbEAAlignModeRsp *)ea_msg->eaPayload.msg;
    align_mode_ptr->errCode = (INT8U)errCode;
    align_mode_ptr->alignId = (INT8U)alignId;

    // Send response to engine
    ret = VbEADriverFrameSend(ea_msg);
  }

  // Always release memory
  VbEAMsgFree(&ea_msg);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode EAAlignClusterStopRspSend(t_vbEAAlignModeRspErrorCode errCode)
{
  t_VB_comErrorCode        ret = VB_COM_ERROR_NONE;
  t_vbEAError              ea_err;
  t_vbEAMsg               *ea_msg = NULL;
  t_vbEAAlignClusterStopRsp      *cluster_stop_ptr;

  // Allocate message
  ea_err = VbEAMsgAlloc(&ea_msg, sizeof(t_vbEAAlignClusterStopRsp), VB_EA_OPCODE_ALIGN_STOP_CLUSTER_RSP);

  if (ea_err != VB_EA_ERR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
    ret = VB_COM_ERROR_EA;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Build the message
    cluster_stop_ptr = (t_vbEAAlignClusterStopRsp *)ea_msg->eaPayload.msg;
    cluster_stop_ptr->errCode = (INT8U)errCode;

    // Send response to engine
    ret = VbEADriverFrameSend(ea_msg);
  }

  // Always release memory
  VbEAMsgFree(&ea_msg);

  return ret;
}

/*******************************************************************/

static void *VBAlignmentCheckProcess(void *arg)
{
  t_VB_comErrorCode          err = VB_COM_ERROR_NONE;
  t_vbEACycQueryRspErrorCode rsp_err = VB_EA_CYCQUERY_RSP_ERR_NONE;
  mqd_t                      vb_queue;
  struct timespec           current_ts;
  INT32S                     cmp;
  INT32S                     err_timer;
  CHAR                       sched_time_str[TIMESPEC_STR_LEN];
  CHAR                       curr_time_str[TIMESPEC_STR_LEN];

  vb_queue = mq_open(VBQUEUENAME, O_WRONLY);

  if (vb_queue == -1)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%s) opening posix queue in thread %s", strerror(errno), ALIGN_CHECK_THREAD_NAME);
    err = VB_COM_ERROR_QUEUE;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Get current time
    clock_gettime(CLOCK_REALTIME, &current_ts);

    // Check sched time
    cmp = VbUtilTimespecCmp(&(vbAlignmentCheckParams.schedTime), &current_ts);

    // Get timeStamp string
    VbUtilTimespecToString(sched_time_str, vbAlignmentCheckParams.schedTime);
    VbUtilTimespecToString(curr_time_str, current_ts);

    if (cmp <= 0)
    {
      // Required time is invalid
      rsp_err = VB_EA_CYCQUERY_RSP_ERR_INVALID_TIME;

      VbLogPrint(VB_LOG_ERROR, "Invalid sched time [%s]; current [%s]", sched_time_str, curr_time_str);

      err = VB_COM_ERROR_PARAM_ERROR;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Configure timer to wait until sched time
    err_timer = VbThreadAbsTimeSleep(&(vbAlignmentCheckParams.schedTime));

    if (err_timer != 0)
    {
      // Send event to exit from ALIGNCHECK state
      rsp_err = VB_EA_CYCQUERY_RSP_ERR_OTHER;

      VbLogPrint(VB_LOG_ERROR, "Error %d configuring timed wait: sched [%s]; current [%s]",
          err_timer,
          sched_time_str,
          curr_time_str);

      err = VB_COM_ERROR_ALIGN;
    }
  }

  if (VBAlignmentCheckStateGet() == TRUE)
  {
    if (err == VB_COM_ERROR_NONE)
    {
      // Launch CycQuery message to G.hn nodes and wait for response
      err = VbAlignmentCycQuerySend();

      if (err != VB_COM_ERROR_NONE)
      {
        // Send event to exit from ALIGNCHECK state
        rsp_err = VB_EA_CYCQUERY_RSP_ERR_NO_NOTIF;

        VbLogPrint(VB_LOG_ERROR, "Error %d waiting for CycQuery notifications: sched [%s]; current [%s]",
            err,
            sched_time_str,
            curr_time_str);
      }
    }

    // Always send response to VB engine

    // Build and send response to VB engine
    err = EACycQueryRspSend(rsp_err);

    if (err != VB_COM_ERROR_NONE)
    {
      // Send event to exit from ALIGNCHECK state
      rsp_err = VB_EA_CYCQUERY_RSP_ERR_OTHER;

      VbLogPrint(VB_LOG_ERROR, "Error %d sending EACycQuery.rsp : sched [%s]; current [%s]",
          err,
          sched_time_str,
          curr_time_str);
    }

    // Always send event to main FSM
    if (vb_queue != -1)
    {
      // Send event to exit from ALIGNCHECK state
      VbMainQEvSend(DRIVER_EV_ALIGNCHECK_END, vb_queue, (void *)rsp_err);
    }

    VbLogPrint(VB_LOG_INFO, "Alignment check end (error %d; rsp_err %d): sched [%s]; start [%s]", err, rsp_err, sched_time_str, curr_time_str);
  }

  if (vb_queue != -1)
  {
    mq_close(vb_queue);
  }

  return NULL;
}

/*******************************************************************/

static void *VBAlignmentChangeProcess(void *arg)
{
  t_VB_comErrorCode           err = VB_COM_ERROR_NONE;
  t_vbEACycChangeRspErrorCode rsp_err = VB_EA_CYCCHANGE_RSP_ERR_NONE;
  mqd_t                       vb_queue;

  vb_queue = mq_open(VBQUEUENAME, O_WRONLY);

  if (vb_queue == -1)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%s) opening posix queue in thread %s", strerror(errno), ALIGN_CHANGE_THREAD_NAME);
    err = VB_COM_ERROR_QUEUE;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Build and send LCMP CycChange.req messages to nodes
    err = CycChangeProcess();

    if (err != VB_COM_ERROR_NONE)
    {
      rsp_err = VB_EA_CYCCHANGE_RSP_ERR_OTHER;
    }
  }

  if (VBAlignmentChangeStateGet() == TRUE)
  {
    // Always send response to VB engine

    // Wait until nodes apply new alignment parameters
    VbThreadSleep(ALIGN_CHANGE_NODES_WAIT);

    // Build and send response to VB engine
    err = EACycChangeRspSend(rsp_err);

    if (err != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending EACycChange.rsp", err);
    }

    // Always send event to main FSM
    if (vb_queue != -1)
    {
      // Send event to exit from ALIGNCHANGE state
      VbMainQEvSend(DRIVER_EV_ALIGNCHANGE_END, vb_queue, NULL);
    }

    VbLogPrint(VB_LOG_INFO, "Alignment change end (error %d; rsp_err %d)", err, rsp_err);
  }

  if (vb_queue != -1)
  {
    mq_close(vb_queue);
  }

  return NULL;
}

/*******************************************************************/

static t_VB_comErrorCode VBCycRespWait( t_Callbacks *callbackInstalled, struct timespec tsTx )
{
  t_VB_comErrorCode                result = VB_COM_ERROR_NONE;
  t_VB_comErrorCode                aux_err = VB_COM_ERROR_NONE;
  t_vb_LCMP_BroadcastResponseCheck response_check = VB_LCMP_BROADCAST_RESPONSE_OK;
  t_HTLVsLists                    *htlv_notify_values = NULL;
  t_HTLVsLists                    *final_confirm_values = NULL;
  t_HTLVValuesList                *htlv_values_list = NULL;
  INT16U                           num_htvl_lists_check;
  t_lcmpCycQueryNotifExtInd       *vb_cyc_query_notifext_param;
  INT16U                           values_check;
  INT16U                           seq_num = 0;
  INT32U                           mac_clock = 0;
  t_syncDetInfo                    synced_dids[VB_ALIGN_GHN_MAX_TX_NODES];
  CHAR                             mac_str[MAC_STR_LEN];

  if (callbackInstalled == NULL)
  {
    result = VB_COM_ERROR_BAD_ARGS;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    final_confirm_values = VbDatamodelHtlvListCreate();
    if(final_confirm_values == NULL)
    {
      result = VB_COM_ERROR_MALLOC;
    }
  }

  if(result == VB_COM_ERROR_NONE)
  {
    BOOLEAN wait_resp;
    BOOLEAN no_resp;
    INT32U  n_retries = 0;
    INT32U  n_retries_max = VbDriverConfLcmpDefaultNAttemptGet();

    do
    {
      wait_resp = FALSE;

      if (result == VB_COM_ERROR_NONE)
      {
        // Wait VB_CYCQUERYNOTIF
        VbThreadSleep(TIMEOUT_CYCQUERY);

        htlv_notify_values = LcmpCallBackReceiveGet(callbackInstalled, NULL);

        if(htlv_notify_values != NULL)
        {
          t_HGF_LCMP_ErrorCode result2;

          result2 = vbLcmpTempListToListAdd(final_confirm_values, htlv_notify_values);
          if (result2 == HGF_LCMP_ERROR_NONE)
          {
            result = VB_COM_ERROR_NONE;
          }
          else
          {
            result = VB_COM_ERROR_LCMP;
          }

          no_resp = FALSE;
        }
        else
        {
          no_resp = TRUE;
        }

        if (no_resp == TRUE)
        {
          VbLogPrint(VB_LOG_ERROR, "htlv_notify_values 0x%p", htlv_notify_values);
          wait_resp = (++n_retries < n_retries_max)?TRUE:FALSE;
          result = VB_COM_ERROR_PROTOCOL;
        }
      }

      if (result == VB_COM_ERROR_NONE)
      {
        // Check nodes response
        response_check = VbLcmpNodesListResponseCheck(final_confirm_values, TRUE, NULL, tsTx);

        if (response_check != VB_LCMP_BROADCAST_RESPONSE_OK)
        {
          wait_resp = (++n_retries < n_retries_max)?TRUE:FALSE;
          result = VB_COM_ERROR_RECEIVE_TIMEOUT;
        }
      }

      // Process data
      if (result == VB_COM_ERROR_NONE)
      {
        t_ValuesArray *values_array;

        htlv_values_list = final_confirm_values->head;
        num_htvl_lists_check = 0;

        while( (htlv_values_list != NULL) && (num_htvl_lists_check < final_confirm_values->NumLists) )
        {
          num_htvl_lists_check++;
          values_array = htlv_values_list->HTLVsArray;

          if (values_array != NULL)
          {
            for (values_check = 0 ; values_check < values_array->NumValues ; values_check++)
            {
              if (values_array->values[values_check].ValueLength == VB_CYCQUERYNOTIFEXT_SIZE)
              {
                vb_cyc_query_notifext_param =
                    (t_lcmpCycQueryNotifExtInd *)values_array->values[values_check].Value;

                if (vb_cyc_query_notifext_param->notifyType == VB_CYCQUERYNOTIF_EXT)
                {
                  INT32U i;

                  //If data correct save it
                  seq_num = _ntohs_ghn(vb_cyc_query_notifext_param->seqNum);
                  mac_clock = _ntohl_ghn(vb_cyc_query_notifext_param->macClock);

                  for (i = 0; i < VB_ALIGN_GHN_MAX_TX_NODES; i++)
                  {
                    synced_dids[i].detDid      = vb_cyc_query_notifext_param->syncDetsInfo[i].syncDetDid;
                    synced_dids[i].hitCount    = _ntohl_ghn(vb_cyc_query_notifext_param->syncDetsInfo[i].hitCount);
                    synced_dids[i].reliability = (INT32S)_ntohl_ghn(vb_cyc_query_notifext_param->syncDetsInfo[i].reliability);
                    synced_dids[i].adcOutRms   = _ntohl_ghn(vb_cyc_query_notifext_param->syncDetsInfo[i].adcOutRms);
                  }

                  // Update align info
                  result = VbDatamodelDomainAlignInfoUpdate(vb_cyc_query_notifext_param->nodeMac, seq_num, mac_clock, synced_dids);

                  MACAddrMem2str(mac_str, vb_cyc_query_notifext_param->nodeMac);

                  VbLogPrint( VB_LOG_INFO,
                      "%s\tseqNum: %u; macClock %u; Is DMRef: %u",
                      mac_str, seq_num, mac_clock, vb_cyc_query_notifext_param->refUnitFlag);

                  VbLogPrint( VB_LOG_INFO,
                      "\tDet [%03u;%lu;%ld;%lu %03u;%lu;%ld;%lu %03u;%lu;%ld;%lu %03u;%lu;%ld;%lu %03u;%lu;%ld;%lu]",
                      synced_dids[0].detDid,
                      synced_dids[0].hitCount,
                      synced_dids[0].reliability,
                      synced_dids[0].adcOutRms,
                      synced_dids[1].detDid,
                      synced_dids[1].hitCount,
                      synced_dids[1].reliability,
                      synced_dids[1].adcOutRms,
                      synced_dids[2].detDid,
                      synced_dids[2].hitCount,
                      synced_dids[2].reliability,
                      synced_dids[2].adcOutRms,
                      synced_dids[3].detDid,
                      synced_dids[3].hitCount,
                      synced_dids[3].reliability,
                      synced_dids[3].adcOutRms,
                      synced_dids[4].detDid,
                      synced_dids[4].hitCount,
                      synced_dids[4].reliability,
                      synced_dids[4].adcOutRms);
                }
              }
            }
          }

          htlv_values_list = htlv_values_list->nextList;
        }
      }
    }while(wait_resp == TRUE);
  }

  if (final_confirm_values != NULL)
  {
    //Free memory
    aux_err = VbDatamodelHtlvsListValueDestroy( &final_confirm_values );

    if (aux_err != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR,"Error releasing TLVs list (err %d)", aux_err);
    }
  }

  return result;
}

/*******************************************************************/

/**
 * @brief Starts alignment check thread
 * @param[in] schedTime Scheduled time to send CycQuery message to G.hn nodes
 * @return TRUE: if thread started OK; FALSE: otherwise
 **/
static BOOLEAN VbAlignmentCheckProcessStart(struct timespec *schedTime)
{
  BOOLEAN running = FALSE;

  if (schedTime != NULL)
  {
    // Stop previous thread (if any)
    VbAlignmentCheckProcessStop();

    // Store sched time to use in thread
    vbAlignmentCheckParams.schedTime = *schedTime;

    VbLogPrint(VB_LOG_INFO, "Starting %s thread", ALIGN_CHECK_THREAD_NAME);

    // Mark the thread as running
    VBAlignmentCheckStateSet(TRUE);

    // Start the thread
    running = VbThreadCreate(ALIGN_CHECK_THREAD_NAME, VBAlignmentCheckProcess, NULL, VB_DRIVER_ALIGNMENT_THREAD_PRIORITY, &vbAlignmentCheckThread);

    if (running == FALSE)
    {
      VbLogPrint(VB_LOG_ERROR, "Can't create %s thread", ALIGN_CHECK_THREAD_NAME);
      VBAlignmentCheckStateSet(FALSE);
    }
  }

  return running;
}

/*******************************************************************/

/**
 * @brief Starts alignment change thread
 * @return TRUE: if thread started OK; FALSE: otherwise
 **/
static BOOLEAN VbAlignmentChangeProcessStart(void)
{
  BOOLEAN running;

  // Stop previous thread (if any)
  VbAlignmentChangeProcessStop();

  VbLogPrint(VB_LOG_INFO, "Starting %s thread", ALIGN_CHANGE_THREAD_NAME);

  // Mark the thread as running
  VBAlignmentChangeStateSet(TRUE);

  // Start the thread
  running = VbThreadCreate(ALIGN_CHANGE_THREAD_NAME, VBAlignmentChangeProcess, NULL, VB_DRIVER_ALIGNMENT_THREAD_PRIORITY, &vbAlignmentChangeThread);

  if (running == FALSE)
  {
    VbLogPrint(VB_LOG_ERROR, "Can't create %s thread", ALIGN_CHANGE_THREAD_NAME);
    VBAlignmentChangeStateSet(FALSE);
  }

  return running;
}

/*******************************************************************/

static t_VB_comErrorCode VbAlignmentSyncLostWait(t_Callbacks *callback)
{
  t_VB_comErrorCode                  ret = VB_COM_ERROR_NONE;
  t_HGF_LCMP_ErrorCode               lcmp_err;
  t_HTLVsLists                      *list_received_values = NULL;

  if (callback == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    lcmp_err = VbLcmpWaitNotifyPermanent(0, &list_received_values, callback);

    if (VBAlignmentSyncLostMonitorStateGet() == FALSE)
    {
      // Sync Lost monitor thread is stopping
      ret = VB_COM_ERROR_NONE;
    }
    else
    {
      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        ret = VB_COM_ERROR_LCMP_WAITNOTIFY;
        VbLogPrint(VB_LOG_ERROR, "Error %d waiting for AlignSyncLost.ind", lcmp_err);
      }

      if (ret == VB_COM_ERROR_NONE)
      {
        if (list_received_values == NULL)
        {
          ret = VB_COM_ERROR_LCMP_WAITNOTIFY;
          VbLogPrint(VB_LOG_ERROR, "Payload empty while receiving AlignSyncLost.ind", list_received_values);
        }
      }

      if (ret == VB_COM_ERROR_NONE)
      {
        t_ValuesArray                     *values_array = NULL;
        t_HTLVValuesList                  *received_values = NULL;
        INT32U                             value_idx = 0;
        INT16U                             list_idx = 0;
        t_lcmpAlignSyncLostInd            *align_sync_lost;

        list_idx = 0;
        received_values = list_received_values->head;

        while ((list_idx < list_received_values->NumLists) && (received_values != NULL))
        {
          list_idx++;
          values_array =  received_values->HTLVsArray;

          for (value_idx = 0; (value_idx < values_array->NumValues) && (ret == VB_COM_ERROR_NONE); value_idx++)
          {
            if (values_array->values[value_idx].ValueLength == sizeof(t_lcmpAlignSyncLostInd))
            {
              align_sync_lost = (t_lcmpAlignSyncLostInd *)values_array->values[value_idx].Value;

              if (align_sync_lost->notifyType == VB_ALIGN_SYNCLOST)
              {
                CHAR  mac_str[MAC_STR_LEN];

                MACAddrMem2str(mac_str, received_values->srcMAC);
                VbLogPrint(VB_LOG_INFO, "AlignSyncLost.ind received from %s : devId lost %u", mac_str, align_sync_lost->syncDid);

                // Send EA message
                ret = VbEAAlignSyncLostSend(received_values->srcMAC, align_sync_lost->syncDid);

                if (ret != VB_COM_ERROR_NONE)
                {
                  VbLogPrint(VB_LOG_ERROR, "Error %d sending EAAlignSyncLost.trg message", ret);
                }

                if (ret == VB_COM_ERROR_NONE)
                {
                  // Force a network discovery to refresh info. Maybe a DM has been switched off
                  VbDomainsMonitorSignal();
                }
              }
            }
          }
          received_values = received_values->nextList;
        }
      }
    }
  }

  //Free memory
  VbDatamodelHtlvsListValueDestroy(&list_received_values);

  return ret;
}

/*******************************************************************/

static void *VbAlignmentSyncLostThread(void *arg)
{
  t_VB_comErrorCode    err = VB_COM_ERROR_NONE;
  BOOLEAN              run = TRUE;
  mqd_t                vb_main_queue;

  vb_main_queue = mq_open(VBQUEUENAME, O_WRONLY);

  if (vbAlignmentSyncLostLcmpCb == NULL)
  {
    err = VB_COM_ERROR_LCMP_WAITNOTIFY;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    while ((VBAlignmentSyncLostMonitorStateGet() == TRUE) && (run == TRUE))
    {
      // Wait for sync lost frames
      err = VbAlignmentSyncLostWait(vbAlignmentSyncLostLcmpCb);

      if ((VBAlignmentSyncLostMonitorStateGet() == FALSE) || (err != VB_COM_ERROR_NONE))
      {
        run = FALSE;
      }
    }
  }

  if (err != VB_COM_ERROR_NONE)
  {
    VbMainQEvSend(DRIVER_EV_SYNC_LOST_KO, vb_main_queue, NULL);
  }

  mq_close(vb_main_queue);

  return NULL;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_VB_comErrorCode VbAlignmentSeqNumGet(INT16U *seqNum, BOOLEAN *valid)
{
  t_VB_comErrorCode    ret = VB_COM_ERROR_NONE;
  INT8U                mac[ETH_ALEN];
  t_ValuesArray       *values_to_read = NULL;
  t_HTLVsLists        *read_htlvs = NULL;
  CHAR                 mac_str[MAC_STR_LEN];
  t_HGF_LCMP_ErrorCode lcmp_err;
  t_lcmpComParams      lcmp_params;
  INT8U                parameters[1];

  if ((seqNum == NULL) || (valid == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *seqNum = 0;
    *valid = FALSE;

    // Get a DM MAC address
    ret = VbDatamodelDMMACGet(0, mac);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    MACAddrMem2str(mac_str, mac);

    parameters[0] = (INT8U)VB_MACSEQNUM;

    ret = VbDatamodelValueToArrayAdd(1, (INT8U *)parameters, &values_to_read);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      ret = VB_COM_ERROR_LCMP;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = UNICAST;
    lcmp_params.dstMac          = mac;
    lcmp_params.paramIdReq      = VB_MACSEQNUM;
    lcmp_params.reqValues       = values_to_read;
    lcmp_params.rspValuesList   = &read_htlvs;

    // Read MAC sequence number
    lcmp_err = VbLcmpRead(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d getting MAC sequence number from %s", lcmp_err, mac_str);

      ret = VB_COM_ERROR_LCMP_READ;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Process received data

    t_HTLVValuesList          *htlv_values_list;
    t_ValuesArray             *values_array;
    INT16U                     cnf_tlv_list_idx;
    INT16U                     values_idx;
    t_lcmpMacSeqNum           *vb_mac_seqnum_param;

    htlv_values_list = read_htlvs->head;
    cnf_tlv_list_idx = 0;

    ret = VB_COM_ERROR_NOT_FOUND;

    while ((htlv_values_list != NULL) && (cnf_tlv_list_idx < read_htlvs->NumLists))
    {
      cnf_tlv_list_idx++;
      values_array = htlv_values_list->HTLVsArray;

      if (values_array != NULL)
      {
        for (values_idx = 0; values_idx < values_array->NumValues; values_idx++)
        {
          if (values_array->values[values_idx].ValueLength == VB_MACSEQNUM_SIZE)
          {
            vb_mac_seqnum_param = (t_lcmpMacSeqNum *)values_array->values[values_idx].Value;

            if (vb_mac_seqnum_param->paramType == VB_MACSEQNUM)
            {
              // Get current sequence number
              *seqNum = _ntohs_ghn(vb_mac_seqnum_param->macSeqNum);
              *valid = TRUE;

              VbLogPrint(VB_LOG_INFO, "MacSeqNum.rsp received from %s\tseqNum: %u", mac_str, *seqNum);

              // Sequence number found, break the loop
              ret = VB_COM_ERROR_NONE;
              break;
            }
          }
        }
      }

      htlv_values_list = htlv_values_list->nextList;
    }
  }

  // Release memory
  VbDatamodelHtlvsListValueDestroy(&read_htlvs);
  VbDatamodelHTLVsArrayDestroy(&values_to_read);

  if (ret == VB_COM_ERROR_NOT_FOUND)
  {
    // No devices present in domains list, return 0 as sequence number
    *seqNum = 0;
    *valid = FALSE;

    ret = VB_COM_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbAlignmentCycQuerySend(void)
{
  t_VB_comErrorCode      result = VB_COM_ERROR_NONE;
  t_ValuesArray         *tlv_notify_values = NULL;
  t_HGF_LCMP_ErrorCode   lcmp_err;
  t_lcmpComParams        lcmp_params;
  t_lcmpCycQueryExtInd   vb_cyc_query_value;
  t_Callbacks           *callback_installed = NULL;

  // Install callback to receive CycQueryNotifExt
  lcmp_err = VbLcmpWaitNotifyPermanentInstall(VB_CYCQUERYNOTIF_EXT, &callback_installed);

  if (lcmp_err != HGF_LCMP_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR,
        "Error installing callback to receive CycQueryNotifExt.ind (err %d)",
        lcmp_err);
    result = VB_COM_ERROR_LCMP_WAITNOTIFY;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    memset(&vb_cyc_query_value, 0, sizeof(vb_cyc_query_value));

    vb_cyc_query_value.notifyType = VB_CYCQUERY_EXT;
    result = LcmpMacGet(&vb_cyc_query_value.aeMac[0]);

    // Reset detection statistics
    vb_cyc_query_value.resetStats = 1;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    result = VbDatamodelValueToArrayAdd(VB_CYCQUERYEXT_SIZE,(INT8U *)&vb_cyc_query_value,
        &tlv_notify_values);
  }

  if (result == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      result = VB_COM_ERROR_LCMP;
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = MULTICAST_DMS;
    lcmp_params.paramIdReq      = VB_CYCQUERY_EXT;
    lcmp_params.reqValues       = tlv_notify_values;

    // Send CycQuery
    lcmp_err = VbLcmpNotify(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending CycQueryExt.ind", lcmp_err);
      result = VB_COM_ERROR_LCMP_WRITE;
    }
  }

  // Always release allocated memory
  VbDatamodelHTLVsArrayDestroy( &tlv_notify_values );

  if (result == VB_COM_ERROR_NONE)
  {
    // Wait CycQuery responses
    result = VBCycRespWait(callback_installed, lcmp_params.tsTx);

    if (result != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error waiting CycQuery responses (err %d)", result);
    }
  }

  if (callback_installed != NULL)
  {
    // Remove callback to release resources and avoid receiving unexpected CycQuery.ind messages
    VbLcmpWaitNotifyPermanentUninstall(&callback_installed);
  }

  return result;
}

/*******************************************************************/

t_VB_comErrorCode VbAlignmentEACycQueryReqProcess(INT8U *msg)
{
  t_VB_comErrorCode       ret = VB_COM_ERROR_NONE;
  t_vbEACycQueryReq      *cycquery_req;
  struct timespec        sched_time;
  BOOLEAN                 running;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Get sched time
    cycquery_req = (t_vbEACycQueryReq *)msg;
    sched_time.tv_sec = _ntohl((INT32U)cycquery_req->tvSec);
    sched_time.tv_nsec = _ntohl((INT32U)cycquery_req->tvNsec);

    // Launch alignment check thread
    running = VbAlignmentCheckProcessStart(&sched_time);

    if (running == FALSE)
    {
      ret = VB_COM_ERROR_THREAD;
    }
  }

  return ret;
}

/*******************************************************************/

void VbAlignmentCheckProcessStop(void)
{
  if (VBAlignmentCheckStateGet() == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", ALIGN_CHECK_THREAD_NAME);

    VBAlignmentCheckStateSet(FALSE);

    // Wake up thread
    VbThreadWakeUp(vbAlignmentCheckThread);

    VbThreadJoin(vbAlignmentCheckThread, ALIGN_CHECK_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", ALIGN_CHECK_THREAD_NAME);
  }
}

/*******************************************************************/

t_VB_comErrorCode VbAlignmentEACycChangeReqProcess(INT8U *msg)
{
  t_VB_comErrorCode       ret = VB_COM_ERROR_NONE;
  BOOLEAN                 running;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    INT32U                         num_nodes;
    INT32U                         idx;
    INT32U                         offset;
    t_vbEACycChangeReqCommon      *align_change_common = (t_vbEACycChangeReqCommon *)msg;
    t_vbEACycChangeReqNode        *align_change_node;

    // Get number of nodes to change alignment
    num_nodes = _ntohs(align_change_common->numNodes);

    offset = VB_EA_CYCCHANGE_REQ_COMMON_SIZE;

    for (idx = 0; idx < num_nodes; idx++)
    {
      align_change_node = (t_vbEACycChangeReqNode *)&(msg[offset]);

      ret = VbDatamodelDomainAlignChangeUpdate(align_change_node->MAC,
          _ntohs(align_change_node->seqNum),
          align_change_node->changeEdge);

      if (ret != VB_COM_ERROR_NONE)
      {
        CHAR mac_str[MAC_STR_LEN];

        MACAddrMem2str(mac_str, align_change_node->MAC);
        VbLogPrint(VB_LOG_ERROR, "Error %d processing EA_CycChange.req message for node %s", ret, mac_str);

        // Ignore this error and continue with next node
        ret = VB_COM_ERROR_NONE;
      }

      offset += VB_EA_CYCCHANGE_REQ_NODE_SIZE;
    }

    // Launch alignment change thread
    running = VbAlignmentChangeProcessStart();

    if (running == FALSE)
    {
      ret = VB_COM_ERROR_THREAD;
    }
  }

  return ret;
}

/*******************************************************************/

void VbAlignmentChangeProcessStop(void)
{
  if (VBAlignmentChangeStateGet() == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", ALIGN_CHANGE_THREAD_NAME);

    VBAlignmentChangeStateSet(FALSE);

    VbThreadJoin(vbAlignmentChangeThread, ALIGN_CHANGE_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", ALIGN_CHANGE_THREAD_NAME);
  }
}

/*******************************************************************/

t_VB_comErrorCode VbAlignmentModeProcess(INT8U *msg)
{
  t_VB_comErrorCode           ret = VB_COM_ERROR_NONE;
  t_VB_comErrorCode           err = VB_COM_ERROR_NONE;
  t_vbEAAlignModeRspErrorCode rsp_err = VB_EA_ALIGNMODE_RSP_ERR_NONE;
  t_vbAlignModeParam         *eng_align_mode;
  t_vbAlignModeRelayParam    *relays = NULL;
  t_vbAlignModeSeedParam     *seeds = NULL;
  t_vbCdtaEngineConf         *cdta_conf;
  t_vbBoostBandLastCarrier   *boosted_band_last_carrier;
  INT8U                       num_relays;
  INT8U                       num_seeds;
  INT8U                       num_boost_bands;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    eng_align_mode = (t_vbAlignModeParam *)msg;
    cdta_conf      = (t_vbCdtaEngineConf *)((msg + sizeof(t_vbAlignModeParam)));

    // Get number of potential boosted bands
    num_boost_bands = cdta_conf->nBoostBands;

    boosted_band_last_carrier = (t_vbBoostBandLastCarrier *)((INT8U*)cdta_conf + sizeof(t_vbCdtaEngineConf));

    // Get AlignId
    vbAlignmentId = eng_align_mode->alignId;

    // Get number of nodes to change alignment
    num_relays = eng_align_mode->numRelays;

    if(num_relays > 0)
    {
      relays = (t_vbAlignModeRelayParam *)(msg + sizeof(t_vbAlignModeParam) +
                                                 sizeof(t_vbCdtaEngineConf) +
                                                 num_boost_bands*sizeof(t_vbBoostBandLastCarrier));
    }

    // Get number of seeds
    num_seeds = eng_align_mode->numSeeds;

    if(num_seeds > 0)
    {
      seeds = (t_vbAlignModeSeedParam *)(msg + sizeof(t_vbAlignModeParam) +
                                                 sizeof(t_vbCdtaEngineConf) +
                                                 num_boost_bands*sizeof(t_vbBoostBandLastCarrier) +
                                                 num_relays*sizeof(t_vbAlignModeRelayParam));
    }

    // Build and send LCMP AlignMode.req messages to nodes
    err = LCMPAlignmentModeSend(eng_align_mode, cdta_conf, boosted_band_last_carrier, relays, seeds);
    if (err != VB_COM_ERROR_NONE)
    {
      rsp_err = VB_EA_ALIGNMODE_RSP_ERR_DRV;
      VbLogPrint(VB_LOG_ERROR, "Error %d sending LCMP AlignMode.req", err);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Build and send response to VB engine
    ret = EAAlignModeRspSend(rsp_err, vbAlignmentId);

    if (ret != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending EAAlignmentMode.rsp", ret);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbAlignmentClusterStopProcess(INT8U *msg)
{
  t_VB_comErrorCode           ret = VB_COM_ERROR_NONE;
  t_VB_comErrorCode           err = VB_COM_ERROR_NONE;
  t_vbEAAlignModeRspErrorCode rsp_err = VB_EA_ALIGNMODE_RSP_ERR_NONE;
  t_vbEAAlignClusterStopReq  *cluster_stop_req;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {

    cluster_stop_req = (t_vbEAAlignClusterStopReq *)msg;


    // Build and send LCMP AlignClusterStop.req messages to nodes
    err = VbAlignmentLCMPClusterStopSend(cluster_stop_req->stopTxFlag);

    if (err != VB_COM_ERROR_NONE)
    {
      rsp_err = VB_EA_ALIGNMODE_RSP_ERR_DRV;
      ret = VB_COM_ERROR_LCMP;
      VbLogPrint(VB_LOG_ERROR, "Error %d sending LCMP ClusterStop.req", err);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Build and send response to VB engine
    ret = EAAlignClusterStopRspSend(rsp_err);

    if (ret != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending EAClusterStop.rsp", ret);
    }
  }

  return ret;
}

/*******************************************************************/

BOOLEAN VbAlignmentSyncLostMonitorRun(void)
{
  BOOLEAN              running = FALSE;
  t_HGF_LCMP_ErrorCode lcmp_err;

  // Stop previous thread (if any)
  VbAlignmentSyncLostMonitorStop();

  VbLogPrint(VB_LOG_INFO, "Starting %s thread", ALIGN_SYNC_LOST_THREAD_NAME);

  // Mark the thread as running
  VBAlignmentSyncLostMonitorStateSet(TRUE);

  // Install callback to receive AlignSyncLost.ind LCMP message
  lcmp_err = VbLcmpWaitNotifyPermanentInstall(VB_ALIGN_SYNCLOST, &vbAlignmentSyncLostLcmpCb);

  if (lcmp_err != HGF_LCMP_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d installing LCMP callback", lcmp_err);
  }
  else
  {
    // Start the thread
    running = VbThreadCreate(ALIGN_SYNC_LOST_THREAD_NAME, VbAlignmentSyncLostThread, NULL, VB_DRIVER_ALIGNMENT_THREAD_PRIORITY, &vbAlignmentSyncLostThread);
  }

  if (running == FALSE)
  {
    VbLogPrint(VB_LOG_ERROR, "Can't create %s thread", ALIGN_SYNC_LOST_THREAD_NAME);

    VBAlignmentSyncLostMonitorStateSet(FALSE);

    if (vbAlignmentSyncLostLcmpCb != NULL)
    {
      // Uninstall callback
      lcmp_err = VbLcmpWaitNotifyPermanentUninstall(&vbAlignmentSyncLostLcmpCb);

      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d uninstalling LCMP callback", lcmp_err);
      }

      vbAlignmentSyncLostLcmpCb = NULL;
    }
  }

  return running;
}

/*******************************************************************/

void VbAlignmentSyncLostMonitorStop(void)
{
  if (VBAlignmentSyncLostMonitorStateGet() == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", ALIGN_SYNC_LOST_THREAD_NAME);

    VBAlignmentSyncLostMonitorStateSet(FALSE);

    if (vbAlignmentSyncLostLcmpCb != NULL)
    {
      // Wake up thread
      VbLcmpCallbackWakeUp(&(vbAlignmentSyncLostLcmpCb->CallbackData));
    }

    VbThreadJoin(vbAlignmentSyncLostThread, ALIGN_SYNC_LOST_THREAD_NAME);

    if (vbAlignmentSyncLostLcmpCb != NULL)
    {
      t_HGF_LCMP_ErrorCode lcmp_err;

      // Uninstall callback
      lcmp_err = VbLcmpWaitNotifyPermanentUninstall(&vbAlignmentSyncLostLcmpCb);

      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d uninstalling LCMP callback", lcmp_err);
      }

      vbAlignmentSyncLostLcmpCb = NULL;
    }

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", ALIGN_SYNC_LOST_THREAD_NAME);
  }
}

/*******************************************************************/

 t_VB_comErrorCode VbAlignmentLCMPClusterStopSend(BOOLEAN stopTx)
{
  t_VB_comErrorCode    ret = VB_COM_ERROR_NONE;
  t_ValuesArray       *control_values_array = NULL;
  t_HGF_LCMP_ErrorCode lcmp_err;
  t_lcmpComParams      lcmp_params;

  if (ret == VB_COM_ERROR_NONE)
  {
    t_lcmpClusterStopReq       cluster_stop;

    // Build message
    bzero(&cluster_stop, sizeof(cluster_stop));
    cluster_stop.ctrlType = VB_CLUSTER_STOP;
    cluster_stop.stopTxFlag = stopTx;

    ret = VbDatamodelValueToArrayAdd(sizeof(cluster_stop), (INT8U *)(&cluster_stop), &control_values_array);
    if (ret != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d building ClusterStop.req LCMP message", ret);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      ret = VB_COM_ERROR_LCMP;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = MULTICAST_DMS;
    lcmp_params.paramIdReq      = VB_CLUSTER_STOP;
    lcmp_params.reqValues       = control_values_array;

    // Send control frame to stop cluster
    lcmp_err = VbLcmpControl(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending ClusterStop.req", lcmp_err);
       ret = VB_COM_ERROR_LCMP_CONTROL;
    }
  }

  // Release req memory
  VbDatamodelHTLVsArrayDestroy(&control_values_array);

  return ret;
}
/*******************************************************************/

/**
 * @}
 **/
