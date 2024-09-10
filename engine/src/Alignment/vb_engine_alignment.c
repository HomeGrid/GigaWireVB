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
 * @file vb_engine_alignment.c
 * @brief VB Drivers alignment
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-10
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"
#include <time.h>
#include <strings.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

#include "vb_log.h"
#include "vb_util.h"
#include "vb_metrics.h"

#include "vb_counters.h"
#include "vb_engine_alignment.h"
#include "vb_engine_alignment_metrics.h"
#include "vb_engine_EA_interface.h"
#include "vb_engine_clock.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_conf.h"
#include "Process/vb_engine_process.h"

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

#define VB_ALIGN_MAX_NUM_CLUSTERS             (24)

#define VB_ALIGN_MAX_CLOCK_DEV                (MS_TO_10NS_UNITS(MAC_CYCLE_DURATION / 2))   // In 10ns
#define VB_ALIGN_PERIODIC_CHECK_TO            (300000) // In msec
#define VB_ALIGN_MAX_SYNC_FRAME_DURATION      (16000)
#define VB_ALIGN_GHN_REF_TX_IDX               (0)
#define VB_ALIGN_GHN_OFFSET_TYPE              (VB_ALIGN_OFFSET_REL)

// Translate ADC RMS from message format
#define VB_ALIGN_ADC_RMS_MSG_DEC_BITS         (14)
#define VB_ALIGN_ADC_RMS_FROM_MSG_FORMAT(X)   (10*log10((FP32)(X) / (1 << VB_ALIGN_ADC_RMS_MSG_DEC_BITS)))

// Minimum ADC RMS level
#define VB_ALIGN_ADC_RMS_MIN                  (-42)  // In dB

// Maximum number of devices in the same domain
#define VB_ALIGN_MAX_DEVICE_ID                (250)

// Invalid number of hops. It means no valid path to reference node
#define VB_ALIGN_INVALID_HOPS                 (MAX_INT8U)

// Max number of hops
#define VB_ALIGN_MAX_HOPS                     (VB_EA_ALIGN_GHN_MAX_TX_NODES)

// Num cycles of extra time in case of sync lost
#define VB_ALIGN_GHN_NUM_CYCLES_EXTRA_TIME    (3)


/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

/// Alignment offset type
typedef enum
{
  VB_ALIGN_OFFSET_REL = 0,
  VB_ALIGN_OFFSET_ABS,
  VB_ALIGN_OFFSET_LAST,
} t_vbAlignOffsetType;

typedef struct
{
  BOOLEAN            aligned;
  t_alignInfo        ref;
  INT8U             *payload;
  INT32U             length;
  INT32U             idx;
} t_vbAlignChangeInfo;

typedef struct
{
  BOOLEAN            found;
  t_domain          *targetDomain;
  t_alignRole        targetRole;
} t_vbAlignRoleSearch;

typedef struct
{
  BOOLEAN            found;
  t_domain          *targetDomain;
  INT8U              targetDid;
} t_vbAlignDidSearch;

typedef struct
{
  INT32U             offset;
  INT32U             duration;
} t_vbAlignGhnTxParams;

typedef struct
{
  BOOLEAN            found;
  t_domain          *currRef;
  t_domain          *nextCandidate;
} t_vbAlignGhnRefInfo;

typedef struct
{
  INT8U                    syncDids[VB_EA_ALIGN_GHN_MAX_TX_NODES];
  t_vbAlignGhnTxNodesInfo *txNodesInfo;
  INT32U                   numNodes;
  BOOLEAN                  aligned;
} t_vbAlignGhnVisibilityInfo;

typedef struct
{
  BOOLEAN             used;
  INT8U               macAddr[6];
} t_vbAlignGhnSeedInfo;

typedef struct
{
  INT16U              seed;
  INT8U               macAddr[6];
} t_vbAlignGhnDidInfo;


typedef struct
{
  INT32U              numClusters;
  INT32U              clusterId[VB_ALIGN_MAX_NUM_CLUSTERS];
} t_vbAlignGhnClustersStopList;

typedef struct
{
  t_alignRole         currRole;
  t_alignRole         newRole;
} t_vbAlignChangeRoleParams;

typedef struct
{
  BOOLEAN             force;
  INT32U              clusterId;
} t_vbAlignPeriodicCheckForce;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_vbAlignChangeInfo      vbAlignChangeInfo;
static t_vbAlignPeriodicCheckForce vbAlignPeriodicCheckForce;
static BOOLEAN                  initSeedArray = FALSE;
static t_vbAlignGhnSeedInfo     *seedArray;
static INT16U                   seedArraySize = 0;
static t_vbAlignGhnDidInfo      didArray[VB_ALIGN_MAX_DEVICE_ID+1] = {{0}};
static INT8U                    vbAlignEngineConfId = 0;

static const t_vbAlignGhnTxParams  vbAlignGhnTxInfoArray[VB_EA_ALIGN_GHN_MAX_TX_NODES] =
{
  // Offset (DS Slot #)       Duration
    {1,                       VB_ALIGN_MAX_SYNC_FRAME_DURATION},       ///< Tx params for reference node
    {2,                       VB_ALIGN_MAX_SYNC_FRAME_DURATION},       ///< Tx params for relay #0
    {3,                       VB_ALIGN_MAX_SYNC_FRAME_DURATION},       ///< Tx params for relay #1
    {4,                       VB_ALIGN_MAX_SYNC_FRAME_DURATION},       ///< Tx params for relay #2
    {5,                       VB_ALIGN_MAX_SYNC_FRAME_DURATION},       ///< Tx params for relay #3
};

static const CHAR *vbAlignRoleString[VB_ALIGN_ROLE_LAST] =
{
   "NOT_INIT",
   "SLAVE",
   "REFERENCE",
   "RELAY",
   "RELAY_CAND",
};

static const CHAR *vbAlignModeString[VB_ALIGN_MODE_LAST] =
{
   "COMMON_CLOCK",
   "GHN",
   "PTP",
};

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

static inline void AlignGhnNodeRoleSet(t_node *dm, t_alignRole role, INT32U clusterId)
{
  if ((dm != NULL) && (role < VB_ALIGN_ROLE_LAST))
  {
    t_alignRole prev_role = dm->nodeAlignInfo.role;

    dm->nodeAlignInfo.role = role;

    VbeUpdateDMsHistory(dm->MAC, clusterId, role, TRUE);

    if (role == VB_ALIGN_ROLE_REF)
    {
      // Init number of hops for reference node. For rest of roles, maintain current number of hops
      dm->nodeAlignInfo.hops = 0;
    }

    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
        "Domain %s - DID %3u - Role %10s -> %10s",
        dm->MACStr,
        dm->devID,
        VbEngineAlignRoleStringGet(prev_role),
        VbEngineAlignRoleStringGet(role));

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR,
        "[%-10s] Domain %s - DID %3u - Role %10s -> %10s",
        "Conf",
        dm->MACStr,
        dm->devID,
        VbEngineAlignRoleStringGet(prev_role),
        VbEngineAlignRoleStringGet(role));
  }
}

/*******************************************************************/

static inline BOOLEAN AlignGhnDetectedDidIsValid(t_syncDetInfo *info)
{
  BOOLEAN is_valid;

  if (info != NULL)
  {
    INT32U pow_thr;

    if (info->allowed == TRUE)
    {
      // Use hysteresis
      pow_thr = VbEngineConfAlignMinPowHystGet();
    }
    else
    {
      pow_thr = VbEngineConfAlignMinPowGet();
    }

    if ((info->detDid != 0) &&
        (info->adcOutRms > pow_thr) &&
        (info->reliability < VbEngineConfAlignRelThrGet()))
    {
      is_valid = TRUE;
    }
    else
    {
      is_valid = FALSE;
    }
  }
  else
  {
    is_valid = FALSE;
  }

  return is_valid;
}

/*******************************************************************/

static inline BOOLEAN AlignNodeIsSynced(t_domain *domain)
{
  BOOLEAN              synced = FALSE;

  if (domain != NULL)
  {
    synced = domain->dm.nodeAlignInfo.synced;
  }

  return synced;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignCorrectionsGet(t_alignInfo *reference, t_alignInfo *target, t_alignChange *corrections)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((reference == NULL) || (target == NULL) || (corrections == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U             min_win;
    INT32U             max_win;
    INT32U             mac_cycle_duration = MS_TO_10NS_UNITS(MAC_CYCLE_DURATION);
    INT16U             adj_target_seq_num;
    t_VBAlignmentMode  alignment_mode;

    ret = VbEngineConfAlignmentModeGet(&alignment_mode);
    if (ret == VB_ENGINE_ERROR_NONE)
    {
      if(alignment_mode == VB_ALIGN_MODE_COMMON_CLOCK)
      {

        // Calculate limits to check clock edge (reference clock +- 10ms)
        min_win = DIFANY(reference->macClock, (VB_ALIGN_MAX_CLOCK_DEV / 2), mac_cycle_duration);
        max_win = reference->macClock + (VB_ALIGN_MAX_CLOCK_DEV / 2);

        // Check overflow
        if (max_win > mac_cycle_duration)
        {
          max_win -= mac_cycle_duration;
        }

        // Check clock edge
        if (max_win > min_win)
        {
          /*
           *         min_win      max_win
           *   |        {     !      }           |
           *   |        {     !      }           |
           *  =========================================
           *   0           refClock            40ms
           *
           */
          if ((target->macClock >= min_win) && (target->macClock <= max_win))
          {
            /*
             *         min_win ref  max_win
             *   |        {     !   *  }           |
             *   |        {     !   *  }           |
             *  =========================================
             *   0             targetClock        40ms
             *
             */

            // Clock is located in same edge than reference one
            corrections->clockEdge = FALSE;
          }
          else
          {
            /*
             *         min_win ref  max_win
             *   |        {     !      }    *      |
             *   |        {     !      }    *      |
             *  =========================================
             *   0                     targetClock 40ms
             *
             */

            // Clock is in different edge outside min and max limits
            corrections->clockEdge = TRUE;
          }
        }
        else
        {
          /*
           *                     min_win          max_win
           *   |                    {      !    |    }
           *   |                    {      !    |    }
           *  ==========================================================
           *   0                      refClock 40ms
           *
           */

          if ((target->macClock >= min_win) || (target->macClock <= max_win))
          {
            /*
             *                     min_win  ref     max_win
             *   |                    {  *   !    |    }
             *   |                    {  *   !    |    }
             *  ==========================================================
             *   0                  targetClock  40ms
             *
             * OR
             *
             *                     min_win  ref     max_win
             *   |                    {      !    |   *}
             *   |                    {      !    |   *}
             *  ==========================================================
             *   0                             40ms  targetClock
             */

            // Clock is located in same edge than reference one
            corrections->clockEdge = FALSE;
          }
          else
          {
            /*
             *                     min_win  ref     max_win
             *   |          *         {      !    |    }
             *   |          *         {      !    |    }
             *  ==========================================================
             *   0      targetClock             40ms
             *
             */

            // Clock is in different edge
            corrections->clockEdge = TRUE;
          }
        }

        if (corrections->clockEdge == FALSE)
        {
          if (max_win > min_win)
          {
            /*
             *                SEQNUM X                 SEQNUM X+1
             *         min_win ref  max_win
             *   |        {     !  *   }           |
             *   |        {     !  *   }           |
             *  =========================================
             *   0            targetClock         40ms
             *
             */

            // Target clock is in the same cycle than reference clock
            adj_target_seq_num = target->seqNum;
          }
          else
          {
            /*
             *              SEQNUM X                          SEQNUM X+1
             *                     min_win          max_win
             *   |                    {           |    }
             *   |                    {           |    }
             *  ==========================================================
             *   0                               40ms
             *
             */

            if ((target->macClock > max_win) && (reference->macClock < min_win))
            {
              /*
               *              SEQNUM X                          SEQNUM X+1
               *                     min_win          max_win
               *   |                    {   *       |  !  }
               *   |                    {   *       |  !  }
               *  ==========================================================
               *   0                      target  40ms ref
               *
               * Target clock is near to cross from cycle X to X+1.
               * Reference clock is already in next cycle, so we shall adjust target
               * sequence number to compare with reference one.
               */
              adj_target_seq_num = target->seqNum + 1;
            }
            else if ((reference->macClock > max_win) && (target->macClock < min_win))
            {
              /*
               *              SEQNUM X                          SEQNUM X+1
               *                     min_win          max_win
               *   |                    {   !       |  *  }
               *   |                    {   !       |  *  }
               *  ==========================================================
               *   0                       ref    40ms target
               *
               * Reference clock is near to cross from cycle X to X+1.
               * Target clock is already in next cycle, so we shall adjust target
               * sequence number to compare with reference one.
               */
              adj_target_seq_num = target->seqNum - 1;
            }
            else
            {
              /*
               *              SEQNUM X                          SEQNUM X+1
               *                     min_win          max_win
               *   |                    {   !     *      |    }
               *   |                    {   !     *      |    }
               *  ==========================================================
               *   0                       ref  target  40ms
               *
               * Both clocks are in the same cycle.
               */
              adj_target_seq_num = target->seqNum;
            }
          }
        }
        else
        {
          // Target clock is far from reference clock, use target sequence number as is, do not adjust its value
          adj_target_seq_num = target->seqNum;
        }

        // Calculate sequence number offset
        corrections->seqNumOffset = DIF16(reference->seqNum, adj_target_seq_num);
      }
      else
      {
        corrections->clockEdge = FALSE;
        corrections->seqNumOffset = 0;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode CycQueryReqLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEACycQueryReq   *cycquery_req;

  if ((driver == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ((driver->FSMState == ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY) || (driver->FSMState == ENGINE_STT_DISCONNECTED ))
    {
      // Skip drivers in "ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY" or in "ENGINE_STT_DISCONNECTED" state
      ret = VB_ENGINE_ERROR_SKIP;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    cycquery_req = (t_vbEACycQueryReq *)args;

    // Send CycQuery.req message
    ret = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_CYCQUERY_REQ, (INT8U *)cycquery_req, VB_EA_CYCQUERY_REQ_SIZE, driver);
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current driver
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode CycChangeReqLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignChangeInfo *align_change_info = (t_vbAlignChangeInfo *)args;
  t_alignChange        align_corrections;

  if ((driver == NULL) || (domain == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (align_change_info->payload == NULL)
    {
      // This is the first iteration, alloc memory for message
      align_change_info->length = VB_EA_CYCCHANGE_REQ_COMMON_SIZE + (driver->domainsList.numDomains * VB_EA_CYCCHANGE_REQ_NODE_SIZE);
      align_change_info->payload = (INT8U *)malloc(align_change_info->length);

      if (align_change_info->payload == NULL)
      {
        ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        t_vbEACycChangeReqCommon *align_change_common = (t_vbEACycChangeReqCommon *)(align_change_info->payload);

        // Init common fields
        align_change_common->numNodes = _htons(driver->domainsList.numDomains);

        // Update idx
        align_change_info->idx = VB_EA_CYCCHANGE_REQ_COMMON_SIZE;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Calculate alignment corrections
    ret = AlignCorrectionsGet(&(align_change_info->ref), &(domain->alignInfo), &align_corrections);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_vbEACycChangeReqNode *align_change_node = (t_vbEACycChangeReqNode *)&(align_change_info->payload[align_change_info->idx]);

    // Fill message fields
    MACAddrClone(align_change_node->MAC, domain->dm.MAC);
    align_change_node->changeEdge = align_corrections.clockEdge;
    align_change_node->seqNum = _htons(align_corrections.seqNumOffset);

    // Update idx
    align_change_info->idx += VB_EA_CYCCHANGE_REQ_NODE_SIZE;

    VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Align change info for %s : changeEdge %u; seqNumOffset %u",
        domain->dm.MACStr, align_corrections.clockEdge, align_corrections.seqNumOffset);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignCheckLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignChangeInfo *align_info = (t_vbAlignChangeInfo *)args;

  if ((driver == NULL) || (domain == NULL) || (align_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_alignChange align_corrections;

    // Compare reference sequence number and clock with domains

    // Calculate alignment corrections
    ret = AlignCorrectionsGet(&(align_info->ref), &(domain->alignInfo), &align_corrections);

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      if ((align_corrections.clockEdge == TRUE) || (align_corrections.seqNumOffset != 0))
      {
        // Mismatch detected in sequence numbers or clock edge
        align_info->aligned = FALSE;

        // Break the domains loop
        ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
      }
    }

    if ((ret != VB_ENGINE_ERROR_NONE) || (align_info->aligned == FALSE))
    {
      // Sequence number or clock edges are different between domains

      VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "MAC Align", "Domain %s - error - seqNum (r %d / %d); macClock (r %u / %u); clockEdge %u; seqNumOffset %u; err %d",
          domain->dm.MACStr,
          align_info->ref.seqNum,
          domain->alignInfo.seqNum,
          align_info->ref.macClock,
          domain->alignInfo.macClock,
          align_corrections.clockEdge,
          align_corrections.seqNumOffset,
          ret);

      VbLogPrintExt(VB_LOG_WARNING, driver->vbDriverID, "[%-10s] Domain %s - error - seqNum (r %d / %d); macClock (r %u / %u); clockEdge %u; seqNumOffset %u; err %d",
          "MAC Align",
          domain->dm.MACStr,
          align_info->ref.seqNum,
          domain->alignInfo.seqNum,
          align_info->ref.macClock,
          domain->alignInfo.macClock,
          align_corrections.clockEdge,
          align_corrections.seqNumOffset,
          ret);
    }
    else
    {
      VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "MAC Align", "Domain %s - ok - firstSeqNum %d checked %d; firstClock %u checked %u",
          domain->dm.MACStr,
          align_info->ref.seqNum,
          domain->alignInfo.seqNum,
          align_info->ref.macClock,
          domain->alignInfo.macClock);

      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "[%-10s] Domain %s - ok - firstSeqNum %d checked %d; firstClock %u checked %u",
          "MAC Align",
          domain->dm.MACStr,
          align_info->ref.seqNum,
          domain->alignInfo.seqNum,
          align_info->ref.macClock,
          domain->alignInfo.macClock);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignGhnBestDetectedDidInfoGet(t_domain *domain, t_syncDetInfo *info)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               best_idx = 0;

  if ((domain == NULL) || (info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U  idx;
    INT32U  max_pow = 0;

    ret = VB_ENGINE_ERROR_NOT_FOUND;

    for (idx = 0; idx < VB_EA_ALIGN_GHN_MAX_TX_NODES; idx++)
    {
      if ((AlignGhnDetectedDidIsValid(&(domain->alignInfo.syncDetsInfo[idx])) == TRUE) &&
          (domain->alignInfo.syncDetsInfo[idx].adcOutRms > max_pow) &&
          (domain->alignInfo.syncDetsInfo[idx].allowed == TRUE))
      {
        max_pow = domain->alignInfo.syncDetsInfo[idx].adcOutRms;
        best_idx = idx;
        ret = VB_ENGINE_ERROR_NONE;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *info = domain->alignInfo.syncDetsInfo[best_idx];
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignGhnVisibilityListInsert(t_vbAlignGhnVisibilityInfo *info, INT8U did)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((info == NULL) || (did == 0))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U  idx;

    ret = VB_ENGINE_ERROR_NOT_FOUND;

    for (idx = 0; idx < VB_EA_ALIGN_GHN_MAX_TX_NODES; idx++)
    {
      if (info->syncDids[idx] == did)
      {
        // Did already present in visibility list, break the loop
        ret = VB_ENGINE_ERROR_NONE;
        break;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NOT_FOUND)
  {
    // Insert new did in list
    if (info->numNodes < VB_EA_ALIGN_GHN_MAX_TX_NODES)
    {
      info->syncDids[info->numNodes] = did;
      info->numNodes++;
      ret = VB_ENGINE_ERROR_NONE;
    }
    else
    {
      ret = VB_ENGINE_ERROR_NO_MEMORY;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignGhnNodeFromTxInfoGet(INT8U did, t_vbAlignGhnTxNodesInfo *txNodesInfo, t_domain **domain)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;

  if ((domain == NULL) || (txNodesInfo == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U idx;

    ret = VB_ENGINE_ERROR_NOT_FOUND;

    for (idx = 0; idx < txNodesInfo->numTxNodes; idx++)
    {
      if ((txNodesInfo->txNodes[idx] != NULL) &&
          (txNodesInfo->txNodes[idx]->dm.devID == did))
      {
        // Node found
        *domain = txNodesInfo->txNodes[idx];
        ret = VB_ENGINE_ERROR_NONE;
        break;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignGhnCheckLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignGhnVisibilityInfo *align_info = (t_vbAlignGhnVisibilityInfo *)args;

  if ((driver == NULL) || (domain == NULL) || (align_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if(domain->dm.nodeAlignInfo.role != VB_ALIGN_ROLE_REF)
    {
      INT32U               idx;
      INT8U                min_hops = VB_ALIGN_INVALID_HOPS;
      BOOLEAN              is_synced = FALSE;

      /*
       * Loop through detected nodes to:
       * - Update sync status of current node
       * - Update visibility list
       */
      for (idx = 0; idx < VB_EA_ALIGN_GHN_MAX_TX_NODES; idx++)
      {
        // Init allowed state
        BOOLEAN is_allowed = FALSE;

        if (AlignGhnDetectedDidIsValid(&(domain->alignInfo.syncDetsInfo[idx])) == TRUE)
        {
          t_VB_engineErrorCode  err;
          t_domain             *target_domain;
          INT8U                 did = domain->alignInfo.syncDetsInfo[idx].detDid;

          // Get node info
          err = AlignGhnNodeFromTxInfoGet(did, align_info->txNodesInfo, &target_domain);;

          if (err == VB_ENGINE_ERROR_NONE)
          {
            /*
             * Check if node has a valid path to reference node.
             * If "domain" is a RELAY or RELAY_CANDIDATE, ensure the detected domain
             * ("target_domain") has a lower hops value to ensure proper path to reference node.
             */
            if ((target_domain->dm.nodeAlignInfo.hops != VB_ALIGN_INVALID_HOPS) &&
                ((domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_SLAVE) ||
                 (target_domain->dm.nodeAlignInfo.hops < domain->dm.nodeAlignInfo.hops)) &&
                (VbEngineConfAlignBlackListIsMacAllowed(domain->dm.MAC, target_domain->dm.MAC) == TRUE))
            {
              // Update sync status
              is_synced = TRUE;

              // Update allowed state
              is_allowed = TRUE;

              // Get min number of hops
              min_hops = MIN(min_hops, target_domain->dm.nodeAlignInfo.hops + 1);

              // Insert node in visibility list
              err = AlignGhnVisibilityListInsert(align_info, did);

              if (err != VB_ENGINE_ERROR_NONE)
              {
                VbLogPrintExt(VB_LOG_WARNING, driver->vbDriverID, "Domain %s: Error %d inserting did %u in visibility list (numNodes = %lu)",
                    domain->dm.MACStr,
                    err,
                    did,
                    align_info->numNodes);
              }
            }
          }
        }

        domain->alignInfo.syncDetsInfo[idx].allowed = is_allowed;
      }

      // Update sync status of node
      domain->dm.nodeAlignInfo.synced = is_synced;

      // Update number of hops
      domain->dm.nodeAlignInfo.hops = min_hops;

      if (is_synced == FALSE)
      {
        // No visibility
        align_info->aligned = FALSE;
      }
    }
    else
    {
      // Ref node, force synced variable
      domain->dm.nodeAlignInfo.synced = TRUE;
      // Update number of hops
      domain->dm.nodeAlignInfo.hops = 0;
    }

    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "GHN Align", "Domain %s - DID %3u - Role %10s - Hops %3u - HasBeenCandidate %1u - Alignment %s",
        domain->dm.MACStr,
        domain->dm.devID,
        VbEngineAlignRoleStringGet(domain->dm.nodeAlignInfo.role),
        domain->dm.nodeAlignInfo.hops,
        domain->dm.nodeAlignInfo.hasBeenCandidate,
        domain->dm.nodeAlignInfo.synced?"ok":"error");

    VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "[%-10s] Domain %s - DID %3u - Role %10s - Hops %3u - HasBeenCandidate %1u - Alignment %s",
        "GHN Align",
        domain->dm.MACStr,
        domain->dm.devID,
        VbEngineAlignRoleStringGet(domain->dm.nodeAlignInfo.role),
        domain->dm.nodeAlignInfo.hops,
        domain->dm.nodeAlignInfo.hasBeenCandidate,
        domain->dm.nodeAlignInfo.synced?"ok":"error");
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignNonGhnForceSyncLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
     domain->dm.nodeAlignInfo.synced = TRUE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignGhnVisibilityLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignGhnVisibilityInfo *visibility_info = (t_vbAlignGhnVisibilityInfo *)args;

  if ((driver == NULL) || (domain == NULL) || (visibility_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U idx;

    // Reset visibility for given domain
    domain->dm.nodeAlignInfo.visible = FALSE;

    for (idx = 0; idx < visibility_info->numNodes; idx++)
    {
      // Check if domain is present in visibility list
      if ( ((domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_REF) ||
            (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY)) &&
            (domain->dm.devID == visibility_info->syncDids[idx])
         )
      {
        domain->dm.nodeAlignInfo.visible = TRUE;

        VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "Visibility", "Domain %s - is visible by crosstalk domains", domain->dm.MACStr);
        VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "[%-10s] Domain %s - is visible by crosstalk domains", "Visibility", domain->dm.MACStr);

        break;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignRefLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignChangeInfo *align_change_info = (t_vbAlignChangeInfo *)args;

  if ((domain == NULL) || (align_change_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get align info
    align_change_info->ref = domain->alignInfo;

    if (VbEngineDatamodelDomainIsNew(domain) == FALSE)
    {
      /*
       * If this domain is not new, break the loop and take this
       * alignment info as ref.
       */
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignRoleSearchLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignRoleSearch *align_role_search = (t_vbAlignRoleSearch *)args;

  if ((domain == NULL) || (align_role_search == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check alignment role
    if (domain->dm.nodeAlignInfo.role == align_role_search->targetRole)
    {
      // Node found, break the loop and exit
      align_role_search->found = TRUE;
      align_role_search->targetDomain = domain;

      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode FirstMatchingRoleGet(t_alignRole targetRole, t_domain **domain, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignRoleSearch  align_role_search;

  bzero(&align_role_search, sizeof(align_role_search));

  if ((targetRole >= VB_ALIGN_ROLE_LAST) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init search params
    align_role_search.found = FALSE;
    align_role_search.targetRole = targetRole;
    align_role_search.targetDomain = NULL;

    // Loop through all domains and check role
    ret = VbEngineDatamodelClusterXAllDomainsLoop(AlignRoleSearchLoopCb, clusterId, &align_role_search);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (align_role_search.found == FALSE)
    {
      // Target not found
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }
    else
    {
      *domain = align_role_search.targetDomain;
    }
  }

  if ((ret != VB_ENGINE_ERROR_NONE) && (ret != VB_ENGINE_ERROR_NOT_FOUND))
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d searching alignment role %s", ret, VbEngineAlignRoleStringGet(targetRole));
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AllRelaysInfoLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignGhnTxNodesInfo *tx_nodes_info = (t_vbAlignGhnTxNodesInfo *)args;

  if ((domain == NULL) || (tx_nodes_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ((domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY) ||
        (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY_CANDIDATE))
    {
      // Add node to Tx array
      if (tx_nodes_info->numTxNodes < VB_EA_ALIGN_GHN_MAX_TX_NODES)
      {
        // Add to Tx array
        tx_nodes_info->txNodes[tx_nodes_info->numTxNodes] = domain;

        // Update number of nodes
        tx_nodes_info->numTxNodes++;
      }
    }

    if (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY)
    {
      if (tx_nodes_info->numRelays < VB_EA_ALIGN_GHN_MAX_RELAYS)
      {
        // Add to relays array
        tx_nodes_info->relays[tx_nodes_info->numRelays] = domain;

        // Update number of relays
        tx_nodes_info->numRelays++;
      }

      // Update relay found
      VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "Relay found: %s; numRelays %u", domain->dm.MACStr, tx_nodes_info->numRelays);
    }
    else if (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY_CANDIDATE)
    {
      if (tx_nodes_info->numRelaysCandidate < VB_EA_ALIGN_GHN_MAX_RELAYS)
      {
        // Add to relays array
        tx_nodes_info->relaysCandidate[tx_nodes_info->numRelaysCandidate] = domain;

        // Update number of relays
        tx_nodes_info->numRelaysCandidate++;
      }

      // Update relay found
      VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "Relay found: %s; numRelaysCandidate %u", domain->dm.MACStr, tx_nodes_info->numRelaysCandidate);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode TxNodesInfoGet(t_vbAlignGhnTxNodesInfo *txNodesInfo, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_domain            *ref_domain = NULL;

  if (txNodesInfo == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Search for reference node (if any)
    ret = FirstMatchingRoleGet(VB_ALIGN_ROLE_REF, &ref_domain, clusterId);

    if (ret == VB_ENGINE_ERROR_NOT_FOUND)
    {
      ret = VB_ENGINE_ERROR_NONE;
    }
    else if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d searching ref node", ret);
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init params
    memset(txNodesInfo, 0, sizeof(*txNodesInfo));
    txNodesInfo->numRelays = 0;
    txNodesInfo->numRelaysCandidate = 0;
    txNodesInfo->ref = ref_domain;
    txNodesInfo->txNodes[0] = ref_domain;
    txNodesInfo->numTxNodes = 1;

    // Loop through all domains and get relays info
    ret = VbEngineDatamodelClusterXAllDomainsLoop(AllRelaysInfoLoopCb, clusterId, txNodesInfo);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignNextRefLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignGhnRefInfo *align_role_search = (t_vbAlignGhnRefInfo *)args;

  if ((domain == NULL) || (align_role_search == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ((align_role_search->nextCandidate == NULL) &&
        (align_role_search->found == FALSE) &&
        (domain->dm.nodeAlignInfo.role != VB_ALIGN_ROLE_REF))
    {
      align_role_search->nextCandidate = domain;
    }
    else
    {
      if (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_REF)
      {
        // Node found, break the loop and exit
        align_role_search->found = TRUE;
        align_role_search->currRef = domain;
      }
      else if (align_role_search->found == TRUE)
      {
        align_role_search->nextCandidate = domain;

        ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
      }
    }

    VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "Domain %s: found %u", domain->dm.MACStr, align_role_search->found);

    if (align_role_search->nextCandidate != NULL)
    {
      VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "   next domain %s", align_role_search->nextCandidate->dm.MACStr);
    }

    if (align_role_search->currRef != NULL)
    {
      VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "   target domain %s", align_role_search->currRef->dm.MACStr);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode NextReferenceNodeGet(t_domain **domain, INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  t_vbAlignGhnRefInfo  align_ref_info;
  t_domain            *relay_domain = NULL;
  t_domain            *next_ref_domain = NULL;

  // Init search params
  align_ref_info.found = FALSE;
  align_ref_info.currRef = NULL;
  align_ref_info.nextCandidate = NULL;

  // Loop through all domains and check role
  ret = VbEngineDatamodelClusterXAllDomainsLoop(AlignNextRefLoopCb, clusterId, &align_ref_info);

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (align_ref_info.nextCandidate == NULL)
    {
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ((align_ref_info.found == TRUE) &&
        (align_ref_info.currRef != NULL))
    {
      // If previous reference domain was found, set it to slave role
      AlignGhnNodeRoleSet(&align_ref_info.currRef->dm, VB_ALIGN_ROLE_SLAVE, clusterId);
    }

    // Try to search a relay node to promote to reference role
    ret = FirstMatchingRoleGet(VB_ALIGN_ROLE_RELAY, &relay_domain, clusterId);

    if ((ret == VB_ENGINE_ERROR_NONE) && (relay_domain != NULL))
    {
      // Relay domain found, set this domain as next reference domain
      next_ref_domain = relay_domain;
    }
    else
    {
      // Relay not found, set previous found domain as next reference domain
      next_ref_domain = align_ref_info.nextCandidate;

      ret = VB_ENGINE_ERROR_NONE;
    }

    if (VbEngineConfAlignPrioRefEnabled() == TRUE)
    {
      // Priority list enabled
      INT32U num_elements = VbEngineConfAlignPrioRefListSizeGet();
      INT32U idx;

      for (idx = 0; idx < num_elements; idx++)
      {
        t_VB_engineErrorCode  vb_err;
        t_domain             *target_domain;
        INT8U                *prio_mac;

        vb_err = VbEngineConfAlignPrioRefItemGet(idx, &prio_mac);

        if (vb_err == VB_ENGINE_ERROR_NONE)
        {
          vb_err = VbEngineDatamodelClusterXDomainByMacGet(clusterId, prio_mac, &target_domain);
        }

        if (vb_err == VB_ENGINE_ERROR_NONE)
        {
          // Domain found
          next_ref_domain = target_domain;

          VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
              "Domain %s - is present in high prio list and it will be selected as next reference",
              next_ref_domain->dm.MACStr);

          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR,
              "[%-10s] Domain %s - is present in high prio list and it will be selected as next reference",
              "Conf",
              next_ref_domain->dm.MACStr);

          break;
        }
      }
    }

    if (next_ref_domain == NULL)
    {
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Update role
    AlignGhnNodeRoleSet(&next_ref_domain->dm, VB_ALIGN_ROLE_REF, clusterId);

    if (domain != NULL)
    {
      // Return new reference domain
      *domain = next_ref_domain;
    }
  }

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Next reference selected (err %d)", ret);

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignRoleChangeLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignChangeRoleParams  *role_params = (t_vbAlignChangeRoleParams *)args;

  if ((driver == NULL) || (domain == NULL) || (role_params == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->dm.nodeAlignInfo.role == role_params->currRole)
    {
      AlignGhnNodeRoleSet(&domain->dm, role_params->newRole, driver->clusterId);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignRoleChange(t_alignRole currRole, t_alignRole newRole, INT32U clusterId)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignChangeRoleParams   role_params;

  if ((currRole >= VB_ALIGN_ROLE_LAST) || (newRole >= VB_ALIGN_ROLE_LAST))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    role_params.currRole = currRole;
    role_params.newRole = newRole;

    // Loop through all domains and set slave role
    ret = VbEngineDatamodelClusterXAllDomainsLoop(AlignRoleChangeLoopCb, clusterId, (void *)&role_params);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d configuring Slave role", ret);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterInfoBuildLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignClusterBuildInfo *cluster_info = (t_vbAlignClusterBuildInfo *)args;

  if ((domain == NULL) || (cluster_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Number of lines in cluster
    cluster_info->numLinesInCluster++;

    if (AlignNodeIsSynced(domain) == TRUE)
    {
      // Number of lines having detected a sync
      cluster_info->numLinesSyncedInCluster++;

      VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s synced", domain->dm.MACStr);
    }

    // Reset number of exclusive hits
    domain->dm.nodeAlignInfo.numExclusiveHit = 0;
  }

  return ret;
}

/*******************************************************************/

static BOOLEAN AlignGhnDomainSyncThroughRelayCandidate(t_domain *domain, t_vbAlignClusterBuildInfo *clusterInfo)
{
  BOOLEAN sync_only_through_candidate = TRUE;

  if ((domain != NULL) && (clusterInfo != NULL))
  {
    INT32U det_idx;

    if (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_REF)
    {
      sync_only_through_candidate = FALSE;
    }
    else
    {
      sync_only_through_candidate = AlignNodeIsSynced(domain);
    }

    if (sync_only_through_candidate == TRUE)
    {
      // Check that this domain does not see the Ref or any already set relay
      for (det_idx = 0; det_idx < VB_EA_ALIGN_GHN_MAX_TX_NODES; det_idx++)
      {
        // Check that detected Device is valid
        if (AlignGhnDetectedDidIsValid(&(domain->alignInfo.syncDetsInfo[det_idx])) == TRUE)
        {
          t_VB_engineErrorCode err;
          INT8U                detected_did = domain->alignInfo.syncDetsInfo[det_idx].detDid;
          t_domain            *detected_domain;

          err = AlignGhnNodeFromTxInfoGet(detected_did, &(clusterInfo->txNodesInfo), &detected_domain);

          if (err == VB_ENGINE_ERROR_NONE)
          {
            // Check domain is synced (and path to reference is OK)
            if ((AlignNodeIsSynced(detected_domain) == TRUE) &&
                ((detected_domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_REF) ||
                 (detected_domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY)) &&
                (detected_domain->dm.nodeAlignInfo.hops < domain->dm.nodeAlignInfo.hops) &&
                (VbEngineConfAlignBlackListIsMacAllowed(domain->dm.MAC, detected_domain->dm.MAC) == TRUE))
            {
              sync_only_through_candidate = FALSE;
            }
          }
          else
          {
            sync_only_through_candidate = FALSE;
          }
        }

        if (sync_only_through_candidate == FALSE)
        {
          break;
        }
      }
    }
  }
  else
  {
    sync_only_through_candidate = FALSE;
  }

  return sync_only_through_candidate;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterSyncOnlyWithCandidateBuildLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignClusterBuildInfo *cluster_info = (t_vbAlignClusterBuildInfo *)args;

  if ((domain == NULL) || (cluster_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (AlignGhnDomainSyncThroughRelayCandidate(domain, cluster_info) == TRUE)
    {
      cluster_info->numLinesSyncedExclusivelyWithRelayCandidate++;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterRelayListLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignClusterBuildInfo *cluster_info = (t_vbAlignClusterBuildInfo *)args;

  if ((domain == NULL) || (cluster_info == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if ((ret == VB_ENGINE_ERROR_NONE) &&
      (AlignGhnDomainSyncThroughRelayCandidate(domain, cluster_info) == TRUE))
  {
    INT32U det_idx;

    // Loop through detected array and check candidate relays
    for (det_idx = 0; det_idx < VB_EA_ALIGN_GHN_MAX_TX_NODES; det_idx++)
    {
      // Check that detected Device is valid
      if (AlignGhnDetectedDidIsValid(&(domain->alignInfo.syncDetsInfo[det_idx])) == TRUE)
      {
        t_VB_engineErrorCode  err;
        t_domain             *detected_domain;
        INT8U                 detected_did = domain->alignInfo.syncDetsInfo[det_idx].detDid;

        err = AlignGhnNodeFromTxInfoGet(detected_did, &(cluster_info->txNodesInfo), &detected_domain);

        if (err == VB_ENGINE_ERROR_NONE)
        {
          // Check domain is synced (and path to reference is OK)
          if ((AlignNodeIsSynced(detected_domain) == TRUE) &&
              (detected_domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY_CANDIDATE) &&
              (detected_domain->dm.nodeAlignInfo.hops < domain->dm.nodeAlignInfo.hops) &&
              (VbEngineConfAlignBlackListIsMacAllowed(domain->dm.MAC, detected_domain->dm.MAC) == TRUE))
          {
            detected_domain->dm.nodeAlignInfo.numExclusiveHit++;

            VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Node %s - Relay candidate %u hit %d",
                domain->dm.MACStr,
                detected_domain->dm.devID,
                detected_domain->dm.nodeAlignInfo.numExclusiveHit);
          }
        }
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterRoleResetLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;

  if (domain == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    memset(&(domain->dm.nodeAlignInfo), 0, sizeof(domain->dm.nodeAlignInfo));
    domain->dm.nodeAlignInfo.role = VB_ALIGN_ROLE_SLAVE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterNextCandidatesLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;
  t_vbAlignClusterBuildInfo *relays_info = (t_vbAlignClusterBuildInfo *)args;

  if( (domain == NULL) && (relays_info == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if( (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_SLAVE) &&
        (domain->dm.nodeAlignInfo.hasBeenCandidate == FALSE) &&
        (AlignNodeIsSynced(domain) == TRUE) &&
        (relays_info->txNodesInfo.numRelays + relays_info->txNodesInfo.numRelaysCandidate < VB_EA_ALIGN_GHN_MAX_RELAYS)
      )
    {
      // I'm a slave
      // I haven't been candidate yet
      // I 'm synced
      // The number of Relay + Candidate is less than  VB_EA_ALIGN_GHN_MAX_RELAYS
      // Then I can be a next Relay Candidate
      domain->dm.nodeAlignInfo.role = VB_ALIGN_ROLE_RELAY_CANDIDATE;
      domain->dm.nodeAlignInfo.hasBeenCandidate = TRUE;
      relays_info->txNodesInfo.numRelaysCandidate++;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClustersIdTagLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    driver->clusterId = *((INT32U *)args);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode DomainSyncedLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;

  if (domain == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (AlignNodeIsSynced(domain) == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_SYNCED;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClustersIdSyncedTagLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelDomainsLoop(driver, DomainSyncedLoopCb, NULL);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U clusterId = *((INT32U *)args);
    if((driver->domainsList.numDomains > 0) && (clusterId != 0))
    {
      driver->clusterId = clusterId;
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Tagged with cluster %d", driver->clusterId);
    }
  }
  else if (ret == VB_ENGINE_ERROR_NOT_SYNCED)
  {
    // Possible error, do not report above
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterRelayCandidatesGet(t_vbAlignClusterBuildInfo  *clusterBuildInfo, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (clusterBuildInfo == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init params

    // Loop through all domains in cluster Id  and get relays info
    ret = VbEngineDatamodelClusterXAllDomainsLoop(ClusterRelayListLoopCb, clusterId, clusterBuildInfo);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterCandidateToRelaySet(t_vbAlignClusterBuildInfo *clusterBuildInfo, INT8U *promotedDevId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if( (clusterBuildInfo == NULL) || (promotedDevId == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U    i;
    INT32U    max = 0;
    t_domain *target_domain = NULL;

    for(i=0; i<clusterBuildInfo->txNodesInfo.numRelaysCandidate; i++)
    {
      if ((clusterBuildInfo->txNodesInfo.relaysCandidate[i] != NULL) &&
          (clusterBuildInfo->txNodesInfo.relaysCandidate[i]->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY_CANDIDATE) &&
          (max < clusterBuildInfo->txNodesInfo.relaysCandidate[i]->dm.nodeAlignInfo.numExclusiveHit))
      {
        max = clusterBuildInfo->txNodesInfo.relaysCandidate[i]->dm.nodeAlignInfo.numExclusiveHit;
        target_domain = clusterBuildInfo->txNodesInfo.relaysCandidate[i];
      }
    }

    if ((clusterBuildInfo->txNodesInfo.numRelaysCandidate > 0) &&
        (target_domain != NULL) &&
        (target_domain->dm.nodeAlignInfo.numExclusiveHit > 0))
    {
      target_domain->dm.nodeAlignInfo.role = VB_ALIGN_ROLE_RELAY;
      *promotedDevId = target_domain->dm.devID;

      VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterBuildInfo->clusterId, "Conf",
          "Domain %s - DID %3u - Role %10s -> %10s",
          target_domain->dm.MACStr,
          target_domain->dm.devID,
          VbEngineAlignRoleStringGet(VB_ALIGN_ROLE_RELAY_CANDIDATE),
          VbEngineAlignRoleStringGet(target_domain->dm.nodeAlignInfo.role));

      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR,
          "[%-10s] Domain %s - DID %3u - Role %10s -> %10s",
          "Conf",
          target_domain->dm.MACStr,
          target_domain->dm.devID,
          VbEngineAlignRoleStringGet(VB_ALIGN_ROLE_RELAY_CANDIDATE),
          VbEngineAlignRoleStringGet(target_domain->dm.nodeAlignInfo.role));
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterRoleNextCandidateListSet(t_vbAlignClusterBuildInfo  *relays_info, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (relays_info == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    relays_info->txNodesInfo.numRelaysCandidate = 0;

    // Loop through all domains and get relays info
    ret = VbEngineDatamodelClusterXAllDomainsLoop(ClusterNextCandidatesLoopCb, clusterId, relays_info);
  }

  return ret;
}

/*******************************************************************/

static BOOLEAN AlignGhnNodePathToRefCalc(t_domain *domain, t_vbAlignGhnTxNodesInfo *txNodesInfo)
{
  BOOLEAN path_to_ref = FALSE;

  if ((domain != NULL) && (txNodesInfo != NULL))
  {
    if (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_REF)
    {
      // This node is already the reference one
      domain->dm.nodeAlignInfo.hops = 0;
      path_to_ref = TRUE;
    }
    else
    {
      INT32U idx;

      for (idx = 0; idx < VB_EA_ALIGN_GHN_MAX_TX_NODES; idx++)
      {
        if (AlignGhnDetectedDidIsValid(&(domain->alignInfo.syncDetsInfo[idx])) == TRUE)
        {
          t_VB_engineErrorCode  err;
          t_domain             *tx_node;

          // Check if detected node is a RELAY with direct visibility with reference node
          err = AlignGhnNodeFromTxInfoGet(domain->alignInfo.syncDetsInfo[idx].detDid, txNodesInfo, &tx_node);

          if (err == VB_ENGINE_ERROR_NONE)
          {
            if ((tx_node->dm.nodeAlignInfo.hops != VB_ALIGN_INVALID_HOPS) &&
                (VbEngineConfAlignBlackListIsMacAllowed(domain->dm.MAC, tx_node->dm.MAC) == TRUE))
            {
              // Path to reference found
              domain->dm.nodeAlignInfo.hops = tx_node->dm.nodeAlignInfo.hops + 1;
              path_to_ref = TRUE;
              break;
            }
          }
          else
          {
            VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d looking for did %u in Tx nodes info",
                err, domain->alignInfo.syncDetsInfo[idx].detDid);
          }
        }
      }
    }
  }

  return path_to_ref;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignGhnPathToRefCheck(BOOLEAN *pathOk, INT32U clusterId, t_vbAlignGhnTxNodesInfo *txNodesInfo)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;
  BOOLEAN                 path_ok = TRUE;

  if ((pathOk == NULL) || (txNodesInfo == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U    idx;
    INT32U    max_iters = 0;

    // Clear path to ref
    for (idx = 0; idx < txNodesInfo->numTxNodes; idx++)
    {
      if (txNodesInfo->txNodes[idx] != NULL)
      {
        txNodesInfo->txNodes[idx]->dm.nodeAlignInfo.hops = VB_ALIGN_INVALID_HOPS;
      }
    }

    // Check path to reference
    while (max_iters < VB_EA_ALIGN_GHN_MAX_TX_NODES)
    {
      path_ok = TRUE;

      for (idx = 0; idx < txNodesInfo->numTxNodes; idx++)
      {
        t_domain *node_to_check = txNodesInfo->txNodes[idx];

        if (node_to_check != NULL)
        {
          if (AlignGhnNodePathToRefCalc(node_to_check, txNodesInfo) == TRUE)
          {
            // Path to reference is OK for this relay

            VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Path", "Domain %s - DID %3u - Role %10s - Hops %3u - path to reference OK",
                node_to_check->dm.MACStr,
                node_to_check->dm.devID,
                VbEngineAlignRoleStringGet(node_to_check->dm.nodeAlignInfo.role),
                node_to_check->dm.nodeAlignInfo.hops);

            VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "[%-10s] Domain %s - DID %3u - Role %10s - Hops %3u - path to reference OK",
                "Path",
                node_to_check->dm.MACStr,
                node_to_check->dm.devID,
                VbEngineAlignRoleStringGet(node_to_check->dm.nodeAlignInfo.role),
                node_to_check->dm.nodeAlignInfo.hops);
          }
          else
          {
            VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "[%-10s] Domain %s - DID %3u - Role %10s - Hops %3u - path to reference KO",
                "Path",
                node_to_check->dm.MACStr,
                node_to_check->dm.devID,
                VbEngineAlignRoleStringGet(node_to_check->dm.nodeAlignInfo.role),
                node_to_check->dm.nodeAlignInfo.hops);
            path_ok = FALSE;
          }
        }
        else
        {
          VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Unexpected reference to NULL in Tx nodes array");
        }
      }

      if (path_ok == TRUE)
      {
        break;
      }

      max_iters++;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *pathOk = path_ok;
  }

  VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Path", "Path to reference node %s (err %d)", path_ok?"OK":"KO", ret);
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "[%-10s] Path to reference node %s (err %d)", "Path", path_ok?"OK":"KO", ret);

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode checkRelayCandidate(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;
  BOOL                       *no_relay_candidate = (BOOL *)args;

  if ((driver == NULL) || (domain == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if(domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY_CANDIDATE)
    {
      *no_relay_candidate = FALSE;
      VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "RC Check", "Domain %s - Relay candidate found", domain->dm.MACStr);
      VbLogPrintExt(VB_LOG_WARNING, driver->vbDriverID, "[%-10s] Domain %s - Relay candidate found", "RC Check", domain->dm.MACStr);
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode EngineConfReqPldGet(INT32U clusterId, INT8U **pld, INT32U *pldLen)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;

  if((pld == NULL) || (pldLen == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBCluster *cluster;

    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      *pld = cluster->confReqBuffer;
      *pldLen = cluster->confReqBufferLen;
    }
    else
    {
      *pld = NULL;
      ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode EngineConfReqPldUpdate(INT32U clusterId, INT8U *pld, INT32U pldLen)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;

  if (pld == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBCluster *cluster;

    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      if(cluster->confReqBuffer != NULL)
      {
        free(cluster->confReqBuffer);
      }

      cluster->confReqBuffer = pld;
      cluster->confReqBufferLen = pldLen;
    }
    else
    {
      ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineGetFirstFreeSeed(INT16U *seedIdx)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (seedIdx == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT16U i = 0;
    i = 0;
    while ((seedArray[i].used != FALSE) && (i < seedArraySize))
    {
      i++;
    }

    if (i < seedArraySize)
    {
      *seedIdx = i;
    }
    else
    {
      ret = VB_ENGINE_ERROR_NO_AVAILABLE_SEED;
    }
  }
  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineRemoveDeprecatedSeeds()
{
  t_VB_engineErrorCode ret;
  t_nodesMacList all_nodes_mac_list;

  ret = VbEngineDatamodelAllNodesMacGet(&all_nodes_mac_list, FALSE);

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    BOOLEAN free_seed = FALSE;
    INT16U i = 0;

    // First, check if all DMs that have an assigned seed are alive, if not, free the assigned seed
    while (i < seedArraySize)
    {
      INT32U idx;
      INT8U  *dm_mac_ptr;
      for (idx = 0; idx < all_nodes_mac_list.dmsMacs.numNodes; idx++)
      {
        // Get next DM's MAC
        dm_mac_ptr = all_nodes_mac_list.dmsMacs.ptr + (ETH_ALEN * idx);

        // If DM is found, then skip to next seed
        if ((memcmp(seedArray[i].macAddr, dm_mac_ptr, ETH_ALEN) == 0))
        {
          break;
        }
      }

      // If DM has not been found, then free its assigned seed
      if (idx == all_nodes_mac_list.dmsMacs.numNodes)
      {
        VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Remove seed (%d+min_index) assigned to %X:%X:%X:%X:%X:%X", i,
            seedArray[i].macAddr[0], seedArray[i].macAddr[1], seedArray[i].macAddr[2], seedArray[i].macAddr[3], seedArray[i].macAddr[4], seedArray[i].macAddr[5]);

        seedArray[i].used = FALSE;
        memset(seedArray[i].macAddr, 0, ETH_ALEN);
        free_seed = TRUE;
      }
      i++;
    }

    if (free_seed == FALSE)
    {
      // Exit if NO seed can be assigned
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "No more seeds available ! Please review allowed range in .ini file. ");

      // Give time to print message
      sleep(1);

      // It is not safe to run the system with duplicated seeds, so if no more seed can be assigned,
      // stop the system to review the configuration
      exit(EXIT_FAILURE);
    }
  }

  if ((all_nodes_mac_list.dmsMacs.numNodes > 0) || (all_nodes_mac_list.epsMacs.numNodes > 0))
  {
    // Always release memory
    VbEngineDatamodelMacListRelease(&all_nodes_mac_list);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineCheckSeed(INT16U *seedIdx, INT8U* macAddr, INT16U minIndex)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U seed_index_tmp;

  if ((seedIdx == NULL) || (macAddr == NULL) || (*seedIdx < minIndex))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // We use a temporal seed index starting from 0 to improve operations with the seed's array.
    // Seed index will be later re-adjust to custom range.
    seed_index_tmp =  *seedIdx - minIndex;

    // If seed index calculated for this MAC is free, use it
    if (seedArray[seed_index_tmp].used == FALSE)
    {
      seedArray[seed_index_tmp].used = TRUE;
      memcpy(seedArray[seed_index_tmp].macAddr, macAddr, ETH_ALEN);
      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "New seed index reserved.");
    }
    // If seed index calculated for this MAC is already used, check if it is used by this MAC
    else if ((seedArray[seed_index_tmp].used == TRUE) && (memcmp(seedArray[seed_index_tmp].macAddr, macAddr, ETH_ALEN) == 0))
    {
      // Return same seedIdx if it was previously used by this MAC (before a reset)
      VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "Seed index previously reserved by this MAC.");
    }
    else
    {
      // This part of code is used to manage conflicts between DMs if the calculated seed index is the same for
      // more than one MAC address. Seed index must be unique, so if calculated seed index is previously used
      // by another DM, we will try to find another one, starting from the first index. If all seeds have been
      // allocated and no more are available, the program will exit.
      INT16U i = 0;

      // Check if DM is using another seed index because of conflict
      while ((i < seedArraySize) && (memcmp(seedArray[i].macAddr, macAddr, ETH_ALEN) != 0))
      {
        i++;
      }

      if (i < seedArraySize)
      {
        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Seed index %d (instead of %d) previously reserved by this MAC.",
                                                              i+minIndex, seed_index_tmp+minIndex);
        seed_index_tmp = i;
      }
      else
      {
        // If DM is not using any previous seed index, return the first free one.
        ret = VbEngineGetFirstFreeSeed(&i);

        if (ret == VB_ENGINE_ERROR_NO_AVAILABLE_SEED)
        {
          // If there is no more free seeds, try to removed obsolete ones
          ret = VbEngineRemoveDeprecatedSeeds();
          if (ret == VB_ENGINE_ERROR_NONE)
          {
            ret = VbEngineGetFirstFreeSeed(&i);
          }
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "New seed index %d (instead of %d) reserved for this MAC.",
                                                                 i+minIndex, seed_index_tmp+minIndex);
          seed_index_tmp = i;
          seedArray[seed_index_tmp].used = TRUE;
          memcpy(seedArray[seed_index_tmp].macAddr, macAddr, ETH_ALEN);
        }
      }
    }

    // Return seed index
    *seedIdx =  seed_index_tmp + minIndex;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineInitSeedArray()
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U i;

  // Reserve memory for seed's array
  seedArray = (t_vbAlignGhnSeedInfo *)malloc(sizeof(t_vbAlignGhnSeedInfo) * seedArraySize);

  // Initialize seed's array
  if (seedArray != NULL)
  {
    for (i=0; i<seedArraySize; i++)
    {
      seedArray[i].used = FALSE;
      memset(seedArray[i].macAddr, 0, 6);
    }
    initSeedArray = TRUE;
  }
  else
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error allocating memory for seed's array");
    ret = VB_ENGINE_ERROR_MALLOC;
  }

  return ret;
}



/*******************************************************************/

static t_VB_engineErrorCode VbEngineGenerateSeedIndex(INT8U *macAddr, INT16U minIndex, INT16U maxIndex, INT16U *seedIndex)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               hash = 0;

  if ((macAddr == NULL) || (seedIndex == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  // Initialize seed's array the first time
  if ((ret == VB_ENGINE_ERROR_NONE) && (initSeedArray == FALSE))
  {
    // Store seed's array size
    seedArraySize = maxIndex - minIndex + 1;

    ret = VbEngineInitSeedArray();
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Calculate a hash value depending of the MAC address
    hash = macAddr[3] + 2*macAddr[4] + 4*macAddr[5];

    // Modulo the hash to get seed index between Min and Max custom values
    *seedIndex = (hash%(maxIndex-minIndex+1)) + minIndex;

    // Check if seed index can be used, if not, a new free one will be automatically returned by this function
    ret = VbEngineCheckSeed(seedIndex,macAddr,minIndex);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineGetFirstFreeDid(INT16U *did)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (did == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT16U i = 1;
    while ((didArray[i].seed != 0) && (i < VB_ALIGN_MAX_DEVICE_ID))
    {
      i++;
    }

    if (i < VB_ALIGN_MAX_DEVICE_ID)
    {
      *did = i;
    }
    else
    {
      ret = VB_ENGINE_ERROR_NO_AVAILABLE_DID;
    }
  }
  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineRemoveDeprecatedDids()
{
  t_VB_engineErrorCode ret;
  t_nodesMacList all_nodes_mac_list;

  ret = VbEngineDatamodelAllNodesMacGet(&all_nodes_mac_list, FALSE);

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT16U i = 1;
    BOOLEAN free_did = FALSE;

    // First, check if all DMs that have an assigned did are alive, if not, free the assigned did
    while (i < VB_ALIGN_MAX_DEVICE_ID)
    {
      INT32U idx;
      INT8U  *dm_mac_ptr;
      for (idx = 0; idx < all_nodes_mac_list.dmsMacs.numNodes; idx++)
      {
        // Get next DM's MAC
        dm_mac_ptr = all_nodes_mac_list.dmsMacs.ptr + (ETH_ALEN * idx);

        // If DM is found, then skip to next seed
        if (memcmp(didArray[i].macAddr, dm_mac_ptr, ETH_ALEN) == 0)
        {
          break;
        }
      }

      // If DM has not been found, then free its assigned did
      if (idx == VB_ALIGN_MAX_DEVICE_ID)
      {
        VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Remove did %d assigned to %X:%X:%X:%X:%X:%X", i,
            didArray[i].macAddr[0], didArray[i].macAddr[1], didArray[i].macAddr[2], didArray[i].macAddr[3], didArray[i].macAddr[4], didArray[i].macAddr[5]);

        didArray[i].seed = 0;
        memset(didArray[i].macAddr, 0, ETH_ALEN);
        free_did = TRUE;
      }
      i++;
    }

    if (free_did == FALSE)
    {
      // Exit if NO did can be assigned
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "No more dids available ! Maximum of %d dids has been reached !",VB_ALIGN_MAX_DEVICE_ID);

      // Give time to print message
      sleep(1);

      // It is not safe to run the system with duplicated dids, so if no more dids can be assigned,
      // stop the system to review the configuration
      exit(EXIT_FAILURE);
    }
  }

  if ((all_nodes_mac_list.dmsMacs.numNodes > 0) || (all_nodes_mac_list.epsMacs.numNodes > 0))
  {
    // Always release memory
    VbEngineDatamodelMacListRelease(&all_nodes_mac_list);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineGenerateDid(INT8U *macAddr,INT16U seedIndex, INT16U *did)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U               tmp_did = VB_ALIGN_MAX_DEVICE_ID;

  if ((macAddr == NULL) || (did == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    tmp_did = seedIndex % VB_ALIGN_MAX_DEVICE_ID;

    // 0 is NOT a valid device number
    if (tmp_did == 0)
    {
      tmp_did = VB_ALIGN_MAX_DEVICE_ID;
    }

    // If calculated device number is free, use it
    if (didArray[tmp_did].seed == 0)
    {
      didArray[tmp_did].seed = seedIndex;
      memcpy(didArray[tmp_did].macAddr, macAddr, ETH_ALEN);
    }
    // If did index calculated for this MAC is already used, check if it is used by this MAC
    else if ((didArray[tmp_did].seed == seedIndex) && (memcmp(didArray[tmp_did].macAddr, macAddr, ETH_ALEN) == 0))
    {
      // Return same seedIdx if it was previously used by this MAC (before a reset)
      VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "Did previously reserved by this MAC.");
    }
    else
    {
      // This part of code is used to manage conflicts between device numbers if the calculated one is equal to
      // another one. Device numbers must be unique, so if they are repeated, we must try to find another one,
      // starting from number 1. If all 250 device numbers have been allocated, the program will exit.
      INT16U i = 1;

      // Check if DM is using another did because of conflict
      while ((i < VB_ALIGN_MAX_DEVICE_ID) && (memcmp(didArray[i].macAddr, macAddr, ETH_ALEN) != 0))
      {
        i++;
      }

      if (i < VB_ALIGN_MAX_DEVICE_ID)
      {
        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Did %d (instead of %d) previously reserved by this MAC.",
            i, tmp_did);
        tmp_did = i;
      }
      else
      {
        // If DM is not using any previous DID, return the first free one.
        ret = VbEngineGetFirstFreeDid(&i);

        if (ret == VB_ENGINE_ERROR_NO_AVAILABLE_DID)
        {
          // If there is no more free dids, try to removed obsolete ones
          ret = VbEngineRemoveDeprecatedDids();
          if (ret == VB_ENGINE_ERROR_NONE)
          {
            ret = VbEngineGetFirstFreeDid(&i);
          }
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "New did %d (instead of %d) reserved for this MAC.",
              i, tmp_did);
          tmp_did = i;
          didArray[tmp_did].seed = seedIndex;
          memcpy(didArray[tmp_did].macAddr, macAddr, ETH_ALEN);
        }
      }
    }

    //Return device number
    *did = tmp_did;
  }

  return ret;
}

/*******************************************************************/

static BOOLEAN VbEngineAutomaticSeedIsMacAllowed (INT8U  *dm_mac_ptr)
{
  BOOLEAN allowed = TRUE;

  if (dm_mac_ptr == NULL)
  {
    allowed = FALSE;
  }
  else if (VbEngineConfSeedExcludedMacListGet() != NULL)
  {
    INT16U idx;
    INT8U  *excluded_mac_ptr;

    for (idx = 0; idx < VbEngineConfSeedExcludedMacListSizeGet(); idx++)
    {
      // Set pointer to excluded MAC list
      excluded_mac_ptr = VbEngineConfSeedExcludedMacListGet() + (ETH_ALEN * idx);

      // Check if DM MAC matches with an excluded MAC
      if ((excluded_mac_ptr != NULL) && (memcmp(excluded_mac_ptr, dm_mac_ptr, ETH_ALEN) == 0))
      {
        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "%X:%X:%X:%X:%X:%X excluded from automatic seed",
            dm_mac_ptr[0],dm_mac_ptr[1], dm_mac_ptr[2],dm_mac_ptr[3],dm_mac_ptr[4],dm_mac_ptr[5]);
        allowed = FALSE;
        break;
      }
    }
  }

  return allowed;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAutomaticSeedsAndDids (INT32U clusterId, t_vbAlignModeSeedParam  **seedsArray, INT16U* numSeedsAndDids)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_nodesMacList       all_nodes_mac_list;

  // Init node list structure
  all_nodes_mac_list.dmsMacs.numNodes = 0;
  all_nodes_mac_list.epsMacs.numNodes = 0;

  if ((seedsArray == NULL) || (numSeedsAndDids == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    *numSeedsAndDids = 0;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Collect all MACs
    ret = VbEngineDatamodelClusterXAllNodesMacGet(&all_nodes_mac_list, FALSE,clusterId);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U                   idx;

    // Allocate memory to store seeds and dids for all DMs
    *seedsArray = (t_vbAlignModeSeedParam *)malloc(all_nodes_mac_list.dmsMacs.numNodes * sizeof(t_vbAlignModeSeedParam));

    if (*seedsArray == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
    else
    {
      // Assign a seed for each DM
      for (idx = 0; idx < all_nodes_mac_list.dmsMacs.numNodes; idx++)
      {
        // Calculate seed
        INT8U  *dm_mac_ptr;
        INT16U seedIdx;
        INT16U did;

        // Get next DM's MAC
        dm_mac_ptr = all_nodes_mac_list.dmsMacs.ptr + (ETH_ALEN * idx);

        // Check if that current MAC is not excluded from automatic seed
        if (VbEngineAutomaticSeedIsMacAllowed(dm_mac_ptr) == TRUE)
        {
          ret = VbEngineGenerateSeedIndex(dm_mac_ptr, VbEngineConfSeedMinIndexGet(),VbEngineConfSeedMaxIndexGet(), &seedIdx);

          if (ret == VB_ENGINE_ERROR_NONE)
          {
            ret = VbEngineGenerateDid(dm_mac_ptr, seedIdx, &did);
          }

          if (ret == VB_ENGINE_ERROR_NONE)
          {
            MACAddrClone((*seedsArray)[*numSeedsAndDids].dmMAC, dm_mac_ptr);
            (*seedsArray)[*numSeedsAndDids].dmSeed = seedIdx;
            (*seedsArray)[*numSeedsAndDids].did    = did;

            VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Seed for %X:%X:%X:%X:%X:%X will be %u and DID %u",
                          (*seedsArray)[*numSeedsAndDids].dmMAC[0],
                          (*seedsArray)[*numSeedsAndDids].dmMAC[1],
                          (*seedsArray)[*numSeedsAndDids].dmMAC[2],
                          (*seedsArray)[*numSeedsAndDids].dmMAC[3],
                          (*seedsArray)[*numSeedsAndDids].dmMAC[4],
                          (*seedsArray)[*numSeedsAndDids].dmMAC[5],
                          seedIdx, did);
            VbEngineSeedIndexByMacSet((*seedsArray)[*numSeedsAndDids].dmMAC, seedIdx,did);

            // Increment number of seeds and dids
            (*numSeedsAndDids)++;
          }
          else
          {
            VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d generating seeds ", ret);
          }
        }
      }
    }
  }

  if ((all_nodes_mac_list.dmsMacs.numNodes > 0) || (all_nodes_mac_list.epsMacs.numNodes > 0))
  {
    // Always release memory
    VbEngineDatamodelMacListRelease(&all_nodes_mac_list);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignHasBeenCandidateResetLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode        ret = VB_ENGINE_ERROR_NONE;

  if (domain == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    domain->dm.nodeAlignInfo.hasBeenCandidate = FALSE;

    VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s - hasBeenCandidate = FALSE", domain->dm.MACStr);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignTxNodesCook(t_vbAlignGhnTxNodesInfo *txNodesInfo, t_domain **txNodesCooked)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_domain            *tx_nodes_precooked[VB_EA_ALIGN_GHN_MAX_TX_NODES];

  if ((txNodesInfo == NULL) || (txNodesCooked == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U tx_nodes_idx;

    // Ensure Tx nodes don't exceed maximum number of hops
    for (tx_nodes_idx = 0; tx_nodes_idx < txNodesInfo->numTxNodes; tx_nodes_idx++)
    {
      if ((txNodesInfo->txNodes[tx_nodes_idx] != NULL) &&
          (txNodesInfo->txNodes[tx_nodes_idx]->dm.nodeAlignInfo.hops > VB_ALIGN_MAX_HOPS))
      {
        VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR,
            "Node %s - Exceeded number of hops (%u) (role %u)",
            txNodesInfo->txNodes[tx_nodes_idx]->dm.MACStr,
            txNodesInfo->txNodes[tx_nodes_idx]->dm.nodeAlignInfo.hops,
            txNodesInfo->txNodes[tx_nodes_idx]->dm.nodeAlignInfo.role);

        txNodesInfo->txNodes[tx_nodes_idx]->dm.nodeAlignInfo.hops = VB_ALIGN_MAX_HOPS;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Reset cooked list
    memset(&(txNodesCooked[0]), 0, sizeof(txNodesCooked[0]) * VB_EA_ALIGN_GHN_MAX_TX_NODES);

    // Copy Tx nodes to a temp list to be processed
    memcpy(tx_nodes_precooked, txNodesInfo->txNodes, sizeof(tx_nodes_precooked));
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U   tx_nodes_idx;
    INT32U   tx_nodes_cooked_idx = 0;
    INT32U   curr_hop = 0;

    while ((tx_nodes_cooked_idx < txNodesInfo->numTxNodes) && (curr_hop <= VB_ALIGN_MAX_HOPS))
    {
      t_domain *target_domain = NULL;

      for (tx_nodes_idx = 0; tx_nodes_idx < txNodesInfo->numTxNodes; tx_nodes_idx++)
      {
        if ((tx_nodes_precooked[tx_nodes_idx] != NULL) &&
            (tx_nodes_precooked[tx_nodes_idx]->dm.nodeAlignInfo.hops == curr_hop))
        {
          // Node found with expected number of hops
          target_domain = tx_nodes_precooked[tx_nodes_idx];

          // Remove entry from pre-cooked list
          tx_nodes_precooked[tx_nodes_idx] = NULL;

          break;
        }
      }

      if (target_domain != NULL)
      {
        txNodesCooked[tx_nodes_cooked_idx] = target_domain;
        tx_nodes_cooked_idx++;
      }
      else
      {
        curr_hop++;
      }
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

t_VB_engineErrorCode VbEngineAlignCycQueryRequest(INT32U clusterId)
{
   t_VB_engineErrorCode ret;
   t_vbEACycQueryReq    cycquery_req;
   struct timespec      cycquery_ts;

  // Get a future TS to schedule CycQuery operation
  ret = VbEngineClockFutureTSGet(&cycquery_ts, clusterId, NULL, NULL);

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %ld calculating a future TS", ret);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Fill msg data
    cycquery_req.tvSec = _htonl((INT32U)cycquery_ts.tv_sec);
    cycquery_req.tvNsec = _htonl((INT32U)cycquery_ts.tv_nsec);

    // Send message to all drivers in cluster
    ret = VbEngineDatamodelClusterXDriversLoop(CycQueryReqLoopCb, clusterId, &cycquery_req);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Alignment done
    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_CHECK_START, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Check",
          "*** Alignment process CHECK start ***");
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineAlignCycQueryRspErrGet(INT8U *payload, t_vbEACycQueryRspErrorCode *errCod, INT32U clusterId)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;
  t_vbEACycQueryRspCommon   *common_fields;
  struct timespec           cycquery_ts;
  struct timespec           cycquery_tx_ts;

  if ((payload == NULL) || (errCod == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get common fields
    common_fields = (t_vbEACycQueryRspCommon *)payload;

    *errCod = _ntohs(common_fields->errorCode);
    cycquery_ts.tv_sec = _ntohl(common_fields->tvSec);
    cycquery_ts.tv_nsec = _ntohl(common_fields->tvNsec);

    ret =  VbEngineClusterTimeApplyDrvGet(clusterId, &cycquery_tx_ts);
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    /*
     * Check if this response is the expected one:
     * EACycQuery.req was sent with a specific timestamp.
     * We shall check at this point that EACycQuery.rsp conveys
     * the same timestamp used in EACycQuery.req.
     */
    if ((cycquery_ts.tv_sec != cycquery_tx_ts.tv_sec) ||
        (cycquery_ts.tv_nsec != cycquery_tx_ts.tv_nsec))
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Rx %ld vs %ld %ld vs %ld",
          cycquery_tx_ts.tv_sec, cycquery_ts.tv_sec,
          cycquery_tx_ts.tv_nsec, cycquery_ts.tv_nsec);

      ret = VB_ENGINE_ERROR_PARAMS;
    }
  }
  else
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Rx Rsp %lu", ret);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineAlignCycQueryRspProcess(INT8U* payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;
  t_vbEACycQueryRspCommon   *common_fields;
  INT32U                     num_nodes;

  if ((payload == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_vbEACycQueryRspErrorCode err_cod;

    // Get common fields
    common_fields = (t_vbEACycQueryRspCommon *)payload;
    err_cod = _ntohs(common_fields->errorCode);
    num_nodes = _ntohs(common_fields->numNodes);

    if (err_cod != VB_EA_CYCQUERY_RSP_ERR_NONE)
    {
      ret = VB_ENGINE_ERROR_ALIGN;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_vbEACycQueryRspNode   *node_fields;
    INT32U                   node_idx;
    INT32U                   idx_to_read;
    INT32U                   i;
    t_alignInfo              align_info;
    CHAR                     mac_str[MAC_STR_LEN];

    for (node_idx = 0; node_idx < num_nodes; node_idx++)
    {
      BOOLEAN detected_did = FALSE;

      idx_to_read = (node_idx * VB_EA_CYCQUERY_RSP_NODE_SIZE) + VB_EA_CYCQUERY_RSP_COMMON_SIZE;
      node_fields = (t_vbEACycQueryRspNode *)&(payload[idx_to_read]);

      align_info.macClock = _ntohl(node_fields->macClock);
      align_info.seqNum   = _ntohs(node_fields->seqNum);

      MACAddrMem2str(mac_str, node_fields->MAC);

      for (i = 0; i < VB_EA_ALIGN_GHN_MAX_TX_NODES; i++)
      {
        align_info.syncDetsInfo[i].allowed = FALSE;
        align_info.syncDetsInfo[i].detDid = node_fields->syncDetsInfo[i].syncDetDid;
        align_info.syncDetsInfo[i].hitCount = _ntohl(node_fields->syncDetsInfo[i].hitCount);
        align_info.syncDetsInfo[i].reliability = _ntohl(node_fields->syncDetsInfo[i].reliability);
        align_info.syncDetsInfo[i].adcOutRms = _ntohl(node_fields->syncDetsInfo[i].adcOutRms);

        if(align_info.syncDetsInfo[i].reliability < 0)
        {
          align_info.syncDetsInfo[i].reliability = 0;
        }

        if(align_info.syncDetsInfo[i].hitCount > 0)
        {
          align_info.syncDetsInfo[i].reliability = (align_info.syncDetsInfo[i].reliability*100)/(align_info.syncDetsInfo[i].hitCount + align_info.syncDetsInfo[i].reliability);
        }

        if (align_info.syncDetsInfo[i].detDid != 0)
        {
          detected_did = TRUE;

          VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "Detection",
              "Domain %s - Idx %d [Did:%3d Hits:%5lu Rel:%5ld %% Rx:%7.2f(%04lu)]",
              mac_str, i,
              align_info.syncDetsInfo[i].detDid,
              align_info.syncDetsInfo[i].hitCount,
              align_info.syncDetsInfo[i].reliability,
              VbEngineAlignAdcOutToFPGet(align_info.syncDetsInfo[i].adcOutRms),
              align_info.syncDetsInfo[i].adcOutRms);

          VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID,
              "[%-10s] Domain %s - Idx %d [Did:%3d Hits:%5lu Rel:%5ld %% Rx:%7.2f(%04lu)]",
              "Detection",
              mac_str, i,
              align_info.syncDetsInfo[i].detDid,
              align_info.syncDetsInfo[i].hitCount,
              align_info.syncDetsInfo[i].reliability,
              VbEngineAlignAdcOutToFPGet(align_info.syncDetsInfo[i].adcOutRms),
              align_info.syncDetsInfo[i].adcOutRms);
        }
      }

      if (detected_did == FALSE)
      {
        VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "Detection",
            "Domain %s - No detected devices", mac_str);

        VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID,
            "[%-10s] Domain %s - No detected devices", "Detection", mac_str);
      }

      // Update alignment info for given domain
      VbEngineDatamodelAlignInfoSet(driver, node_fields->MAC, &align_info);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineAlignSyncLostTrgProcess(INT8U* payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;

  if ((payload == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_vbEAAlignSyncLost *sync_lost_pld;
    CHAR                 mac_str[MAC_STR_LEN];

    sync_lost_pld = (t_vbEAAlignSyncLost *)payload;

    MACAddrMem2str(mac_str, sync_lost_pld->reporterMac);

    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, driver->vbDriverID, driver->clusterId, "Event",
        "Sync Lost received from %s : devId lost %u", mac_str, sync_lost_pld->syncDid);

    VbLogPrintExt(VB_LOG_WARNING, driver->vbDriverID, "Sync Lost received from %s : devId lost %u", mac_str, sync_lost_pld->syncDid);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineAlignCycChangeRspErrGet(INT8U *payload, t_vbEACycChangeRspErrorCode *errCod)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;
  t_vbEACycChangeRsp        *cycchange_rsp;

  if ((payload == NULL) || (errCod == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    cycchange_rsp = (t_vbEACycChangeRsp *)payload;
    *errCod = cycchange_rsp->errCode;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineAlignModeRspErrGet(INT8U *payload, t_vbEAAlignModeRspErrorCode *errCod, t_VBDriver *driver)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;
  t_vbEAAlignModeRsp        *alignmode_rsp;
  INT8U                      align_id_rsp;
  INT8U                      align_id_cluster;

  if ((payload == NULL) || (errCod == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    alignmode_rsp = (t_vbEAAlignModeRsp *)payload;
    *errCod = alignmode_rsp->errCode;
    align_id_rsp = alignmode_rsp->alignId;

    ret = VbEngineAlignmentIdGet(driver->clusterId, NULL, &align_id_cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      if(align_id_rsp != align_id_cluster)
      {
        *errCod = VB_EA_ALIGNMODE_RSP_ERR_ID;
      }
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineAlignClusterStopRspProcess(INT8U* payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEAAlignClusterStopRsp *cluster_stop_rsp_payload;

  cluster_stop_rsp_payload = (t_vbEAAlignClusterStopRsp *)payload;

  if ((cluster_stop_rsp_payload == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (cluster_stop_rsp_payload->errCode != VB_EA_ALIGNCLUSTERSTOP_RSP_ERR_NONE)
    {
      // Some error detected, Stop cluster rejected by driver
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error %d reported by driver while processing stop cluster %u",
          cluster_stop_rsp_payload->errCode, driver->clusterId);

      VbCounterIncrease(VB_ENGINE_COUNTER_EV_RX_CLUSTER_STOP_RSP_KO);
      ret = VB_ENGINE_ERROR_ALIGN;
    }
    else
    {
      // Cluster Stopped accepted Ok
      ret = VB_ENGINE_ERROR_NONE;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEngineConfSend(t_VBDriver *driver)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;
  INT8U                   *pld;
  INT32U                   pld_len;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = EngineConfReqPldGet(driver->clusterId, &pld, &pld_len);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Send AlignMode.req message
    ret = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_ENGINE_CONF_REQ, pld, pld_len, driver);

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "EngineConf.req sent");
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error sending EngineConf (%d)", ret);
    }
  }

  return ret;
}

/*******************************************************************/

BOOLEAN VbEngineAlignCheck(INT32U clusterId)
{
  t_VB_engineErrorCode       err;
  BOOLEAN                    ret = FALSE;
  t_vbAlignGhnVisibilityInfo ghn_visibility_info;
  BOOLEAN                    path_to_ref_ok;
  BOOLEAN                    no_relay_candidate;

  // Init align check info
  vbAlignChangeInfo.aligned = TRUE;

  // Init visibility info
  memset(&ghn_visibility_info, 0, sizeof(ghn_visibility_info));
  ghn_visibility_info.aligned = TRUE;

  // Init path to reference
  path_to_ref_ok = TRUE;

  no_relay_candidate = TRUE;

  // Obtain reference
  err = VbEngineAlignRefCalc(clusterId);

  if (err != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d RefCalc alignment", err);
    ret = FALSE;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    // Loop through all domains and check alignment
    err = VbEngineDatamodelClusterXAllDomainsLoop(AlignCheckLoopCb, clusterId, &vbAlignChangeInfo);
    if (err != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d checking alignment", err);
      ret = FALSE;
    }
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    t_VBAlignmentMode alignment_mode;

    VbEngineConfAlignmentModeGet(&alignment_mode);

    if (alignment_mode == VB_ALIGN_MODE_GHN)
    {
      t_vbAlignGhnTxNodesInfo tx_nodes_info;

      // Get tx nodes info
      err = TxNodesInfoGet(&tx_nodes_info, clusterId);

      if (err != VB_ENGINE_ERROR_NONE)
      {
        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d getting Tx nodes info", err);
      }

      if (err == VB_ENGINE_ERROR_NONE)
      {
        // Check path to reference node
        err = AlignGhnPathToRefCheck(&path_to_ref_ok, clusterId, &tx_nodes_info);

        if (err != VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d checking path to reference", err);
          ret = FALSE;
        }
      }

      if (err == VB_ENGINE_ERROR_NONE)
      {
        ghn_visibility_info.txNodesInfo = &tx_nodes_info;

        // Loop through all domains and check alignment
        err = VbEngineDatamodelClusterXAllDomainsLoop(AlignGhnCheckLoopCb, clusterId, &ghn_visibility_info);

        if (err != VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d checking alignment", err);
          ret = FALSE;
        }
      }

      if (err == VB_ENGINE_ERROR_NONE)
      {
        // Update visibility for each node being Ref or Relay
        err = VbEngineDatamodelClusterXAllDomainsLoop(AlignGhnVisibilityLoopCb, clusterId, &ghn_visibility_info);

        if (err != VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d checking visibility", err);
          ret = FALSE;
        }
      }

      if (err == VB_ENGINE_ERROR_NONE)
      {
        // If there is still relay candidate -> report aligned failed
        // Candidate shall be removed or promoted to relay
        err = VbEngineDatamodelClusterXAllDomainsLoop(checkRelayCandidate, clusterId, &no_relay_candidate);
      }
    }
    else
    {
      // Artificially force synced variable to allow cluster creation
      err = VbEngineDatamodelClusterXAllDomainsLoop(AlignNonGhnForceSyncLoopCb, clusterId, &ghn_visibility_info);
    }
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    ret = (vbAlignChangeInfo.aligned && ghn_visibility_info.aligned && path_to_ref_ok && no_relay_candidate);

    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_CHECK_END, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Check",
        "*** Alignment process CHECK finish (align status %s) ***", ret?"OK":"KO");
  }

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Align status: %s - err %d", ret?"OK":"KO", err);

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignRefCalc(INT32U clusterId)
{
  t_VB_engineErrorCode       ret;

  // Init align check info
  vbAlignChangeInfo.ref.macClock = 0;
  vbAlignChangeInfo.ref.seqNum = 0;

  // Loop through all domains and check alignment
  ret = VbEngineDatamodelClusterXAllDomainsLoop(AlignRefLoopCb, clusterId, &vbAlignChangeInfo);

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d retrieving reference MAC clock and sequence number", ret);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Alignment reference : MAC Clock %u; SeqNum %u, cluster %lu",
        vbAlignChangeInfo.ref.macClock, vbAlignChangeInfo.ref.seqNum, clusterId);
  }

  return ret;
}

/*******************************************************************/

t_alignInfo VbEngineAlignRefGet(void)
{
  return vbAlignChangeInfo.ref;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignCycChangeSend(t_VBDriver *driver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init fields
    vbAlignChangeInfo.payload = NULL;
    vbAlignChangeInfo.idx = 0;
    vbAlignChangeInfo.length = 0;

    /*
     * Loop through domains to fill message payload.
     * Buffer will be allocated inside the loop to ensure domains mutex is grabbed.
     */
    ret = VbEngineDatamodelDomainsLoop(driver, CycChangeReqLoopCb, &vbAlignChangeInfo);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (vbAlignChangeInfo.payload == NULL)
    {
      // No domains present in this driver, do not send CycChange.req message
    }
    else
    {
      // Send CycChange.req message
      ret = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_CYCCHANGE_REQ, vbAlignChangeInfo.payload, vbAlignChangeInfo.length, driver);
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (vbAlignChangeInfo.payload != NULL)
    {
      free(vbAlignChangeInfo.payload);
      vbAlignChangeInfo.payload = NULL;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignDoneNotify(INT32U clusterId)
{
  t_VB_engineErrorCode  ret;
  t_VBCluster          *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Update last alignment done TS
    clock_gettime(CLOCK_MONOTONIC, &cluster->clusterInfo.lastAlignCheck);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Clear "hasBeenCandidate" flag to allow nodes become RELAY_CANDIDATE if needed
    ret = VbEngineAlignmentHasBeenCandidateReset(clusterId);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Alignment done
    VbEngineAlignMetricsNodeInfoReport(clusterId);

    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_DONE, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Done",
          "***** Alignment process DONE *****");
  }

  return ret;
}

/*******************************************************************/

void VbEngineAlignPeriodicCheckForce(INT32U clusterId)
{
  vbAlignPeriodicCheckForce.force = TRUE;
  vbAlignPeriodicCheckForce.clusterId = clusterId;
}

/*******************************************************************/

BOOLEAN VbEngineAlignPeriodicCheckNeeded(struct timespec *lastCheckTS, INT32U clusterId)
{
  BOOLEAN          periodic_check = FALSE;
  struct timespec current_ts;
  INT64S           elapsed_ms;

  // Check if a new alignment check is needed

  // Get current time
  clock_gettime(CLOCK_MONOTONIC, &current_ts);

  // Get elapsed time since last alignment check
  elapsed_ms = VbUtilElapsetimeTimespecMs(*lastCheckTS, current_ts);

  if ((elapsed_ms > VB_ALIGN_PERIODIC_CHECK_TO) ||
      ((vbAlignPeriodicCheckForce.force == TRUE) && (vbAlignPeriodicCheckForce.clusterId == clusterId)))
  {
    *lastCheckTS = current_ts;
    periodic_check = TRUE;
    vbAlignPeriodicCheckForce.force = FALSE;
    vbAlignPeriodicCheckForce.clusterId = 0;
  }

  return periodic_check;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignLastCheckTSGet(struct timespec *lastCheckTS,INT32U  clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (lastCheckTS == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBCluster              *cluster;

    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      *lastCheckTS = cluster->applyOwnTs;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignGhnNextTry(t_vbAlignNextStep *next, t_vbAlignClusterBuildInfo  *clusterBuildInfo, INT32U clusterId)
{
  t_VB_engineErrorCode     ret;
  t_domain                *ref_domain = NULL;
  BOOLEAN                  select_next_ref = FALSE;
  BOOLEAN                  ref_lonely_cluster = FALSE;

  // Search for reference node (if any)
  ret = FirstMatchingRoleGet(VB_ALIGN_ROLE_REF, &ref_domain, clusterId);

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (ref_domain->dm.nodeAlignInfo.visible == FALSE)
    {
      // Reference node is not seen by any other node, create a cluster for this line and keep aligning the others
      select_next_ref = TRUE;

      VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
          "Reference node is not seen by any other node - Align Done for node %s (did %d)",
          ref_domain->dm.MACStr,
          ref_domain->dm.devID);

      VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Align Done for Ref %d", ref_domain->dm.devID);

      *next |= 1 << VB_ENGINE_ALIGNMENT_NEXT_DONE_EVT;
      ref_lonely_cluster = TRUE;
    }
  }
  else if (ret == VB_ENGINE_ERROR_NOT_FOUND)
  {
    // Reference node not found, select next one
    select_next_ref = TRUE;

    ret = VB_ENGINE_ERROR_NONE;
  }
  else
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d searching ref node", ret);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (select_next_ref == TRUE)
    {
      if(ref_lonely_cluster == FALSE)
      {
        // Use next node as reference, not in the case one cluster is about to be created
        // In the latter case (a cluster is about to be created, the ref is selected in the EngineConf Build call
        ret = NextReferenceNodeGet(NULL, clusterId);
      }

      *next |= 1 << VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT;
    }
    else
    {
      INT32U                        max_iters;
      BOOL                          loop = FALSE;
      INT8U                         promoted_dev_id = 0;

      // Current reference shall be maintained, so select a new relay

      // 1. Get number of lines in Cluster i (C0)
      // 2. Get number of lines seeing a sync (either DM Ref, Relay or Relay Candidate) (C1)
      // 3. Get number of lines seeing only a Relay Candidate (C2)
      // 4. Tag lines seeing only Relay Candidate

      // Get  1. 2. 3. 4.
      if (ret == VB_ENGINE_ERROR_NONE)
      {
        ret = VbEngineAlignClusterAlignInfoGet(clusterBuildInfo, clusterId);

        if (clusterBuildInfo->txNodesInfo.ref != NULL)
        {
          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "clusterId %d  in:Line in C %d, syncedLines %d, syncedR %d, numR %d, num RC %d, Ref %d, ret %d",
              clusterId,
              clusterBuildInfo->numLinesInCluster,
              clusterBuildInfo->numLinesSyncedInCluster,
              clusterBuildInfo->numLinesSyncedExclusivelyWithRelayCandidate,
              clusterBuildInfo->txNodesInfo.numRelays,
              clusterBuildInfo->txNodesInfo.numRelaysCandidate,
              clusterBuildInfo->txNodesInfo.ref->dm.devID,
              ret);
        }
      }

      // Set maximum number of iterations
      max_iters = VB_EA_ALIGN_GHN_MAX_RELAYS;

      do
      {
        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // 5. Get list of Relay Candidate by order of largest exclusive visibility
          ret = ClusterRelayCandidatesGet(clusterBuildInfo, clusterId);
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // 6. Promote most seen Relay candidate to Relay
          ret = ClusterCandidateToRelaySet(clusterBuildInfo, &promoted_dev_id);
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // ... update C2 (lines seeing only a relay candidate (and not the just promoted one)
          ret = VbEngineAlignClusterAlignInfoGet(clusterBuildInfo, clusterId);
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // 8. Go back to 5 if C2 is still > 0
          if( (clusterBuildInfo->numLinesSyncedExclusivelyWithRelayCandidate > 0) &&
              (clusterBuildInfo->txNodesInfo.numRelays <= VB_EA_ALIGN_GHN_MAX_RELAYS) )
          {
            loop = TRUE;
          }
          else
          {
            loop = FALSE;
          }
        }

        if (max_iters > 0)
        {
          max_iters--;
        }

        if ((ret != VB_ENGINE_ERROR_NONE) || (max_iters == 0))
        {
          loop = FALSE;
        }
      }
      while (loop == TRUE);

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        if (clusterBuildInfo->txNodesInfo.ref != NULL)
        {
          VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "clusterId %d out:Line in C %d, syncedLines %d, syncedR %d, numR %d, num RC %d, Ref %d",
              clusterId,
              clusterBuildInfo->numLinesInCluster,
              clusterBuildInfo->numLinesSyncedInCluster,
              clusterBuildInfo->numLinesSyncedExclusivelyWithRelayCandidate,
              clusterBuildInfo->txNodesInfo.numRelays,
              clusterBuildInfo->txNodesInfo.numRelaysCandidate,
              clusterBuildInfo->txNodesInfo.ref->dm.devID);
        }

        // 9.1.2 Clear CANDIDATE Role first
        ret = AlignRoleChange(VB_ALIGN_ROLE_RELAY_CANDIDATE, VB_ALIGN_ROLE_SLAVE, clusterId);

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // 9.1 if C0 (number of lines in cluster) > C1 (number of lines in synced with the Ref or a Relay) select new Relay Candidate(s) and RESTART
          if(clusterBuildInfo->numLinesInCluster > clusterBuildInfo->numLinesSyncedInCluster)
          {
            if (ret == VB_ENGINE_ERROR_NONE)
            {
              // 9.1.3 Set list of next relay candidate
              ret =  ClusterRoleNextCandidateListSet(clusterBuildInfo, clusterId);
            }

            if (ret == VB_ENGINE_ERROR_NONE)
            {
              if (clusterBuildInfo->txNodesInfo.numRelaysCandidate == 0)
              {
                // No more relay candidate and still not synced lines -> Build cluster and carry on alignment with remaining lines
                // SEND Restart and DONE

                if (clusterBuildInfo->txNodesInfo.numRelays == VB_EA_ALIGN_GHN_MAX_RELAYS)
                {
                  INT8U align_id = 0;
                  VbEngineAlignmentIdGet(clusterId, NULL, &align_id);

                  VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
                      "Maximum number of RELAYS reached (%u) and there are lines not synced",
                      clusterBuildInfo->txNodesInfo.numRelays);

                  VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR,
                      "[%-10s] Align Id %u - Maximum number of RELAYS reached (%u) and there are lines not synced",
                      "Conf",
                      align_id,
                      clusterBuildInfo->txNodesInfo.numRelays);
                }

                VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
                    "No more suitable RELAY Candidates, cluster split (Done/Restart)");

                VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR,
                    "[%-10s] No more suitable RELAY Candidates, cluster split (Done/Restart)",
                    "Conf");

                *next  = 1 << VB_ENGINE_ALIGNMENT_NEXT_DONE_EVT;
                *next |= 1 << VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT;
              }
              // Check if there is enough room for a new relay and if a proper candidate was found
              else
              {
                // Add new relay Candidate(s) and Restart alignment
                // SEND Restart
                VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
                    "New RELAY Candidates selected - (num relays %u; candidates %u) - (Restart)",
                    clusterBuildInfo->txNodesInfo.numRelays,
                    clusterBuildInfo->txNodesInfo.numRelaysCandidate);

                VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR,
                    "[%-10s] New RELAY Candidates selected - (num relays %u; candidates %u) - (Restart)",
                    "Conf",
                    clusterBuildInfo->txNodesInfo.numRelays,
                    clusterBuildInfo->txNodesInfo.numRelaysCandidate);

                *next = 1 << VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT;
              }
            }
            else
            {
              // Error configuring new relays candidates

              VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
                  "Error %d configuring a new relay candidate - (num relays %u; candidates %u) - (Restart)",
                  ret,
                  clusterBuildInfo->txNodesInfo.numRelays,
                  clusterBuildInfo->txNodesInfo.numRelaysCandidate);

              VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR,
                  "[%-10s] Error %d configuring a new relay candidate - (num relays %u; candidates %u) - (Restart)",
                  "Conf",
                  ret,
                  clusterBuildInfo->txNodesInfo.numRelays,
                  clusterBuildInfo->txNodesInfo.numRelaysCandidate);

              *next = 1 << VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT;
            }
          }
          else
          {
            // 9.2 C0 = C1 (number of lines in clusters = number of lines synced ->

            VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
                "Update done - (num relays %u; candidates %u) - (Restart)",
                clusterBuildInfo->txNodesInfo.numRelays,
                0);

            VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR,
                "[%-10s] Update done - (num relays %u; candidates %u) - (Restart)",
                "Conf",
                clusterBuildInfo->txNodesInfo.numRelays,
                0);

            *next = 1 << VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT;
          }
        }
      }
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_INFO, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Conf",
        "Error %d selecting a new Tx node - (Restart)",
        ret);

    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR,
        "[%-10s] Error %d selecting a new Tx node - (Restart)",
        "Conf",
        ret);

    *next |= 1 << VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEngineConfReqBuild(INT32U clusterId, BOOLEAN forceRelayCandidates)
{
  t_VB_engineErrorCode     ret;
  t_VBAlignmentMode        alignment_mode;
  INT8U                   *conf_req_buffer = NULL;
  INT32U                   conf_req_buffer_len = 0;

  ret = VbEngineConfAlignmentModeGet(&alignment_mode);
  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (alignment_mode == VB_ALIGN_MODE_GHN)
    {
      t_domain               *ref_domain = NULL;
      t_vbAlignGhnTxNodesInfo tx_nodes_info;
      t_psdBandAllocation    *psd_band_allocation;
      t_vbAlignModeSeedParam  *seeds = NULL;
      INT16U                  num_dms = 0;
      t_domain               *tx_nodes_cooked[VB_EA_ALIGN_GHN_MAX_TX_NODES];
      psd_band_allocation = VbEngineConfPSDBandAllocationGet();
      if(psd_band_allocation == NULL)
      {
        ret = VB_ENGINE_ERROR_UNKNOWN;
      }

      // Generate automatic seeds and dids for all DMs if feature is enabled
      if ((ret == VB_ENGINE_ERROR_NONE) && (VbEngineConfSeedAutomaticEnableGet() == TRUE))
      {
        ret = VbEngineAutomaticSeedsAndDids(clusterId, &seeds, &num_dms);
      }

      if(ret == VB_ENGINE_ERROR_NONE)
      {
        // Search for reference node (if any)
        ret = FirstMatchingRoleGet(VB_ALIGN_ROLE_REF, &ref_domain, clusterId);
      }

      if (ret == VB_ENGINE_ERROR_NOT_FOUND)
      {
        // No reference found, select a new one
        ret = NextReferenceNodeGet(&ref_domain, clusterId);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        if (ref_domain == NULL)
        {
          ret = VB_ENGINE_ERROR_NOT_FOUND;
        }
      }

      if ((ret == VB_ENGINE_ERROR_NONE) &&
          (forceRelayCandidates == TRUE))
      {
        // Configure all RELAYS as RELAY_CANDIDATES to re-evaluate them
        ret = AlignRoleChange(VB_ALIGN_ROLE_RELAY, VB_ALIGN_ROLE_RELAY_CANDIDATE, clusterId);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Reference domain selected
        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Node %s (did %u) selected as %s",
            ref_domain->dm.MACStr, ref_domain->dm.devID, VbEngineAlignRoleStringGet(ref_domain->dm.nodeAlignInfo.role));

        // Get tx nodes info
        ret = TxNodesInfoGet(&tx_nodes_info, clusterId);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Check number of relays
        if ((tx_nodes_info.numRelays + tx_nodes_info.numRelaysCandidate) > VB_EA_ALIGN_GHN_MAX_RELAYS)
        {
          VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error. Incorrect number of relays detected (%u/%lu).",
              tx_nodes_info.numRelays, tx_nodes_info.numRelaysCandidate);

          // Configure all relays as SLAVEs to start from scratch
          ret = AlignRoleChange(VB_ALIGN_ROLE_RELAY, VB_ALIGN_ROLE_SLAVE, clusterId);

          if (ret == VB_ENGINE_ERROR_NONE)
          {
            // Configure all relays candidate as SLAVEs to start from scratch
            ret = AlignRoleChange(VB_ALIGN_ROLE_RELAY_CANDIDATE, VB_ALIGN_ROLE_SLAVE, clusterId);
            tx_nodes_info.numRelays = 0;
            tx_nodes_info.numRelaysCandidate = 0;
          }
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Number of relays %u Candidate %u; number of bands %lu (err %d)", tx_nodes_info.numRelays, tx_nodes_info.numRelaysCandidate, psd_band_allocation->numBands200Mhz, ret);

        // Configure rest of nodes as SLAVEs
        ret = AlignRoleChange(VB_ALIGN_ROLE_NOT_INIT, VB_ALIGN_ROLE_SLAVE, clusterId);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Allocate memory for message
          conf_req_buffer_len = sizeof(t_vbAlignModeParam) + sizeof(t_vbCdtaEngineConf) +
                                (psd_band_allocation->numBands200Mhz*sizeof(t_vbBoostBandLastCarrier)) +
                                ((tx_nodes_info.numRelays + tx_nodes_info.numRelaysCandidate) * sizeof(t_vbAlignModeRelayParam)) +
                                 (num_dms*sizeof(t_vbAlignModeSeedParam));

          conf_req_buffer = (INT8U *)calloc(1, conf_req_buffer_len);
        
          if (conf_req_buffer == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        BOOLEAN sync_lost_flag = FALSE;

        // Build the common part of the message (Alignment bits)
        t_vbAlignModeParam *align_mode_req = (t_vbAlignModeParam *)conf_req_buffer;

        align_mode_req->alignMode = alignment_mode;
        align_mode_req->alignId   = ++vbAlignEngineConfId;
        align_mode_req->absOffset = VB_ALIGN_GHN_OFFSET_TYPE;
        MACAddrClone(align_mode_req->DMMAC, ref_domain->dm.MAC);
        align_mode_req->refDID = ref_domain->dm.devID;
        align_mode_req->macCycleDur = _htonl((MS_TO_10NS_UNITS(MAC_CYCLE_DURATION)));
        align_mode_req->maxDurSyncFrm = _htonl(vbAlignGhnTxInfoArray[VB_ALIGN_GHN_REF_TX_IDX].duration);
        align_mode_req->syncFrmOffset = _htonl(vbAlignGhnTxInfoArray[VB_ALIGN_GHN_REF_TX_IDX].offset);
        align_mode_req->numRelays = tx_nodes_info.numRelays+tx_nodes_info.numRelaysCandidate;
        align_mode_req->clusterId = _htonl(clusterId);

        VbEngineClusterSyncLostGet(clusterId, &sync_lost_flag);
        if(sync_lost_flag == TRUE)
        {
          srand(clusterId);
          align_mode_req->numCyclesApplyExtraTime = VB_ALIGN_GHN_NUM_CYCLES_EXTRA_TIME;
          align_mode_req->tmpMacCycleDurExtraTime = _htonl((rand()%VB_ALIGN_MAX_SYNC_FRAME_DURATION + VB_ALIGN_MAX_SYNC_FRAME_DURATION));

          VbEngineClusterSyncLostSet(clusterId, FALSE);
        }
        else
        {
          align_mode_req->numCyclesApplyExtraTime = 0;
          align_mode_req->tmpMacCycleDurExtraTime = _htonl((INT32U)0);
        }

        if (VbEngineConfSeedAutomaticEnableGet() == TRUE)
        {
          align_mode_req->numSeeds = num_dms;
        }
        else
        {
          align_mode_req->numSeeds = 0;
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Build the common part of the message (CDTA bits)
        t_vbCdtaEngineConf *cdta_conf;

        if (psd_band_allocation != NULL)
        {
          t_vbBoostBandLastCarrier *boosted_band_def;
          INT32U                   idx;

          cdta_conf = (t_vbCdtaEngineConf *)(conf_req_buffer + sizeof(t_vbAlignModeParam));
          cdta_conf->defaultRate = VbCdtaDefaultQosRateGet();
          cdta_conf->currentRate = VbCdtaQosRateGet(clusterId);

          cdta_conf->numBandsFirstPSD = 1;
          cdta_conf->nBoostBands = psd_band_allocation->numBands200Mhz;

          boosted_band_def = (t_vbBoostBandLastCarrier *)(conf_req_buffer + sizeof(t_vbAlignModeParam) + sizeof(t_vbCdtaEngineConf));
          for(idx=0; idx< cdta_conf->nBoostBands; idx++, boosted_band_def++)
          {
            boosted_band_def->boostBandLastCarrier = _htons(psd_band_allocation->lastCarrier[idx]);
          }
        }
        else
        {
          ret = VB_ENGINE_ERROR_UNKNOWN;
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        ret = AlignTxNodesCook(&tx_nodes_info, tx_nodes_cooked);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        INT32U                   idx;
        t_vbAlignModeRelayParam *relays;

        // Update relays info
        relays = (t_vbAlignModeRelayParam *)(conf_req_buffer +
                                             sizeof(t_vbAlignModeParam) +
                                             sizeof(t_vbCdtaEngineConf) +
                                             psd_band_allocation->numBands200Mhz*sizeof(t_vbBoostBandLastCarrier));

        for (idx = 0; idx < VB_EA_ALIGN_GHN_MAX_RELAYS; idx++)
        {
          if (tx_nodes_cooked[idx + 1] != NULL)
          {
            MACAddrClone(relays[idx].relayMAC, tx_nodes_cooked[idx + 1]->dm.MAC);
            relays[idx].deviceId      =  tx_nodes_cooked[idx + 1]->dm.devID;
            relays[idx].duration      = _htonl(vbAlignGhnTxInfoArray[idx + 1].duration);
            relays[idx].syncFrmOffset = _htonl(vbAlignGhnTxInfoArray[idx + 1].offset);

            if (tx_nodes_cooked[idx + 1]->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY)
            {
              VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "R  - %d %d %lu", idx, relays[idx].deviceId, vbAlignGhnTxInfoArray[idx + 1].offset);
            }
            else if (tx_nodes_cooked[idx + 1]->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY_CANDIDATE)
            {
              VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "RC - %ld %d %lu", idx, relays[idx].deviceId, vbAlignGhnTxInfoArray[idx + 1].offset);
            }
            else
            {
              VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Unexpected role (%u) for node %s",
                  tx_nodes_cooked[idx + 1]->dm.nodeAlignInfo.role,
                  tx_nodes_cooked[idx + 1]->dm.MACStr);
            }
          }
          else
          {
            // Last relay reached
            break;
          }
        }
      }

      // Build the seed part of the engine conf message
      if ((ret == VB_ENGINE_ERROR_NONE) && (VbEngineConfSeedAutomaticEnableGet() == TRUE) && (seeds != NULL))
      {
          INT32U                   idx;
          t_vbAlignModeSeedParam  *seeds_conf;

          // Go to seed location
          seeds_conf = (t_vbAlignModeSeedParam *)(conf_req_buffer +
              sizeof(t_vbAlignModeParam) +
              sizeof(t_vbCdtaEngineConf) +
              psd_band_allocation->numBands200Mhz*sizeof(t_vbBoostBandLastCarrier) +
              (tx_nodes_info.numRelays+tx_nodes_info.numRelaysCandidate)*sizeof(t_vbAlignModeRelayParam));

          // Update seed information
          for (idx = 0; idx < num_dms; idx++)
          {
              // Get next DM's MAC
              MACAddrClone(seeds_conf[idx].dmMAC, seeds[idx].dmMAC);
              seeds_conf[idx].dmSeed = _htons(seeds[idx].dmSeed);
              seeds_conf[idx].did = _htons(seeds[idx].did);
          }

          // Free memory used for seed information
          if (seeds != NULL)
          {
            free (seeds);
            seeds = NULL;
          }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        ret = EngineConfReqPldUpdate(clusterId, conf_req_buffer, conf_req_buffer_len);
      }
    }
    else if (alignment_mode == VB_ALIGN_MODE_COMMON_CLOCK)
    {
      t_psdBandAllocation    *psd_band_allocation;
      t_vbAlignModeSeedParam *seeds = NULL;
      INT16U                 num_dms = 0;
      psd_band_allocation = VbEngineConfPSDBandAllocationGet();
      if(psd_band_allocation == NULL)
      {
        ret = VB_ENGINE_ERROR_UNKNOWN;
      }

      // Generate automatic seeds and dids for all DMs if feature is enabled
      if ((ret == VB_ENGINE_ERROR_NONE) && (VbEngineConfSeedAutomaticEnableGet() == TRUE))
      {
        ret = VbEngineAutomaticSeedsAndDids(clusterId, &seeds, &num_dms);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Allocate memory for message
        conf_req_buffer_len = sizeof(t_vbAlignModeParam) + sizeof(t_vbCdtaEngineConf) + psd_band_allocation->numBands200Mhz*sizeof(t_vbBoostBandLastCarrier) +
        (num_dms*sizeof(t_vbAlignModeSeedParam));

        conf_req_buffer = (INT8U *)calloc(1, conf_req_buffer_len);

        if (conf_req_buffer == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // Build the common part of the message (Alignment bits)
          t_vbAlignModeParam *align_mode_req = (t_vbAlignModeParam *)conf_req_buffer;
          if (VbEngineConfSeedAutomaticEnableGet() == TRUE)
          {
            align_mode_req->numSeeds = num_dms;
          }
          else
          {
            align_mode_req->numSeeds = 0;
          }
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // Build the common part of the message (CDTA bits)
          t_vbCdtaEngineConf *cdta_conf;

          if (conf_req_buffer != NULL)
          {
            t_vbBoostBandLastCarrier *boosted_band_def;
            INT32U                   idx;

            cdta_conf = (t_vbCdtaEngineConf *)(conf_req_buffer + sizeof(t_vbAlignModeParam));
            cdta_conf->defaultRate = VbCdtaDefaultQosRateGet();
            cdta_conf->currentRate = VbCdtaQosRateGet(clusterId);

            cdta_conf->numBandsFirstPSD = 1;
            cdta_conf->nBoostBands = psd_band_allocation->numBands200Mhz;

            boosted_band_def = (t_vbBoostBandLastCarrier *)(conf_req_buffer + sizeof(t_vbAlignModeParam) + sizeof(t_vbCdtaEngineConf));
            for(idx=0; idx < cdta_conf->nBoostBands; idx++, boosted_band_def++)
            {
              boosted_band_def->boostBandLastCarrier = _htons(psd_band_allocation->lastCarrier[idx]);
            }
          }
        }

        // Build the seed part of the engine conf message
        if ((ret == VB_ENGINE_ERROR_NONE) && (VbEngineConfSeedAutomaticEnableGet() == TRUE) && (seeds != NULL))
        {
            INT32U                   idx;
            t_vbAlignModeSeedParam  *seeds_conf;

            // Go to seed location
            // Update seed info
            seeds_conf = (t_vbAlignModeSeedParam *)(conf_req_buffer +
                sizeof(t_vbAlignModeParam) +
                sizeof(t_vbCdtaEngineConf) +
                psd_band_allocation->numBands200Mhz*sizeof(t_vbBoostBandLastCarrier));

            // Update seed information
            for (idx = 0; idx < num_dms; idx++,seeds_conf++)
            {
                // Get next DM's MAC
                MACAddrClone(seeds_conf->dmMAC, seeds[idx].dmMAC);
                seeds_conf->dmSeed = _htons(seeds[idx].dmSeed);
                seeds_conf->did = _htons(seeds[idx].did);
            }

            // Free memory used for seed information
            if (seeds != NULL)
            {
              free (seeds);
              seeds = NULL;
            }
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          t_vbAlignModeParam *align_mode_req = (t_vbAlignModeParam *)conf_req_buffer;

          align_mode_req->alignMode = alignment_mode;
          align_mode_req->numRelays = 0;
          ret = EngineConfReqPldUpdate(clusterId, conf_req_buffer, conf_req_buffer_len);
        }
      }

    }
    else
    {
      ret = VB_ENGINE_ERROR_UNKNOWN;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "AlignMode.req message build: mode %s, %lu", VbEngineAlignModeStringGet(alignment_mode), clusterId);
  }

  if (ret == VB_ENGINE_ERROR_NOT_FOUND)
  {
    t_vbEngineNumNodes num_nodes;

    ret = VbEngineDataModelNumNodesInClusterXGet(clusterId, &num_nodes);

    if ((ret == VB_ENGINE_ERROR_NONE) && (num_nodes.numDms == 0))
    {
      // Expected error VB_ENGINE_ERROR_NOT_FOUND when no domains are discovered. Wait until a new topology change arrives
    }
    else
    {
      // Maintain error
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBAlignmentMode align_mode;
    INT8U             align_id = 0;

    VbEngineAlignmentIdGet(clusterId, NULL, &align_id);
    VbEngineConfAlignmentModeGet(&align_mode);

    // Metrics
    if (forceRelayCandidates == TRUE)
    {
      // First time
      VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_START, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Start",
          "***** Alignment process START (mode %s) *****",
          VbEngineAlignModeStringGet(align_mode));

      VbEngineAlignMetricsNodeInfoReport(clusterId);

      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Alignment process START (mode %s; alignId %u; clusterId %u)",
          VbEngineAlignModeStringGet(align_mode), align_id, clusterId);
    }
    else
    {
      // Align restart
      VbEngineAlignMetricsEvReport(VB_METRICS_EVENT_ALIGN_RESTART, VB_ENGINE_ALL_DRIVERS_STR, clusterId, "Restart",
          "*** Alignment process RESTART (mode %s) ***",
           VbEngineAlignModeStringGet(align_mode));

      VbEngineAlignMetricsNodeInfoReport(clusterId);

      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Alignment process RESTART (mode %s; alignId %u; clusterId %u)",
          VbEngineAlignModeStringGet(align_mode), align_id, clusterId);
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d building AlignMode.req message", ret);
  }

  return ret;
}

/*******************************************************************/

const CHAR *VbEngineAlignRoleStringGet(t_alignRole role)
{
  const CHAR *role_str;

  if (role < VB_ALIGN_ROLE_LAST)
  {
    role_str = vbAlignRoleString[role];
  }
  else
  {
    role_str = "INVALID";
  }

  return role_str;
}

/*******************************************************************/

const CHAR *VbEngineAlignModeStringGet(t_VBAlignmentMode mode)
{
  const CHAR *mode_str;

  if (mode < VB_ALIGN_MODE_LAST)
  {
    mode_str = vbAlignModeString[mode];
  }
  else
  {
    mode_str = "INVALID";
  }

  return mode_str;
}

/*******************************************************************/

FP32 VbEngineAlignAdcOutToFPGet(INT32U adcOutRms)
{
  FP32 adc_out = 0;

  if (adcOutRms > 0)
  {
    adc_out = VB_ALIGN_ADC_RMS_FROM_MSG_FORMAT(adcOutRms);
  }
  else
  {
    adc_out = VB_ALIGN_ADC_RMS_MIN;
  }

  return adc_out;
}

/*******************************************************************/

FP32 VbEngineAlignAdcOutRmsGet(t_domain *domain)
{
  FP32 adc_out = 0;

  if (domain != NULL)
  {
    INT32U adc_out_rms = VbEngineAlignGhnBestDetectedPowGet(domain);

    if (adc_out_rms == 0)
    {
      if (domain->dm.nodeAlignInfo.role != VB_ALIGN_ROLE_REF)
      {
        adc_out = VB_ALIGN_ADC_RMS_MIN;
      }
      else
      {
        adc_out = 0;
      }
    }
    else
    {
      adc_out = VbEngineAlignAdcOutToFPGet(adc_out_rms);
    }
  }

  return adc_out;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineOldClusterAdd(INT32U oldClusterId)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster             *cluster = NULL;

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelCreateCluster(&cluster);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    cluster->clusterInfo.clusterId = oldClusterId;
//    cluster->clusterInfo.numLines  = clusterBuildInfo->numLinesSyncedInCluster;
//    cluster->clusterInfo.alignRef  = (clusterBuildInfo->txNodesInfo.ref == NULL)? 0:clusterBuildInfo->txNodesInfo.ref->dm.devID;
//    cluster->clusterInfo.numRelays = clusterBuildInfo->txNodesInfo.numRelays;
    cluster->timeoutCnf.clusterCast.list[0] = cluster->clusterInfo.clusterId;
    cluster->timeoutCnf.clusterCast.numCLuster = 1;
    cluster->epChange = FALSE;
    cluster->skipMeasPlan = FALSE;
    cluster->syncLostFlag = FALSE;

    cluster->cdtaData = (INT8U *)calloc(1, sizeof(t_clusterCdtaInfo));
    if(cluster->cdtaData == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }

    // Allocate CDTA info now as it is fixed in length and is needed if cluster goes back to align phase (The currentQosRate needs to be up to date)
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      t_clusterCdtaInfo *cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

      bzero(&cluster_cdta_info->stats, sizeof(cluster_cdta_info->stats));
      bzero(&cluster_cdta_info->hist, sizeof(cluster_cdta_info->hist));
      bzero(&cluster_cdta_info->metrics, sizeof(cluster_cdta_info->metrics));

      cluster_cdta_info->finalQosRate = VbCdtaQosRateGet(0);
      pthread_mutex_init(&(cluster_cdta_info->hist.mutex), NULL);
    }

//    for(i=0; i< clusterBuildInfo->txNodesInfo.numRelays; i++)
//    {
//      cluster->clusterInfo.relays[i] = clusterBuildInfo->txNodesInfo.relays[i]->dm.devID;
//    }

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "VbEngineAlignClusterAdd Cluster Id %d, numLines %d, RefDid %d",
                  cluster->clusterInfo.clusterId, cluster->clusterInfo.numLines, cluster->clusterInfo.alignRef);

  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineCltListClusterAdd(cluster);
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    if (cluster != NULL)
    {
      // Release allocated memory
      VbEngineDatamodelClusterDel(&cluster);
    }

    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d Old Cluster Add", ret);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignClusterAdd(INT32U currentClusterId, INT32U *newClusterId, t_vbAlignClusterBuildInfo *clusterBuildInfo)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster             *cluster = NULL;
  INT32U                   i;

  if ( (newClusterId == NULL) || (clusterBuildInfo == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelCreateCluster(&cluster);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    cluster->clusterInfo.clusterId = VbEngineDataModelNextClusterIdGet();
    cluster->clusterInfo.numLines  = clusterBuildInfo->numLinesSyncedInCluster;
    cluster->clusterInfo.alignRef  = (clusterBuildInfo->txNodesInfo.ref == NULL)? 0:clusterBuildInfo->txNodesInfo.ref->dm.devID;
    cluster->clusterInfo.numRelays = clusterBuildInfo->txNodesInfo.numRelays;
    cluster->timeoutCnf.clusterCast.list[0] = cluster->clusterInfo.clusterId;
    cluster->timeoutCnf.clusterCast.numCLuster = 1;
    cluster->epChange = FALSE;
    cluster->skipMeasPlan = FALSE;
    cluster->syncLostFlag = FALSE;

    cluster->cdtaData = (INT8U *)calloc(1, sizeof(t_clusterCdtaInfo));
    if(cluster->cdtaData == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }

    // Allocate CDTA info now as it is fixed in length and is needed if cluster goes back to align phase (The currentQosRate needs to be up to date)
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      t_clusterCdtaInfo *cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

      bzero(&cluster_cdta_info->stats, sizeof(cluster_cdta_info->stats));
      bzero(&cluster_cdta_info->hist, sizeof(cluster_cdta_info->hist));
      bzero(&cluster_cdta_info->metrics, sizeof(cluster_cdta_info->metrics));

      cluster_cdta_info->finalQosRate = VbCdtaQosRateGet(0);
      pthread_mutex_init(&(cluster_cdta_info->hist.mutex), NULL);
    }

    for(i=0; i< clusterBuildInfo->txNodesInfo.numRelays; i++)
    {
      cluster->clusterInfo.relays[i] = clusterBuildInfo->txNodesInfo.relays[i]->dm.devID;
    }

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "VbEngineAlignClusterAdd Cluster Id %d, numLines %d, RefDid %d",
                  cluster->clusterInfo.clusterId, cluster->clusterInfo.numLines, cluster->clusterInfo.alignRef);

  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineCltListClusterAdd(cluster);
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // All lines in cluster Id currentClusterId -> cluster->clusterInfo.clusterId
    ret = VbEngineAlignClusterXSyncedTagY(currentClusterId, cluster->clusterInfo.clusterId);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *newClusterId = cluster->clusterInfo.clusterId;
  }
  else
  {
    if (cluster != NULL)
    {
      // Release allocated memory
      VbEngineDatamodelClusterDel(&cluster);
    }

    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d Cluster Add", ret);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignClusterUpdate(INT32U clusterId, t_vbAlignClusterBuildInfo *clusterBuildInfo)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster             *cluster;
  INT32U                   i;

  if (clusterBuildInfo == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(clusterId, &cluster);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    cluster->clusterInfo.numLines = clusterBuildInfo->numLinesInCluster;
    cluster->clusterInfo.alignRef = (clusterBuildInfo->txNodesInfo.ref == NULL)? 0:clusterBuildInfo->txNodesInfo.ref->dm.devID;
    memset(cluster->clusterInfo.relays, 0, VB_EA_ALIGN_GHN_MAX_RELAYS);
    for(i=0;i<clusterBuildInfo->txNodesInfo.numRelays;i++)
    {
      cluster->clusterInfo.relays[i] = clusterBuildInfo->txNodesInfo.relays[i]->dm.devID;
    }

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "VbEngineAlignClusterAdd Cluster Id %d, numLines %d, RefDid %d",
                  cluster->clusterInfo.clusterId, cluster->clusterInfo.numLines, cluster->clusterInfo.alignRef);

  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d Cluster Add", ret);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignCluster0Add(void)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster              *cluster;

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelCreateCluster(&cluster);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    cluster->clusterInfo.clusterId = 0;
    cluster->clusterInfo.numLines = 0;
    cluster->clusterInfo.alignRef = 0;
    cluster->timeoutCnf.clusterCast.list[0] = 0;
    cluster->timeoutCnf.clusterCast.numCLuster = 0;
    cluster->epChange = FALSE;
    cluster->skipMeasPlan = FALSE;
    cluster->syncLostFlag = FALSE;

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "VbEngineAlignClusterAdd Cluster Id %d, numLines %d, RefDid %d",
                  cluster->clusterInfo.clusterId, cluster->clusterInfo.numLines, cluster->clusterInfo.alignRef);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineCltListClusterAdd(cluster);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate CDTA memory...
    ret = VbEngineCdtaClusterResourcesAlloc(0);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // ... and set default Qos Rate of cluster 0
      ret = VbCdtaQosRateForce(0, VB_ENGINE_QOS_RATE_70_30);
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d Cluster Add", ret);
  }

  return ret;
}


/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignClusterRemove(INT32U clusterId)
{
  t_VB_engineErrorCode     ret;
  t_VBCluster              *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Remove cluster from clusters list
    VbEngineCltListRemoveCluster(cluster);

    // Release its related memory
    VbEngineDatamodelClusterMemFree(cluster);
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d Cluster Rem", ret);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignClusterStopGet(INT32U clusterId, BOOL *stopTx)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBAlignmentMode    alignment_mode;

  if (stopTx == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineConfAlignmentModeGet(&alignment_mode);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (alignment_mode == VB_ALIGN_MODE_GHN)
    {
      *stopTx = FALSE;

      if(VbEngineDataModelNumClustersGet() > 1) // 0 + biggest one
      {
        INT32U biggest_cluster_id = 0;
        biggest_cluster_id = VbEngineDataModelBiggestClusterIdGet();
        if(biggest_cluster_id != clusterId)
        {
          *stopTx = TRUE;
        }
      }
      else
      {
        // Just cluster 0
        *stopTx = FALSE;
      }
    }
    else
    {
      *stopTx = FALSE;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignStopClusterSend(t_VBDriver *driver, BOOLEAN stopTx)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;
  t_vbEAAlignClusterStopReq     cluster_stop_req;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    cluster_stop_req.stopTxFlag = stopTx;

    // Send AlignStopCluster.req message
    ret = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_ALIGN_STOP_CLUSTER_REQ, (INT8U *)&cluster_stop_req, VB_EA_CLUSTER_STOP_REQ_SIZE, driver);

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "StopCluster.req sent");
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error sending StopCluster (%d)", ret);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignAllDriversClusterTag(INT32U clusterId)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelDriversLoop(ClustersIdTagLoopCb, &clusterId);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignClusterXSyncedTagY(INT32U currentClusterId, INT32U newClusterId)
{
  t_VB_engineErrorCode     ret = VB_ENGINE_ERROR_NONE;

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelClusterXDriversLoop(ClustersIdSyncedTagLoopCb, currentClusterId, (void*)&newClusterId);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignClusterAlignInfoGet(t_vbAlignClusterBuildInfo *clusterInfoBuild, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (clusterInfoBuild == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = TxNodesInfoGet(&(clusterInfoBuild->txNodesInfo), clusterId);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d getting Tx nodes info", ret);
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    clusterInfoBuild->clusterId = clusterId;
    clusterInfoBuild->numLinesInCluster = 0;
    clusterInfoBuild->numLinesSyncedInCluster = 0;
    clusterInfoBuild->numLinesSyncedExclusivelyWithRelayCandidate = 0;

    // Loop through all domains in cluster Id 0 and get relays info
    ret = VbEngineDatamodelClusterXAllDomainsLoop(ClusterInfoBuildLoopCb, clusterId, clusterInfoBuild);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelClusterXAllDomainsLoop(ClusterSyncOnlyWithCandidateBuildLoopCb, clusterId, clusterInfoBuild);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignClusterRoleReset(INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through all domains and get relays info
    ret = VbEngineDatamodelClusterXAllDomainsLoop(ClusterRoleResetLoopCb, clusterId, NULL);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignReleaseResources(void)
{
  if (seedArray != NULL)
  {
    free(seedArray);
  }

  return VB_ENGINE_ERROR_NONE;
}

/*******************************************************************/

INT8U VbEngineAlignGhnBestDetectedDidGet(t_domain *domain)
{
  t_VB_engineErrorCode err;
  t_syncDetInfo        info;
  INT8U                best_did;

  err = AlignGhnBestDetectedDidInfoGet(domain, &info);

  if (err == VB_ENGINE_ERROR_NONE)
  {
    best_did = info.detDid;
  }
  else
  {
    best_did = 0;
  }

  return best_did;
}

/*******************************************************************/

INT32U VbEngineAlignGhnBestDetectedPowGet(t_domain *domain)
{
  t_VB_engineErrorCode err;
  t_syncDetInfo        info;
  INT32U               best_pow;

  err = AlignGhnBestDetectedDidInfoGet(domain, &info);

  if (err == VB_ENGINE_ERROR_NONE)
  {
    best_pow = info.adcOutRms;
  }
  else
  {
    best_pow = 0;
  }

  return best_pow;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignmentIdGet(INT32U clusterId, t_VBCluster *cluster, INT8U *alignId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if(alignId == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if (cluster == NULL)
    {
      ret = VbEngineClusterByIdGet(clusterId, &cluster);
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if (cluster->confReqBuffer == NULL)
      {
        ret = VB_ENGINE_ERROR_ALIGN;
      }
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      t_vbAlignModeParam *align_param;

      align_param = (t_vbAlignModeParam *)cluster->confReqBuffer;
      *alignId = align_param->alignId;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignmentHasBeenCandidateReset(INT32U clusterId)
{
  t_VB_engineErrorCode ret;

  // Loop through all domains
  ret = VbEngineDatamodelClusterXAllDomainsLoop(AlignHasBeenCandidateResetLoopCb, clusterId, NULL);

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineAlignDriverXSyncedTagY(t_VBDriver *driver, INT32U clusterId)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelDomainsLoop(driver, DomainSyncedLoopCb, NULL);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if(clusterId != 0)
    {
      driver->clusterId = clusterId;
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Tagged with cluster %d", driver->clusterId);
    }
  }
  else if (ret == VB_ENGINE_ERROR_NOT_SYNCED)
  {
    // Possible error, do not report above
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
