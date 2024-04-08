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
 * @file vb_engine_cdta.c
 * @brief Implements engine C-DTA functions
 *
 * @internal
 *
 * @author Y.Raoul
 * @date 2017/08/21
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>

#include "vb_log.h"
#include "vb_util.h"
#include "vb_engine_datamodel.h"
#include "vb_engine_EA_interface.h"
#include "vb_engine_clock.h"
#include "vb_counters.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_psd_shape.h"
#include "vb_engine_cdta.h"
#include "vb_engine_conf.h"
#include "vb_engine_metrics_reports.h"


/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

t_cdtaGlobalConf cdtaGlobalConf;
/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */
#define VB_ENGINE_CDTA_MIN_QOS_RATE         (VB_ENGINE_QOS_RATE_20_80)
#define VB_ENGINE_CDTA_MAX_QOS_RATE         (VB_ENGINE_QOS_RATE_80_20)

/*
 * Those defines depends on the value of the boosting alg period.
 * They should be ~1s
 */
#define VB_ENGINE_CDTA_HYSTERESIS           (CEIL(MSEC_IN_SEC, VbEngineConfBoostAlgPeriodGet()))
#define VB_ENGINE_AGE_TO_TRUST_BPS_REPORT   (CEIL(MSEC_IN_SEC, VbEngineConfBoostAlgPeriodGet()))

#define VB_ENGINE_CDTA_MIN_TRAFFIC          (10)
#define VB_ENGINE_CDTA_MIN_TCPACK_TRAFFIC   (2)

#define VB_ENGINE_CDTA_LOW_BAND_NUM_LINES   (100)  // Any high number > 1 will do

#define VB_ENGINE_MAX_DOWN_UP_WEIGHT        (100)
#define VB_ENGINE_MAX_INDIVIDUAL_METRIC     (100000)

#define METRIC_MULT_FACTOR(X)               ((X)<<10)
#define METRIC_MULT_FACTOR_UP_DOWN(X, Y)    ((X)<<Y)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */
typedef struct s_cdtaAnalyseArgs
{
  INT8U qosRate;
  INT8U bandIdx;
  INT8U nBands;
  INT8U numNodes;
  t_vbEngineBpsBandStatus bandStatus;
} t_cdtaAnalyseArgs;

typedef struct s_cdtaMsgBuildArgs
{
  INT16U  seqNum;
  BOOLEAN forceChannelEstimation;
  t_vbEngineQosRate qosRate;
} t_cdtaMsgBuildArgs;
/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

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

static t_VB_engineErrorCode VbCdtaGlobalInfoReset(t_clusterCdtaInfo *clusterCdtaInfo)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U rate_idx;

  if (clusterCdtaInfo == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    memset((INT8U*)&clusterCdtaInfo->stats, 0, sizeof(clusterCdtaInfo->stats));
    memset((INT8U*)&clusterCdtaInfo->metrics, 0, sizeof(clusterCdtaInfo->metrics));

    for(rate_idx = VB_ENGINE_QOS_RATE_10_90; rate_idx < VB_ENGINE_QOS_RATE_90_10; rate_idx++)
    {
      // Force all lines to use band 0
      clusterCdtaInfo->stats.nBoostedDownLines[0][rate_idx] = VB_ENGINE_CDTA_LOW_BAND_NUM_LINES;
      clusterCdtaInfo->stats.nBoostedUpLines[0][rate_idx] = VB_ENGINE_CDTA_LOW_BAND_NUM_LINES;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaMinUpGet(INT8U idxLow, INT8U idxHigh, INT32U *metrics, INT32U *minIdx)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U rate_idx;
  INT32U min = MAX_INT32U;
  BOOLEAN all_zeros = TRUE;

  if( (metrics == NULL) || (idxLow < VB_ENGINE_QOS_RATE_10_90) || (idxHigh > VB_ENGINE_QOS_RATE_90_10) || (minIdx == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    for(rate_idx = idxLow; rate_idx <= idxHigh; rate_idx++)
    {
      if(metrics[rate_idx] != 0)
      {
        all_zeros = FALSE;
        break;
      }
    }

    if(all_zeros == FALSE)
    {
      for(rate_idx = idxLow; rate_idx <= idxHigh; rate_idx++)
      {
        if(metrics[rate_idx] < min)
        {
          min = metrics[rate_idx];
          *minIdx = rate_idx;
        }
      }
    }
    else
    {
      *minIdx = idxHigh;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaMinDownGet(INT8U idxLow, INT8U idxHigh, INT32U *metrics, INT32U *minIdx)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U rate_idx;
  INT32U min = MAX_INT32U;
  BOOLEAN all_zeros = TRUE;

  if( (metrics == NULL) || (idxLow < VB_ENGINE_QOS_RATE_10_90) || (idxHigh > VB_ENGINE_QOS_RATE_90_10) || (minIdx == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    for(rate_idx = idxLow; rate_idx <= idxHigh; rate_idx++)
    {
      if(metrics[rate_idx] != 0)
      {
        all_zeros = FALSE;
        break;
      }
    }

    if(all_zeros == FALSE)
    {
      for(rate_idx = idxLow; rate_idx <= idxHigh; rate_idx++)
      {
        if(metrics[rate_idx] <= min)
        {
          min = metrics[rate_idx];
          *minIdx = rate_idx;
        }
      }
    }
    else
    {
      *minIdx = idxLow;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaAdequationMetricsSet(t_nodeCdtaInfo* nodeCdtaInfo, t_clusterCdtaInfo* clusterCdtaInfo, INT16U neededXput, t_nodeType nodeType, INT8U maxNumBands, INT32U rateIdx)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if((nodeCdtaInfo == NULL) || (clusterCdtaInfo == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if(neededXput > VB_ENGINE_CDTA_MIN_TRAFFIC)
    {
      INT32S capacity;
      INT32S metric;
      INT32S weight;

      capacity = nodeCdtaInfo->capacityPerRate[rateIdx];
      weight = nodeCdtaInfo->profile.userWeight*nodeCdtaInfo->profile.slaWeight;
      if(capacity > 0)
      {
        INT16U allowed_SLA_xput;
        INT32U thr_inc;
        thr_inc = VbEngineConfBoostThrGet(VB_BOOST_THR_TYPE_INC_BOOST);
        capacity = (thr_inc*capacity)/100 + 1;
        allowed_SLA_xput = (nodeCdtaInfo->profile.userSLA != 0)?(MIN(nodeCdtaInfo->profile.userSLA, neededXput)):neededXput;
        metric = (METRIC_MULT_FACTOR((INT32S)(allowed_SLA_xput - capacity)))/capacity;
        metric = MIN(VB_ENGINE_MAX_INDIVIDUAL_METRIC, metric);
        if((metric > 0) && (nodeCdtaInfo->bandSet.numActiveBands[rateIdx] < maxNumBands))
        {
          VbLogPrint(VB_LOG_DEBUG, "Metric (%ld / %u) > 0 while not all bands set (%u vs %u)", metric, allowed_SLA_xput, nodeCdtaInfo->bandSet.numActiveBands[rateIdx], maxNumBands);
          metric = 0;
        }
        nodeCdtaInfo->rateAdequationValue[rateIdx] = weight*metric;
      }
      else
      {
        VbLogPrint(VB_LOG_ERROR, "Capacity = 0");
      }

      if(nodeType == VB_NODE_DOMAIN_MASTER)
      {
        clusterCdtaInfo->metrics.sumWeightsDown[rateIdx] += weight;
      }
      else
      {
        clusterCdtaInfo->metrics.sumWeightsUp[rateIdx] += weight;
      }
    }
    else
    {
      nodeCdtaInfo->rateAdequationValue[rateIdx] = 0;
    }

    // Saturate to 0
    if(nodeCdtaInfo->rateAdequationValue[rateIdx] < 0)
    {
      nodeCdtaInfo->rateAdequationValue[rateIdx] = 0;
    }

    if(nodeType == VB_NODE_DOMAIN_MASTER)
    {
      clusterCdtaInfo->metrics.rateMetricsDown[rateIdx] += nodeCdtaInfo->rateAdequationValue[rateIdx];
    }
    else
    {
      clusterCdtaInfo->metrics.rateMetricsUp[rateIdx] += nodeCdtaInfo->rateAdequationValue[rateIdx];
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaGetBandCapacity(t_nodeCdtaInfo* nodeCdtaInfo, INT32U validBandsBitmap, INT16U numLinesBoosted, INT8U qosRateIdx, INT8U bandIdx, INT32U *cap, INT8U numNodes)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if( (nodeCdtaInfo == NULL) || (cap == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if( (bandIdx > 0) &&
        (validBandsBitmap & (1 << bandIdx)) &&
        (nodeCdtaInfo->bpsReportedNBands > bandIdx) &&
        (nodeCdtaInfo->bpsReportedCapacities[bandIdx][qosRateIdx] != 0))
    {
      // Get Capaciy reported in Traffic Report
      *cap = nodeCdtaInfo->bpsReportedCapacities[bandIdx][qosRateIdx];
    }
    else
    {
      // Trust Capacity from Engine computation
      if(numLinesBoosted > 1)
      {
        *cap = nodeCdtaInfo->bandCapacities[bandIdx][qosRateIdx].capacityAllBoosted;
        if((numLinesBoosted < numNodes) && (bandIdx > 0))
        {
          *cap += (nodeCdtaInfo->bandCapacities[bandIdx][qosRateIdx].capacity1Boosted - nodeCdtaInfo->bandCapacities[bandIdx][qosRateIdx].capacityAllBoosted)/(1<<numLinesBoosted);
        }
      }
      else
      {
        *cap = nodeCdtaInfo->bandCapacities[bandIdx][qosRateIdx].capacity1Boosted;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaBoostingAnalyseCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster          *cluster;

  if ((node == NULL) || (driver == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(driver->clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if(cluster->cdtaData == NULL)
      {
        ret = VB_ENGINE_ERROR_BAD_CLUSTER_ID;
      }
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    t_clusterCdtaInfo     *cluster_cdta_info;

    cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

    if ( (VbEngineDatamodelNodeIsReadyToBoost(node) == FALSE) ||
         ((node->type == VB_NODE_DOMAIN_MASTER) && (domain->eps.epsArray == NULL)) )
    {
      // No reports received or channel capacity not calculated -> node is not ready
      VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s not ready", node->MACStr);

      // Skip current node
      ret = VB_ENGINE_ERROR_SKIP;
    }
    else
    {
      INT32U n_lines_boosted;
      INT32U perc;
      INT32U capacity_current_band;
      INT16U allowed_SLA_xput;
      INT16U n_bands = ((t_cdtaAnalyseArgs *)(args))->nBands;
      INT8U  band_idx = ((t_cdtaAnalyseArgs *)(args))->bandIdx;
      INT8U  rate_idx = ((t_cdtaAnalyseArgs *)(args))->qosRate;
      INT8U  num_nodes = ((t_cdtaAnalyseArgs *)(args))->numNodes;

      if(band_idx == 0)
      {
        node->cdtaInfo.capacityPerRate[rate_idx]= 0;
        node->cdtaInfo.bandSet.forceNextTwoBandsBitmap[rate_idx]= 0;
        node->cdtaInfo.bandSet.numActiveBands[rate_idx] = 1;
        node->cdtaInfo.bandSet.skipBand = FALSE;
      }

      if(node->cdtaInfo.bandSet.skipBand == FALSE)
      {
        allowed_SLA_xput = (node->cdtaInfo.profile.userSLA != 0)?(MIN(node->cdtaInfo.profile.userSLA, node->trafficReports.neededL2Xput)):node->trafficReports.neededL2Xput;
        n_lines_boosted = (node->type == VB_NODE_DOMAIN_MASTER)?cluster_cdta_info->stats.nBoostedDownLines[band_idx][rate_idx]:cluster_cdta_info->stats.nBoostedUpLines[band_idx][rate_idx];

        ret = VbCdtaGetBandCapacity(&node->cdtaInfo, cluster_cdta_info->stats.bpsValidBandsBitmap, n_lines_boosted, rate_idx, band_idx, &capacity_current_band, num_nodes);
        if(ret == VB_ENGINE_ERROR_NONE)
        {
          node->cdtaInfo.capacityPerRate[rate_idx] += capacity_current_band;
          ret = VbEngineReqCapacityRatioCalc(allowed_SLA_xput, node->cdtaInfo.capacityPerRate[rate_idx], &perc);
          if(ret == VB_ENGINE_ERROR_NONE)
          {
            BOOLEAN enable_next_band = FALSE;
            INT32U thr_inc_add;
            INT32U thr_inc_rem;

            thr_inc_add = VbEngineConfBoostThrGet(VB_BOOST_THR_TYPE_INC_BOOST);
            thr_inc_rem = VbEngineConfBoostThrGet(VB_BOOST_THR_TYPE_DEC_BOOST);

            if( (node->channelSettings.boostInfo.forcedBandsBitmap & (1<<(band_idx+1))) ||
                (node->cdtaInfo.bandSet.forceNextTwoBandsBitmap[rate_idx] & (1<<(band_idx+1))) ||
                (perc > thr_inc_add) ||
                ((perc > thr_inc_rem) && (((node->channelSettings.boostInfo.level-1) > band_idx)))
              )
            {
              // Enable next band if band has been forced (Bad SNR or VDSL Coex)
              // Or Next band is needed (xput > bi * 0.85 -> Give next band)
              // Or next band is still needed due to remove band hysteresis (xput > bi * 0.70 and bi+1 is set)
              enable_next_band = TRUE;
            }

            if(enable_next_band == TRUE)
            {
              // Decide level of PSD increments
              // It is usually +1 but if TCP is "detected, it could be force to a +2
              if( (node->channelSettings.boostInfo.lastLevel == (band_idx+1)) &&
                  ((node->cdtaInfo.bandSet.forceNextTwoBandsBitmap[rate_idx] & (1<<(band_idx+1))) == 0) &&
                  (node->channelSettings.boostInfo.maxNumBands == VB_ENGINE_HIGH_GRANULARITY_PSD_MNGT) &&
                  ((node->linkedNode != NULL) && (node->linkedNode->trafficReports.neededL2Xput >= VB_ENGINE_CDTA_MIN_TCPACK_TRAFFIC)) &&
                  ((band_idx == 0) || (node->cdtaInfo.bandSet.tendency == 1)) )
              {
                // "Trick" to improve TCP reaction speed, Set 2 bands more under certain conditions:
                // Current band level = band idx
                // Band not already forced
                // SISO 200
                // Down & Up (TCP?)
                // Last band change was a band increase (or current band is low band)
                // -> if those conditions are matched
                //    Allocate band i+1 & band i+2
                node->cdtaInfo.bandSet.forceNextTwoBandsBitmap[rate_idx] |= (1<<(band_idx+2));
              }

              if(band_idx < (n_bands-1))
              {
                // Current band(s) is not enough, this node will have the next band boosted too
                (node->type == VB_NODE_DOMAIN_MASTER)?cluster_cdta_info->stats.nBoostedDownLines[band_idx+1][rate_idx]++:cluster_cdta_info->stats.nBoostedUpLines[band_idx+1][rate_idx]++;
              }

              // Sum up number of active bands in this rate
              n_bands = MIN(node->channelSettings.boostInfo.maxNumBands, n_bands);
              if(node->cdtaInfo.bandSet.numActiveBands[rate_idx] < n_bands)
              {
                node->cdtaInfo.bandSet.numActiveBands[rate_idx]++;
              }
            }
            else
            {
              node->cdtaInfo.bandSet.skipBand = TRUE;
            }
          }
        }
      } // Skip band
    }

    if (ret == VB_ENGINE_ERROR_SKIP)
    {
      // Expected error code. Skip current node
      ret = VB_ENGINE_ERROR_NONE;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaAdequationMetricFillCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster          *cluster;

  if ((node == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(driver->clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if(cluster->cdtaData == NULL)
      {
        ret = VB_ENGINE_ERROR_BAD_CLUSTER_ID;
      }
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    t_clusterCdtaInfo     *cluster_cdta_info;

    cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

    if ( (VbEngineDatamodelNodeIsReadyToBoost(node) == FALSE) ||
         ((node->type == VB_NODE_DOMAIN_MASTER) && (domain->eps.epsArray == NULL)) )
    {
      // No reports received or channel capacity not calculated -> node is not ready
      VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s not ready", node->MACStr);

      // Skip current node
      ret = VB_ENGINE_ERROR_SKIP;
    }
    else
    {
      INT32U i;

      for(i = VB_ENGINE_QOS_RATE_10_90; i < VB_ENGINE_QOS_RATE_90_10; i++)
      {
        // Fill metric for this Qos Rate
        VbCdtaAdequationMetricsSet(&node->cdtaInfo,
                                   cluster_cdta_info,
                                   node->trafficReports.neededL2Xput,
                                   node->type,
                                   node->channelSettings.boostInfo.maxNumBands,
                                   i);
      }

      if( (node->type == VB_NODE_DOMAIN_MASTER) && (node->trafficReports.neededL2Xput > VB_ENGINE_CDTA_MIN_TRAFFIC) )
      {
        cluster_cdta_info->stats.nDowns++;
        cluster_cdta_info->stats.aggregatedDowns += node->trafficReports.neededL2Xput;
      }
      else if ( (node->type == VB_NODE_END_POINT) && (node->trafficReports.neededL2Xput > VB_ENGINE_CDTA_MIN_TRAFFIC) )
      {
        cluster_cdta_info->stats.nUps++;
        cluster_cdta_info->stats.aggregatedUps += node->trafficReports.neededL2Xput;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current node
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaBpsValidBandsSetCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((node == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ( (VbEngineDatamodelNodeIsReadyToBoost(node) == FALSE) ||
         ((node->type == VB_NODE_DOMAIN_MASTER) && (domain->eps.epsArray == NULL)) )
    {
      // No reports received or channel capacity not calculated -> node is not ready
      VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s not ready", node->MACStr);

      // Skip current node
      ret = VB_ENGINE_ERROR_SKIP;
    }
    else
    {
      t_vbEngineBpsBandStatus *status =  &((t_cdtaAnalyseArgs *)(args))->bandStatus;
      INT8U  band_idx = ((t_cdtaAnalyseArgs *)(args))->bandIdx;

      if(*status != VB_ENGINE_BPS_BAND_INVALID)
      {
        if((node->channelSettings.boostInfo.level > band_idx) && (node->channelSettings.boostInfo.bandsAge[band_idx]> VB_ENGINE_AGE_TO_TRUST_BPS_REPORT))
        {
          *status = VB_ENGINE_BPS_BAND_VALID;
        }
        else if(node->channelSettings.boostInfo.level > band_idx)
        {
          // Not enough stable yet
          *status = VB_ENGINE_BPS_BAND_INVALID;
        }
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current node
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaInfoPreUpdateCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((node == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if(node->channelSettings.boostInfo.lastLevel < node->channelSettings.boostInfo.level)
    {
      node->cdtaInfo.bandSet.tendency = 1;
    }
    else if(node->channelSettings.boostInfo.lastLevel > node->channelSettings.boostInfo.level)
    {
      node->cdtaInfo.bandSet.tendency = 0;
    }

    node->channelSettings.boostInfo.lastLevel = node->channelSettings.boostInfo.level;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaInfoUpdateCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster          *cluster;

  if ((node == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(driver->clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if(cluster->cdtaData == NULL)
      {
        ret = VB_ENGINE_ERROR_BAD_CLUSTER_ID;
      }
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    t_clusterCdtaInfo     *cluster_cdta_info;

    cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

    if(node->cdtaInfo.bandSet.numActiveBands[cluster_cdta_info->finalQosRate] < node->channelSettings.boostInfo.level)
    {
      // Could reduce number of bands in this settings
      if(node->cdtaInfo.hysteresisCounter < VB_ENGINE_CDTA_HYSTERESIS)
      {
        // Force to use current number of bands during hysteresis time
        node->cdtaInfo.bandSet.numActiveBands[cluster_cdta_info->finalQosRate] = node->channelSettings.boostInfo.level;
        node->cdtaInfo.hysteresisCounter++;
      }
      else
      {
        node->cdtaInfo.hysteresisCounter = 0;
      }
    }
    else
    {
      node->cdtaInfo.hysteresisCounter = 0;
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U i;

    for(i = 1; i<node->channelSettings.boostInfo.maxNumBands ; i++)
    {
      if((i < node->channelSettings.boostInfo.lastLevel) && (i < node->channelSettings.boostInfo.level))
      {
        node->channelSettings.boostInfo.bandsAge[i]++;
      }
      else
      {
        node->channelSettings.boostInfo.bandsAge[i] = 0;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbCdtaAdequationMetricMinSearch(t_clusterCdtaInfo *clusterCdtaInfo, t_vbEngineQosRate *qosRateToApply)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  INT32U i;
  INT32S min = MAX_INT32S;
  INT32U min_idx = 0;
  INT32U min_idx_1 = 0;
  INT32U min_idx_2 = 0;
  INT32U n_consec_0 = 0;

  if((qosRateToApply == NULL) || (clusterCdtaInfo == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // No traffic, set to 70-30
    if((clusterCdtaInfo->stats.nUps == 0) && (clusterCdtaInfo->stats.nDowns == 0))
    {
      *qosRateToApply = VB_ENGINE_QOS_RATE_70_30;
    }
    else
    {
      INT32U sum_slas    = 0;
      INT32U weight_down = 0;
      INT32U weight_up   = 0;
      INT32U current_metric   = 0;
      BOOLEAN write_hist = FALSE;

      // Up & Down, get min metrics
      weight_down =  (METRIC_MULT_FACTOR_UP_DOWN(cdtaGlobalConf.weights.downPref, cdtaGlobalConf.weights.downUpMultFactor));
      current_metric = clusterCdtaInfo->selectedMetrics ;
      weight_up =    (METRIC_MULT_FACTOR_UP_DOWN(cdtaGlobalConf.weights.upPref, cdtaGlobalConf.weights.downUpMultFactor));

      pthread_mutex_lock(&(clusterCdtaInfo->hist.mutex));
      for(i=cdtaGlobalConf.minQosRate;i<=cdtaGlobalConf.maxQosRate;i++)
      {
        sum_slas = clusterCdtaInfo->metrics.sumWeightsDown[i] + clusterCdtaInfo->metrics.sumWeightsUp[i];
        if(sum_slas == 0)
        {
          sum_slas = 1;
        }

        clusterCdtaInfo->metrics.rateMetrics[i] = weight_down*(clusterCdtaInfo->metrics.rateMetricsDown[i]/sum_slas) +
                                                 weight_up*(clusterCdtaInfo->metrics.rateMetricsUp[i]/sum_slas);

        if(clusterCdtaInfo->metrics.rateMetrics[i] < min)
        {
          min = clusterCdtaInfo->metrics.rateMetrics[i];
          min_idx = i;
        }

        if (clusterCdtaInfo->metrics.rateMetrics[i] == 0)
        {
          if(n_consec_0++ == 0)
          {
            min_idx_1 = i;
          }

          min_idx_2 = i;
        }

        if(clusterCdtaInfo->hist.wPointer < VB_ENGINE_NUM_HIST_DATA)
        {
          if (clusterCdtaInfo->metrics.rateMetrics[i] != clusterCdtaInfo->hist.data[clusterCdtaInfo->hist.wPointer].metrics[i])
          {
            if(write_hist == FALSE)
            {
              INT32U w_ptr = clusterCdtaInfo->hist.wPointer;

              clusterCdtaInfo->hist.wPointer = ((++w_ptr) % VB_ENGINE_NUM_HIST_DATA);
              write_hist = TRUE;
            }
          }
        }
      }

      if(write_hist == TRUE)
      {
        struct timespec     ts;

        clock_gettime(CLOCK_REALTIME, &ts);
        clusterCdtaInfo->hist.data[clusterCdtaInfo->hist.wPointer].timeStamp = ts;
        memcpy(clusterCdtaInfo->hist.data[clusterCdtaInfo->hist.wPointer].metrics,
            clusterCdtaInfo->metrics.rateMetrics,
               (sizeof(clusterCdtaInfo->hist.data[clusterCdtaInfo->hist.wPointer].metrics[0])*VB_ENGINE_QOS_RATE_LAST));
      }

      pthread_mutex_unlock(&(clusterCdtaInfo->hist.mutex));

      if(min == 0)
      {
        // System can handle requested Xputs
        // Get first idx of metrics 0 and last index of metric 0 and pick the one in the middle
        if(n_consec_0 > 1)
        {
          // Down & Up have non null weight
          if( (cdtaGlobalConf.weights.downPref > 0) && (cdtaGlobalConf.weights.upPref > 0) )
          {
            if(clusterCdtaInfo->stats.nUps == 0)
            {
              // No up traffic, round up
              min_idx = ((min_idx_1 + min_idx_2)/2) + 1;
            }
            else
            {
              min_idx = ((min_idx_1 + min_idx_2)/2);
            }
          }
          else if(cdtaGlobalConf.weights.upPref == 0)
          {
            INT32U min_idx_aux = 0;

            // Up weight = 0, Give to Down first and then the best you can to Up (If there is up traffic)
            ret = VbCdtaMinUpGet(min_idx_1, min_idx_2, clusterCdtaInfo->metrics.rateMetricsUp, &min_idx_aux);

            if((ret == VB_ENGINE_ERROR_NONE) && (clusterCdtaInfo->stats.nUps > 0))
            {
              min_idx = min_idx_aux;
            }
            else
            {
              // No up traffic, round up
              min_idx = ((min_idx_1 + min_idx_2)/2) + 1;
            }
          }
          else if(cdtaGlobalConf.weights.downPref == 0)
          {
            INT32U min_idx_aux = 0;

            // Down weight = 0, Give to Up first and then the best you can to Down (If there is up traffic)
            ret = VbCdtaMinDownGet(min_idx_1, min_idx_2, clusterCdtaInfo->metrics.rateMetricsDown, &min_idx_aux);
            if( (ret == VB_ENGINE_ERROR_NONE) && (clusterCdtaInfo->stats.nDowns > 0) )
            {
              // Down weight = 0, Give to Up first and then the best you can to Down
              min_idx = min_idx_aux;
            }
            else
            {
              // No down traffic traffic, round
              min_idx = ((min_idx_1 + min_idx_2)/2);
            }
          }
        }
      }

      if( (min_idx >= cdtaGlobalConf.minQosRate) && (min_idx <= cdtaGlobalConf.maxQosRate) )
      {
        INT32U perc;

        // See if the change is worth it
        if(min > 0)
        {
          ret = VbEngineReqCapacityRatioCalc(current_metric, min, &perc);
          if( (perc >= (100 + cdtaGlobalConf.percMetricChange)) || (perc <= (100 - cdtaGlobalConf.percMetricChange)) )
          {
            // New metric is percMetricChange% different than the previous one -> change
            // Worth the change
            clusterCdtaInfo->selectedMetrics = clusterCdtaInfo->metrics.rateMetrics[min_idx];
            *qosRateToApply = min_idx;
          }
          else
          {
            if( (clusterCdtaInfo->finalQosRate >= cdtaGlobalConf.minQosRate) && (clusterCdtaInfo->finalQosRate  <= cdtaGlobalConf.maxQosRate) )
            {
              // Not worth changing, remain in current Qos Rate
              clusterCdtaInfo->selectedMetrics = clusterCdtaInfo->metrics.rateMetrics[clusterCdtaInfo->finalQosRate];
              *qosRateToApply = clusterCdtaInfo->finalQosRate;
              VbCounterIncrease(VB_ENGINE_COUNTER_CDTA_FORCE_NO_CHANGE);
            }
            else
            {
              clusterCdtaInfo->selectedMetrics = clusterCdtaInfo->metrics.rateMetrics[min_idx];
              *qosRateToApply = min_idx;
            }
          }
        }
        else
        {
          // min is 0, if previous min was 0 too and near new min index, do not move to avoid potential continious oscillation
          // the typical case is
          // 40/60 50/50 60/40 70/30
          //  0     0      0     0  -> Qos rate 5
          //  x     0      0     0  -> Qos rate 6
          //  0     0      0     0  -> Qos rate 5
          //  x     0      0     0  -> Qos rate 6
          // The x in 40/60 is from a node with BPS vs capcity from Measure plan discrepancy
          // (All Boosted differs from BPS reported when not all nodes are interfering))
          if((clusterCdtaInfo->selectedMetrics == 0) && (ABS_DIFF(min_idx, clusterCdtaInfo->finalQosRate) == 1))
          {
            clusterCdtaInfo->selectedMetrics = clusterCdtaInfo->metrics.rateMetrics[clusterCdtaInfo->finalQosRate];
            *qosRateToApply = clusterCdtaInfo->finalQosRate;
            VbCounterIncrease(VB_ENGINE_COUNTER_CDTA_FORCE_NO_CHANGE);
          }
          else
          {
            clusterCdtaInfo->selectedMetrics = clusterCdtaInfo->metrics.rateMetrics[min_idx];
            *qosRateToApply = min_idx;
          }
        }
      }
      else
      {
        *qosRateToApply = VB_ENGINE_QOS_RATE_50_50;
      }

      clusterCdtaInfo->hist.data[clusterCdtaInfo->hist.wPointer].rate = *qosRateToApply;
    }

#if VB_ENGINE_METRICS_ENABLED
    if(clusterCdtaInfo->finalQosRate != *qosRateToApply)
    {
      VbMetricsReportDeviceEvent(VB_METRICS_EVENT_UD_RATE_CHANGE, NULL,
          VB_NODE_DOMAIN_MASTER, NULL, 0, *qosRateToApply);
    }
#endif

    clusterCdtaInfo->finalQosRate = *qosRateToApply;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineCDTADriverCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_psdShapeArgs       shape_args = {0, 0, NULL, NULL};
  t_vbEngineNumNodes   num_nodes;

  if ((driver == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get number of lines in driver
    ret = VbEngineDatamodelNumNodesInDriverGet(driver, &num_nodes);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check that number of complete lines in driver is not 0
    if (driver->FSMState == ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY)
    {
      // Skip drivers in "ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY" state or with no complete lines
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "No complete lines detected -> Skip sending EACDTAShape.req");
      ret = VB_ENGINE_ERROR_SKIP;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Add room for header
    shape_args.length = VB_EA_CDTA_REQ_HDR_SIZE;

    // Loop through all nodes to calculate message length
    ret = VbEngineDatamodelNodesLoop(driver, VbEnginePSDShapeLengthCalcCb, &shape_args);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for message
    shape_args.payload = calloc(1, shape_args.length);

    if (shape_args.payload == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Send CDTA to %u nodes", shape_args.numPsds);

    // Update pointer to start writing PSDs
    shape_args.ptrToWrite = shape_args.payload + VB_EA_CDTA_REQ_HDR_SIZE;

    // Loop through all nodes to build the message
    ret = VbEngineDatamodelNodesLoop(driver, VbEnginePSDShapeBuildCb, &shape_args);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_vbEACDTAHdr             *psd_hdr;
    INT16U                    seq_num =                  ((t_cdtaMsgBuildArgs*)args)->seqNum;
    BOOLEAN                   force_channel_estimation = ((t_cdtaMsgBuildArgs*)args)->forceChannelEstimation;
    t_vbEngineQosRate         qos_rate                 = ((t_cdtaMsgBuildArgs*)args)->qosRate;
    INT32U                    idx;
    t_psdBandAllocation      *psd_band_allocation;

    psd_band_allocation = VbEngineConfPSDBandAllocationGet();
    if(psd_band_allocation == NULL)
    {
      ret = VB_ENGINE_ERROR_UNKNOWN;
    }

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Fill PSD Shape header
      psd_hdr = (t_vbEACDTAHdr *)shape_args.payload;
      psd_hdr->numNodes = shape_args.numPsds;
      psd_hdr->seqNumber = _htons(seq_num);
      psd_hdr->qosRate = qos_rate;
      psd_hdr->defaultQosRate = cdtaGlobalConf.defaultQosRate;
      psd_hdr->forceChannelEstimation = force_channel_estimation;

      psd_hdr->numBandsFirstPSD = 1;
      psd_hdr->nBoostBands = psd_band_allocation->numBands200Mhz;
      for(idx=0; idx < MAX(VB_PSD_NUM_BANDS, psd_band_allocation->numBands200Mhz); idx++)
      {
        psd_hdr->bandCarrierDef[idx] = _htons(psd_band_allocation->lastCarrier[idx]);
      }

      // Send EA message
      ret = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_CDTA_CFG,
          shape_args.payload,
          shape_args.length,
          driver);
    }
  }

  if (shape_args.payload != NULL)
  {
    free(shape_args.payload);
    shape_args.payload = NULL;
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current driver
    ret = VB_ENGINE_ERROR_NONE;
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    if (driver != NULL)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error %d sending new CDTA", ret);
    }
  }
  else
  {
    VbCounterIncrease(VB_ENGINE_COUNTER_CDTA_CFG);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineCdtaCnfNotifyLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (node == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    node->channelSettings.psdShape.sendUpdate = FALSE;
    node->channelSettings.boostInfo.levelCnf = node->channelSettings.boostInfo.level;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineCdtaCnfNotify(t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (thisDriver != NULL)
  {
    // Loop through all nodes of given driver to notify Psd Shape confirmation
    ret = VbEngineDatamodelNodesLoop(thisDriver, VbEngineCdtaCnfNotifyLoopCb, NULL);
    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Error %d notifying for PSD shape confirmation", ret);
    }
  }
  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineCdtaInfoFill(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  INT32U band_idx;
  INT32U rate_idx;
  INT8U  n_bands;
  t_vbEngineNumNodes        num_nodes;
  t_cdtaAnalyseArgs args;

  n_bands = VbEngineConfNumPSDBandAllocationGet(VB_TX_MODE_200_MHZ);
  ret = VbEngineDataModelNumNodesGet(&num_nodes);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    args.nBands = n_bands;
    args.numNodes = num_nodes.numCompleteLines;
    for(rate_idx = VB_ENGINE_QOS_RATE_10_90; rate_idx < VB_ENGINE_QOS_RATE_90_10; rate_idx++)
    {
      args.qosRate = rate_idx;
      for(band_idx=0; band_idx < n_bands; band_idx++)
      {
        args.bandIdx = band_idx;
        ret = VbEngineDatamodelClusterXAllNodesLoop(VbCdtaBoostingAnalyseCb, clusterId, &args);
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineCdtaBpsReportValidInfoFill(t_clusterCdtaInfo *clusterCdtaInfo, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U band_idx;
  INT8U  n_bands;
  t_cdtaAnalyseArgs args;

  if (clusterCdtaInfo == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    n_bands = VbEngineConfNumPSDBandAllocationGet(VB_TX_MODE_200_MHZ);
    clusterCdtaInfo->stats.bpsValidBandsBitmap = 1;
    for(band_idx=1; band_idx < n_bands; band_idx++)
    {
      args.bandIdx = band_idx;
      args.bandStatus = VB_ENGINE_BPS_BAND_NOT_AVAILABLE;
      ret = VbEngineDatamodelClusterXAllNodesLoop(VbCdtaBpsValidBandsSetCb, clusterId, &args);
      if(args.bandStatus == VB_ENGINE_BPS_BAND_VALID)
      {
        clusterCdtaInfo->stats.bpsValidBandsBitmap |= (1<<band_idx);
      }
    }
  }

  return ret;
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_VB_engineErrorCode VbCdtaAnalyseRun(t_vbEngineQosRate *qosRate, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_clusterCdtaInfo     *cluster_cdta_info;
  t_VBCluster          *cluster;
  if(qosRate == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if(cluster->cdtaData == NULL)
      {
        ret = VB_ENGINE_ERROR_BAD_CLUSTER_ID;
      }
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

    // Reset CDTA Decision data
    VbCdtaGlobalInfoReset(cluster_cdta_info);

    // Update cdtaNodeInfo related to band set information
    ret = VbEngineDatamodelClusterXAllNodesLoop(VbCdtaInfoPreUpdateCb, clusterId, NULL);

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Update bps valid bands info
      ret = VbEngineCdtaBpsReportValidInfoFill(cluster_cdta_info, clusterId);
    }

    if(ret == VB_ENGINE_ERROR_NONE)
    {
     // Analyse traffic needed vs Qos Rate & Boosting need for all nodes reporting traffics
     ret = VbEngineCdtaInfoFill(clusterId);
    }

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Analyse traffic needed vs current Max possible traffic for all nodes reporting traffics
      ret = VbEngineDatamodelClusterXAllNodesLoop(VbCdtaAdequationMetricFillCb, clusterId, NULL);
    }

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      if(VbEngineConfCDTAEnableGet() == TRUE)
      {
        // Sum Down & Up and look for min
        ret = VbCdtaAdequationMetricMinSearch(cluster_cdta_info, qosRate);
      }
      else
      {
        *qosRate = cdtaGlobalConf.defaultQosRate;
        cluster_cdta_info->finalQosRate = cdtaGlobalConf.defaultQosRate;
      }

      if(ret == VB_ENGINE_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_DEBUG,"CDTA SumDownUp: %5d %5d %5d %5d %5d %5d %5d\n",
                   cluster_cdta_info->metrics.rateMetrics[2],
                   cluster_cdta_info->metrics.rateMetrics[3],
                   cluster_cdta_info->metrics.rateMetrics[4],
                   cluster_cdta_info->metrics.rateMetrics[5],
                   cluster_cdta_info->metrics.rateMetrics[6],
                   cluster_cdta_info->metrics.rateMetrics[7],
                   cluster_cdta_info->metrics.rateMetrics[8]);

        // Update cdtaNodeInfo variable once the qosRate is known
        ret = VbEngineDatamodelClusterXAllNodesLoop(VbCdtaInfoUpdateCb, clusterId, NULL);
      }
      else
      {
        VbLogPrint(VB_LOG_ERROR,"CDTA Metrics Error: %d\n", ret);
      }

      VbLogPrint(VB_LOG_DEBUG,"CDTA QosRate: %u\n", cluster_cdta_info->finalQosRate);
    }
  }

  if(ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR,"CDTA VbCdtaAnalyseRun Error: %d\n", ret);
  }


  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineCDTAConfigureAll(BOOLEAN forceChannelEstimation, t_vbEngineQosRate nextQosRate, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  struct timespec      apply_ts;
  INT16U               apply_seq_num;

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get a future time
    ret = VbEngineClockFutureTSGet(NULL, clusterId, &apply_seq_num, &apply_ts);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_cdtaMsgBuildArgs args;

    // Loop through all drivers to build and send CDTA message
    args.seqNum = apply_seq_num;
    args.forceChannelEstimation = forceChannelEstimation;
    args.qosRate = nextQosRate;
    ret = VbEngineDatamodelClusterXDriversLoop(VbEngineCDTADriverCb, clusterId, &args);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    CHAR apply_ts_str[TIMESPEC_STR_LEN];

    // Convert TS to string
    VbUtilTimespecToString(apply_ts_str, apply_ts);

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "PSD Shape sent to all drivers : applyTS %s; applySeqNum %u",
        apply_ts_str, apply_seq_num);
  }

  return ret;
}


/*******************************************************************/

t_VB_engineErrorCode VbCdtaGlobalInfoDump(t_writeFun writeFun, INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  INT32U i;
  t_clusterCdtaInfo *cluster_cdta_info;
  t_VBCluster *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Check previous allocated memory
    if(cluster->cdtaData == NULL)
    {
      ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

    writeFun("nDowns        : %u\n", cluster_cdta_info->stats.nDowns);
    writeFun("nUps          : %u\n", cluster_cdta_info->stats.nUps);

    writeFun("finalQosRate  : %u\n", cluster_cdta_info->finalQosRate);
    writeFun("minQosRate    : %u\n", cdtaGlobalConf.minQosRate);
    writeFun("maxQosRate    : %u\n", cdtaGlobalConf.maxQosRate);
    writeFun("DefaultQosRate: %u\n", cdtaGlobalConf.defaultQosRate);

    writeFun("aggredated Down      : %u\n", cluster_cdta_info->stats.aggregatedDowns);
    writeFun("aggregated Up        : %u\n", cluster_cdta_info->stats.aggregatedUps);
    writeFun("max Down             : %u\n", cluster_cdta_info->stats.maxDown);
    writeFun("max Up               : %u\n", cluster_cdta_info->stats.maxUp);
    writeFun("Bitmap BpsReport     : %lx\n", cluster_cdta_info->stats.bpsValidBandsBitmap);
    writeFun("PercMetricChange     : %u\n", cdtaGlobalConf.percMetricChange);

    writeFun("Down Weight          : %u\n", cdtaGlobalConf.weights.downPref);
    writeFun("Up Weight            : %u\n", cdtaGlobalConf.weights.upPref);
    writeFun("Mult Down Up         : %u\n", cdtaGlobalConf.weights.downUpMultFactor);
    writeFun("Sum Weight Down      : %u\n", cluster_cdta_info->metrics.sumWeightsDown[cluster_cdta_info->finalQosRate]);
    writeFun("Sum Weight Up        : %u\n", cluster_cdta_info->metrics.sumWeightsUp[cluster_cdta_info->finalQosRate]);

    writeFun("\n Rate          :   28     37     46     55     64     73     82\n");
    writeFun("-----------------------------------------------------------------\n");

    for(i=0; i< VbEngineConfNumPSDBandAllocationGet(VB_TX_MODE_200_MHZ); i++)
    {
      writeFun("NbandsDown (%1u) : %5d  %5d  %5d  %5d  %5d  %5d  %5d\n", i,
          cluster_cdta_info->stats.nBoostedDownLines[i][2],
          cluster_cdta_info->stats.nBoostedDownLines[i][3],
          cluster_cdta_info->stats.nBoostedDownLines[i][4],
          cluster_cdta_info->stats.nBoostedDownLines[i][5],
          cluster_cdta_info->stats.nBoostedDownLines[i][6],
          cluster_cdta_info->stats.nBoostedDownLines[i][7],
          cluster_cdta_info->stats.nBoostedDownLines[i][8]);
    }

    writeFun("-----------------------------------------------------------------\n");

    for(i=0; i< VbEngineConfNumPSDBandAllocationGet(VB_TX_MODE_200_MHZ); i++)
    {
      writeFun("NbandsUp   (%1u) : %5d  %5d  %5d  %5d  %5d  %5d  %5d\n", i,
          cluster_cdta_info->stats.nBoostedUpLines[i][2],
          cluster_cdta_info->stats.nBoostedUpLines[i][3],
          cluster_cdta_info->stats.nBoostedUpLines[i][4],
          cluster_cdta_info->stats.nBoostedUpLines[i][5],
          cluster_cdta_info->stats.nBoostedUpLines[i][6],
          cluster_cdta_info->stats.nBoostedUpLines[i][7],
          cluster_cdta_info->stats.nBoostedUpLines[i][8]);
    }

    writeFun("\nRate        :|     28    |     37    |     46    |     55    |     64    |     73    |     82    |\n");
    writeFun("----------------------------------------------------------------------------------------------------\n");

    writeFun("CDTA SumDown:  %7d     %7d     %7d     %7d     %7d     %7d     %7d  \n",
        cluster_cdta_info->metrics.rateMetricsDown[2],
        cluster_cdta_info->metrics.rateMetricsDown[3],
        cluster_cdta_info->metrics.rateMetricsDown[4],
        cluster_cdta_info->metrics.rateMetricsDown[5],
        cluster_cdta_info->metrics.rateMetricsDown[6],
        cluster_cdta_info->metrics.rateMetricsDown[7],
        cluster_cdta_info->metrics.rateMetricsDown[8]);

    writeFun("CDTA SumUp  :  %7d     %7d     %7d     %7d     %7d     %7d     %7d  \n",
        cluster_cdta_info->metrics.rateMetricsUp[2],
        cluster_cdta_info->metrics.rateMetricsUp[3],
        cluster_cdta_info->metrics.rateMetricsUp[4],
        cluster_cdta_info->metrics.rateMetricsUp[5],
        cluster_cdta_info->metrics.rateMetricsUp[6],
        cluster_cdta_info->metrics.rateMetricsUp[7],
        cluster_cdta_info->metrics.rateMetricsUp[8]);

    writeFun("\nRate        :|     28    |     37    |     46    |     55    |     64    |     73    |     82    |\n");
    writeFun("CDTA        :  %7d     %7d     %7d     %7d     %7d     %7d     %7d  \n",
        cluster_cdta_info->metrics.rateMetrics[2],
        cluster_cdta_info->metrics.rateMetrics[3],
        cluster_cdta_info->metrics.rateMetrics[4],
        cluster_cdta_info->metrics.rateMetrics[5],
        cluster_cdta_info->metrics.rateMetrics[6],
        cluster_cdta_info->metrics.rateMetrics[7],
        cluster_cdta_info->metrics.rateMetrics[8]);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbCdtaGlobalHistInfoDump(t_writeFun writeFun, INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  INT32U i;
  struct tm tmu;
  t_clusterCdtaInfo *cluster_cdta_info;
  t_VBCluster *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Check previous allocated memory
    if(cluster->cdtaData == NULL)
    {
      ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

    pthread_mutex_lock(&(cluster_cdta_info->hist.mutex));

    writeFun("\n Rate          :             28        37        46        55        64        73        82    -\n");
    writeFun("--------------------------------------------------------------------------------------------------\n");

    for(i=0; i<VB_ENGINE_NUM_HIST_DATA; i++)
    {
      localtime_r(&(cluster_cdta_info->hist.data[i].timeStamp.tv_sec), &tmu);

      writeFun("(%2u)/%2d:%02d:%02d %03dms  : %8d  %8d  %8d  %8d  %8d  %8d  %8d - [%02d]\n", i,
                tmu.tm_hour, tmu.tm_min, tmu.tm_sec,
                (int)(cluster_cdta_info->hist.data[i].timeStamp.tv_nsec/1000000),
                cluster_cdta_info->hist.data[i].metrics[2],
                cluster_cdta_info->hist.data[i].metrics[3],
                cluster_cdta_info->hist.data[i].metrics[4],
                cluster_cdta_info->hist.data[i].metrics[5],
                cluster_cdta_info->hist.data[i].metrics[6],
                cluster_cdta_info->hist.data[i].metrics[7],
                cluster_cdta_info->hist.data[i].metrics[8],
                cluster_cdta_info->hist.data[i].rate
             );
    }

    writeFun("--------------------------------------------------------------------------------------------------\n");
    pthread_mutex_unlock(&(cluster_cdta_info->hist.mutex));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineCDTAConfigurationParse( ezxml_t cdta_conf )
{
  t_VB_engineErrorCode err_code;
  ezxml_t              ez_temp;
  t_cdtaConf           *cdta_conf_data = NULL;

  err_code = VbEngineConfCDTADataGet(&cdta_conf_data);

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    if (cdta_conf == NULL)
    {
      err_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(cdta_conf, "Enable");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      cdta_conf_data->enabled =  (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Enabled value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading Enabled parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(cdta_conf, "DownUpWeight");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      cdta_conf_data->downUpWeight = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid DownUpWeight value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
      else
      {
        if(cdta_conf_data->downUpWeight <= VB_ENGINE_MAX_DOWN_UP_WEIGHT)
        {
          cdtaGlobalConf.weights.downPref = cdta_conf_data->downUpWeight;
          cdtaGlobalConf.weights.upPref = VB_ENGINE_MAX_DOWN_UP_WEIGHT - cdta_conf_data->downUpWeight;
          cdtaGlobalConf.weights.downUpMultFactor = 0;
        }
        else
        {
          printf("ERROR (%d:%s) parsing .ini file: Invalid DownUpWeight value\n", errno, strerror(errno));
          err_code = VB_ENGINE_ERROR_INI_FILE;
        }
      }
    }
    else
    {
      printf("Engine Conf: Error reading DownUpWeight parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(cdta_conf, "MinDownUpRate");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      cdta_conf_data->minDownUpRate = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid MinDownUpRate value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
      else
      {
        cdtaGlobalConf.minQosRate = cdta_conf_data->minDownUpRate;
      }
    }
    else
    {
      printf("Engine Conf: Error reading MinDownUpRate parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(cdta_conf, "MaxDownUpRate");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      cdta_conf_data->maxDownUpRate = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid MaxDownUpRate value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
      else
      {
        cdtaGlobalConf.maxQosRate = cdta_conf_data->maxDownUpRate;
      }
    }
    else
    {
      printf("Engine Conf: Error reading MaxDownUpRate parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(cdta_conf, "DefaultDownUpRate");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      cdta_conf_data->defaultDownUpRate = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid DefaultDownUpRate value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
      else
      {
        cdtaGlobalConf.defaultQosRate = cdta_conf_data->defaultDownUpRate;
        if(cdta_conf_data->enabled == FALSE)
        {
          // CDTA disabled, max = min
          cdtaGlobalConf.maxQosRate = cdtaGlobalConf.defaultQosRate;
          cdtaGlobalConf.minQosRate = cdtaGlobalConf.defaultQosRate;
        }
      }
    }
    else
    {
      printf("Engine Conf: Error reading DefaultDownUpRate parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(cdta_conf, "percMetricChange");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      cdta_conf_data->percMetricChange = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid percMetricChange value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
      else
      {
        cdtaGlobalConf.percMetricChange = cdta_conf_data->percMetricChange;
      }
    }
    else
    {
      printf("Engine Conf: Error reading percMetricChange parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  return err_code;
}

/*******************************************************************/

t_vbEngineQosRate VbCdtaQosRateGet(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  t_clusterCdtaInfo   *cluster_cdta_info = NULL;
  t_VBCluster         *cluster;
  t_vbEngineQosRate    qos_rate = VB_ENGINE_QOS_RATE_70_30;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Check previous allocated memory
    if(cluster->cdtaData == NULL)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "QosRate Invalid cluster Id %lu", clusterId);
      ret = VB_ENGINE_ERROR_MEASURE_PLAN;
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;
    qos_rate = cluster_cdta_info->finalQosRate;
  }

  return qos_rate;
}

/*******************************************************************/

t_VB_engineErrorCode VbCdtaQosRateForce(INT32U clusterId, INT8U qosRate)
{
  t_VB_engineErrorCode ret;
  t_clusterCdtaInfo   *cluster_cdta_info = NULL;
  t_VBCluster         *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if(cluster->cdtaData != NULL)
    {
      cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;
      cluster_cdta_info->finalQosRate = qosRate;
    }
    else
    {
      ret = VB_ENGINE_ERROR_BAD_CLUSTER_ID;
    }
  }

  return ret;
}

/*******************************************************************/

t_vbEngineQosRate VbCdtaDefaultQosRateGet(void)
{
  return cdtaGlobalConf.defaultQosRate;
}

/*******************************************************************/

void VbCdtaWeightDownUpMultFactorSet(INT16U multFactor)
{
  cdtaGlobalConf.weights.downUpMultFactor = multFactor;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineCdtaRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEACdtaCnf *cdta_cnf = (t_vbEACdtaCnf *)payload;

  if ((cdta_cnf == NULL) || (thisDriver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    if (cdta_cnf->status == VB_EA_PSD_SHAPE_STATUS_OK)
    {
      // Notify PSD Shape confirmation
      VbEngineCdtaCnfNotify(thisDriver);
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "PSD shape confirmation error");
    }
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineCdtaReportedBpsUpdate(t_node *node, t_trafficReport *trafficReport, t_nodeType nodeType)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;

  if ((node == NULL) || (trafficReport == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    INT32U band_idx;
    INT32U rate_idx;
    INT32U capacity;
    INT32U eff;

    node->cdtaInfo.bpsReportedNBands = trafficReport->nBandsBps;
    memset((INT8U *)node->cdtaInfo.bpsReportedCapacities, 0x0, sizeof(node->cdtaInfo.bpsReportedCapacities));

    for(band_idx = 0; band_idx < node->channelSettings.boostInfo.maxNumBands; band_idx++)
    {
      for(rate_idx = VB_ENGINE_QOS_RATE_10_90; rate_idx<VB_ENGINE_QOS_RATE_LAST; rate_idx++)
      {
        eff = (nodeType == VB_NODE_DOMAIN_MASTER)?((rate_idx*10)-VB_ENGINE_STXOP_LOSS_EFF):((100 - (rate_idx*10))-VB_ENGINE_STXOP_LOSS_EFF);
        if(band_idx == 0)
        {
          node->cdtaInfo.bpsReportedCapacities[band_idx][rate_idx] = (trafficReport->bpsBand[band_idx]*eff)/100;
          if ( ( node->cdtaInfo.bpsReportedCapacities[band_idx][rate_idx] > 0) )
          {
            node->cdtaInfo.bandCapacities[band_idx][rate_idx].capacityAllBoosted = node->cdtaInfo.bpsReportedCapacities[band_idx][rate_idx];
          }
        }
        else
        {
          if(trafficReport->bpsBand[band_idx] > trafficReport->bpsBand[band_idx-1])
          {
            capacity = trafficReport->bpsBand[band_idx] - trafficReport->bpsBand[band_idx-1];
            node->cdtaInfo.bpsReportedCapacities[band_idx][rate_idx] = (capacity*eff)/100;
          }
          else
          {
            node->cdtaInfo.bpsReportedCapacities[band_idx][rate_idx] = 0;
          }
        }
      }
    }
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineCdtaClusterResourcesAlloc(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  t_VBCluster          *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Check previous allocated memory
    if (cluster->cdtaData != NULL)
    {
      free(cluster->cdtaData);
      cluster->cdtaData = NULL;
    }

    cluster->cdtaData = (INT8U *)calloc(1, sizeof(t_clusterCdtaInfo));
    if(cluster->cdtaData == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    t_clusterCdtaInfo *cluster_cdta_info = (t_clusterCdtaInfo *)cluster->cdtaData;

    bzero(&cluster_cdta_info->stats, sizeof(cluster_cdta_info->stats));
    bzero(&cluster_cdta_info->hist, sizeof(cluster_cdta_info->hist));
    bzero(&cluster_cdta_info->metrics, sizeof(cluster_cdta_info->metrics));

    pthread_mutex_init(&(cluster_cdta_info->hist.mutex), NULL);

  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineCdtaFreeClusterResources(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  t_VBCluster          *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Check previous allocated memory
    if (cluster->cdtaData != NULL)
    {
      free(cluster->cdtaData);
      cluster->cdtaData = NULL;
    }
  }

  return ret;
}


/**
 * @}
 **/
