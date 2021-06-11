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
 * @file vb_engine_l2rPSD_calculation.c
 * @brief PSD Left to right algorithm implementation
 *
 * @internal
 *
 * @author
 * @date 19/02/2015
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <stdlib.h>

#include "vb_engine_drivers_list.h"
#include "vb_log.h"
#include "vb_engine_l2rPSD_calculation.h"
#include "vb_counters.h"
#include "vb_util.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_conf.h"
#include "vb_engine_process.h"
#include "vb_engine_cdta.h"
#include "vb_measure_utils.h"

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

#define VB_ENGINE_LAST_CARRIER_IDX                              (4095)
#define VB_ENGINE_PSD_FULL_POWER                                (0)
#define VB_ENGINE_PSD_NO_POWER                                  (100)

#define VB_ENGINE_l2rPSD_NO_TRAFFIC_MARGIN                      (10) // Mbps

#define VB_ENGINE_l2rPSD_MEAUSREMENT_NEED_CAPACITY_DIFFERENCE   (0.5)//%
#define VB_ENGINE_l2rPSD_MEAUSREMENT_NEED_CAPACITY_MINIMUM      (50)//Mbps
#define VB_ENGINE_BPS_DEGRADATE_HYSTERESIS                      (5)
#define VB_ENGINE_BACK_TO_LOW_BAND_HYST                         (3)

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

 static  t_VB_engineErrorCode VbChannelCapacityPSDNBandsSet(t_nodeChannelSettings *nodeChannelSettings, INT8U nBands, t_psdBandAllocation *psdBandAllocation)
 {
   t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
   t_psdShape *psd;

   if (nodeChannelSettings != NULL)
   {
     psd = &(nodeChannelSettings->psdShape);

     if(nodeChannelSettings->boostInfo.level != nBands)
     {
       psd->sendUpdate = TRUE;
       nodeChannelSettings->boostInfo.level = nBands;
       nodeChannelSettings->boostInfo.perc = IN_PERC(psdBandAllocation->lastCarrier[nBands-1], psdBandAllocation->lastCarrier[nodeChannelSettings->boostInfo.maxNumBands-1]);
       if(nBands >= nodeChannelSettings->boostInfo.maxNumBands)
       {
         // All bands used -> Full Band Full power
         psd->numPSDBands = 1;
         psd->psdBandLevel[0].attLevel = VB_ENGINE_PSD_FULL_POWER;
         psd->psdBandLevel[0].stopCarrier = VB_LAST_CARRIER_IDX;
       }
       else
       {
         if(nBands >= 1)
         {
           // Not all bands in use, split in 2 bands, first one with full power, second one to 0
           psd->numPSDBands = 2;
           psd->psdBandLevel[0].attLevel = VB_ENGINE_PSD_FULL_POWER;
           psd->psdBandLevel[0].stopCarrier = psdBandAllocation->lastCarrier[nBands-1];

           psd->psdBandLevel[1].attLevel = VB_ENGINE_PSD_NO_POWER;
           psd->psdBandLevel[1].stopCarrier = VB_LAST_CARRIER_IDX;
         }
         else
         {
           result = VB_ENGINE_ERROR_PARAMS;
         }
       }
     }
   }
   else
   {
     result = VB_ENGINE_ERROR_DATA_MODEL;
   }

   return result;
 }

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

 t_VB_engineErrorCode VbEngineLastRelativeLowBandCarrierIdxGet(INT16U firstValidCarrier, INT16U *lowBandRelativeIdx)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  t_psdBandAllocation *psd_band_allocation;

  if(lowBandRelativeIdx == NULL)
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(result == VB_ENGINE_ERROR_NONE)
  {
    psd_band_allocation = VbEngineConfPSDBandAllocationGet();
    if(psd_band_allocation != NULL)
    {
      if(psd_band_allocation->lastCarrier[0] > firstValidCarrier)
      {
        *lowBandRelativeIdx = psd_band_allocation->lastCarrier[0] - firstValidCarrier;
      }
      else
      {
        result = VB_ENGINE_ERROR_DATA_MODEL;
      }
    }
    else
    {
      result = VB_ENGINE_ERROR_UNKNOWN;
    }
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineSnrCalculatedCheck(t_trafficReport* trafficReport, t_nodeChannelSettings *nodeChannelSettings,
                                                t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  float capacity_difference;

  if ((trafficReport != NULL) && (nodeChannelSettings != NULL) && (thisDriver != NULL))
  {
    if (nodeChannelSettings->boostInfo.level == 1)
    {
      capacity_difference =  (float)((float)trafficReport->bpsCapacity / (float)nodeChannelSettings->boostInfo.lowBandCapacity);
      //  VbLogPrint(VB_LOG_INFO, "Non boosted capacity_difference == %f", capacity_difference);
      if( (nodeChannelSettings->boostInfo.lowBandCapacity > VB_ENGINE_l2rPSD_MEAUSREMENT_NEED_CAPACITY_MINIMUM) &&
          (capacity_difference <  VB_ENGINE_l2rPSD_MEAUSREMENT_NEED_CAPACITY_DIFFERENCE) )
      {
        nodeChannelSettings->interferenceDetectionCounter++;
        if(nodeChannelSettings->interferenceDetectionCounter > VB_ENGINE_BPS_DEGRADATE_HYSTERESIS)
        {
          nodeChannelSettings->interferenceDetectionCounter = 0;
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "Measurement necessary. Non boosted and capacity_difference == %f", capacity_difference);

          // EP CHANGE will trigger a new measure plan
          result = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_NETWORK_DIFF_EP_CHANGE, NULL, thisDriver->clusterId);
        }
      }
      else
      {
        nodeChannelSettings->interferenceDetectionCounter = 0;
      }
    }
  }
  else
  {
    if (thisDriver != NULL)
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Node channel settings not found");
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Node channel settings not found");
    }

    result = VB_ENGINE_ERROR_DATA_MODEL;
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineLeftToRightPSDShapeRun(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
#if VB_ENGINE_METRICS_ENABLED
  INT8U                nBands = 0;
  INT32U               traffic;
#endif

  if ((node == NULL) || (driver == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineDatamodelNodeIsReadyToBoost(node) == FALSE)
    {
      // No reports received or channel capacity not calculated -> node is not ready
      VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s not ready", node->MACStr);

      // Skip current node
      ret = VB_ENGINE_ERROR_SKIP;
    }
    else
    {
      t_psdl2rArgs         *psd_l2r_args = ((t_psdl2rArgs *)args);

      if (node->channelSettings.boostInfo.mode == VB_ENGINE_BOOST_MODE_AUTO)
      {
        VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s, num Bands %u", node->MACStr, node->cdtaInfo.bandSet.numActiveBands[psd_l2r_args->qos]);
        ret = VbChannelCapacityPSDNBandsSet( &(node->channelSettings),
                                             node->cdtaInfo.bandSet.numActiveBands[psd_l2r_args->qos],
                                             psd_l2r_args->psdBandsAllocation );

#if VB_ENGINE_METRICS_ENABLED
        nBands = node->cdtaInfo.bandSet.numActiveBands[psd_l2r_args->qos];
#endif

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error %d Setting PSD Bands for node %s", ret, node->MACStr);
        }

#if VB_ENGINE_METRICS_ENABLED
        traffic = node->trafficReports.ingressTrafficP0;
        traffic += node->trafficReports.ingressTrafficP1;
        traffic += node->trafficReports.ingressTrafficP2;
        traffic += node->trafficReports.ingressTrafficP3;
        VbMetricsReportDeviceEvent(VB_METRICS_EVENT_BAND_CHANGE, node->MAC, (node->type  == VB_NODE_DOMAIN_MASTER), driver, traffic, nBands);
#endif

      }
      else if (node->channelSettings.boostInfo.mode == VB_ENGINE_BOOST_MODE_FORCED_FULL)
      {
        // Set full band
        ret = VbChannelCapacityPSDNBandsSet( &(node->channelSettings),
                                             node->channelSettings.boostInfo.maxNumBands,
                                             psd_l2r_args->psdBandsAllocation );

#if VB_ENGINE_METRICS_ENABLED
        nBands = psd_l2r_args->psdBandsAllocation->numBands200Mhz;
#endif

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s set to FULL band (mode %u; sendUpdate %u; reportsReceived %u)",
              node->MACStr,
              node->channelSettings.boostInfo.mode,
              node->channelSettings.psdShape.sendUpdate,
              node->trafficReports.reportsReceived);
        }
      }
      else if (node->channelSettings.boostInfo.mode == VB_ENGINE_BOOST_MODE_FORCED_LOW)
      {
        // Set low Band
        ret = VbChannelCapacityPSDNBandsSet( &(node->channelSettings),
                                             1,
                                             psd_l2r_args->psdBandsAllocation );

#if VB_ENGINE_METRICS_ENABLED
        nBands = 1;
#endif

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Node %s set to LOW band (mode %u; sendUpdate %u; reportsReceived %u)",
              node->MACStr,
              node->channelSettings.boostInfo.mode,
              node->channelSettings.psdShape.sendUpdate,
              node->trafficReports.reportsReceived);
        }
      }
      else
      {
        VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Unexpected boost mode (%u)", node->channelSettings.boostInfo.mode);
        ret = VB_ENGINE_ERROR_PARAMS;
      }
    }

#if VB_ENGINE_METRICS_ENABLED
    if(node->channelSettings.psdShape.sendUpdate)
    {
      traffic = node->trafficReports.ingressTrafficP0;
      traffic += node->trafficReports.ingressTrafficP1;
      traffic += node->trafficReports.ingressTrafficP2;
      traffic += node->trafficReports.ingressTrafficP3;
      VbMetricsReportDeviceEvent(VB_METRICS_EVENT_BAND_CHANGE, node->MAC, (node->type  == VB_NODE_DOMAIN_MASTER), driver, traffic, nBands);
    }
#endif
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current node
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/**
 * @}
 **/
