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
 * @file vb_engine_console.c
 * @brief Console commands implementation
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016-11-30
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <string.h>
#include <pthread.h>

#include "types.h"
#include "vb_mac_utils.h"
#include "vb_engine_datamodel.h"
#include "vb_engine_process.h"
#include "vb_counters.h"
#include "vb_log.h"
#include "vb_util.h"
#include "vb_engine_main.h"
#include "vb_measure_utils.h"
#include "vb_console.h"
#include "vb_metrics.h"
#include "vb_thread.h"
#include "vb_timer.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_conf.h"
#include "vb_engine_clock.h"
#include "vb_engine_alignment.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_measure.h"
#include "vb_engine_cdta.h"
#include "vb_ea_communication.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */
typedef struct s_clusterListLoopArgs
{
  INT32U     numClusters;
  INT32U     clusterIdx;
  INT32U    *clusterIdList;
} t_clusterListLoopArgs;

typedef struct s_consoleLoopArgs
{
  void       (*writeFun)(const char *fmt, ...);
  CHAR        *formatStr;
  CHAR       **cmd;
  void        *data;
} t_consoleLoopArgs;


/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

static void VbEngineConsoleCdtaXputPrint(char* MAC, char* driverId, t_node* node, void       (*writeFun)(const char *fmt, ...))
{
  INT32U band_idx;
  t_psdBandAllocation *psd_band_allocation = VbEngineConfPSDBandAllocationGet();

  for(band_idx = 0; band_idx < psd_band_allocation->numBands200Mhz; band_idx++)
  {
    {
      writeFun("| %s  | %4s | %2u | %20s | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps |\n",
               MAC,
               "1B",
               band_idx,
               driverId,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_20_80].capacity1Boosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_30_70].capacity1Boosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_40_60].capacity1Boosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_50_50].capacity1Boosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_60_40].capacity1Boosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_70_30].capacity1Boosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_80_20].capacity1Boosted);

      writeFun("| %s  | %4s | %2u | %20s | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps |\n",
               MAC,
               "AB",
               band_idx,
               driverId,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_20_80].capacityAllBoosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_30_70].capacityAllBoosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_40_60].capacityAllBoosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_50_50].capacityAllBoosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_60_40].capacityAllBoosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_70_30].capacityAllBoosted,
               node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_80_20].capacityAllBoosted);
    }
  }
}

/*******************************************************************/

static void VbEngineConsoleCdtaBpsReportedPrint(char* MAC, char* driverId, t_node* node, void       (*writeFun)(const char *fmt, ...))
{
  INT32U band_idx;
  t_psdBandAllocation *psd_band_allocation = VbEngineConfPSDBandAllocationGet();

  for(band_idx = 0; band_idx < psd_band_allocation->numBands200Mhz; band_idx++)
  {
    {
      writeFun("| %s  | %2u | %20s | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps |\n",
               MAC,
               band_idx,
               driverId,
               node->cdtaInfo.bpsReportedCapacities[band_idx][VB_ENGINE_QOS_RATE_20_80],
               node->cdtaInfo.bpsReportedCapacities[band_idx][VB_ENGINE_QOS_RATE_30_70],
               node->cdtaInfo.bpsReportedCapacities[band_idx][VB_ENGINE_QOS_RATE_40_60],
               node->cdtaInfo.bpsReportedCapacities[band_idx][VB_ENGINE_QOS_RATE_50_50],
               node->cdtaInfo.bpsReportedCapacities[band_idx][VB_ENGINE_QOS_RATE_60_40],
               node->cdtaInfo.bpsReportedCapacities[band_idx][VB_ENGINE_QOS_RATE_70_30],
               node->cdtaInfo.bpsReportedCapacities[band_idx][VB_ENGINE_QOS_RATE_80_20]);
    }
  }
}

/*******************************************************************/

static void VbEngineConsoleCdtaNbandsPrint(char* MAC, char* driverId, t_node* node, void       (*writeFun)(const char *fmt, ...))
{
  writeFun("| %s  | %20s |    %4u  |    %4u   |    %4u   |    %4u   |    %4u   |    %4u   |    %4u   |\n",
             MAC,
             driverId,
             node->cdtaInfo.bandSet.numActiveBands[VB_ENGINE_QOS_RATE_20_80],
             node->cdtaInfo.bandSet.numActiveBands[VB_ENGINE_QOS_RATE_30_70],
             node->cdtaInfo.bandSet.numActiveBands[VB_ENGINE_QOS_RATE_40_60],
             node->cdtaInfo.bandSet.numActiveBands[VB_ENGINE_QOS_RATE_50_50],
             node->cdtaInfo.bandSet.numActiveBands[VB_ENGINE_QOS_RATE_60_40],
             node->cdtaInfo.bandSet.numActiveBands[VB_ENGINE_QOS_RATE_70_30],
             node->cdtaInfo.bandSet.numActiveBands[VB_ENGINE_QOS_RATE_80_20]);

}

/*******************************************************************/

static void VbEngineConsoleCdtaCapacityPerRatePrint(char* MAC, char* driverId, t_node* node, void       (*writeFun)(const char *fmt, ...))
{
  writeFun("| %s  | %20s |    %4u  |    %4u   |    %4u   |    %4u   |    %4u   |    %4u   |    %4u   |\n",
             MAC,
             driverId,
             node->cdtaInfo.capacityPerRate[VB_ENGINE_QOS_RATE_20_80],
             node->cdtaInfo.capacityPerRate[VB_ENGINE_QOS_RATE_30_70],
             node->cdtaInfo.capacityPerRate[VB_ENGINE_QOS_RATE_40_60],
             node->cdtaInfo.capacityPerRate[VB_ENGINE_QOS_RATE_50_50],
             node->cdtaInfo.capacityPerRate[VB_ENGINE_QOS_RATE_60_40],
             node->cdtaInfo.capacityPerRate[VB_ENGINE_QOS_RATE_70_30],
             node->cdtaInfo.capacityPerRate[VB_ENGINE_QOS_RATE_80_20]);

}
/*******************************************************************/

static void VbEngineConsoleCdtaMetricsPrint(char* driverId, t_node* node, void       (*writeFun)(const char *fmt, ...))
{
  if (node != NULL)
  {
    writeFun("| %s  | %20s |    %4d  |    %4d  |    %4d  |    %4d  |    %4d  |    %4d  |    %4d  |\n",
        node->MACStr,
        driverId,
        node->cdtaInfo.rateAdequationValue[VB_ENGINE_QOS_RATE_20_80],
        node->cdtaInfo.rateAdequationValue[VB_ENGINE_QOS_RATE_30_70],
        node->cdtaInfo.rateAdequationValue[VB_ENGINE_QOS_RATE_40_60],
        node->cdtaInfo.rateAdequationValue[VB_ENGINE_QOS_RATE_50_50],
        node->cdtaInfo.rateAdequationValue[VB_ENGINE_QOS_RATE_60_40],
        node->cdtaInfo.rateAdequationValue[VB_ENGINE_QOS_RATE_70_30],
        node->cdtaInfo.rateAdequationValue[VB_ENGINE_QOS_RATE_80_20]);
  }
  else
  {
    writeFun("| %s  | %20s |    %4d  |    %4d  |    %4d  |    %4d  |    %4d  |    %4d  |    %4d  |\n",
        "LOST", driverId, 0, 0, 0, 0, 0, 0, 0);
  }
}

/*******************************************************************/

static void VbEngineConsoleCdtaProfilePrint(char* MAC, char* driverId, INT32S userSLA, INT32S slaWeight, INT32S userWeight, void (*writeFun)(const char *fmt, ...))
{
  writeFun("| %s  | %20s |    %5d  |     %5d     |      %5d    |\n",
           MAC,
           driverId,
           userSLA,
           slaWeight,
           userWeight);
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleBasicReportDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_node              *ep = NULL;
  INT32U               low_band_phy = 0;
  INT32U               current_phy = 0;
  INT32U               req_phy = 0;
  INT32U               l2_xput = 0;
  INT32U               max_l2_xput = 0;
  INT32U               n_boosted_bands = 0;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    low_band_phy = domain->dm.channelSettings.boostInfo.lowBandCapacity;
    current_phy = domain->dm.trafficReports.bpsCapacity;
    req_phy = domain->dm.trafficReports.neededTheoricCapacity;
    l2_xput = domain->dm.trafficReports.realCapacity;
    max_l2_xput = MAX(((domain->dm.channelSettings.boostInfo.maxCapacity * domain->dm.trafficReports.macEfficiency) / 100), l2_xput);
    n_boosted_bands = domain->dm.channelSettings.boostInfo.level;

    // Dump DM info
    loop_args->writeFun("| %s    | %4s | %20s |     %04d    |     %04d    |      %04d     |     %04d    |        %04d       |      %1u/%1u       |\n",
        domain->dm.MACStr,
        "DM",
        driver->vbDriverID,
        (int)low_band_phy,
        (int)current_phy,
        (int)req_phy,
        (int)l2_xput,
        (int)max_l2_xput,
        n_boosted_bands,
        domain->dm.channelSettings.boostInfo.maxNumBands);

    if (ep == NULL)
    {
      // EP not present

      ep_mac_str = "LOST";
      low_band_phy = 0;
      current_phy = 0;
      req_phy = 0;
      l2_xput = 0;
      max_l2_xput = 0;

      // Dump EP info
      loop_args->writeFun("|    %17s | %4s | %20s |     %04s    |     %04s    |      %04s     |     %04s    |        %04s       |      %3s       |\n",
          ep_mac_str,
          "EP",
          driver->vbDriverID,
          "-",
          "-",
          "-",
          "-",
          "-",
          "-");
    }
    else if (VbEngineConfVbInUpstreamGet() == FALSE)
    {
      // EP present, but VB in upstream is disabled

      ep_mac_str = ep->MACStr;
      current_phy = ep->trafficReports.bpsCapacity;
      req_phy = ep->trafficReports.neededTheoricCapacity;
      l2_xput = ep->trafficReports.realCapacity;

      // Dump EP info
      loop_args->writeFun("|    %17s | %4s | %20s |     %04s    |     %04d    |      %04d     |     %04d    |        %04s       |      %3s       |\n",
          ep_mac_str,
          "EP",
          driver->vbDriverID,
          "-",
          (int)current_phy,
          (int)req_phy,
          (int)l2_xput,
          "-",
          "-");
    }
    else
    {
      // EP present and VB in upstream is enabled

      ep_mac_str = ep->MACStr;
      low_band_phy = ep->channelSettings.boostInfo.lowBandCapacity;
      current_phy = ep->trafficReports.bpsCapacity;
      req_phy = ep->trafficReports.neededTheoricCapacity;
      l2_xput = ep->trafficReports.realCapacity;
      max_l2_xput = MAX(((ep->channelSettings.boostInfo.maxCapacity * ep->trafficReports.macEfficiency) / 100), l2_xput);
      n_boosted_bands = ep->channelSettings.boostInfo.level;

      // Dump EP info
      loop_args->writeFun("|    %17s | %4s | %20s |     %04d    |     %04d    |      %04d     |     %04d    |        %04d       |      %1u/%1u       |\n",
          ep_mac_str,
          "EP",
          driver->vbDriverID,
          (int)low_band_phy,
          (int)current_phy,
          (int)req_phy,
          (int)l2_xput,
          (int)max_l2_xput,
          n_boosted_bands,
          ep->channelSettings.boostInfo.maxNumBands);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleBasicReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleBasicReportDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleBasicReport(void *arg, t_writeFun writeFun, char **cmd)
{
  time_t               t;
  struct               tm *tmu;
  struct               timeval tv;
  t_consoleLoopArgs    loop_args;
  const CHAR         *qos_rate_str;

  t = time(NULL);
  tmu = localtime(&t);
  gettimeofday(&tv, NULL);

/*
Drivers Report     [10:54:33 528ms]
VDSLCoex           : OFF
Engine version     : 3.0 r046
DS/US split ratio  : 80/20
Notes: All values in Mbps.
=====================================================================================================================================================
|         Node         | Type |       Driver id      | Lowband PHY | Current PHY | Requested PHY |L2 Throughput| Max L2 Throughput | Boosting State |
=====================================================================================================================================================
| 00:13:9D:00:1A:E0    |   DM |              driver0 |     0223    |     0223    |      0000     |     0161    |        0931       |      1/5       |
|    00:13:9D:00:1A:CB |   EP |              driver0 |     0231    |     0231    |      0000     |     0032    |        0157       |      1/5       |
| 00:13:9D:00:1A:EA    |   DM |              driver0 |     0233    |     0233    |      0000     |     0168    |        1013       |      1/5       |
|    00:13:9D:00:1A:D1 |   EP |              driver0 |     0238    |     0238    |      0000     |     0033    |        0170       |      1/5       |
| 00:13:9D:00:1A:D8    |   DM |              driver0 |     0235    |     0235    |      0000     |     0170    |        1063       |      1/5       |
|    00:13:9D:00:1A:CC |   EP |              driver0 |     0234    |     0234    |      0000     |     0033    |        0179       |      1/5       |
| 00:13:9D:00:1A:CF    |   DM |              driver0 |     0240    |     0240    |      0000     |     0173    |        0901       |      1/5       |
|    00:13:9D:00:1A:DC |   EP |              driver0 |     0215    |     0215    |      0000     |     0030    |        0126       |      1/5       |
=====================================================================================================================================================
 */

  if (tmu != NULL)
  {
    writeFun("Drivers Report     [%02d:%02d:%02d %03dms]\n", tmu->tm_hour, tmu->tm_min, tmu->tm_sec, (int)(tv.tv_usec/ 1000));
  }

  loop_args.writeFun = writeFun;
  loop_args.cmd = cmd;

  loop_args.writeFun("VDSLCoex           : %s\n", VbEngineConfVdslCoexGet()?"ON":"OFF");
  loop_args.writeFun("Engine version     : %s\n", VB_ENGINE_VERSION);

  // YYRR Review
  qos_rate_str = VbEngineQosRateToStrGet(VbCdtaQosRateGet(0));
  loop_args.writeFun("DS/US split ratio  : %s\n", qos_rate_str);

  loop_args.writeFun("Notes: All values in Mbps. \n");

  // Show extended basic report (DS/US)
  loop_args.writeFun("=====================================================================================================================================================\n");
  loop_args.writeFun("|         Node         | Type |       Driver id      | Lowband PHY | Current PHY | Requested PHY |L2 Throughput| Max L2 Throughput | Boosting State |\n");
  loop_args.writeFun("=====================================================================================================================================================\n");

  // Loop through existing drivers and dump report
  VbEngineDatamodelDriversLoop(VbEngineConsoleBasicReportDriversCb, &loop_args);

  loop_args.writeFun("=====================================================================================================================================================\n");

  return TRUE;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterIdListCb(t_VBCluster *cluster, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_clusterListLoopArgs  *cluster_id_list = (t_clusterListLoopArgs *)args;

  if((cluster == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if ((cluster_id_list->clusterIdx < cluster_id_list->numClusters))
    {
      cluster_id_list->clusterIdList[cluster_id_list->clusterIdx] = cluster->clusterInfo.clusterId;
      cluster_id_list->clusterIdx++;
    }
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleBasicReportCluster(void *arg, t_writeFun writeFun, char **cmd)
{
  t_VB_engineErrorCode   ret;
  t_consoleLoopArgs      loop_args;
  const CHAR            *qos_rate_str;
  INT32U                 num_clusters;
  INT32U                 cluster_id;
  INT32U                *cluster_id_list_ptr = NULL;
  INT32U                 i;
  t_clusterListLoopArgs  cluster_id_list_arg;
/*
Num Clusters       : 1
ClusterId          : 2
DS/US split ratio  : 80/20
Notes: All values in Mbps.
=====================================================================================================================================================
|         Node         | Type |       Driver id      | Lowband PHY | Current PHY | Requested PHY |L2 Throughput| Max L2 Throughput | Boosting State |
=====================================================================================================================================================
| 00:13:9D:00:1A:E0    |   DM |              driver0 |     0223    |     0223    |      0000     |     0161    |        0931       |      1/5       |
|    00:13:9D:00:1A:CB |   EP |              driver0 |     0231    |     0231    |      0000     |     0032    |        0157       |      1/5       |
| 00:13:9D:00:1A:EA    |   DM |              driver0 |     0233    |     0233    |      0000     |     0168    |        1013       |      1/5       |
|    00:13:9D:00:1A:D1 |   EP |              driver0 |     0238    |     0238    |      0000     |     0033    |        0170       |      1/5       |
| 00:13:9D:00:1A:D8    |   DM |              driver0 |     0235    |     0235    |      0000     |     0170    |        1063       |      1/5       |
|    00:13:9D:00:1A:CC |   EP |              driver0 |     0234    |     0234    |      0000     |     0033    |        0179       |      1/5       |
| 00:13:9D:00:1A:CF    |   DM |              driver0 |     0240    |     0240    |      0000     |     0173    |        0901       |      1/5       |
|    00:13:9D:00:1A:DC |   EP |              driver0 |     0215    |     0215    |      0000     |     0030    |        0126       |      1/5       |
=====================================================================================================================================================
...
 */

  loop_args.writeFun = writeFun;
  loop_args.cmd = cmd;

  num_clusters = VbEngineDataModelNumClustersGet();
  cluster_id_list_ptr = malloc(num_clusters*sizeof(INT32U));

  if (cluster_id_list_ptr != NULL)
  {
    cluster_id_list_arg.numClusters = num_clusters;
    cluster_id_list_arg.clusterIdList  = cluster_id_list_ptr;
    cluster_id_list_arg.clusterIdx = 0;
    ret = VbEngineDatamodelClustersLoop(ClusterIdListCb, &cluster_id_list_arg);

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      for(i=0; i<num_clusters; i++)
      {
        cluster_id = cluster_id_list_ptr[i];

        loop_args.writeFun("ClusterId          : %d\n", cluster_id);

        qos_rate_str = VbEngineQosRateToStrGet(VbCdtaQosRateGet(cluster_id));
        loop_args.writeFun("DS/US split ratio  : %s\n", qos_rate_str);

        loop_args.writeFun("Notes: All values in Mbps. \n");

        // Show extended basic report (DS/US)
        loop_args.writeFun("=====================================================================================================================================================\n");
        loop_args.writeFun("|         Node         | Type |       Driver id      | Lowband PHY | Current PHY | Requested PHY |L2 Throughput| Max L2 Throughput | Boosting State |\n");
        loop_args.writeFun("=====================================================================================================================================================\n");

        // Loop through existing drivers and dump report
        VbEngineDatamodelClusterXDriversLoop(VbEngineConsoleBasicReportDriversCb, cluster_id, &loop_args);

        loop_args.writeFun("=====================================================================================================================================================\n");
      }
    }
  }
  else
  {
    writeFun("Error allocating memory!\n");
  }

  if (cluster_id_list_ptr != NULL)
  {
    free(cluster_id_list_ptr);
    cluster_id_list_ptr = NULL;
  }

  return TRUE;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleAddInfoReportDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  INT8U                dm_ver[VB_FW_VERSION_LENGTH];
  INT8U                ep_ver[VB_FW_VERSION_LENGTH];
  CHAR                *ep_mac_str;
  t_node              *ep = NULL;
  INT16U               qos_rate = 0;
  INT16U               max_length_txop = 0;
  INT16U               ext_seed = 0;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    strncpy((char *)dm_ver, (char *)domain->dm.addInfo1.fwVersion, VB_FW_VERSION_LENGTH);
    dm_ver[VB_FW_VERSION_LENGTH - 1] = '\0';

    qos_rate = domain->dm.addInfo1.qosRate;
    max_length_txop = domain->dm.addInfo1.maxLengthTxop;
    ext_seed = domain->dm.addInfo1.extSeed;

    if(ep != NULL)
    {
      strncpy((char *)ep_ver, (char *)domain->eps.epsArray[0].addInfo1.fwVersion, VB_FW_VERSION_LENGTH);
      ep_ver[VB_FW_VERSION_LENGTH - 1] = '\0';
      ep_mac_str = domain->eps.epsArray[0].MACStr;
    }
    else
    {
      ep_ver[0] = '\0';
      ep_mac_str = "LOST";
    }


    loop_args->writeFun("| %s | %3u | %20s | %10s |     %04d   |     %04d   |     %04d   | %17s | %10s |\n",
        domain->dm.MACStr,
        domain->dm.devID,
        driver->vbDriverID,
        dm_ver,
        qos_rate,
        max_length_txop,
        ext_seed,
        ep_mac_str,
        ep_ver);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleAddInfoReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleAddInfoReportDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleAddInfoReport(void *arg, t_writeFun writeFun, char **cmd)
{
  time_t               t;
  struct               tm *tmu;
  struct               timeval tv;
  t_consoleLoopArgs    loop_args;

  t = time(NULL);
  tmu = localtime(&t);
  gettimeofday(&tv, NULL);

  if (tmu != NULL)
  {
    writeFun("AddInfo     [%02d:%02d:%02d %03dms]\n", tmu->tm_hour, tmu->tm_min, tmu->tm_sec, (int)(tv.tv_usec/ 1000));
  }

  loop_args.writeFun = writeFun;

  loop_args.writeFun("=======================================================================================================================================\n");
  loop_args.writeFun("|   DM MAC          | DID |       Driver id      | DM FW Ver  | DM qosRate | DM maxTxop | DM ExtSeed |  EP MAC           | EP FW Ver  |\n");
  loop_args.writeFun("=======================================================================================================================================\n");

  // Loop through existing drivers and dump report
  VbEngineDatamodelDriversLoop(VbEngineConsoleAddInfoReportDriversCb, &loop_args);

  loop_args.writeFun("=======================================================================================================================================\n");


  return TRUE;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleTrafficReportDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  INT16U               ingress_p0 = 0;
  INT16U               ingress_p1 = 0;
  INT16U               ingress_p2 = 0;
  INT16U               ingress_p3 = 0;
  INT8U                buff_p0 = 0;
  INT8U                buff_p1 = 0;
  INT8U                buff_p2 = 0;
  INT8U                buff_p3 = 0;
  INT8U                mac_eff = 0;
  INT32U               rx_reports = 0;
  t_node              *ep = NULL;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ingress_p0 = domain->dm.trafficReports.ingressTrafficP0;
    ingress_p1 = domain->dm.trafficReports.ingressTrafficP1;
    ingress_p2 = domain->dm.trafficReports.ingressTrafficP2;
    ingress_p3 = domain->dm.trafficReports.ingressTrafficP3;
    buff_p0 = domain->dm.trafficReports.usageBuffP0;
    buff_p1 = domain->dm.trafficReports.usageBuffP1;
    buff_p2 = domain->dm.trafficReports.usageBuffP2;
    buff_p3 = domain->dm.trafficReports.usageBuffP3;
    mac_eff = domain->dm.trafficReports.macEfficiency;
    rx_reports = domain->dm.trafficReports.rxReports;

    // Show traffic report (DS/US)

    loop_args->writeFun("| %s    | %4s | %20s | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u %% | %4u %% | %4u %% | %4u %% | %4u %% | %9u |\n",
        domain->dm.MACStr,
        "DM",
        driver->vbDriverID,
        (int)ingress_p0,
        (int)ingress_p1,
        (int)ingress_p2,
        (int)ingress_p3,
        (int)buff_p0,
        (int)buff_p1,
        (int)buff_p2,
        (int)buff_p3,
        (int)mac_eff,
        (int)rx_reports);

    if (ep == NULL)
    {
      ep_mac_str = "LOST";
      ingress_p0 = 0;
      ingress_p1 = 0;
      ingress_p2 = 0;
      ingress_p3 = 0;
      buff_p0 = 0;
      buff_p1 = 0;
      buff_p2 = 0;
      buff_p3 = 0;
      mac_eff = 0;
      rx_reports = 0;
    }
    else
    {
      ep_mac_str = ep->MACStr;
      ingress_p0 = ep->trafficReports.ingressTrafficP0;
      ingress_p1 = ep->trafficReports.ingressTrafficP1;
      ingress_p2 = ep->trafficReports.ingressTrafficP2;
      ingress_p3 = ep->trafficReports.ingressTrafficP3;
      buff_p0 = ep->trafficReports.usageBuffP0;
      buff_p1 = ep->trafficReports.usageBuffP1;
      buff_p2 = ep->trafficReports.usageBuffP2;
      buff_p3 = ep->trafficReports.usageBuffP3;
      mac_eff = ep->trafficReports.macEfficiency;
      rx_reports = ep->trafficReports.rxReports;
    }

    loop_args->writeFun("|    %17s | %4s | %20s | %4u Mbps | %4u Mbps | %4u Mbps | %4u Mbps | %4u %% | %4u %% | %4u %% | %4u %% | %4u %% | %9u |\n",
        ep_mac_str,
        "EP",
        driver->vbDriverID,
        (int)ingress_p0,
        (int)ingress_p1,
        (int)ingress_p2,
        (int)ingress_p3,
        (int)buff_p0,
        (int)buff_p1,
        (int)buff_p2,
        (int)buff_p3,
        (int)mac_eff,
        (int)rx_reports);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleTrafficBpsReportDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  INT8U                n_bands;
  INT32U               rx_reports = 0;
  INT16U               b0;
  INT16U               b1;
  INT16U               b2;
  INT16U               b3;
  INT16U               b4;
  t_node              *ep = NULL;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    n_bands = domain->dm.trafficReports.nBandsBps;
    b0 = domain->dm.trafficReports.bpsBand[0];
    b1 = domain->dm.trafficReports.bpsBand[1];
    b2 = domain->dm.trafficReports.bpsBand[2];
    b3 = domain->dm.trafficReports.bpsBand[3];
    b4 = domain->dm.trafficReports.bpsBand[4];
    rx_reports = domain->dm.trafficReports.rxReports;

    loop_args->writeFun("| %s    | %4s | %20s | %6u | %5u | %5u | %5u | %5u | %5u | %9u |\n",
        domain->dm.MACStr,
        "DM",
        driver->vbDriverID,
        (int)n_bands,
        (int)b0,
        (int)b1,
        (int)b2,
        (int)b3,
        (int)b4,
        rx_reports);

    if (ep == NULL)
    {
      ep_mac_str = "LOST";
      n_bands = 0;
      b0 = 0;

      rx_reports = 0;
    }
    else
    {
      ep_mac_str = ep->MACStr;
      n_bands = ep->trafficReports.nBandsBps;
      b0 = ep->trafficReports.bpsBand[0];
      b1 = ep->trafficReports.bpsBand[1];
      b2 = ep->trafficReports.bpsBand[2];
      b3 = ep->trafficReports.bpsBand[3];
      b4 = ep->trafficReports.bpsBand[4];
      rx_reports = ep->trafficReports.rxReports;
    }

    loop_args->writeFun("|    %17s | %4s | %20s | %6u | %5u | %5u | %5u | %5u | %5u | %9u |\n",
        ep_mac_str,
        "EP",
        driver->vbDriverID,
        (int)n_bands,
        (int)b0,
        (int)b1,
        (int)b2,
        (int)b3,
        (int)b4,
        rx_reports);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaReportDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_node              *node;
  t_node              *ep = NULL;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {

    if ((loop_args->cmd != NULL) &&
        (loop_args->cmd[3] != NULL) &&
        (!strcmp(loop_args->cmd[3], "down")))
    {
      // Show Down Xput estimation
      node = &domain->dm;

      if(node != NULL)
      {
        loop_args->writeFun("Requested %u \n", node->trafficReports.neededL2Xput);
        VbEngineConsoleCdtaXputPrint(domain->dm.MACStr, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
    else
    {
      // Show only DS traffic report

      if (ep == NULL)
      {
        ep_mac_str = "LOST";
      }
      else
      {
        ep_mac_str = ep->MACStr;
      }

      node = &domain->eps.epsArray[0];

      if(node != NULL)
      {
        loop_args->writeFun("Requested %u\n", node->trafficReports.neededL2Xput);
        VbEngineConsoleCdtaXputPrint(ep_mac_str, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaBpsReportDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_node              *node;
  t_node              *ep = NULL;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {

    if ((loop_args->cmd != NULL) &&
        (loop_args->cmd[3] != NULL) &&
        (!strcmp(loop_args->cmd[3], "down")))
    {
      // Show Down Xput estimation
      node = &domain->dm;

      if(node != NULL)
      {
        VbEngineConsoleCdtaBpsReportedPrint(domain->dm.MACStr, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
    else
    {
      // Show only DS traffic report

      if (ep == NULL)
      {
        ep_mac_str = "LOST";
      }
      else
      {
        ep_mac_str = ep->MACStr;
      }

      node = &domain->eps.epsArray[0];

      if(node != NULL)
      {
        VbEngineConsoleCdtaBpsReportedPrint(ep_mac_str, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
  }

  return ret;
}


/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaNbandsDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_node              *node;
  t_node              *ep = NULL;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {

    if ((loop_args->cmd != NULL) &&
        (loop_args->cmd[3] != NULL) &&
        (!strcmp(loop_args->cmd[3], "down")))
    {
      // Show Down Xput estimation
      node = &domain->dm;

      if(node != NULL)
      {
        VbEngineConsoleCdtaNbandsPrint(domain->dm.MACStr, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
    else
    {
      // Show only DS traffic report

      if (ep == NULL)
      {
        ep_mac_str = "LOST";
      }
      else
      {
        ep_mac_str = ep->MACStr;
      }

      node = &domain->eps.epsArray[0];

      if(node != NULL)
      {
        VbEngineConsoleCdtaNbandsPrint(ep_mac_str, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaCapPerRateDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_node              *node;
  t_node              *ep = NULL;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {

    if ((loop_args->cmd != NULL) &&
        (loop_args->cmd[3] != NULL) &&
        (!strcmp(loop_args->cmd[3], "down")))
    {
      // Show Down Xput estimation
      node = &domain->dm;

      if(node != NULL)
      {
        VbEngineConsoleCdtaCapacityPerRatePrint(domain->dm.MACStr, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
    else
    {
      // Show only DS traffic report

      if (ep == NULL)
      {
        ep_mac_str = "LOST";
      }
      else
      {
        ep_mac_str = ep->MACStr;
      }

      node = &domain->eps.epsArray[0];

      if(node != NULL)
      {
        VbEngineConsoleCdtaCapacityPerRatePrint(ep_mac_str, driver->vbDriverID, node, loop_args->writeFun);
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaMetricsDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_node              *node;
  t_node              *ep = NULL;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ((loop_args->cmd != NULL) &&
        (loop_args->cmd[3] != NULL) &&
        (!strcmp(loop_args->cmd[3], "down")))
    {
      // Show Down Xput estimation
      node = &domain->dm;
      VbEngineConsoleCdtaMetricsPrint(driver->vbDriverID, node, loop_args->writeFun);
    }
    else
    {
      // Show Up Xput estimation
      VbEngineConsoleCdtaMetricsPrint(driver->vbDriverID, ep, loop_args->writeFun);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaProfileDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_node              *node;
  t_node              *ep = NULL;
  CHAR                *ep_mac_str;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {

    if ((loop_args->cmd != NULL) &&
        (loop_args->cmd[3] != NULL) &&
        (!strcmp(loop_args->cmd[3], "down")))
    {
      // Show Down Xput estimation
      node = &domain->dm;
      VbEngineConsoleCdtaProfilePrint(domain->dm.MACStr, driver->vbDriverID,
          node->cdtaInfo.profile.userSLA, node->cdtaInfo.profile.slaWeight,
          node->cdtaInfo.profile.userWeight, loop_args->writeFun);
    }
    else
    {
      // Show only DS traffic report

      if (ep == NULL)
      {
        ep_mac_str = "LOST";
        VbEngineConsoleCdtaProfilePrint(ep_mac_str, driver->vbDriverID, 0, 0, 0, loop_args->writeFun);
      }
      else
      {
        ep_mac_str = ep->MACStr;
        node = &domain->eps.epsArray[0];
        VbEngineConsoleCdtaProfilePrint(ep_mac_str, driver->vbDriverID,
            node->cdtaInfo.profile.userSLA, node->cdtaInfo.profile.slaWeight,
            node->cdtaInfo.profile.userWeight, loop_args->writeFun);
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleTrafficReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleTrafficReportDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleTrafficBpsReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleTrafficBpsReportDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleCdtaReportDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaBpsReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleCdtaBpsReportDomainsCb, loop_args);
  }

  return ret;
}


/*******************************************************************/

static t_VB_engineErrorCode   VbEngineConsoleCdtaNbandsDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleCdtaNbandsDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode   VbEngineConsoleCdtaCapPerRateDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleCdtaCapPerRateDomainsCb, loop_args);
  }

  return ret;
}
/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaMetricsDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleCdtaMetricsDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleCdtaProfileDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleCdtaProfileDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleTrafficReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs    loop_args;

  loop_args.writeFun = writeFun;
  loop_args.cmd = cmd;

  // Show traffic report (DS/US)

  loop_args.writeFun("===============================================================================================================================================================\n");
  loop_args.writeFun("|         Node         | Type |       Driver id      | IngressP0 | IngressP1 | IngressP2 | IngressP3 | BuffP0 | BuffP1 | BuffP2 | BuffP3 | MACeff | RxReports |\n");
  loop_args.writeFun("===============================================================================================================================================================\n");

  // Loop through existing drivers and dump report
  VbEngineDatamodelDriversLoop(VbEngineConsoleTrafficReportDriversCb, &loop_args);

  loop_args.writeFun("===============================================================================================================================================================\n");

  return TRUE;
}

/*******************************************************************/

static BOOL VbEngineConsoleTrafficBpsReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs    loop_args;

  loop_args.writeFun = writeFun;
  loop_args.cmd = cmd;

  // Show extended traffic report (DS/US)

  loop_args.writeFun("===================================================================================================================\n");
  loop_args.writeFun("|         Node         | Type |       Driver id      | nBands |  b0   |  b1   |  b2   |  b3   |  b4   | RxReports |\n");
  loop_args.writeFun("===================================================================================================================\n");

  // Loop through existing drivers and dump report
  VbEngineDatamodelDriversLoop(VbEngineConsoleTrafficBpsReportDriversCb, &loop_args);

  loop_args.writeFun("===================================================================================================================\n");

  return TRUE;
}


/*******************************************************************/

static BOOL VbEngineConsoleCdtaReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs    loop_args;

  loop_args.writeFun = writeFun;
  loop_args.cmd = cmd;

  if ((loop_args.cmd != NULL) &&
       (loop_args.cmd[2] != NULL) &&
       (!strcmp(loop_args.cmd[2], "xput")))
  {
    // Show only DS traffic report

    loop_args.writeFun("=============================================================================================================================================\n");
    loop_args.writeFun("|         MAC        | Cond | bn |       Driver id      |   20-80   |   30-70   |   40-60   |   50-50   |   60-40   |   70-30   |   80-20   |\n");
    loop_args.writeFun("=============================================================================================================================================\n");

    // Loop through existing drivers and dump report
    VbEngineDatamodelDriversLoop(VbEngineConsoleCdtaReportDriversCb, &loop_args);

    loop_args.writeFun("=============================================================================================================================================\n");
  }
  if ((loop_args.cmd != NULL) &&
         (loop_args.cmd[2] != NULL) &&
         (!strcmp(loop_args.cmd[2], "bpsReported")))
  {
    // Show only DS traffic report

    loop_args.writeFun("======================================================================================================================================\n");
    loop_args.writeFun("|         MAC        | bn |      Driver id       |   20-80   |   30-70   |   40-60   |   50-50   |   60-40   |   70-30   |   80-20   |\n");
    loop_args.writeFun("======================================================================================================================================\n");

    // Loop through existing drivers and dump report
    VbEngineDatamodelDriversLoop(VbEngineConsoleCdtaBpsReportDriversCb, &loop_args);

    loop_args.writeFun("=============================================================================================================================================\n");
  }
  else if((loop_args.cmd != NULL) &&
      (loop_args.cmd[2] != NULL) &&
      (!strcmp(loop_args.cmd[2], "nbands")))
  {
    loop_args.writeFun("================================================================================================================================\n");
    loop_args.writeFun("|         MAC        |       Driver id      |   20-80  |   30-70   |   40-60   |   50-50   |   60-40   |   70-30   |   80-20   |\n");
    loop_args.writeFun("================================================================================================================================\n");

    // Loop through existing drivers and dump report
    VbEngineDatamodelDriversLoop(  VbEngineConsoleCdtaNbandsDriversCb, &loop_args);

    loop_args.writeFun("================================================================================================================================\n");
  }
  else if((loop_args.cmd != NULL) &&
        (loop_args.cmd[2] != NULL) &&
        (!strcmp(loop_args.cmd[2], "cap")))
  {
    loop_args.writeFun("================================================================================================================================\n");
    loop_args.writeFun("|         MAC        |       Driver id      |   20-80  |   30-70   |   40-60   |   50-50   |   60-40   |   70-30   |   80-20   |\n");
    loop_args.writeFun("================================================================================================================================\n");

    // Loop through existing drivers and dump report
    VbEngineDatamodelDriversLoop(  VbEngineConsoleCdtaCapPerRateDriversCb, &loop_args);

    loop_args.writeFun("================================================================================================================================\n");
  }
  else if((loop_args.cmd != NULL) &&
       (loop_args.cmd[2] != NULL) &&
       (!strcmp(loop_args.cmd[2], "metrics")))
  {
     loop_args.writeFun("==========================================================================================================================\n");
     loop_args.writeFun("|         MAC        |       Driver id      |   20-80  |   30-70  |   40-60  |   50-50  |   60-40  |   70-30  |   80-20  |\n");
     loop_args.writeFun("==========================================================================================================================\n");

     // Loop through existing drivers and dump report
     VbEngineDatamodelDriversLoop(VbEngineConsoleCdtaMetricsDriversCb, &loop_args);

     loop_args.writeFun("==========================================================================================================================\n");
  }
  else if((loop_args.cmd != NULL) &&
         (loop_args.cmd[2] != NULL) &&
         (!strcmp(loop_args.cmd[2], "profiles")))
  {
     loop_args.writeFun("=========================================================================================\n");
     loop_args.writeFun("|         MAC        |       Driver id      |    SLA    |   SLA Weight  |   UserWeight  |\n");
     loop_args.writeFun("=========================================================================================\n");

     // Loop through existing drivers and dump report
     VbEngineDatamodelDriversLoop(VbEngineConsoleCdtaProfileDriversCb, &loop_args);

     loop_args.writeFun("=========================================================================================\n");
  }

  return TRUE;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleBoostReportDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  char                *boost_mode_strs[VB_ENGINE_BOOST_MODE_LAST] = {"AUTO", "FORCED FULL", "FORCED LOW"};
  char                *boost_mode = NULL;
  t_node              *ep = NULL;

  if ((loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->eps.epsArray != NULL)
    {
      ep = &(domain->eps.epsArray[0]);
    }
    else
    {
      ep = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (domain->dm.channelSettings.boostInfo.mode < VB_ENGINE_BOOST_MODE_LAST)
    {
      boost_mode = boost_mode_strs[domain->dm.channelSettings.boostInfo.mode];
    }
    else
    {
      boost_mode = "UNKN";
    }

    // DM
    loop_args->writeFun("| %s    | %4s | %20s | %6u Mbps | %5u Mbps |    %1u/%1u (%3u%%)   |  %12s  |\n",
                        domain->dm.MACStr,
                        "DM",
                        driver->vbDriverID,
                        domain->dm.trafficReports.neededL2Xput,
                        domain->dm.cdtaInfo.profile.userSLA,
                        domain->dm.channelSettings.boostInfo.level,
                        domain->dm.channelSettings.boostInfo.maxNumBands,
                        domain->dm.channelSettings.boostInfo.perc,
                        boost_mode);

    // EP
    if (ep == NULL)
    {
      loop_args->writeFun("|    %17s | %4s | %20s | %6u Mbps | %5u Mbps |      %3s        |  %12s  |\n",
          "LOST",
          "EP",
          driver->vbDriverID,
          0,
          0,
          "-",
          "-");
    }
    else
    {
      if (ep->channelSettings.boostInfo.mode < VB_ENGINE_BOOST_MODE_LAST)
      {
        boost_mode = boost_mode_strs[ep->channelSettings.boostInfo.mode];
      }
      else
      {
        boost_mode = "UNKN";
      }

      loop_args->writeFun("|    %s | %4s | %20s | %6u Mbps | %5u Mbps |    %1u/%1u (%3u%%)   |  %12s  |\n",
          ep->MACStr,
          "EP",
          driver->vbDriverID,
          ep->trafficReports.neededL2Xput,
          ep->cdtaInfo.profile.userSLA,
          ep->channelSettings.boostInfo.level,
          ep->channelSettings.boostInfo.maxNumBands,
          ep->channelSettings.boostInfo.perc,
          boost_mode);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleBoostReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleBoostReportDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleBoostReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs    loop_args;
  time_t               t;
  struct               tm *tmu;
  struct               timeval tv;

  t = time(NULL);
  tmu = localtime(&t);
  gettimeofday(&tv, NULL);

/*
Drivers Report     [16:52:28 016ms]
Engine version     : 3.0 r048
====================================================================================================================
|         Node         | Type |       Driver id      | Req L2 Xput |  SLA Mbps  | Boosting State  | B.Mode         |
====================================================================================================================
| 00:13:9D:00:1A:CF    |   DM |              driver0 |    264 Mbps |   100 Mbps |    1/3 ( 29%)   |          AUTO  |
|    00:13:9D:00:1A:DC |   EP |              driver0 |    208 Mbps |  1000 Mbps |    3/3 (100%)   |          AUTO  |
| 00:13:9D:00:1A:EA    |   DM |              driver0 |    264 Mbps |  1000 Mbps |    3/3 (100%)   |          AUTO  |
|    00:13:9D:00:1A:D1 |   EP |              driver0 |    208 Mbps |  1000 Mbps |    3/3 (100%)   |          AUTO  |
| 00:13:9D:00:1A:E0    |   DM |              driver0 |    264 Mbps |  1000 Mbps |    3/3 (100%)   |          AUTO  |
|    00:13:9D:00:1A:CB |   EP |              driver0 |    208 Mbps |  1000 Mbps |    3/3 (100%)   |          AUTO  |
| 00:13:9D:00:1A:D8    |   DM |              driver0 |    264 Mbps |  1000 Mbps |    3/3 (100%)   |          AUTO  |
|    00:13:9D:00:1A:CC |   EP |              driver0 |    208 Mbps |  1000 Mbps |    3/3 (100%)   |          AUTO  |
====================================================================================================================
 */

  if (tmu != NULL)
  {
    writeFun("Drivers Report     [%02d:%02d:%02d %03dms]\n", tmu->tm_hour, tmu->tm_min, tmu->tm_sec, (int)(tv.tv_usec/ 1000));
  }

  loop_args.writeFun = writeFun;

  loop_args.writeFun("Engine version     : %s\n", VB_ENGINE_VERSION);
  loop_args.writeFun("====================================================================================================================\n");
  loop_args.writeFun("|         Node         | Type |       Driver id      | Req L2 Xput |  SLA Mbps  | Boosting State  | B.Mode         |\n");
  loop_args.writeFun("====================================================================================================================\n");

  // Loop through existing drivers and dump report
  VbEngineDatamodelDriversLoop(VbEngineConsoleBoostReportDriversCb, &loop_args);

  loop_args.writeFun("====================================================================================================================\n");


  return TRUE;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleMeasReportNodesCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  INT32U               cfr_idx = 0;
  t_processMeasure    *measure = NULL;
  t_crossMeasure      *cross_measure = NULL;
  INT8U                plan_id = 0;

  if ((loop_args == NULL) || (node == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get expected plan Id
    VbEngineMeasurePlanIdGet(driver->clusterId, &plan_id);

    // Dump BGN measure
    VbMeasureDump(node->MAC, node->type, driver->vbDriverID, VB_MEAS_TYPE_BGN, &(node->measures.BGNMeasure), NULL, NULL, plan_id, loop_args->writeFun);

    // Dump CFR measures
    if ((node->measures.CFRMeasureList.numCrossMeasures > 0) &&
        (node->measures.CFRMeasureList.crossMeasureArray != NULL))
    {
      for (cfr_idx = 0; cfr_idx < node->measures.CFRMeasureList.numCrossMeasures; cfr_idx++)
      {
        cross_measure = &(node->measures.CFRMeasureList.crossMeasureArray[cfr_idx]);
        measure = &(cross_measure->measure);

        if (measure != NULL)
        {
          VbMeasureDump(node->MAC, node->type, driver->vbDriverID, VB_MEAS_TYPE_CFR, measure, cross_measure, NULL, plan_id, loop_args->writeFun);
        }
      }
    }

    // Dump SNR full xtalk measure
    VbMeasureDump(node->MAC, node->type, driver->vbDriverID, VB_MEAS_TYPE_SNRFULLXTALK, &(node->measures.snrFullXtalk), NULL, NULL, plan_id, loop_args->writeFun);

    // Dump SNR low xtalk measure
    VbMeasureDump(node->MAC, node->type, driver->vbDriverID, VB_MEAS_TYPE_SNRLOWXTALK, &(node->measures.snrLowXtalk), NULL, NULL, plan_id, loop_args->writeFun);

    // Dump PSD measure
    VbMeasureDump(node->MAC, node->type, driver->vbDriverID, VB_MEAS_TYPE_PSD, NULL, NULL, &(node->measures.Psd), plan_id, loop_args->writeFun);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleMeasReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing nodes in driver
    ret = VbEngineDatamodelNodesLoop(driver, VbEngineConsoleMeasReportNodesCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleMeasReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs    loop_args;

/*
 Driver Id          : DriverA
Number of lines    : 4
Engine version     : 2.0 r042
State              : VB_ENGINE_BOOSTING -> VB_ENGINE_PROCESS_BOOSTING_WAIT_TRIGGERS
Last Measure Plan Id = 2
==========================================================================================================
|         Node         | Type |    Driver id   | MeasType |     MeasuredMAC    | OwnCFR | Valid | PlanId |
==========================================================================================================
| 00:13:9D:00:13:7C    |   DM |        driverA |      BGN |                  - |      - |   YES |      2 |
| 00:13:9D:00:13:7C    |   DM |        driverA |      CFR |  00:13:9D:00:10:28 |    YES |   YES |      2 |
| 00:13:9D:00:13:7C    |   DM |        driverA |      CFR |  00:13:9D:00:10:2D |     NO |   YES |      2 |
| 00:13:9D:00:13:7C    |   DM |        driverA |      CFR |  00:13:9D:00:10:0F |     NO |   YES |      2 |
| 00:13:9D:00:13:7C    |   DM |        driverA |      CFR |  00:13:9D:00:10:2A |     NO |   YES |      2 |
| 00:13:9D:00:13:7C    |   DM |        driverA |      PSD |                  - |      - |    NO |      - |
|    00:13:9D:00:10:28 |   EP |        driverA |      BGN |                  - |      - |   YES |      2 |
|    00:13:9D:00:10:28 |   EP |        driverA |      CFR |  00:13:9D:00:13:7C |    YES |   YES |      2 |
|    00:13:9D:00:10:28 |   EP |        driverA |      CFR |  00:13:9D:00:13:8F |     NO |   YES |      2 |
|    00:13:9D:00:10:28 |   EP |        driverA |      CFR |  00:13:9D:00:13:A8 |     NO |   YES |      2 |
|    00:13:9D:00:10:28 |   EP |        driverA |      CFR |  00:13:9D:00:13:AE |     NO |   YES |      2 |
|    00:13:9D:00:10:28 |   EP |        driverA |      PSD |                  - |      - |    NO |      - |
| 00:13:9D:00:13:8F    |   DM |        driverA |      BGN |                  - |      - |   YES |      2 |
| 00:13:9D:00:13:8F    |   DM |        driverA |      CFR |  00:13:9D:00:10:28 |     NO |   YES |      2 |
| 00:13:9D:00:13:8F    |   DM |        driverA |      CFR |  00:13:9D:00:10:2D |    YES |   YES |      2 |
| 00:13:9D:00:13:8F    |   DM |        driverA |      CFR |  00:13:9D:00:10:0F |     NO |   YES |      2 |
| 00:13:9D:00:13:8F    |   DM |        driverA |      CFR |  00:13:9D:00:10:2A |     NO |   YES |      2 |
| 00:13:9D:00:13:8F    |   DM |        driverA |      PSD |                  - |      - |    NO |      - |
|    00:13:9D:00:10:2D |   EP |        driverA |      BGN |                  - |      - |   YES |      2 |
|    00:13:9D:00:10:2D |   EP |        driverA |      CFR |  00:13:9D:00:13:7C |     NO |   YES |      2 |
|    00:13:9D:00:10:2D |   EP |        driverA |      CFR |  00:13:9D:00:13:8F |    YES |   YES |      2 |
|    00:13:9D:00:10:2D |   EP |        driverA |      CFR |  00:13:9D:00:13:A8 |     NO |   YES |      2 |
|    00:13:9D:00:10:2D |   EP |        driverA |      CFR |  00:13:9D:00:13:AE |     NO |   YES |      2 |
|    00:13:9D:00:10:2D |   EP |        driverA |      PSD |                  - |      - |    NO |      - |
| 00:13:9D:00:13:A8    |   DM |        driverA |      BGN |                  - |      - |   YES |      2 |
| 00:13:9D:00:13:A8    |   DM |        driverA |      CFR |  00:13:9D:00:10:28 |     NO |   YES |      2 |
| 00:13:9D:00:13:A8    |   DM |        driverA |      CFR |  00:13:9D:00:10:2D |     NO |   YES |      2 |
| 00:13:9D:00:13:A8    |   DM |        driverA |      CFR |  00:13:9D:00:10:0F |    YES |   YES |      2 |
| 00:13:9D:00:13:A8    |   DM |        driverA |      CFR |  00:13:9D:00:10:2A |     NO |   YES |      2 |
| 00:13:9D:00:13:A8    |   DM |        driverA |      PSD |                  - |      - |    NO |      - |
|    00:13:9D:00:10:0F |   EP |        driverA |      BGN |                  - |      - |   YES |      2 |
|    00:13:9D:00:10:0F |   EP |        driverA |      CFR |  00:13:9D:00:13:7C |     NO |   YES |      2 |
|    00:13:9D:00:10:0F |   EP |        driverA |      CFR |  00:13:9D:00:13:8F |     NO |   YES |      2 |
|    00:13:9D:00:10:0F |   EP |        driverA |      CFR |  00:13:9D:00:13:A8 |    YES |   YES |      2 |
|    00:13:9D:00:10:0F |   EP |        driverA |      CFR |  00:13:9D:00:13:AE |     NO |   YES |      2 |
|    00:13:9D:00:10:0F |   EP |        driverA |      PSD |                  - |      - |    NO |      - |
| 00:13:9D:00:13:AE    |   DM |        driverA |      BGN |                  - |      - |   YES |      2 |
| 00:13:9D:00:13:AE    |   DM |        driverA |      CFR |  00:13:9D:00:10:28 |     NO |   YES |      2 |
| 00:13:9D:00:13:AE    |   DM |        driverA |      CFR |  00:13:9D:00:10:2D |     NO |   YES |      2 |
| 00:13:9D:00:13:AE    |   DM |        driverA |      CFR |  00:13:9D:00:10:0F |     NO |   YES |      2 |
| 00:13:9D:00:13:AE    |   DM |        driverA |      CFR |  00:13:9D:00:10:2A |    YES |   YES |      2 |
| 00:13:9D:00:13:AE    |   DM |        driverA |      PSD |                  - |      - |    NO |      - |
|    00:13:9D:00:10:2A |   EP |        driverA |      BGN |                  - |      - |   YES |      2 |
|    00:13:9D:00:10:2A |   EP |        driverA |      CFR |  00:13:9D:00:13:7C |     NO |   YES |      2 |
|    00:13:9D:00:10:2A |   EP |        driverA |      CFR |  00:13:9D:00:13:8F |     NO |   YES |      2 |
|    00:13:9D:00:10:2A |   EP |        driverA |      CFR |  00:13:9D:00:13:A8 |     NO |   YES |      2 |
|    00:13:9D:00:10:2A |   EP |        driverA |      CFR |  00:13:9D:00:13:AE |    YES |   YES |      2 |
|    00:13:9D:00:10:2A |   EP |        driverA |      PSD |                  - |      - |    NO |      - |
==========================================================================================================
 */

  loop_args.writeFun = writeFun;

  loop_args.writeFun("Engine version     : %s\n", VB_ENGINE_VERSION);
  loop_args.writeFun("================================================================================================================================\n");
  loop_args.writeFun("|         Node         | Type |       Driver id      | MeasureType  |     MeasuredMAC    | OwnCFR | ExpPlanId | Valid | PlanId |\n");
  loop_args.writeFun("================================================================================================================================\n");

  // Loop through existing drivers and dump report
  VbEngineDatamodelDriversLoop(VbEngineConsoleMeasReportDriversCb, &loop_args);

  loop_args.writeFun("================================================================================================================================\n");

  return TRUE;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleClockReportDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    CHAR own_clock_str[TIMESPEC_STR_LEN];
    CHAR last_clock_rx_str[TIMESPEC_STR_LEN];
    CHAR adj_clock_str[TIMESPEC_STR_LEN];

    // Get time strings
    VbUtilTimespecToString(own_clock_str, driver->time.ownClock);
    VbUtilTimespecToString(last_clock_rx_str, driver->time.lastClockRx);
    VbUtilTimespecToString(adj_clock_str, driver->time.adjClock);

    loop_args->writeFun("|%-30s|  %13s | %13s | %13s | %6u | %5s | %8llu usec | %9lld ms | %6lld ms |\n",
        driver->vbDriverID,
        own_clock_str,
        last_clock_rx_str,
        adj_clock_str,
        driver->time.lastSeqNumberRx,
        driver->time.lastSeqNumberValid?"YES":"NO",
        NS_TO_US(driver->time.rttNs),
        driver->time.deviationAbs,
        driver->time.deviationRel);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleClockReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_VB_engineErrorCode err;
  t_consoleLoopArgs    loop_args;
  INT64S               max_rtt;
  INT64S               max_dev;
  INT64S               min_dev;

  loop_args.writeFun = writeFun;
  loop_args.cmd = cmd;

  if ((cmd != NULL) &&
      (cmd[2] != NULL) &&
      (!strcmp(cmd[2], "u")))
  {
    // Send an event to update the clock from all drivers
    VbEngineProcessAllDriversEvSend(ENGINE_EV_CLOCK_REQ, NULL);

    VbThreadSleep(MSEC_IN_SEC);
  }

  // Calculate clock deviations
  err = VbEngineClockDevCalc(&max_dev, &min_dev, &max_rtt);

  if (err == VB_ENGINE_ERROR_NONE)
  {
    struct timespec curr_time;
    CHAR             time_str[TIMESPEC_STR_LEN];

    // Show clock info report
    clock_gettime(CLOCK_REALTIME, &curr_time);

    // Get time string
    VbUtilTimespecToString(time_str, curr_time);

    writeFun("Current time  : [%13s]\n", time_str);
    writeFun("Max RTT       : %lld usec\n", NS_TO_US(max_rtt));
    writeFun("Max deviation : %lld ms\n", max_dev);
    writeFun("Min deviation : %lld ms\n", min_dev);
    writeFun("=============================================================================================================================================\n");
    writeFun("|             Driver           |    ownClock    | driverClockRx |    adjClock   | seqNum | valid |      RTT      |     devAbs   |   devRel  |\n");
    writeFun("=============================================================================================================================================\n");

    // Loop through existing drivers and dump report
    VbEngineDatamodelDriversLoop(VbEngineConsoleClockReportDriversCb, &loop_args);

    writeFun("=============================================================================================================================================\n");
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    if ((cmd != NULL) &&
        (cmd[2] != NULL) &&
        (!strcmp(cmd[2], "f")))
    {
      struct timespec future_time;
      struct timespec future_time_own_clock;
      INT16U           future_seq_num;

      // YYRR Review
      err = VbEngineClockFutureTSGet(&future_time, 0, &future_seq_num, &future_time_own_clock);

      if (err == VB_ENGINE_ERROR_NONE)
      {
        CHAR future_time_str[TIMESPEC_STR_LEN];
        CHAR future_time_own_clock_str[TIMESPEC_STR_LEN];

        VbUtilTimespecToString(future_time_str, future_time);
        VbUtilTimespecToString(future_time_own_clock_str, future_time_own_clock);

        writeFun("Future TS      : [%s]\n", future_time_str);
        writeFun("Future TS Own  : [%s]\n", future_time_own_clock_str);
        writeFun("Future Seq Num : [%6u]\n", future_seq_num);
      }
      else
      {
        writeFun("Error          : %d\n", err);
      }
    }
  }

  return TRUE;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleAlignInfoDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  t_VBAlignmentMode    alignment_mode;

  if ((driver == NULL) || (loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineConfAlignmentModeGet(&alignment_mode);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    struct timespec last_check_ts;
    struct timespec current_ts;
    INT64S           elapsed_ms;

    // Get current time
    clock_gettime(CLOCK_MONOTONIC, &current_ts);

    // Get last alignment check timestamp
    VbEngineAlignLastCheckTSGet(&last_check_ts, driver->clusterId);

    // Get elapsed time since last alignment check
    elapsed_ms = VbUtilElapsetimeTimespecMs(last_check_ts, current_ts);

    if (alignment_mode == VB_ALIGN_MODE_GHN)
    {
      loop_args->writeFun("|%-30s| %9u | %17s | %10s | %3u | %9u | %9u | %8u | %7.2f | %3s | %8s | %10llu ms |[%1c%03d %1c%03d %1c%03d %1c%03d %1c%03d]|\n",
          driver->vbDriverID,
          driver->clusterId,
          domain->dm.MACStr,
          VbEngineAlignRoleStringGet(domain->dm.nodeAlignInfo.role),
          domain->dm.devID,
          domain->alignInfo.macClock,
          domain->alignInfo.seqNum,
          VbEngineAlignGhnBestDetectedDidGet(domain),
          VbEngineAlignAdcOutRmsGet(domain),
          domain->dm.nodeAlignInfo.visible?"Y":"N",
          VbDevStateToStr(domain->dm.state),
          elapsed_ms,
          domain->alignInfo.syncDetsInfo[0].allowed?'*':' ',
          domain->alignInfo.syncDetsInfo[0].detDid,
          domain->alignInfo.syncDetsInfo[1].allowed?'*':' ',
          domain->alignInfo.syncDetsInfo[1].detDid,
          domain->alignInfo.syncDetsInfo[2].allowed?'*':' ',
          domain->alignInfo.syncDetsInfo[2].detDid,
          domain->alignInfo.syncDetsInfo[3].allowed?'*':' ',
          domain->alignInfo.syncDetsInfo[3].detDid,
          domain->alignInfo.syncDetsInfo[4].allowed?'*':' ',
          domain->alignInfo.syncDetsInfo[4].detDid);
    }
    else
    {
      loop_args->writeFun("|%-30s| %9u | %17s | %10s | %3u | %9u | %9u | %8s | %7s | %3s | %8s | %10llu ms |[%s]|\n",
          driver->vbDriverID,
          driver->clusterId,
          domain->dm.MACStr,
          "-",
          domain->dm.devID,
          domain->alignInfo.macClock,
          domain->alignInfo.seqNum,
          "-",
          "-",
          domain->dm.nodeAlignInfo.visible?"Y":"N",
          VbDevStateToStr(domain->dm.state),
          elapsed_ms,
          " ---  ---  ---  ---  ---");
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleAlignSyncDetDomainsCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((driver == NULL) || (loop_args == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args->writeFun("|%-30s| %9u | %17s | %10s | %3u | %4s | %8u | %7.2f | %3s | %4s |%4u| %8s |[(%03d|%05lu|%04ld|%6.2f|%1u) (%03d|%05lu|%04ld|%6.2f|%1u) (%03d|%05lu|%04ld|%6.2f|%1u) (%03d|%05lu|%04ld|%6.2f|%1u) (%03d|%05lu|%04ld|%6.2f|%1u)]|\n",
          driver->vbDriverID,
          driver->clusterId,
          domain->dm.MACStr,
          VbEngineAlignRoleStringGet(domain->dm.nodeAlignInfo.role),
          domain->dm.devID,
          domain->dm.nodeAlignInfo.synced?"Y":"N",
          VbEngineAlignGhnBestDetectedDidGet(domain),
          VbEngineAlignAdcOutRmsGet(domain),
          domain->dm.nodeAlignInfo.visible?"Y":"N",
          domain->dm.nodeAlignInfo.hasBeenCandidate?"Y":"N",
          domain->dm.nodeAlignInfo.hops,
          VbDevStateToStr(domain->dm.state),
          domain->alignInfo.syncDetsInfo[0].detDid, domain->alignInfo.syncDetsInfo[0].hitCount, domain->alignInfo.syncDetsInfo[0].reliability, VbEngineAlignAdcOutToFPGet(domain->alignInfo.syncDetsInfo[0].adcOutRms), domain->alignInfo.syncDetsInfo[0].allowed,
          domain->alignInfo.syncDetsInfo[1].detDid, domain->alignInfo.syncDetsInfo[1].hitCount, domain->alignInfo.syncDetsInfo[1].reliability, VbEngineAlignAdcOutToFPGet(domain->alignInfo.syncDetsInfo[1].adcOutRms), domain->alignInfo.syncDetsInfo[1].allowed,
          domain->alignInfo.syncDetsInfo[2].detDid, domain->alignInfo.syncDetsInfo[2].hitCount, domain->alignInfo.syncDetsInfo[2].reliability, VbEngineAlignAdcOutToFPGet(domain->alignInfo.syncDetsInfo[2].adcOutRms), domain->alignInfo.syncDetsInfo[2].allowed,
          domain->alignInfo.syncDetsInfo[3].detDid, domain->alignInfo.syncDetsInfo[3].hitCount, domain->alignInfo.syncDetsInfo[3].reliability, VbEngineAlignAdcOutToFPGet(domain->alignInfo.syncDetsInfo[3].adcOutRms), domain->alignInfo.syncDetsInfo[3].allowed,
          domain->alignInfo.syncDetsInfo[4].detDid, domain->alignInfo.syncDetsInfo[4].hitCount, domain->alignInfo.syncDetsInfo[4].reliability, VbEngineAlignAdcOutToFPGet(domain->alignInfo.syncDetsInfo[4].adcOutRms), domain->alignInfo.syncDetsInfo[4].allowed);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleAlignInfoReport(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL                 ret = TRUE;
  t_consoleLoopArgs    loop_args;
  t_VBAlignmentMode    alignment_mode;

  if (writeFun == NULL)
  {
    ret = FALSE;
  }

  if (ret == TRUE)
  {
    t_VB_engineErrorCode err;

    err = VbEngineConfAlignmentModeGet(&alignment_mode);

    if (err != VB_ENGINE_ERROR_NONE)
    {
      ret = FALSE;
    }
  }

  if (ret == TRUE)
  {
    loop_args.writeFun = writeFun;
    loop_args.cmd = cmd;

    loop_args.writeFun("Alignment mode    : %s\n", VbEngineAlignModeStringGet(alignment_mode));

    loop_args.writeFun("============================================================================================================================================================================================\n");
    loop_args.writeFun("|           Driver             | ClusterID |        DM         |    Role    | DID | MAC Clock |  Seq Num  | Best Did | Rx (dB) | Vis | DevState |  TSinceCheck  |[        sync det        ]|\n");
    loop_args.writeFun("============================================================================================================================================================================================\n");

    // Loop through existing domains and dump report
    VbEngineDatamodelAllDomainsLoop(VbEngineConsoleAlignInfoDomainsCb, &loop_args);

    loop_args.writeFun("============================================================================================================================================================================================\n");
  }

  return TRUE;
}

/*******************************************************************/

static BOOL VbEngineConsoleAlignSyncDetReport(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL                 ret = TRUE;
  t_consoleLoopArgs    loop_args;
  t_VBAlignmentMode    alignment_mode;

  if (writeFun == NULL)
  {
    ret = FALSE;
  }

  if (ret == TRUE)
  {
    t_VB_engineErrorCode err;

    err = VbEngineConfAlignmentModeGet(&alignment_mode);

    if (err != VB_ENGINE_ERROR_NONE)
    {
      ret = FALSE;
    }
  }

  if (ret == TRUE)
  {
    if (alignment_mode == VB_ALIGN_MODE_GHN)
    {
      loop_args.writeFun = writeFun;
      loop_args.cmd = cmd;

      loop_args.writeFun("================================================================================================================================================================================================================================================================================\n");
      loop_args.writeFun("|           Driver             | ClusterID |        DM         |    Role    | DID |Synced| Best Did | Rx (dB) | Vis | Cand |Hops| DevState |[                                                               sync det                                                          ]|\n");
      loop_args.writeFun("================================================================================================================================================================================================================================================================================\n");

      // Loop through existing domains and dump report
      VbEngineDatamodelAllDomainsLoop(VbEngineConsoleAlignSyncDetDomainsCb, &loop_args);

      loop_args.writeFun("================================================================================================================================================================================================================================================================================\n");
    }
    else
    {
      ret = VbEngineConsoleAlignInfoReport(arg, writeFun, cmd);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleAlignSyncDetByClusterDriversCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Loop through existing domains
    ret = VbEngineDatamodelDomainsLoop(driver, VbEngineConsoleAlignSyncDetDomainsCb, loop_args);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleAlignSyncDetByClusterReport(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL                   ret = TRUE;
  t_VBAlignmentMode      alignment_mode;

  if (writeFun == NULL)
  {
    ret = FALSE;
  }

  if (ret == TRUE)
  {
    t_VB_engineErrorCode err;

    err = VbEngineConfAlignmentModeGet(&alignment_mode);

    if (err != VB_ENGINE_ERROR_NONE)
    {
      ret = FALSE;
    }
  }

  if (ret == TRUE)
  {
    if (alignment_mode == VB_ALIGN_MODE_GHN)
    {
      t_consoleLoopArgs      loop_args;
      INT32U                 num_clusters;
      INT32U                *cluster_id_list_ptr = NULL;
      t_clusterListLoopArgs  cluster_id_list_arg;

      if (ret == TRUE)
      {
        num_clusters = VbEngineDataModelNumClustersGet();
        cluster_id_list_ptr = malloc(num_clusters*sizeof(INT32U));

        if (cluster_id_list_ptr == NULL)
        {
          writeFun("Error allocating memory!\n");
          ret = FALSE;
        }
      }

      if (ret == TRUE)
      {
        t_VB_engineErrorCode vb_err;

        cluster_id_list_arg.numClusters = num_clusters;
        cluster_id_list_arg.clusterIdList  = cluster_id_list_ptr;
        cluster_id_list_arg.clusterIdx = 0;

        vb_err = VbEngineDatamodelClustersLoop(ClusterIdListCb, &cluster_id_list_arg);

        if (vb_err != VB_ENGINE_ERROR_NONE)
        {
          ret = FALSE;
        }
      }

      if (ret == TRUE)
      {
        INT32U idx;

        loop_args.writeFun = writeFun;
        loop_args.cmd = cmd;

        writeFun("Num clusters = %lu\n", num_clusters);
        loop_args.writeFun("================================================================================================================================================================================================================================================================================\n");
        loop_args.writeFun("|           Driver             | ClusterID |        DM         |    Role    | DID |Synced| Best Did | Rx (dB) | Vis | Cand |Hops| DevState |[                                                               sync det                                                          ]|\n");
        loop_args.writeFun("================================================================================================================================================================================================================================================================================\n");

        for (idx = 0; idx < num_clusters; idx++)
        {
          INT32U cluster_id;

          cluster_id = cluster_id_list_ptr[idx];

          // Loop through existing domains and dump report
          VbEngineDatamodelClusterXDriversLoop(VbEngineConsoleAlignSyncDetByClusterDriversCb, cluster_id, &loop_args);
        }

        loop_args.writeFun("================================================================================================================================================================================================================================================================================\n");
      }

      if (cluster_id_list_ptr != NULL)
      {
        free(cluster_id_list_ptr);
        cluster_id_list_ptr = NULL;
      }
    }
    else
    {
      ret = VbEngineConsoleAlignInfoReport(arg, writeFun, cmd);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AlignIdListCb(t_VBCluster *cluster, void *args)
{
  t_VB_engineErrorCode    ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs      *loop_args = (t_consoleLoopArgs *)args;

  if ((cluster == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT8U align_id = 0;

    VbEngineAlignmentIdGet(0, cluster, &align_id);

    loop_args->writeFun("| %9u | %11u |\n", cluster->clusterInfo.clusterId, align_id);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleAlignIdReport(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL                   ret = TRUE;
  t_consoleLoopArgs      loop_args;

  if (writeFun == NULL)
  {
    ret = FALSE;
  }

  if (ret == TRUE)
  {
    loop_args.writeFun = writeFun;
    loop_args.cmd = cmd;

    loop_args.writeFun("===========================\n");
    loop_args.writeFun("| ClusterID |   AlignId   |\n");
    loop_args.writeFun("===========================\n");

    VbEngineDatamodelClustersLoop(AlignIdListCb, &loop_args);

    loop_args.writeFun("===========================\n");
  }

  return TRUE;
}

/*******************************************************************/

static BOOL VbEngineConsoleVers(void *arg, t_writeFun writeFun, char **cmd)
{
  writeFun("%s\n", VB_ENGINE_VERSION);

  return TRUE;
}

/*******************************************************************/

static BOOL VbEngineConsoleReport(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] == NULL)
  {
    // Basic report
    ret = VbEngineConsoleBasicReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "clu"))
  {
    // Clusters report
    ret = VbEngineConsoleBasicReportCluster(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "tr"))
  {
    // Traffic report
    ret = VbEngineConsoleTrafficReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "trbps"))
  {
    // Traffic report
    ret = VbEngineConsoleTrafficBpsReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "i"))
  {
    // Additional info report
    ret = VbEngineConsoleAddInfoReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "b"))
  {
    // Boost report
    ret = VbEngineConsoleBoostReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "meas"))
  {
    // Measure report
    ret = VbEngineConsoleMeasReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "thr"))
  {
    // Threads report
    VbThreadListThreadDump(writeFun);
    // Timer tasks report
    VbTimerListTaskDump(writeFun);
    ret = TRUE;
  }
  else if (!strcmp(cmd[1], "cdta"))
  {
     // Traffic report
     ret = VbEngineConsoleCdtaReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "h"))
  {
    show_help = TRUE;
    ret = TRUE;
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("report                          : Shows basic report\n");
    writeFun("report clu                      : Shows basic report per clusters\n");
    writeFun("report h                        : Shows this help\n");
    writeFun("report tr                       : Shows traffic report\n");
    writeFun("report trbps                    : Shows traffic (bps related) report\n");
    writeFun("report i                        : Shows additional domains info\n");
    writeFun("report b                        : Shows boost info\n");
    writeFun("report meas                     : Shows measure info\n");
    writeFun("report thr                      : Shows threads info\n");
    writeFun("report cdta xput down/up        : Shows cdta xput info\n");
    writeFun("report cdta nbands down/up      : Shows cdta bands info\n");
    writeFun("report cdta cap down/up         : Shows cdta capacity info\n");
    writeFun("report cdta metrics down/up     : Shows cdta metrics info\n");
    writeFun("report cdta profiles down/up    : Shows cdta profiles info\n");
    writeFun("report cdta bpsReported down/up : Shows cdta bps reported info\n");
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleDriversGenericHelpCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args->writeFun(loop_args->formatStr, driver->vbDriverID, driver->vbDriverID);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleDriverCheckState(t_VBDriver *thisDriver, void *args)
{
  t_vbEngineProcessFSMState  state_to_check;
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;

  if ((args == NULL) || (thisDriver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    state_to_check = *((t_vbEngineProcessFSMState *)args);

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

      VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "Not in desired state (%s -> %s)",
          FSMSttToStrGet(thisDriver->FSMState), FSMSttToStrGet(state_to_check));
    }
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleMeasure(void *arg, t_writeFun writeFun, char **cmd)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  t_vbEngineProcessFSMState args;
  BOOL                 ret = FALSE;
  BOOL                 meas_done = FALSE;
  BOOL                 show_help = FALSE;
  INT32U               counter;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "conf"))
    {
      VbEngineMeasureConfDump(writeFun);
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "d"))
    {
      INT32U clusterId;

      if(cmd[2] != NULL)
      {
        clusterId = strtoul(cmd[2], NULL, 0);
        VbEngineMeasurePlanDump(writeFun,clusterId);
        ret = TRUE;
      }
      else
      {
        ret = FALSE;
      }
    }
    else if (!strcmp(cmd[1], "save"))
    {
      INT32U clusterId;

      if(cmd[2] != NULL)
      {
        clusterId = strtoul(cmd[2], NULL, 0);
        VbEngineMeasureSave(clusterId);
        ret = TRUE;
      }
      else
      {
        ret = FALSE;
      }
    }
    else if (!strcmp(cmd[1], "f"))
    {
      struct timespec t0;
      struct timespec t1;
      INT64S elapsed_time_ms;
      INT32U clusterId;

      if(cmd[2] != NULL)
      {
        clusterId = strtoul(cmd[2], NULL, 0);

        clock_gettime(CLOCK_MONOTONIC, &t0);

        // Force measure
        err = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEAS_FORCE, NULL, clusterId);

        writeFun("Forcing new measure on all drivers ");

        VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Measure plan requested by user on all drivers");

        // We will wait until all drivers reach this state
        args = ENGINE_STT_BOOSTING_WAIT_TRGS;

        meas_done = FALSE;
        counter = 0;
        while (!meas_done)
        {
          VbThreadSleep(1000);
          writeFun(".");
          err = VbEngineDatamodelClusterXDriversLoop(VbEngineConsoleDriverCheckState, clusterId, &args);
          meas_done = (err == VB_ENGINE_ERROR_NONE)? TRUE : FALSE;
          // Give 30 sec timeout
          if(counter++ > 30)
          {
            writeFun("\nTimeout Error performing Measure Plan!\n");
            break;
          }
        }

        clock_gettime(CLOCK_MONOTONIC, &t1);
        elapsed_time_ms = VbUtilElapsetimeTimespecMs(t0, t1);

        writeFun("done (elapsed %u msecs)\n", elapsed_time_ms);

        if (meas_done == TRUE)
        {
          ret = TRUE;
        }
      }
      else
      {
        ret = FALSE;
      }
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("meas h         : Shows this help\n");
    writeFun("meas conf      : Shows measure configuration\n");
    writeFun("meas d num     : Shows last measure plan created in cluster num\n");
    writeFun("meas f mun     : Force measure plan in cluster num\n");
    writeFun("meas save num  : Saves last measure of cluster num to disk\n");
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleDriverSNRProbeReqCb(t_VBDriver *thisDriver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((thisDriver == NULL) || (loop_args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Force measure
    loop_args->writeFun("Executing SNR probes measure on driver '%s'.\n", thisDriver->vbDriverID);
    VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "SNR probes measure requested by user");
    ret = VbEngineProcessEvSend(thisDriver, ENGINE_EV_SNR_PROBES_MEAS_FORCE, loop_args->data);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleSnrProbesMeasure(void *arg, t_writeFun writeFun, char **cmd)
{
  t_VB_engineErrorCode      err = VB_ENGINE_ERROR_NONE;
  t_VBDriver                *driver = NULL;
  BOOL                      ret = FALSE;
  BOOL                      show_help = FALSE;
  INT8U                     node_mac[ETH_ALEN];

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "a"))
    {
      t_consoleLoopArgs loop_args;

      loop_args.data = NULL;
      loop_args.writeFun = writeFun;

      err = VbEngineDatamodelDriversLoop(VbEngineConsoleDriverSNRProbeReqCb, &loop_args);

      if (err == VB_ENGINE_ERROR_NONE)
      {
        INT32U                    counter;
        BOOL                      meas_done = FALSE;
        t_vbEngineProcessFSMState args;

        // We will wait until all drivers reach this state
        args = ENGINE_STT_BOOSTING_WAIT_TRGS;

        counter = 0;
        while (!meas_done)
        {
          VbThreadSleep(1000);
          writeFun(".");

          err = VbEngineDatamodelDriversLoop(VbEngineConsoleDriverCheckState, &args);
          meas_done = (err == VB_ENGINE_ERROR_NONE)? TRUE : FALSE;

          // Give 30 sec timeout
          if (counter++ > 30)
          {
            writeFun("\nTimeout Error performing SNR probes measurement\n");
            break;
          }
        }
      }

      if (err == VB_ENGINE_ERROR_NONE)
      {
        ret = TRUE;
      }
      else
      {
        ret = FALSE;
      }
    }
    else if (cmd[1] != NULL)
    {
      err = VbEngineDriverByIdGet(cmd[1], &driver);

      if (err == VB_ENGINE_ERROR_NONE)
      {
        ret = TRUE;
      }
      else
      {
        writeFun("Driver <%s> not found (err %d)\n", cmd[1], err);
        ret = FALSE;
      }

      if (ret == TRUE)
      {
        t_consoleLoopArgs loop_args;

        loop_args.writeFun = writeFun;

        // Force measure
        if (cmd[2] == NULL)
        {
          loop_args.data = NULL;
        }
        else
        {
          MACAddrStr2mem(node_mac, cmd[2]);
          loop_args.data = node_mac;
        }

        err = VbEngineConsoleDriverSNRProbeReqCb(driver, &loop_args);

        if (err == VB_ENGINE_ERROR_NONE)
        {
          INT32U                    counter;
          BOOL                      meas_done = FALSE;
          t_vbEngineProcessFSMState args;

          // We will wait until all drivers reach this state
          args = ENGINE_STT_BOOSTING_WAIT_TRGS;

          counter = 0;
          while (!meas_done)
          {
            VbThreadSleep(1000);
            writeFun(".");

            err = VbEngineConsoleDriverCheckState(driver, &args);
            meas_done = (err == VB_ENGINE_ERROR_NONE)? TRUE : FALSE;

            // Give 30 sec timeout
            if (counter++ > 30)
            {
              writeFun("\nTimeout Error performing SNR probes measurement\n");
              break;
            }
          }
        }

        if (err != VB_ENGINE_ERROR_NONE)
        {
          ret = FALSE;
        }
      }
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    t_consoleLoopArgs    loop_args;

    writeFun("Usage:\n");
    writeFun("snr_probes h : Shows this help\n");
    writeFun("snr_probes a : Force SNR measure with PROBEs for all drivers\n");

    loop_args.writeFun = writeFun;
    loop_args.formatStr = "snr_probes %s : Force SNR measure with PROBEs for driver <%s>\n";

    // Loop through existing drivers and dump help
    VbEngineDatamodelDriversLoop(VbEngineConsoleDriversGenericHelpCb, &loop_args);
    writeFun("snr_probes <driverId> <MAC> : Force SNR measure with PROBEs for a specific driver and MAC\n");
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleExecutePSD(void *arg, t_writeFun writeFun, char **cmd)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  t_VBDriver          *driver = NULL;
  t_consoleLoopArgs    loop_args;
  BOOL                 ret = FALSE;
  BOOL                 show_help = FALSE;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      err = VbEngineDriverByIdGet(cmd[1], &driver);

      if (err == VB_ENGINE_ERROR_NONE)
      {
        ret = TRUE;
      }
      else
      {
        writeFun("Driver <%s> not found (err %d)\n", cmd[1], err);
        ret = FALSE;
      }

      if (ret == TRUE)
      {
        // Send event to the FSM
        err = VbEngineProcessAllDriversEvSend(ENGINE_EV_PSD_UPDATE_FORCE, driver);

        writeFun("Forcing a PSD update on driver '%s'.\n", driver->vbDriverID);

        VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "PSD update requested by user (%ld)", err);
      }
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("psd h : Shows this help\n");

    loop_args.writeFun = writeFun;
    loop_args.formatStr = "psd %s : Force a PSD update for driver <%s>\n";

    // Loop through existing drivers and dump help
    VbEngineDatamodelDriversLoop(VbEngineConsoleDriversGenericHelpCb, &loop_args);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleForceBoostMode(void *arg, t_writeFun writeFun, char **cmd)
{
  t_VB_engineErrorCode   err = VB_ENGINE_ERROR_NONE;
  t_vbEngineBoostMode    boost_mode = VB_ENGINE_BOOST_MODE_AUTO;
  INT8U                  node_mac[ETH_ALEN];
  BOOL                   ret = FALSE;
  BOOL                   show_help = FALSE;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else if (cmd[2] != NULL)
    {
      MACAddrStr2mem(node_mac, cmd[1]);

      ret = TRUE;
      if (!strcmp(cmd[2], "a"))
      {
        boost_mode = VB_ENGINE_BOOST_MODE_AUTO;
      }
      else if (!strcmp(cmd[2], "f"))
      {
        boost_mode = VB_ENGINE_BOOST_MODE_FORCED_FULL;
      }
      else if (!strcmp(cmd[2], "l"))
      {
        boost_mode = VB_ENGINE_BOOST_MODE_FORCED_LOW;
      }
      else
      {
        writeFun("Incorrect argument <%s>\n", cmd[2]);
        ret = FALSE;
      }

      if (ret == TRUE)
      {
        err = VbEngineBoostModeByMacSet(node_mac, boost_mode);

        if (err != VB_ENGINE_ERROR_NONE)
        {
          ret = FALSE;
          writeFun("Error %d\n", err);
        }
      }
    }
    else
    {
      ret = FALSE;
      writeFun("Invalid args\n");
    }
  }
  else
  {
    ret = FALSE;
    writeFun("Invalid args\n");
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("boost h       : Shows this help\n");
    writeFun("boost <MAC> f : Force given node in full band mode\n");
    writeFun("boost <MAC> l : Force given node in low band mode\n");
    writeFun("boost <MAC> a : Force given node in auto mode\n");
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleKill(void *arg, t_writeFun writeFun, char **cmd)
{
  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Exit requested from console");
  writeFun("Exiting...\n");

  VbEngineKill();

  return TRUE;
}

/*******************************************************************/

static BOOL VbEngineConfConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL                   ret = FALSE;
  BOOL                   show_help = FALSE;
  INT32U                 boost_thr[VB_BOOST_THR_TYPE_LAST];

  if ((cmd[1] != NULL) && (cmd[2] != NULL) && (cmd[3] != NULL))
  {
    if (!strcmp(cmd[1], "bthr"))
    {
      boost_thr[VB_BOOST_THR_TYPE_DEC_BOOST] = strtoul(cmd[2], NULL, 0);
      boost_thr[VB_BOOST_THR_TYPE_INC_BOOST] = strtoul(cmd[3], NULL, 0);

      VbEngineConfBoostThrSet(boost_thr);
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
      writeFun("Invalid args\n");
    }
  }
  else if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
      writeFun("Invalid args\n");
    }
  }
  else
  {
    VbEngineConfDump(writeFun);
    ret = TRUE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("conf h                      : Shows this help\n");
    writeFun("conf                        : Dump current configuration\n");
    writeFun("conf bthr <decThr> <incThr> : Configures boost thresholds\n");
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleBasicDriversReportLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;
  INT32U               num_domains = 0;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    num_domains = driver->domainsList.numDomains;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    CHAR str_addr[INET6_ADDRSTRLEN];

    if (inet_ntop(AF_INET6, &driver->vbEAConnDesc.clientAddr.sin6_addr, str_addr, sizeof(str_addr)) == NULL)
    {
      strcpy(str_addr,"unknown");
    }

    loop_args->writeFun("| %20s |%33s | %20s |%15s | %15s |  %5u |            %2u |  %8u |%20s | %12u ms |\n",
        driver->vbDriverID,
        FSMSttToStrGet(driver->FSMState),
        driver->remoteState,
        driver->remoteVersion,
        str_addr,
        driver->vbEAConnDesc.clientAddr.sin6_port,
        num_domains,
        driver->clusterId,
        driver->vbEAConnDesc.thrName,
        driver->transactionMinTime);
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleBasicDriversReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs    loop_args;
  t_vbEngineNumNodes   numNodes;

  loop_args.writeFun = writeFun;

  VbEngineDataModelNumNodesGet(&numNodes);

  loop_args.writeFun("Drivers connected   : %u\n", VbEngineDataModelNumDriversGet());
  loop_args.writeFun("Num DMs             : %u\n", numNodes.numDms);
  loop_args.writeFun("Num EPs             : %u\n", numNodes.numEps);
  loop_args.writeFun("Num Complete Lines  : %u\n", numNodes.numCompleteLines);

  loop_args.writeFun("==================================================================================================================================================================================================\n");
  loop_args.writeFun("|       Driver Id      |             FSMState             |     Driver State     | Driver version |    IP Address   |  Port  | Nr of domains | ClusterId |    Process name     | TransactionTime |\n");
  loop_args.writeFun("==================================================================================================================================================================================================\n");

  // Loop through existing drivers and dump drivers report
  VbEngineDatamodelDriversLoop(VbEngineConsoleBasicDriversReportLoopCb, &loop_args);

  loop_args.writeFun("==================================================================================================================================================================================================\n");

  return TRUE;
}


/*******************************************************************/

static t_VB_engineErrorCode DriverIdPerClusterLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((driver == NULL) || (loop_args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBCluster         *cluster = (t_VBCluster *)(loop_args->data);

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      loop_args->writeFun("| %9d |%7d | %8d |%10d | %11x | %11x |[ %3u %3u %3u %3u ]|[ %3u %3u %3u %3u ]| %20s | %33s |\n",
            cluster->clusterInfo.clusterId,
            cluster->clusterInfo.alignRef,
            cluster->clusterInfo.numLines,
            cluster->clusterInfo.numRelays,
            cluster->measurePlanData,
            cluster->cdtaData,
            cluster->clusterInfo.relays[0], cluster->clusterInfo.relays[1], cluster->clusterInfo.relays[2], cluster->clusterInfo.relays[3],
            cluster->clusterInfo.relaysCandidate[0], cluster->clusterInfo.relaysCandidate[1], cluster->clusterInfo.relaysCandidate[2], cluster->clusterInfo.relaysCandidate[3],
            driver->vbDriverID,
            FSMSttToStrGet(driver->FSMState)
          );
    }
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleBasicClustersReportI(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL                   ret = TRUE;
  t_consoleLoopArgs      loop_args;
  INT32U                 num_clusters;
  INT32U                *cluster_id_list_ptr = NULL;
  t_clusterListLoopArgs  cluster_id_list_arg;

  if (writeFun == NULL)
  {
    ret = FALSE;
  }

  if (ret == TRUE)
  {
    num_clusters = VbEngineDataModelNumClustersGet();
    cluster_id_list_ptr = malloc(num_clusters*sizeof(INT32U));

    if (cluster_id_list_ptr == NULL)
    {
      writeFun("Error allocating memory!\n");
      ret = FALSE;
    }
  }

  if (ret == TRUE)
  {
    t_VB_engineErrorCode vb_err;

    cluster_id_list_arg.numClusters = num_clusters;
    cluster_id_list_arg.clusterIdList  = cluster_id_list_ptr;
    cluster_id_list_arg.clusterIdx = 0;

    vb_err = VbEngineDatamodelClustersLoop(ClusterIdListCb, &cluster_id_list_arg);

    if (vb_err != VB_ENGINE_ERROR_NONE)
    {
      ret = FALSE;
    }
  }

  if (ret == TRUE)
  {
    INT32U idx;

    loop_args.writeFun = writeFun;

    loop_args.writeFun("Clusters established: %u\n", num_clusters);

    loop_args.writeFun("============================================================================================================================================================================\n");
    loop_args.writeFun("| ClusterId | RefDid | numLines | numRelays | MeasPlanPtr |   cdtaPtr   |   [relay 0...3   ]| [relay Cand 0..3 ]|       Driver Id      |             FSMState              |\n");
    loop_args.writeFun("============================================================================================================================================================================\n");

    for (idx = 0; idx < num_clusters; idx++)
    {
      t_VB_engineErrorCode    cluster_err;
      INT32U                  cluster_id;
      t_VBCluster            *cluster_ptr;

      // Get next cluster Id
      cluster_id = cluster_id_list_ptr[idx];

      // Get cluster ptr
      cluster_err = VbEngineClusterByIdGet(cluster_id, &cluster_ptr);

      if (cluster_err == VB_ENGINE_ERROR_NONE)
      {
        loop_args.data = cluster_ptr;

        // Loop through existing drivers and dump clusters report
        VbEngineDatamodelClusterXDriversLoop(DriverIdPerClusterLoopCb, cluster_id, &loop_args);
      }
    }

    loop_args.writeFun("============================================================================================================================================================================\n");
  }

  if (cluster_id_list_ptr != NULL)
  {
    free(cluster_id_list_ptr);
    cluster_id_list_ptr = NULL;
  }

  return TRUE;
}
/*******************************************************************/

static BOOL VbEngineConsoleDriversReport(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] == NULL)
  {
    // Basic drivers report
    ret = VbEngineConsoleBasicDriversReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "h"))
  {
    show_help = TRUE;
    ret = TRUE;
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("drivers        : Shows basic VB drivers report\n");
    writeFun("drivers h      : Shows this help\n");
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleClustersReport(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] == NULL)
  {
    ret = FALSE;
  }
  else if (!strcmp(cmd[1], "i"))
  {
    VbEngineConsoleBasicClustersReportI(arg, writeFun, cmd);
    ret = TRUE;
  }
  else if (!strcmp(cmd[1], "evtALL"))
  {
    VbEngineProcessAllDriversEvSend(ENGINE_EV_ALIGN_ALL_CLUSTERS, NULL);
    ret = TRUE;
  }
  else if (!strcmp(cmd[1], "evtI"))
  {
    INT32U cluster_id;

    cluster_id = strtoul(cmd[2], NULL, 0);
    VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CLUSTER_I, NULL, cluster_id);
    ret = TRUE;
  }
  else if (!strcmp(cmd[1], "h"))
  {
    show_help = TRUE;
    ret = TRUE;
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("clusters i         : Shows clusters report\n");
    writeFun("clusters evtALL    : Send Evt ALIGN ALL \n");
    writeFun("clusters evtI <x>  : Send Evt ALIGN I to cluster x\n");
    writeFun("clusters h         : Shows this help\n");
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineConsoleAlignCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] == NULL)
  {
    ret = VbEngineConsoleAlignInfoReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "h"))
  {
    show_help = TRUE;
    ret = TRUE;
  }
  else if (!strcmp(cmd[1], "s"))
  {
    // Extended align report
    ret = VbEngineConsoleAlignSyncDetReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "sc"))
  {
    // Extended align report by cluster Id
    ret = VbEngineConsoleAlignSyncDetByClusterReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "id"))
  {
    // Align Id by clusters
    ret = VbEngineConsoleAlignIdReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "c"))
  {
    // Clock report
    ret = VbEngineConsoleClockReport(arg, writeFun, cmd);
  }
  else if ((!strcmp(cmd[1], "f")) && (cmd[2] != NULL))
  {
    INT32U cluster_id = strtoul(cmd[2], NULL, 0);
    VbEngineAlignPeriodicCheckForce(cluster_id);
    writeFun("Forcing alignment check with cluster Id %lu\n", cluster_id);
    ret = TRUE;
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("align          : Dumps last alignment info\n");
    writeFun("align h        : Shows this help\n");
    writeFun("align c        : Shows clock info\n");
    writeFun("align c u      : Forces clock info update\n");
    writeFun("align c f      : Shows future clock timestamp\n");
    writeFun("align f <id>   : Forces an alignment check with given cluster Id\n");
    writeFun("align s        : Shows complete align sync info\n");
    writeFun("align sc       : Shows complete align sync info ordered by cluster Id\n");
    writeFun("align id       : Shows last Align Ids by cluster\n");
  }

  return ret;
}

/************************************************************************/
static BOOL VbMetricsConsoleHandler(void *arg, void (*write_fun)(const char *fmt, ...), char **cmd)
{
  BOOL                 ret = FALSE;
  BOOL                 show_help = FALSE;
  INT32S               param2;

  if (cmd[1] != NULL)
  {
    if (cmd[2] != NULL)
    {
      param2 = atoi(cmd[2]);
    }
    else
    {
      param2 = 0x7FFFFFFF; // No argument
    }

    if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "start"))
    {
      VbEngineMetricsStart();
      write_fun("\n Metrics capture started\n");
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "stop"))
    {
      VbEngineMetricsStop();
      write_fun("\n Metrics capture stopped\n");
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "d"))
    {
      VbMetricsDumpAllEventsList(write_fun);
      write_fun("\n Metrics logs file         : %s", VbMetricsGetReportsPath());
      write_fun("\n Metrics max log file size : %u KB", (unsigned int)VbMetricsGetMaxLogSize());
      write_fun("\n Metrics save to disk      : %u", VbMetricsGetSaveMetricsToDisk());
      write_fun("\n Enable CPU timing reports : %u", VbMetricsGetEnableCPUTimingMetrics());
      write_fun("\n Enable traffic'n boost rep: %u\n", VbMetricsGetEnableBoostTrafficMetrics());

      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "dis"))
    {
      // Check valid event value
      switch(param2)
      {
      case 0:
        VbMetricsSetEnableCPUTimingMetrics(FALSE);
        break;
      case 1:
        VbMetricsSetEnableBoostTrafficMetrics(FALSE);
        break;
      default:
        write_fun("Invalid event id\n");
        break;
      }
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "en"))
    {
      switch(param2)
      {
      case 0:
        VbMetricsSetEnableCPUTimingMetrics(TRUE);
        break;
      case 1:
        VbMetricsSetEnableBoostTrafficMetrics(TRUE);
        break;
      default:
        write_fun("Invalid event id\n");
        break;
      }
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "max"))
    {
      VbMetricsSetMaxLogSize(param2);
      write_fun("\nMaximum log file size set to : %dKB\n", (int)param2);
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "r"))
    {
      VbMetricsCallReport((t_VBMetricsEventType) param2,  write_fun);
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "c"))
    {
      VbEngineMetricsResetMetrics();
      ret = TRUE;
    }
    else if(!strcmp(cmd[1], "f"))
    {
      VbEngineDumpMetricsToFiles();
      ret = TRUE;
    }
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    write_fun("Usage:\n");
    write_fun("metrics h          : Shows this help\n");
    write_fun("metrics start/stop : Start/stop capture \n");
    write_fun("metrics d          : Debug information \n");
    write_fun("metrics ev <n>     : Show a list of events of type <n>\n");
    write_fun("metrics en/dis <n> : Enable or Disable saving of events of type <n>  where n:\n");
    write_fun("\t\t0 - CPU timing\n\t1 - Traffic and boost\n");
    write_fun("metrics c          : Clear the events list \n");
    write_fun("metrics f          : Dump all reports to files \n");
    write_fun("metrics r <n>      : Show metrics reports where n: \n");

    VbMetricsPrintReportList(write_fun);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleEAMsgDumpDriver(t_VBDriver *driver, t_writeFun writeFun)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEADbgTable       table;

  if ((writeFun == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&(driver->vbEAConnDesc.mutex));
    memcpy(&table, &(driver->vbEAConnDesc.debugTable), sizeof(t_vbEADbgTable));
    pthread_mutex_unlock(&(driver->vbEAConnDesc.mutex));

    writeFun("Driver Id     : %s\n", driver->vbDriverID);

    // Dump EA statistics
    VbEADbgMsgDump(&table, writeFun);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleEAMsgDumpCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if (loop_args == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineConsoleEAMsgDumpDriver(driver, loop_args->writeFun);
    loop_args->writeFun("\n");
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleEAMsgCalculateCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEADbgTable   *table;
  INT32U i;

  if (args == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    table = (t_vbEADbgTable *)args;
    // Dump LCMP statistics
    pthread_mutex_lock(&(driver->vbEAConnDesc.mutex));

    for(i = 0; i < VB_EA_OPCODE_LAST; i++)
    {
      table->rxTable[i].cnt += driver->vbEAConnDesc.debugTable.rxTable[i].cnt;
      if(VbUtilTimespecCmp(&(table->rxTable[i].timeStamp), &(driver->vbEAConnDesc.debugTable.rxTable[i].timeStamp)) < 0)
      {
        table->rxTable[i].timeStamp = driver->vbEAConnDesc.debugTable.rxTable[i].timeStamp;
      }
      table->txTable[i].cnt += driver->vbEAConnDesc.debugTable.txTable[i].cnt;
      if(VbUtilTimespecCmp(&(table->txTable[i].timeStamp), &(driver->vbEAConnDesc.debugTable.txTable[i].timeStamp)) < 0)
      {
        table->txTable[i].timeStamp = driver->vbEAConnDesc.debugTable.txTable[i].timeStamp;
      }
    }

    pthread_mutex_unlock(&(driver->vbEAConnDesc.mutex));
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineConsoleEAMsgResetCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_consoleLoopArgs   *loop_args = (t_consoleLoopArgs *)args;

  if ((loop_args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args->writeFun(".");

    pthread_mutex_lock(&(driver->vbEAConnDesc.mutex));
    // Dump LCMP statistics
    VbEADbgMsgReset(&(driver->vbEAConnDesc.debugTable));

    pthread_mutex_unlock(&(driver->vbEAConnDesc.mutex));

    loop_args->writeFun(".");
  }

  return ret;
}
/*******************************************************************/

static BOOL VbEngineEAConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;
  t_consoleLoopArgs    loop_args;

  if (cmd[1] != NULL)
  {
    loop_args.writeFun = writeFun;

    if (!strcmp(cmd[1], "i"))
    {
      t_VBDriver *driver;
      if (cmd[2] != NULL)
      {
        if(VbEngineDriverByIdGet(cmd[2], &driver) != VB_ENGINE_ERROR_NONE)
        {
          writeFun("Error, driver %s not found\n", cmd[2]);
        }
        else
        {
          VbEngineConsoleEAMsgDumpDriver(driver, writeFun);
          ret = TRUE;
        }
      }
      else
      {
        // Loop through existing drivers and dump report
        VbEngineDatamodelDriversLoop(VbEngineConsoleEAMsgDumpCb, &loop_args);
        ret = TRUE;
      }
    }
    else if (!strcmp(cmd[1], "t"))
    {
      t_vbEADbgTable   table;

      VbEADbgMsgReset(&table);
      // Loop through existing drivers and dump report
      VbEngineDatamodelDriversLoop(VbEngineConsoleEAMsgCalculateCb, &table);

      writeFun("Total Counters report\n=====================\n");
      // Dump LCMP statistics
      VbEADbgMsgDump(&table, writeFun);

      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "r"))
    {
      // Reset counters
      loop_args.writeFun = writeFun;
      loop_args.writeFun("Reset tables");
      // Loop through existing drivers and dump report
      VbEngineDatamodelDriversLoop(VbEngineConsoleEAMsgResetCb, &loop_args);
      loop_args.writeFun(" Done!\n");
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("ea h            : Shows this help\n");
    writeFun("ea t            : Shows total Tx/Rx messages counters\n");
    writeFun("ea i            : Shows Tx/Rx messages per driver\n");
    writeFun("ea i <driverId> : Shows Tx/Rx messages of driverId\n");
    writeFun("ea r            : Reset Tx/Rx table\n");
  }

  return ret;
}

/*******************************************************************/

static BOOL VbEngineCdtaDescConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "i"))
    {
      INT32U clusterId;

      if(cmd[2] != NULL)
      {
        clusterId = strtoul(cmd[2], NULL, 0);
        VbCdtaGlobalInfoDump(writeFun, clusterId);
        ret = TRUE;
      }
      else
      {
        ret = FALSE;
      }
    }
    else if (!strcmp(cmd[1], "s"))
    {
      if (cmd[2] != NULL)
      {
        if (!strcmp(cmd[2], "multDownUp"))
        {
          INT32U mult;
          mult = strtoul(cmd[3], NULL, 0);
          VbCdtaWeightDownUpMultFactorSet(mult);
        }
      }

      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "hist"))
    {
      INT32U clusterId;

      if(cmd[2] != NULL)
      {
        clusterId = strtoul(cmd[2], NULL, 0);
        VbCdtaGlobalHistInfoDump(writeFun, clusterId);
        ret = TRUE;
      }
      else
      {
        ret = FALSE;
      }
    }
    else if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("cdta h                     : Shows this help\n");
    writeFun("cdta i <num>               : Dumps CDTA info of cluster num\n");
    writeFun("cdta hist <num>            : Dumps CDTA Metrics historial of cluster num\n");
    writeFun("cdta s multDownUp <value>  : Sets Down/Up weight\n");
  }

  return ret;
}


/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

t_VB_engineErrorCode VbEngineConsoleInit(INT16U port)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (port != 0)
  {
    VbConsoleInit(port, VbEngineConfEngineIdGet());

    VbConsoleCommandRegister("kill",       VbEngineConsoleKill,             NULL);
    VbConsoleCommandRegister("report",     VbEngineConsoleReport,           NULL);
    VbConsoleCommandRegister("vers",       VbEngineConsoleVers,             NULL);
    VbConsoleCommandRegister("runtime",    VbCountersRunningTime,           NULL);
    VbConsoleCommandRegister("counters",   VbCountersConsoleCmd,            NULL);
    VbConsoleCommandRegister("meas",       VbEngineConsoleMeasure,          NULL);
    VbConsoleCommandRegister("metrics",    VbMetricsConsoleHandler,         NULL);
    VbConsoleCommandRegister("snr_probes", VbEngineConsoleSnrProbesMeasure, NULL);
    VbConsoleCommandRegister("psd",        VbEngineConsoleExecutePSD,       NULL);
    VbConsoleCommandRegister("boost",      VbEngineConsoleForceBoostMode,   NULL);
    VbConsoleCommandRegister("conf",       VbEngineConfConsoleCmd,          NULL);
    VbConsoleCommandRegister("drivers",    VbEngineConsoleDriversReport,    NULL);
    VbConsoleCommandRegister("clusters",   VbEngineConsoleClustersReport,   NULL);
    VbConsoleCommandRegister("align",      VbEngineConsoleAlignCmd,         NULL);
    VbConsoleCommandRegister("ea",         VbEngineEAConsoleCmd,            NULL);
    VbConsoleCommandRegister("cdta",       VbEngineCdtaDescConsoleCmd,      NULL);
    VbConsoleCommandRegister("log",        VbLogConsoleCmd,                 NULL);
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
