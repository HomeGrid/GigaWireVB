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
 * @file vb_engine_communication.c
 * @brief Implements engine communication
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
#include <pthread.h>

#include "vb_engine_datamodel.h"
#include "vb_log.h"
#include "vb_engine_EA_interface.h"
#include "vb_engine_communication.h"
#include "vb_util.h"
#include "vb_engine_conf.h"
#include "vb_engine_clock.h"
#include "vb_engine_socket_alive.h"
#include "vb_ea_communication.h"
#include "vb_engine_l2rPSD_calculation.h"
#include "vb_counters.h"
#include "vb_metrics.h"
#include "vb_thread.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_measure.h"
#include "vb_engine_alignment.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_clock.h"
#include "vb_engine_psd_shape.h"
#include "vb_engine_cdta.h"
#include "vb_engine_process.h"


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

#define VB_ENGINE_L2_XPUT_DECREASE_FILTER (4)
#define VB_ENGINE_BPS_REPORT_CHANGE_THR   (5)

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

static t_VB_engineErrorCode    VbEngineDomainsRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);
static t_VB_engineErrorCode    VbEngineTrafficAwarenessTrgProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);
static t_VB_engineErrorCode    VbEngineNetworkChangeTrgProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);
static t_VB_engineErrorCode    VbEngineDriverVersionRspProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);
static t_VB_engineErrorCode    VbEngineDriverStateRspProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);
static t_trafficReport*        VbEngineTrafficReporterPtrGet(INT8U* macMeasurer, BOOLEAN *found,  t_VBDriver *thisDriver );
static t_nodeChannelSettings*  VbEngineChannelCapacityPtrGet(INT8U* macMeasurer, BOOLEAN *found,  t_VBDriver *thisDriver );

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

static t_trafficReport* VbEngineTrafficReporterPtrGet(INT8U *macMeasurer, BOOLEAN *found, t_VBDriver *thisDriver )
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  t_trafficReport     *traffic_report = NULL;
  t_node              *node = NULL;

  if ((macMeasurer != NULL) && (found != NULL) && (thisDriver != NULL))
  {
    err = VbEngineDatamodelNodeFind(thisDriver, macMeasurer, &node);

    if (err != VB_ENGINE_ERROR_NONE)
    {
      *found = FALSE;
    }
    else
    {
      // Node found
      *found = TRUE;

      traffic_report = &(node->trafficReports);
    }
  }

  return traffic_report;
}

/*******************************************************************/

static t_nodeChannelSettings* VbEngineChannelCapacityPtrGet(INT8U* macMeasurer, BOOLEAN *found,  t_VBDriver *thisDriver )
{
  t_VB_engineErrorCode   err = VB_ENGINE_ERROR_NONE;
  t_nodeChannelSettings *node_channel_settings = NULL;
  t_node                *node = NULL;

  if ((macMeasurer != NULL) && (found != NULL) && (thisDriver != NULL))
  {
    err = VbEngineDatamodelNodeFind(thisDriver, macMeasurer, &node);

    if (err != VB_ENGINE_ERROR_NONE)
    {
      *found = FALSE;
    }
    else
    {
      // Node found
      *found = TRUE;

      node_channel_settings = &(node->channelSettings);
    }
  }

  return node_channel_settings;
}

/*******************************************************************/

static t_VB_engineErrorCode    VbEngineDomainsRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEADomainRspDM   *domRsp;
  t_vbEADomainRspEP   *epRsp;
  t_domainsList        domains_list;
  t_domainsList        temp_domains_list;
  INT16U               num_DMs;
  INT16U               num_EPs;
  INT32U               i;
  INT32U               j;
  INT8U               *pld;
  INT8U               *pld_aux;

  if (payload != NULL)
  {
    num_DMs = VbEngineDomainsRespNumDomainsGet(payload);
    domains_list.numDomains = num_DMs;
    domains_list.domainsArray = NULL;

    if (num_DMs != 0)
    {
      pld = payload;
      pld += 2;

      if (domains_list.numDomains > 0)
      {
        domains_list.domainsArray = calloc(1, num_DMs*sizeof(t_domain));

        if (domains_list.domainsArray == NULL)
        {
          vb_err = VB_ENGINE_ERROR_MALLOC;
        }

        if (vb_err == VB_ENGINE_ERROR_NONE)
        {
          for (i = 0; i < num_DMs; i++)
          {
            domRsp = (t_vbEADomainRspDM*)pld;
            num_EPs = _ntohs(domRsp->NumEps);

            // Init Measures
            domains_list.domainsArray[i].dm.measures.BGNMeasure.measuresRx1 = NULL;
            domains_list.domainsArray[i].dm.measures.BGNMeasure.measuresRx2 = NULL;
            domains_list.domainsArray[i].dm.measures.SNRProbesMeasure.measuresRx1 = NULL;
            domains_list.domainsArray[i].dm.measures.SNRProbesMeasure.measuresRx2 = NULL;
            domains_list.domainsArray[i].dm.measures.CFRMeasureList.numCrossMeasures = 0;
            domains_list.domainsArray[i].dm.measures.CFRMeasureList.crossMeasureArray = NULL;

            domains_list.domainsArray[i].dm.measures.Psd.MSBsList.NumMSBs = 0;
            domains_list.domainsArray[i].dm.measures.Psd.MSBsList.MSB = NULL;
            domains_list.domainsArray[i].dm.measures.Psd.PSDStepsList.NumPSDs = 0;
            domains_list.domainsArray[i].dm.measures.Psd.PSDStepsList.PSD = NULL;
            domains_list.domainsArray[i].dm.measures.snrFullXtalk.measuresRx1 = NULL;
            domains_list.domainsArray[i].dm.measures.snrFullXtalk.measuresRx2 = NULL;
            domains_list.domainsArray[i].dm.measures.snrLowXtalk.measuresRx1 = NULL;
            domains_list.domainsArray[i].dm.measures.snrLowXtalk.measuresRx2 = NULL;

            // Init channel capacity and channel settings
            domains_list.domainsArray[i].dm.channelSettings.psdShape.numPSDBands = 0;
            domains_list.domainsArray[i].dm.channelSettings.psdShape.sendUpdate = FALSE;

            memset(&domains_list.domainsArray[i].dm.channelSettings.boostInfo, 0, sizeof(t_boostInfo));
            domains_list.domainsArray[i].dm.channelSettings.interferenceDetectionCounter = 0;

            memset(&domains_list.domainsArray[i].dm.cdtaInfo, 0, sizeof(t_nodeCdtaInfo));
            domains_list.domainsArray[i].dm.channelSettings.boostInfo.forcedBandsBitmap = (1 | (((VbEngineConfVdslCoexGet() == TRUE)?1:0)<<1));
            domains_list.domainsArray[i].dm.channelSettings.boostInfo.lastLevel = 1;

            VbEngineConfProfileGet(domRsp->DM_MAC,
                                   &(domains_list.domainsArray[i].dm.cdtaInfo.profile.userSLA),
                                   &(domains_list.domainsArray[i].dm.cdtaInfo.profile.slaWeight),
                                   &(domains_list.domainsArray[i].dm.cdtaInfo.profile.userWeight) );

            // Init Traffic report
            memset(&domains_list.domainsArray[i].dm.trafficReports, 0, sizeof(t_trafficReport));

            // Init node Align info
            memset(&(domains_list.domainsArray[i].dm.nodeAlignInfo), 0, sizeof(domains_list.domainsArray[i].dm.nodeAlignInfo));

            domains_list.domainsArray[i].eps.numEPs = num_EPs;
            domains_list.domainsArray[i].eps.epsArray = NULL;

            memcpy(domains_list.domainsArray[i].dm.MAC, domRsp->DM_MAC, ETH_ALEN);
            MACAddrMem2str(domains_list.domainsArray[i].dm.MACStr, domRsp->DM_MAC);
            memcpy(domains_list.domainsArray[i].dm.addInfo1.fwVersion, domRsp->fwVersion, 10);
            domains_list.domainsArray[i].dm.devID = domRsp->DM_ID;
            domains_list.domainsArray[i].dm.linkedNode = NULL;
            domains_list.domainsArray[i].dm.type = VB_NODE_DOMAIN_MASTER;
            domains_list.domainsArray[i].dm.addInfo1.extSeed = _ntohs(domRsp->DM_Extseed);
            domains_list.domainsArray[i].dm.addInfo1.qosRate = _ntohs(domRsp->qosRate);
            domains_list.domainsArray[i].dm.addInfo1.maxLengthTxop = _ntohs(domRsp->maxLengthTxop);

            // Update state file name
            snprintf(domains_list.domainsArray[i].dm.stateFileName, VB_ENGINE_MAX_FILE_NAME_SIZE, "portGhn_%02X%02X%02X%02X%02X%02X",
                domains_list.domainsArray[i].dm.MAC[0],
                domains_list.domainsArray[i].dm.MAC[1],
                domains_list.domainsArray[i].dm.MAC[2],
                domains_list.domainsArray[i].dm.MAC[3],
                domains_list.domainsArray[i].dm.MAC[4],
                domains_list.domainsArray[i].dm.MAC[5]);
            // Force the null byte in last position
            domains_list.domainsArray[i].dm.stateFileName[VB_ENGINE_MAX_FILE_NAME_SIZE - 1] = '\0';

            if (num_EPs > 0)
            {
              pld_aux = pld + sizeof(t_vbEADomainRspDM);
              epRsp = (t_vbEADomainRspEP *)pld_aux;
              domains_list.domainsArray[i].eps.epsArray = calloc(1, num_EPs*sizeof(t_node));

              if (domains_list.domainsArray[i].eps.epsArray == NULL)
              {
                vb_err = VB_ENGINE_ERROR_MALLOC;
              }

              if (vb_err == VB_ENGINE_ERROR_NONE)
              {
                for (j = 0; j < num_EPs; j++)
                {
                  // Init measure
                  domains_list.domainsArray[i].eps.epsArray[j].measures.BGNMeasure.measuresRx1 = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.BGNMeasure.measuresRx2 = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.SNRProbesMeasure.measuresRx1 = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.SNRProbesMeasure.measuresRx2 = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.CFRMeasureList.numCrossMeasures = 0;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.CFRMeasureList.crossMeasureArray = NULL;

                  domains_list.domainsArray[i].eps.epsArray[j].measures.Psd.MSBsList.MSB = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.Psd.MSBsList.NumMSBs = 0;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.Psd.PSDStepsList.PSD = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.Psd.PSDStepsList.NumPSDs = 0;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.snrFullXtalk.measuresRx1 = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.snrFullXtalk.measuresRx2 = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.snrLowXtalk.measuresRx1 = NULL;
                  domains_list.domainsArray[i].eps.epsArray[j].measures.snrLowXtalk.measuresRx2 = NULL;


                  // Init channel capacity and channel settings
                  domains_list.domainsArray[i].eps.epsArray[j].channelSettings.psdShape.numPSDBands = 0;
                  domains_list.domainsArray[i].eps.epsArray[j].channelSettings.psdShape.sendUpdate = FALSE;
                  memset(&domains_list.domainsArray[i].eps.epsArray[j].channelSettings.boostInfo, 0, sizeof(t_boostInfo));
                  domains_list.domainsArray[i].eps.epsArray[j].channelSettings.interferenceDetectionCounter = 0;

                  // Init Traffic report
                  memset(&domains_list.domainsArray[i].eps.epsArray[j].trafficReports, 0, sizeof(t_trafficReport));
                  memcpy(domains_list.domainsArray[i].eps.epsArray[j].MAC, epRsp->EP_MAC, ETH_ALEN);

                  memset(&(domains_list.domainsArray[i].eps.epsArray[j].cdtaInfo), 0, sizeof(t_nodeCdtaInfo));
                  VbEngineConfProfileGet(epRsp->EP_MAC,
                                         &(domains_list.domainsArray[i].eps.epsArray[j].cdtaInfo.profile.userSLA),
                                         &(domains_list.domainsArray[i].eps.epsArray[j].cdtaInfo.profile.slaWeight),
                                         &(domains_list.domainsArray[i].eps.epsArray[j].cdtaInfo.profile.userWeight) );
                  domains_list.domainsArray[i].eps.epsArray[j].channelSettings.boostInfo.forcedBandsBitmap = (1 | (((VbEngineConfVdslCoexGet() == TRUE)?1:0)<<1));;
                  domains_list.domainsArray[i].eps.epsArray[j].channelSettings.boostInfo.lastLevel = 1;

                  MACAddrMem2str(domains_list.domainsArray[i].eps.epsArray[j].MACStr, epRsp->EP_MAC);
                  memcpy(domains_list.domainsArray[i].eps.epsArray[j].addInfo1.fwVersion, epRsp->fwVersion, VB_FW_VERSION_LENGTH);
                  domains_list.domainsArray[i].eps.epsArray[j].devID = epRsp->EP_ID;
                  domains_list.domainsArray[i].eps.epsArray[j].linkedNode = &(domains_list.domainsArray[i].dm);
                  domains_list.domainsArray[i].dm.linkedNode = &(domains_list.domainsArray[i].eps.epsArray[j]);
                  domains_list.domainsArray[i].eps.epsArray[j].type = VB_NODE_END_POINT;

                  // Update state file name
                  snprintf(domains_list.domainsArray[i].eps.epsArray[j].stateFileName, VB_ENGINE_MAX_FILE_NAME_SIZE, "portGhn_%02X%02X%02X%02X%02X%02X",
                      domains_list.domainsArray[i].eps.epsArray[j].MAC[0],
                      domains_list.domainsArray[i].eps.epsArray[j].MAC[1],
                      domains_list.domainsArray[i].eps.epsArray[j].MAC[2],
                      domains_list.domainsArray[i].eps.epsArray[j].MAC[3],
                      domains_list.domainsArray[i].eps.epsArray[j].MAC[4],
                      domains_list.domainsArray[i].eps.epsArray[j].MAC[5]);
                  // Force the null byte in last position
                  domains_list.domainsArray[i].eps.epsArray[j].stateFileName[VB_ENGINE_MAX_FILE_NAME_SIZE - 1] = '\0';

                  pld_aux +=sizeof(t_vbEADomainRspEP);
                }
              }
            }

            pld += sizeof(t_vbEADomainRspDM) + (num_EPs * sizeof(t_vbEADomainRspEP));
          }
        }
      }
    }

    if (vb_err == VB_ENGINE_ERROR_NONE)
    {
      VbEngineDatamodelListDomainsLostCheck(thisDriver, &domains_list);

      VbEngineDatamodelListDomainsRecoverData(thisDriver, &domains_list);
    }

    if (vb_err == VB_ENGINE_ERROR_NONE)
    {
      pthread_mutex_lock(&(thisDriver->domainsMutex));

      temp_domains_list = thisDriver->domainsList;
      thisDriver->domainsList = domains_list;

      pthread_mutex_unlock(&(thisDriver->domainsMutex));

      VbEngineDatamodelListDmDestroy(thisDriver, &temp_domains_list);
    }
  }
  else
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  return vb_err;
}

/*******************************************************************/

static t_VB_engineErrorCode UpdateDMs(t_VBDriver *thisDriver, void *dms, INT16U num_changed_dms,
                INT16U *num_changed_old_dms, BOOL *old_dm_ref_added, BOOL isAdding)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  INT32U               i;
  BOOL                 found = FALSE;

  if (num_changed_old_dms)
    *num_changed_old_dms = 0;
  if (old_dm_ref_added)
    *old_dm_ref_added = FALSE;

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    t_alignRole role = VB_ALIGN_ROLE_NOT_INIT;
    INT32U newClusterId = 0;

    for(i=0; i<num_changed_dms; i++)
    {
      INT32U anotherClusterId = 0;
      INT8U *dmMAC;

      if (isAdding)
      {
        t_vbEADomainDiffRspDMAdded *added_dms = dms;
        dmMAC = added_dms[i].dmMAC;
      }
      else
      {
        t_vbEADomainDiffRspNodeRem *rem_dms = dms;
        dmMAC = rem_dms[i].mac;
      }

      if (vb_err == VB_ENGINE_ERROR_NONE)
      {
        vb_err = VbeFindDMsClusterIdByMAC(dmMAC, &anotherClusterId, &role);
      }

      if (vb_err == VB_ENGINE_ERROR_NONE)
      {
        found = TRUE;
        if (newClusterId == 0)
          newClusterId = anotherClusterId;
        if (!newClusterId || anotherClusterId != newClusterId)
        {
          newClusterId = 0;
          if (num_changed_old_dms)
            *num_changed_old_dms = 0;
          break;
        }
        else if (newClusterId)
        {
          if (num_changed_old_dms)
            (*num_changed_old_dms)++;
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "DMs found [%02X:%2X:%02X:%02X:%02X:%02X] -> clusterId %d, role %d",
                        dmMAC[0],
                        dmMAC[1],
                        dmMAC[2],
                        dmMAC[3],
                        dmMAC[4],
                        dmMAC[5],
                        newClusterId, role);
        }
      }
      else if (vb_err == VB_ENGINE_ERROR_NOT_FOUND)
      {
        VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "DMs not found [%02X:%2X:%02X:%02X:%02X:%02X]",
                        dmMAC[0],
                        dmMAC[1],
                        dmMAC[2],
                        dmMAC[3],
                        dmMAC[4],
                        dmMAC[5]);
        vb_err = VbeUpdateDMsHistory(dmMAC, thisDriver->clusterId, VB_ALIGN_ROLE_NOT_INIT, isAdding);
      }
    }

    if (newClusterId && found == TRUE && vb_err == VB_ENGINE_ERROR_NONE && isAdding == TRUE)
    {
      t_VBCluster *cluster;

      vb_err = VbEngineClusterByIdGet(newClusterId, &cluster);
      if (vb_err == VB_ENGINE_ERROR_NONE)
      {
        vb_err = VbEngineAlignDriverXSyncedTagY(thisDriver, newClusterId);
      }
      else
      {
        vb_err = VbEngineOldClusterAdd(newClusterId);
        if (vb_err == VB_ENGINE_ERROR_NONE)
        {
          vb_err = VbEngineAlignDriverXSyncedTagY(thisDriver, newClusterId);
        }
        if (vb_err == VB_ENGINE_ERROR_NONE)
        {
          if (old_dm_ref_added)
            *old_dm_ref_added = TRUE;
        }
      }
    }
  }

  return vb_err;
}

/*******************************************************************/

static t_VB_engineErrorCode    VbEngineDomainsChangeRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  INT8U               *pld_ptr;
  INT8U               *pld_ptr_init;
  t_vbEADomainDiffHdrRsp report_hdr;
  INT32U               i;
  INT16U               num_added_dms;
  INT16U               num_added_old_dms = 0;
  BOOL                 old_dm_ref_added = FALSE;
  INT16U               num_added_eps;
  INT16U               num_rem_dms;
  INT16U               num_rem_eps;
  BOOL                 dm_added      = FALSE;
  BOOL                 dm_rem        = FALSE;
  BOOL                 ep_added      = FALSE;
  BOOL                 ep_rem        = FALSE;
  BOOL                 ref_or_relay  = FALSE;

  if ((payload == NULL) || (thisDriver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    // Added DMs
    report_hdr = *((t_vbEADomainDiffHdrRsp *)payload);
    VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "******** Domain Report Type %s", (report_hdr.reportType == VB_EA_DOMAIN_REPORT_FULL)?"FULL":"DIFF");

    if(report_hdr.reportType == VB_EA_DOMAIN_REPORT_FULL)
    {
      // This a full report, clear current list of domains and process new DM and new EP
      thisDriver->domainsList.numDomains = 0;
      if(thisDriver->domainsList.domainsArray != NULL)
      {
        VbEngineDatamodelListDmDestroy(thisDriver, &thisDriver->domainsList);
      }
      thisDriver->domainsList.domainsArray = NULL;
    }

    pld_ptr = payload + sizeof(t_vbEADomainDiffHdrRsp);

    num_added_dms = ntohs(*((INT16U *)pld_ptr));
    pld_ptr += 2;
    pld_ptr_init = pld_ptr;
    if(num_added_dms > 0)
    {
      t_vbEADomainDiffRspDMAdded *added_dms;

      // Change status of existing lines from x (probably new) to PRESENT
      VbEngineLineStatusUpdate(thisDriver->clusterId, VB_DEV_PRESENT);

      vb_err = UpdateDMs(thisDriver, pld_ptr, num_added_dms, &num_added_old_dms, &old_dm_ref_added, TRUE);

      for(i=0; i<num_added_dms; i++)
      {
        added_dms = (t_vbEADomainDiffRspDMAdded *)pld_ptr;
        VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "++++++++");
        VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "++++++++ Added DMs devId %d; MAC: [%02x %02x %02x %02x %02x %02x] ++++++++",
                   added_dms->dmDevId,
                   added_dms->dmMAC[0],
                   added_dms->dmMAC[1],
                   added_dms->dmMAC[2],
                   added_dms->dmMAC[3],
                   added_dms->dmMAC[4],
                   added_dms->dmMAC[5]);
        VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "++++++++");
        pld_ptr += sizeof(t_vbEADomainDiffRspDMAdded);
      }

      vb_err = VbEngineDatamodelListDomainsDMsAdd(thisDriver, num_added_dms, (t_vbEADomainDiffRspDMAdded *)pld_ptr_init);
      if(vb_err == VB_ENGINE_ERROR_NONE)
      {
        // New line added -> Send ALIGN_ALL event (Stop smallest clusters, keep biggest one ...)
        dm_added = TRUE;
      }
    }

    if(vb_err == VB_ENGINE_ERROR_NONE)
    {
      // Added EPs
      num_added_eps = ntohs(*((INT16U *)pld_ptr));
      pld_ptr += 2;
      pld_ptr_init = pld_ptr;

      if(num_added_eps > 0)
      {
        t_vbEADomainDiffRspEPAdded *added_eps;
        for(i=0; i<num_added_eps; i++)
        {
          added_eps = (t_vbEADomainDiffRspEPAdded *)pld_ptr;
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "++++++++");
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "++++++++ Added EPs devId %d; MAC: [%02x %02x %02x %02x %02x %02x] / [%02x %02x %02x %02x %02x %02x] ++++++++",
                        added_eps->epDevId,
                        added_eps->dmMAC[0],
                        added_eps->dmMAC[1],
                        added_eps->dmMAC[2],
                        added_eps->dmMAC[3],
                        added_eps->dmMAC[4],
                        added_eps->dmMAC[5],
                        added_eps->epMAC[0],
                        added_eps->epMAC[1],
                        added_eps->epMAC[2],
                        added_eps->epMAC[3],
                        added_eps->epMAC[4],
                        added_eps->epMAC[5]);
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "++++++++");

          pld_ptr += sizeof(t_vbEADomainDiffRspEPAdded);
        }

        vb_err = VbEngineDatamodelListDomainsEPsAdd(thisDriver, num_added_eps, (t_vbEADomainDiffRspEPAdded *)pld_ptr_init);
        if(vb_err == VB_ENGINE_ERROR_NONE)
        {
          // New EP -> Send measure plan request to cluster x
          ep_added = TRUE;
        }
      }
    }

    if(vb_err == VB_ENGINE_ERROR_NONE)
    {
      // Removed DMs
      num_rem_dms = ntohs(*((INT16U *)pld_ptr));
      pld_ptr += 2;
      pld_ptr_init = pld_ptr;

      if(num_rem_dms > 0)
      {
        t_vbEADomainDiffRspNodeRem *rem_dms;

        vb_err = UpdateDMs(thisDriver, pld_ptr, num_rem_dms, NULL, NULL, FALSE);

        for(i=0; i<num_rem_dms; i++)
        {
          rem_dms = (t_vbEADomainDiffRspNodeRem *)pld_ptr;
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "--------");
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "-------- Rem DMs MAC: [%02x %02x %02x %02x %02x %02x] --------",
                        rem_dms->mac[0],
                        rem_dms->mac[1],
                        rem_dms->mac[2],
                        rem_dms->mac[3],
                        rem_dms->mac[4],
                        rem_dms->mac[5]);
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "--------");

          pld_ptr += sizeof(t_vbEADomainDiffRspNodeRem);
        }

        vb_err = VbEngineDatamodelListDomainsDMsRem(thisDriver, num_rem_dms, pld_ptr_init, &ref_or_relay);
        if(vb_err == VB_ENGINE_ERROR_NONE)
        {
          // DM gone -> Send ALIGN_I event to cluster x (Reevaluate Cluster x alignment)
          dm_rem = TRUE;
        }
      }
    }

    if(vb_err == VB_ENGINE_ERROR_NONE)
    {
      // Removed EPs
      num_rem_eps = ntohs(*((INT16U *)pld_ptr));
      pld_ptr += 2;
      pld_ptr_init = pld_ptr;
      if(num_rem_eps > 0)
      {
        t_vbEADomainDiffRspNodeRem *rem_eps;
        for(i=0; i<num_rem_eps; i++)
        {
          rem_eps = (t_vbEADomainDiffRspNodeRem *)pld_ptr;
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "--------");
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "-------- Rem EPs MAC: [%02x %02x %02x %02x %02x %02x] --------",
                        rem_eps->mac[0],
                        rem_eps->mac[1],
                        rem_eps->mac[2],
                        rem_eps->mac[3],
                        rem_eps->mac[4],
                        rem_eps->mac[5]);
          VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "--------");

          pld_ptr += sizeof(t_vbEADomainDiffRspNodeRem);
        }

        vb_err = VbEngineDatamodelListDomainsEPsRem(thisDriver, num_rem_eps, pld_ptr_init);
        if(vb_err == VB_ENGINE_ERROR_NONE)
        {
          // EP Gone -> Send measure plan request to cluster x
          ep_rem = TRUE;
        }
      }
    }
  }

  if(vb_err == VB_ENGINE_ERROR_NONE)
  {
    if(report_hdr.reportType == VB_EA_DOMAIN_REPORT_DIFF)
    {
      if(dm_added == TRUE)
      {
        // At least one line in driver, Send NON EMPTY event to potentially get out of the EMPTY state
        vb_err = VbEngineProcessEvSend(thisDriver, ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP, NULL);
      }
      else if(dm_rem == TRUE)
      {
        if(thisDriver->domainsList.numDomains > 0)
        {
          vb_err = VbEngineProcessEvSend(thisDriver, ENGINE_EV_NETWORK_DIFF_DM_REM, INT2VOIDP(ref_or_relay));
        }
        else
        {
          // No lines in driver, put him aside in DOMAIN_WAIT state until a DM get back in
          vb_err = VbEngineProcessEvSend(thisDriver, ENGINE_EV_RX_EMPTY_DOMAIN_RSP, INT2VOIDP(ref_or_relay));
        }
      }
      else if(ep_added == TRUE)
      {
        t_VBCluster *cluster;

        vb_err = VbEngineClusterByIdGet(thisDriver->clusterId, &cluster);
        if(vb_err == VB_ENGINE_ERROR_NONE)
        {
          cluster->skipMeasPlan = FALSE;
          if(thisDriver->FSMState != ENGINE_STT_ALIGNMENT_CHECK_SYNC)
          {

            if(vb_err == VB_ENGINE_ERROR_NONE)
            {
              if((thisDriver->FSMState <= ENGINE_STT_ALIGNMENT_DONE_SYNC) || (thisDriver->FSMState  >= ENGINE_STT_BOOSTING_CALCULATE_SNR))
              {
                // Delayed action (Or nothing to do (alignment phase) or to be done before running CDTA algorithm (boosted))
                cluster->epChange = TRUE;
              }
              else
              {
                // In measure plan -> Cancel and send new plan
                vb_err = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_RESTART, NULL, thisDriver->clusterId);
              }
            }
          }
          else
          {
            // likely to be waiting for EP to go to next stage (Measure)
            vb_err = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_ALIGN_CHECK_RESTART, NULL, thisDriver->clusterId);
          }
        }
      }
      else if(ep_rem == TRUE)
      {
        t_VBCluster *cluster;

        vb_err = VbEngineClusterByIdGet(thisDriver->clusterId, &cluster);
        if(vb_err == VB_ENGINE_ERROR_NONE)
        {
          cluster->skipMeasPlan = FALSE;
          if((thisDriver->FSMState <= ENGINE_STT_ALIGNMENT_DONE_SYNC) || (thisDriver->FSMState  >= ENGINE_STT_BOOSTING_CALCULATE_SNR))
          {
            // Delayed action (Or nothing to do (alignment phase) or to be done before running CDTA algorithm (boosted))
            cluster->epChange = TRUE;
          }
          else
          {
            // In measure plan -> Cancel and send new plan
            vb_err = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_MEASPLAN_RESTART, NULL, thisDriver->clusterId);
          }
        }
      }
      else
      {
        VbLogPrintExt(VB_LOG_WARNING, thisDriver->vbDriverID, "Empty Diff report");
      }
    }
    else
    {
      // Full report are sent only at first connection with the driver, so send new DM event if there is any
      if(thisDriver->domainsList.numDomains > 0 && num_added_old_dms == 0)
      {
        // At least one line in driver, Send NON EMPTY event to potentially get out of the EMPTY state
        vb_err = VbEngineProcessEvSend(thisDriver, ENGINE_EV_RX_NON_EMPTY_DOMAIN_RSP, NULL);
      }
      else if (num_added_old_dms == 0)
      {
        // No lines in driver, put him aside in DOMAIN_WAIT state until a DM get back in
        vb_err = VbEngineProcessEvSend(thisDriver, ENGINE_EV_RX_EMPTY_DOMAIN_RSP, NULL);
      }
      else if (old_dm_ref_added == FALSE)
      {
        vb_err = VbEngineProcessEvSend(thisDriver, ENGINE_EV_RX_EMPTY_DOMAIN_RSP, NULL);
      }
      else
      {
        vb_err = VbEngineProcessEvSend(thisDriver, ENGINE_EV_ALIGN_CLUSTER_I, NULL);
      }
    }
  }
  else
  {
    // Would be nice to force a resend on the driver side
    VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Empty Diff report");
  }


  return vb_err;
}

/*******************************************************************/

static t_VB_engineErrorCode    VbEngineDriverVersionRspProcess(INT8U* payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEAVersionRsp    *ptr;

  if ((payload == NULL) || (driver == NULL) || (length == 0))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U wait_time_out;
    INT32U n_attempt;

    ptr = (t_vbEAVersionRsp *)payload;

    // Copy remote state to internal structure
    strncpy(driver->remoteVersion, ptr->version, VB_EA_VERSION_MAX_SIZE); // DRIVER_REMOTE_VERSION_MAX_LEN
    // Force the null byte in last position
    driver->remoteVersion[VB_EA_VERSION_MAX_SIZE - 1] = '\0';

    ret = VbEngineDatamodelDriverIdSet(ptr->driverId, driver, TRUE);

    wait_time_out = _ntohl(ptr->lcmpMcastTimeOut);
    n_attempt     = _ntohl(ptr->lcmpMcastNAttempt);

    driver->transactionMinTime = wait_time_out*n_attempt;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "New ID %s; Version %s, trMinTime %lu",
                  driver->vbDriverID, driver->remoteVersion, driver->transactionMinTime);

    // Copy remote version to file
    VbLogSaveStringToTextFile(driver->versionFileName, "w+", "VBDriverVersion=%s\n", driver->remoteVersion);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode    VbEngineDriverStateRspProcess(INT8U* payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((payload == NULL) || (driver == NULL) || (length == 0))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Copy remote state to internal structure
    strncpy(driver->remoteState, (CHAR *)payload, DRIVER_REMOTE_STATE_MAX_LEN);

    // Force '\0' character
    driver->remoteState[DRIVER_REMOTE_STATE_MAX_LEN - 1] = '\0';

    VbLogSaveStringToTextFile(driver->remoteStateFileName, "w+", "VBDriverState=%s\n", driver->remoteState);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineTrafficAwarenessTrgProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode    vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEATrafficReportRsp *traffic_report_rsp = NULL;
  t_vbEATrafficBpsReport *traffic_bps_report;
  t_vbEATrafficReportHdr *traffic_report_hdr;
  t_trafficReport        *traffic_report = NULL;
  BOOLEAN                 found = FALSE;
  t_nodeChannelSettings  *nodeChannelSettings;
  t_node                 *node;
  CHAR                    mac_str[MAC_STR_LEN];
#if VB_ENGINE_METRICS_ENABLED
  INT32U                  traffic;
#endif
  INT32U                  num_reports;
  INT32U                  i;
  INT32U                  j;
  INT16U                  n_bands;
  INT8U                   *aux_ptr = NULL;


  if ((payload == NULL) || (thisDriver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    BOOLEAN stable_set_of_bps = TRUE;

    traffic_report_hdr = (t_vbEATrafficReportHdr *)payload;
    num_reports = _ntohl(traffic_report_hdr->numReports);

    pthread_mutex_lock(&(thisDriver->domainsMutex));

    aux_ptr = (INT8U*)(payload + VB_EA_TRAFFIC_REPORT_HDR_SIZE);

    for(i = 0; i < num_reports; i++)
    {
      traffic_report_rsp = (t_vbEATrafficReportRsp *)(aux_ptr);
      traffic_report =  VbEngineTrafficReporterPtrGet(traffic_report_rsp->MAC, &found, thisDriver);
      traffic_bps_report = (t_vbEATrafficBpsReport *)((INT8U*)traffic_report_rsp + VB_EA_TRAFFIC_REPORT_RSP_SIZE);

      if ((found == TRUE) && (traffic_report != NULL))
      {
        INT16U l2_xput;
        INT16U bps_band;

        traffic_report->rxReports++;
        traffic_report->reportsReceived = TRUE;

        traffic_report->ingressTrafficP0 = _ntohs(traffic_report_rsp->trafficPrio0);
        traffic_report->ingressTrafficP1 = _ntohs(traffic_report_rsp->trafficPrio1);
        traffic_report->ingressTrafficP2 = _ntohs(traffic_report_rsp->trafficPrio2);
        traffic_report->ingressTrafficP3 = _ntohs(traffic_report_rsp->trafficPrio3);

        traffic_report->usageBuffP0 = traffic_report_rsp->maxBuffPrio0;
        traffic_report->usageBuffP1 = traffic_report_rsp->maxBuffPrio1;
        traffic_report->usageBuffP2 = traffic_report_rsp->maxBuffPrio2;
        traffic_report->usageBuffP3 = traffic_report_rsp->maxBuffPrio3;

        traffic_report->bpsCapacity = _ntohs(traffic_report_rsp->bpsCapacity);
        traffic_report->realCapacity = _ntohs(traffic_report_rsp->realCapacity);
        traffic_report->neededTheoricCapacity = _ntohs(traffic_report_rsp->neededTheoricCapacity);

        l2_xput = traffic_report->ingressTrafficP0 +
                  traffic_report->ingressTrafficP1 +
                  traffic_report->ingressTrafficP2 +
                  traffic_report->ingressTrafficP3;

        if( (l2_xput >= traffic_report->neededL2Xput) || (++traffic_report->consecL2XputSmaller > VB_ENGINE_L2_XPUT_DECREASE_FILTER) )
        {
          traffic_report->neededL2Xput  = l2_xput;
          traffic_report->consecL2XputSmaller = 0;
        }

        traffic_report->neededL2Xput = l2_xput;
        traffic_report->macEfficiency = traffic_report_rsp->macEfficiency;
        traffic_report->nBandsBps = _ntohs(traffic_bps_report->nBands);
        memset(traffic_report->bpsBand, 0x0, sizeof(traffic_report->bpsBand));
        for(j = 0; ((j < traffic_report->nBandsBps) && (j < VB_PSD_NUM_BANDS)) ; j++)
        {
          bps_band = _ntohs(traffic_bps_report->bpsBand[j]);
          if(ABS_DIFF(traffic_report->bpsBand[j], bps_band) >= VB_ENGINE_BPS_REPORT_CHANGE_THR)
          {
            stable_set_of_bps = FALSE;
            traffic_report->bpsBand[j] = bps_band;
          }
        }

        nodeChannelSettings = VbEngineChannelCapacityPtrGet(traffic_report_rsp->MAC,  &found, thisDriver);
        if ((found == TRUE) && (nodeChannelSettings != NULL))
        {
          vb_err = VbEngineSnrCalculatedCheck(traffic_report, nodeChannelSettings, thisDriver);
          if(vb_err == VB_ENGINE_ERROR_NONE)
          {
            if (traffic_report->nBandsBps > 0)
            {
              if(traffic_report->bpsBand[0] > 0)
              {
                nodeChannelSettings->boostInfo.lowBandCapacity = traffic_report->bpsBand[0];
              }
            }
            else
            {
              nodeChannelSettings->boostInfo.lowBandCapacity = 0;
            }
          }
        }

        vb_err = VbEngineDatamodelNodeFind(thisDriver, traffic_report_rsp->MAC, &node);
        if ( (vb_err == VB_ENGINE_ERROR_NONE) && (stable_set_of_bps == FALSE) )
        {
          vb_err = VbEngineCdtaReportedBpsUpdate(node, traffic_report, node->type);
        }

  #if VB_ENGINE_METRICS_ENABLED
        // Traffic reports received
        traffic = traffic_report->ingressTrafficP0;
        traffic += traffic_report->ingressTrafficP1;
        traffic += traffic_report->ingressTrafficP2;
        traffic += traffic_report->ingressTrafficP3;
        VbMetricsReportDeviceEvent(VB_METRICS_EVENT_TRAFFIC_REPORT,
            traffic_report_rsp->MAC, FALSE, thisDriver, traffic, 0);
  #endif
      }
      else
      {
        MACAddrMem2str(mac_str, traffic_report_rsp->MAC);
        VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "Node %s not found to store traffic report", mac_str);
      }

      n_bands = _ntohs(traffic_bps_report->nBands);
      aux_ptr += (VB_EA_TRAFFIC_REPORT_RSP_SIZE + n_bands*sizeof(INT16U) + sizeof(INT16U));
    }

    pthread_mutex_unlock(&(thisDriver->domainsMutex));
  }

  return vb_err;
}

/*******************************************************************/

static t_VB_engineErrorCode    VbEngineNetworkChangeTrgProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;

#if 0
  if(thisDriver != NULL)
  {
    thisDriver->FSMState = ENGINE_STT_DISCOVER_DOMAIN_REQ;
    vb_err = VbEngineProcessInsertEvent(VB_EAI_EVENT_NETWORK_CHANGES);
  }
  else
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
#endif

  return vb_err;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_VB_engineErrorCode VbEngineFrameRxProcess(t_VBProcessMsg *processMsg)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBDriver          *driver;
  INT8U               *payload_ptr;
  INT32U               payload_len;

  // Ensure there is a message pending to be processed
  if ((processMsg == NULL) || (processMsg->msg == NULL) ||
      (processMsg->senderDriver == NULL) || (processMsg->msg->eaFullMsg.msg == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    payload_ptr = processMsg->msg->eaPayload.msg;
    payload_len = processMsg->msg->eaPayload.msgLen;
    driver      = processMsg->senderDriver;

    switch (processMsg->msg->opcode)
    {
      case VB_EA_OPCODE_DOMAIN_RESP:
        ret = VbEngineDomainsRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_BGN_RESP:
        ret = VbEngineBngNoiseRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_CFR_RESP:
        ret = VbEngineCFRRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_SNRPROBES_RESP:
        ret = VbEngineSnrProbesRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_PSD_RESP:
        ret = VbEnginePSDRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_PSD_SHAPE_CFM:
        ret = VbEnginePSDShapeRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_CDTA_CFM:
        ret = VbEngineCdtaRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_TRAFFIC_AWARENESS_TRG:
        ret = VbEngineTrafficAwarenessTrgProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_NETWORK_CHANGE_TRG:
        ret = VbEngineNetworkChangeTrgProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_VERSION_RESP:
        ret = VbEngineDriverVersionRspProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_VBDRIVER_STATE_TRG:
      case VB_EA_OPCODE_VBDRIVER_STATE_RESP:
        ret = VbEngineDriverStateRspProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_CLOCK_RSP:
        ret = VbEngineClockRspProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_CYCQUERY_RSP:
        ret = VbEngineAlignCycQueryRspProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_ALIGN_SYNC_LOST_TRG:
        ret = VbEngineAlignSyncLostTrgProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_ALIGN_STOP_CLUSTER_RSP:
        ret = VbEngineAlignClusterStopRspProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_NETWORK_CHANGE_REPORT:
        ret = VbEngineDomainsChangeRespProcess(payload_ptr, payload_len, driver);
        break;

      case VB_EA_OPCODE_SOCKET_ALIVE_RESP:
        ret = VbEngineSocketAliveRspProcess(payload_ptr, payload_len, driver);
        break;

      default:
        VbLogPrintExt(VB_LOG_WARNING, driver->vbDriverID, "Unexpected EA opcode 0x%X", processMsg->msg->opcode);
        break;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessMeasureSnrProbesRequest( INT8U nMacs, const INT8U *mac, t_VBDriver *thisDriver )
{

  INT8U                *snr_probes_req_payload;
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  INT16U               len;
  INT8U                *buffer;

  if(mac == NULL)
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    len = 1+nMacs*ETH_ALEN;
    buffer = malloc(1+nMacs*ETH_ALEN);

    if(buffer != NULL)
    {
      buffer[0] = nMacs;
      snr_probes_req_payload = &buffer[1];
      memcpy(snr_probes_req_payload, mac, nMacs*ETH_ALEN);

      result = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_SNRPROBES_REQ, buffer, len, thisDriver);
      free(buffer);
    }
    else
    {
      result = VB_ENGINE_ERROR_MALLOC;
    }
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessVersionRequest( t_VBDriver *thisDriver )
{

  return VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_VERSION_REQ,NULL, 0, thisDriver);

}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessDriverStateRequest( t_VBDriver *thisDriver )
{

  return VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_VBDRIVER_STATE_REQ, NULL, 0, thisDriver);

}

/*******************************************************************/

INT32U VbEngineDomainsRespNumDomainsGet(INT8U *payload)
{
  INT32U num_domains = 0;

  if (payload != NULL)
  {
    num_domains = _ntohs(*((INT16U *)payload));
  }

  return num_domains;
}

/*******************************************************************/

/**
 * @}
 **/
