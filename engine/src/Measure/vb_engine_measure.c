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
 * @file vb_engine_measure.c
 * @brief Implements measure plan and collect
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

#include "vb_engine_datamodel.h"
#include "vb_log.h"
#include "vb_engine_EA_interface.h"
#include "vb_engine_communication.h"
#include "vb_engine_process.h"
#include "vb_priorities.h"
#include "vb_measure_utils.h"
#include "vb_engine_measure.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_conf.h"
#include "vb_counters.h"
#include "vb_LCMP_paramId.h"
#include "vb_engine_clock.h"

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

#define VB_ENGINE_EXTRA_MEASURE_PLAN_DURATION              (100) // ms
#define VB_MEASURE_FILE_NAME_SIZE                          (45)
#define VB_MEASURE_ROOT_FOLDER                             ("measures")
#define VB_MEASURE_PROBES_FOLDER                           ("measures/SNRProbes")
#define VB_MEASURE_TO_FLOAT(X)                             ((float)((X) / 4))
#define VB_MEASURE_BUFFER_TO_FILE_SIZE                     (5 * 1024 * 1024)
#define VB_MEASURE_CLUSTER_ID_FMT                          ("Cluster_id_%06u_")
#define VB_MEASURE_CLUSTER_ID_LEN                          (19)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct s_measurePlanInfo
{
  INT8U *ptr;
  INT16U seqNum;
} t_measurePlanInfo;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static INT32U vbMeasurePlanId;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/**
* @brief This function executes the VBCE measure part
* param[in] this_driver Pointer to vbDriver data struct
**/
static t_VB_engineErrorCode VbEngineMeasurementPlanCreate( INT8U planid, INT16U seqNumStart, t_measconfdata *measconfdata,
                                                           INT8U *payload, t_vbEngineNumNodes numNodes, INT16U *endSeqNum, INT32U clusterId );

/**
* @brief This function executes the VBCE measure part
* param[in] this_driver Pointer to vbDriver data struct
**/
static t_VB_engineErrorCode VbEngineMeasureCfrSet(INT8U* macMeasurer, INT8U *macMeasured, t_VBDriver *thisDriver, t_processMeasure *measurePtr);

/**
* @brief This function executes the VBCE measure part
* param[in] this_driver Pointer to vbDriver data struct
**/
static t_VB_engineErrorCode VbEngineMeasureBgnNoiseSet(INT8U* macMeasurer, t_VBDriver *thisDriver, t_processMeasure *measurePtr);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasPlanDMInfoLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  t_MeasureCFRDevice   *measure_cfr_node;
  t_measurePlanInfo    *plan_info = (t_measurePlanInfo *)args;
  t_measconfdata       *measure_conf_data = NULL;

  if ((domain == NULL) || (plan_info == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineConfMeasurePlanGet(&measure_conf_data);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineDatamodelDomainIsComplete(domain) == TRUE)
    {
      measure_cfr_node = (t_MeasureCFRDevice *)plan_info->ptr;

      measure_cfr_node->GeneralMeasureConfiguration.startSeqNumber = _htons_ghn(plan_info->seqNum);
      plan_info->seqNum += measure_conf_data->NumCycles;
      measure_cfr_node->GeneralMeasureConfiguration.endSeqNumber   = _htons_ghn(plan_info->seqNum);
      plan_info->seqNum++;

      measure_cfr_node->GeneralMeasureConfiguration.deviceType     = DMDEVICE;
      measure_cfr_node->nodeBasicInfo.devID                        = domain->dm.devID;
      measure_cfr_node->nodeBasicInfo.extSeed                      = _htons_ghn(domain->dm.addInfo1.extSeed);
      MACAddrClone(measure_cfr_node->nodeBasicInfo.measureDeviceMAC, domain->dm.MAC);

      plan_info->ptr += MEASURE_CFR_DEV_PLAN_SIZE;
    }
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasPlanEPInfoLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode      result = VB_ENGINE_ERROR_NONE;
  t_MeasureCFRDevice       *measure_cfr_node;
  t_measurePlanInfo        *plan_info = (t_measurePlanInfo *)args;
  INT16U                    num_eps_check;
  t_measconfdata           *measure_conf_data = NULL;

  if ((domain == NULL) || (plan_info == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineConfMeasurePlanGet(&measure_conf_data);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineDatamodelDomainIsComplete(domain) == TRUE)
    {
      for (num_eps_check = 0; num_eps_check < domain->eps.numEPs; num_eps_check++)
      {
        measure_cfr_node = (t_MeasureCFRDevice *)plan_info->ptr;

        measure_cfr_node->GeneralMeasureConfiguration.startSeqNumber = _htons_ghn(plan_info->seqNum);
        plan_info->seqNum += measure_conf_data->NumCycles;
        measure_cfr_node->GeneralMeasureConfiguration.endSeqNumber   = _htons_ghn(plan_info->seqNum);
        plan_info->seqNum++;

        measure_cfr_node->GeneralMeasureConfiguration.deviceType     = EPDEVICE;
        measure_cfr_node->nodeBasicInfo.devID                        = domain->eps.epsArray[num_eps_check].devID;
        measure_cfr_node->nodeBasicInfo.extSeed                      = _htons_ghn(domain->dm.addInfo1.extSeed);
        MACAddrClone(measure_cfr_node->nodeBasicInfo.measureDeviceMAC, domain->eps.epsArray[num_eps_check].MAC);

        plan_info->ptr += MEASURE_CFR_DEV_PLAN_SIZE;
      }
    }
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasCollectRequestPerDriverCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode  result = VB_ENGINE_ERROR_NONE;
  t_nodesMacList        driver_mac_list;
  t_nodesMacList       *all_macs_list = (t_nodesMacList *)args;
  t_vbEngineNumNodes    num_nodes;
  INT8U                 plan_id;

  if ((driver == NULL) || (all_macs_list == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Get number of lines in driver
    result = VbEngineDatamodelNumNodesInDriverGet(driver, &num_nodes);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Check that number of complete lines in driver is not 0
    if ((driver->FSMState == ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY) ||
        (num_nodes.numCompleteLines == 0))
    {
      t_VB_engineErrorCode  ev_err;

      // Skip drivers in "ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY" state or with no complete lines
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "No complete lines detected -> Skip sending EAMeasCollect.req");

      // Send event to move FSM to next state
      ev_err = VbEngineProcessEvSend(driver, ENGINE_EV_MEAS_COLLECT_END_NO_LINES, NULL);

      if (ev_err != VB_ENGINE_ERROR_NONE)
      {
        result = VB_ENGINE_ERROR_QUEUE;
      }
      else
      {
        result = VB_ENGINE_ERROR_SKIP;
      }
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Get MACs for this driver
    result = VbEngineDatamodelNodesMacGet(driver, &driver_mac_list, TRUE);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineMeasurePlanIdGet(driver->clusterId, &plan_id);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Send measure collect request to driver
    result = VbEngineMeasCollectReqSend(plan_id,
                                        &(driver_mac_list.dmsMacs),
                                        &(driver_mac_list.epsMacs),
                                        &(all_macs_list->dmsMacs),
                                        &(all_macs_list->epsMacs),
                                        driver);

    // Always relase memory
    VbEngineDatamodelMacListRelease(&driver_mac_list);
  }

  if (result == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current driver
    result = VB_ENGINE_ERROR_NONE;
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineCfrMeasureListCreateCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_nodesMacList      *all_nodes_macs_list = (t_nodesMacList *)args;
  INT32U               num_measured;

  if ((domain == NULL) || (node == NULL) || (all_nodes_macs_list == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Only allocate memory for complete lines
    if ((VbEngineDatamodelDomainIsComplete(domain) == FALSE) ||
        (node->linkedNode == NULL))
    {
      ret = VB_ENGINE_ERROR_DOMAIN_WITHOUT_LINE;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Release previously allocated memory
    if (node->measures.CFRMeasureList.numCrossMeasures > 0)
    {
      VbEngineDatamodelNodeCrossMeasureListDestroy(&(node->measures.CFRMeasureList));
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for crossMeasureArray
    if (node->type == VB_NODE_DOMAIN_MASTER)
    {
      num_measured = all_nodes_macs_list->epsMacs.numNodes;
    }
    else
    {
      num_measured = all_nodes_macs_list->dmsMacs.numNodes;
    }

    node->measures.CFRMeasureList.crossMeasureArray = (t_crossMeasure *)calloc(1, num_measured * sizeof(t_crossMeasure));
    if (node->measures.CFRMeasureList.crossMeasureArray == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for each measured device
    INT32U i;

    node->measures.CFRMeasureList.numCrossMeasures = num_measured;

    for (i = 0; i < num_measured; i++)
    {
      node->measures.CFRMeasureList.crossMeasureArray[i].measure.measuresRx1 = NULL;
      node->measures.CFRMeasureList.crossMeasureArray[i].measure.measuresRx2 = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Fill each measured device MAC
    INT8U *mac_measured_ptr;
    INT32U i;

    if (node->type == VB_NODE_DOMAIN_MASTER)
    {
      mac_measured_ptr = all_nodes_macs_list->epsMacs.ptr;
    }
    else
    {
      mac_measured_ptr = all_nodes_macs_list->dmsMacs.ptr;
    }

    for (i = 0; i < num_measured; i++)
    {
      // Compare with our linked node MAC
      if (memcmp(mac_measured_ptr, node->linkedNode->MAC, ETH_ALEN) == 0)
      {
        // MAC of my linked node
        node->measures.CFRMeasureList.crossMeasureArray[i].ownCFR = TRUE;
      }
      else
      {
        // Crosstalk MAC
        node->measures.CFRMeasureList.crossMeasureArray[i].ownCFR = FALSE;
      }

      MACAddrClone(node->measures.CFRMeasureList.crossMeasureArray[i].MAC, mac_measured_ptr);
      node->measures.CFRMeasureList.crossMeasureArray[i].measure.freqCutProfile = MAX_INT32U;
      mac_measured_ptr += ETH_ALEN;
    }
  }

  if (ret == VB_ENGINE_ERROR_DOMAIN_WITHOUT_LINE)
  {
    // Expected error for incomplete lines, continue with next nodes
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasurementPlanCreate( INT8U planid, INT16U seqNumStart, t_measconfdata *measconfdata,
                                                           INT8U *payload, t_vbEngineNumNodes numNodes, INT16U *endSeqNum, INT32U clusterId )
{
  t_VB_engineErrorCode             result = VB_ENGINE_ERROR_NONE;
  INT8U                           *pvalue;
  INT16U                           seq_number = seqNumStart;
  t_MeasureConfiguration          *measure_configuration;
  t_GeneralMeasureConfiguration   *measure_bgn_node;
  t_measurePlanInfo               plan_info;

  if (payload != NULL)
  {
    payload[0] = VB_MEASPLAN;

    pvalue =  &(payload)[1];
    pvalue[0] = planid;

    //////////////////////////////////
    // Fill CFR measure configuration
    //////////////////////////////////
    pvalue = &(payload[1 + MEASURE_CFR_CONF_OFFSET]);
    measure_configuration = (t_MeasureConfiguration *) pvalue;

    switch(measconfdata->CFRMeasureType)
    {
      case (CFRMODULE_MEASURE):
      case (CFRMODULEPHASE_MEASURE):
      {
        measure_configuration->measureType = measconfdata->CFRMeasureType;
        break;
      }
      default:
      {
        measure_configuration->measureType = CFRMODULE_MEASURE;
        break;
      }
    }

    if (VbEngineConfVbInUpstreamGet() == TRUE)
    {
      measure_configuration->numMeasure        = _htons_ghn(numNodes.numCompleteLines * 2);
    }
    else
    {
      measure_configuration->numMeasure        = _htons_ghn(numNodes.numEps);
    }

    measure_configuration->storageType         = measconfdata->StorageType;
    measure_configuration->symbolsNumber       = measconfdata->SymbolsNumber;
    measure_configuration->time                = measconfdata->TimeAveraging & 0x0F;
    measure_configuration->frequency           = measconfdata->FrecuencyAveraging;
    measure_configuration->offset              = _htonl_ghn(measconfdata->Offset);
    measure_configuration->duration            = _htonl_ghn(measconfdata->Duration);

    //////////////////////////////////
    // Fill CFR nodes plan
    //////////////////////////////////
    pvalue += MEASURE_CONF_SIZE;

    plan_info.ptr = pvalue;
    plan_info.seqNum = seq_number;

    result = VbEngineDatamodelClusterXAllDomainsLoop(VbEngineMeasPlanDMInfoLoopCb, clusterId, &plan_info);

    if ((result == VB_ENGINE_ERROR_NONE) &&
        (VbEngineConfVbInUpstreamGet() == TRUE))
    {
      result = VbEngineDatamodelClusterXAllDomainsLoop(VbEngineMeasPlanEPInfoLoopCb, clusterId, &plan_info);
    }

    //////////////////////////////////
    // Fill BGN measure configuration
    //////////////////////////////////
    if (result == VB_ENGINE_ERROR_NONE)
    {
      // Update sequence number and pointer
      seq_number = plan_info.seqNum;
      pvalue = plan_info.ptr;

      measure_configuration = (t_MeasureConfiguration *)pvalue;
      measure_configuration->measureType = BNGNOISE_MEASURE;

      if(VbEngineConfVbInUpstreamGet() == TRUE)
      {
        measure_configuration->numMeasure = _htons_ghn(2);
      }
      else
      {
        measure_configuration->numMeasure = _htons_ghn(1);
      }

      measure_configuration->storageType        = measconfdata->StorageType;
      measure_configuration->symbolsNumber      = measconfdata->SymbolsNumber;
      measure_configuration->time               = (measconfdata->TimeAveraging & 0x0F);
      measure_configuration->frequency          = measconfdata->FrecuencyAveraging;
      measure_configuration->offset             = _htonl_ghn(measconfdata->Offset);
      measure_configuration->duration           = _htonl_ghn(measconfdata->Duration);

      //////////////////////////////////
      // Fill BGN nodes plan
      //////////////////////////////////
      pvalue += MEASURE_CONF_SIZE;

      measure_bgn_node = (t_GeneralMeasureConfiguration *)pvalue;
      measure_bgn_node->startSeqNumber          = _htons_ghn(seq_number);
      seq_number += measconfdata->NumCycles;
      measure_bgn_node->endSeqNumber            = _htons_ghn(seq_number);
      seq_number++;
      measure_bgn_node->deviceType   = DMDEVICE;

      pvalue += MEASURE_SNR_DEV_PLAN_SIZE;

      if (VbEngineConfVbInUpstreamGet() == TRUE)
      {
        measure_bgn_node = (t_GeneralMeasureConfiguration *)pvalue;
        measure_bgn_node->startSeqNumber        = _htons_ghn(seq_number);
        seq_number += measconfdata->NumCycles;
        measure_bgn_node->endSeqNumber          = _htons_ghn(seq_number);
        seq_number++;
        measure_bgn_node->deviceType = EPDEVICE;
      }

      *endSeqNum = seq_number;
    }
  }
  else
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasureBgnNoiseSet(INT8U* macMeasurer, t_VBDriver *thisDriver, t_processMeasure *measurePtr)
{
  INT32U i, j;
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  t_domain   *domain = NULL;
  t_node     *domainEp = NULL;
  t_domainsList *domainsList;
  BOOL found = FALSE;

  if ((thisDriver != NULL) && (measurePtr != NULL))
  {
    pthread_mutex_lock(&(thisDriver->domainsMutex));

    domainsList = &(thisDriver->domainsList);

    for(i= 0; i < domainsList->numDomains; i++)
    {
      domain = &(domainsList->domainsArray[i]);
      if(memcmp(domain->dm.MAC, macMeasurer, ETH_ALEN) == 0)
      {
        VbDatamodelNodeProcessMeasureDestroy(&(domain->dm.measures.BGNMeasure));
        domain->dm.measures.BGNMeasure = *measurePtr;
        found = TRUE;
      }

      for(j= 0; j < domain->eps.numEPs; j++)
      {
        domainEp = &(domain->eps.epsArray[j]);
        if(memcmp(domainEp->MAC, macMeasurer, ETH_ALEN) == 0)
        {
          VbDatamodelNodeProcessMeasureDestroy(&(domainEp->measures.BGNMeasure));
          domainEp->measures.BGNMeasure = *measurePtr;
          found = TRUE;
        }
      }
    }
    pthread_mutex_unlock(&(thisDriver->domainsMutex));
  }

  if(found == FALSE)
  {
    err = VB_ENGINE_ERROR_NOT_FOUND;
  }
  return err;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasureCfrSet(INT8U* macMeasurer, INT8U *macMeasured, t_VBDriver *thisDriver, t_processMeasure *measurePtr )
{
  INT32U i;
  INT32U j;
  INT32U k;
  INT32U freq_cut_profile;
  t_domain   *domain = NULL;
  t_node     *domainEp = NULL;
  t_crossMeasure *cfrMeasInfo = NULL;
  t_domainsList *domainsList;
  BOOL found = FALSE;
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

  if ((thisDriver != NULL) && (measurePtr != NULL))
  {
    pthread_mutex_lock(&(thisDriver->domainsMutex));
    domainsList = &(thisDriver->domainsList);

    for(i= 0; i < domainsList->numDomains; i++)
    {
      domain = &(domainsList->domainsArray[i]);
      if(memcmp(domain->dm.MAC, macMeasurer, ETH_ALEN) == 0)
      {
        // Measurer is DM, search in measured MAC
        for(j = 0; j < domain->dm.measures.CFRMeasureList.numCrossMeasures; j++)
        {
          cfrMeasInfo = &(domain->dm.measures.CFRMeasureList.crossMeasureArray[j]);
          if(memcmp(cfrMeasInfo->MAC, macMeasured, ETH_ALEN) == 0)
          {
            freq_cut_profile = cfrMeasInfo->measure.freqCutProfile;
            VbDatamodelNodeProcessMeasureDestroy(&(cfrMeasInfo->measure));
            cfrMeasInfo->measure = *measurePtr;
            cfrMeasInfo->measure.freqCutProfile = freq_cut_profile;
            cfrMeasInfo->measure.carrierGridIdxCutProfile = FREQ2GRIDCARRIERIDX(freq_cut_profile, cfrMeasInfo->measure.spacing);
            found = TRUE;
          }
        }
      }
      else
      {
        for(j = 0; j < domain->eps.numEPs; j++)
        {
          domainEp = &(domain->eps.epsArray[j]);
          if(memcmp(domainEp->MAC, macMeasurer, ETH_ALEN) == 0)
          {
            for(k= 0; k < domainEp->measures.CFRMeasureList.numCrossMeasures; k++)
            {
              cfrMeasInfo = &(domainEp->measures.CFRMeasureList.crossMeasureArray[k]);
              if(memcmp(cfrMeasInfo->MAC, macMeasured, ETH_ALEN) == 0)
              {
                freq_cut_profile = cfrMeasInfo->measure.freqCutProfile;
                VbDatamodelNodeProcessMeasureDestroy(&(cfrMeasInfo->measure));
                cfrMeasInfo->measure = *measurePtr;
                cfrMeasInfo->measure.freqCutProfile = freq_cut_profile;
                cfrMeasInfo->measure.carrierGridIdxCutProfile = FREQ2GRIDCARRIERIDX(freq_cut_profile, cfrMeasInfo->measure.spacing);
                found = TRUE;
              }
            }
          }
        }
      }
    }
    pthread_mutex_unlock(&(thisDriver->domainsMutex));
  }

  if(found == FALSE)
  {
    err = VB_ENGINE_ERROR_NOT_FOUND;
  }
  return err;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasureSaveHelper ( const char *folder,
    const  t_processMeasure *bgnMeasure, const t_crossMeasureList *cfrMeasureList,
    const INT8U *macMeasurer, const t_processMeasure *snrCalculated,
    const t_processMeasure *snrCalculatedLow, const char *driverId, INT32S clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U               actual_carrier;
  t_crossMeasure      *cfr_measure = NULL;
  CHAR                *file_path = NULL;
  CHAR                *buffer = NULL;
  INT16U               measure_idx;
  float               temp_float;
  INT32U               snr_idx;
  INT32U               bgn_idx;
  INT32U               cfr_idx;

  if ((bgnMeasure == NULL) || (cfrMeasureList == NULL) || (macMeasurer == NULL) || (folder == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate file path
    file_path = (CHAR *)calloc(1, strlen(folder) + VB_MEASURE_FILE_NAME_SIZE + 2);

    if (file_path == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate buffer to write
    buffer = (CHAR *)malloc(VB_MEASURE_BUFFER_TO_FILE_SIZE);

    if (buffer == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U remaining_size = VB_MEASURE_BUFFER_TO_FILE_SIZE;
    CHAR  *ptr_to_write = buffer;

    // Create file name
    sprintf(file_path, "%s/Measures_Device_MAC_%02X_%02X_%02X_%02X_%02X_%02X.csv", folder, macMeasurer[0], macMeasurer[1], macMeasurer[2], macMeasurer[3], macMeasurer[4], macMeasurer[5]);

    // Print header
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "MAC Measurer: " MAC_PRINTF_FORMAT "\n", MAC_PRINTF_DATA(macMeasurer));
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "Driver Id: %s\n", driverId);
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "Cluster Id: %d\n", clusterId);
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "First Carriers: %d\n",bgnMeasure->firstCarrier);
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "Spacing: %d\n", ((bgnMeasure->mimoInd == TRUE)?(bgnMeasure->spacing<<1):bgnMeasure->spacing));
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "Flags: %d\n",bgnMeasure->flags);
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "MIMO Ind: %d\n",bgnMeasure->mimoInd);

    // Print CFR headers
    if (cfrMeasureList->numCrossMeasures > 0)
    {
      // First print ownCFR
      for (measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures; measure_idx++)
      {
        cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

        if ((cfr_measure->measure.measuresRx1 != NULL) &&
            (cfr_measure->ownCFR))
        {
          VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "CFR " MAC_PRINTF_FORMAT "; MIMO Ind %d\n", MAC_PRINTF_DATA(cfr_measure->MAC), cfr_measure->measure.mimoInd);
          break;
        }
      }

      // Now the rest of CFRs
      for(measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures; measure_idx++ )
      {
        cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

        if ((cfr_measure->measure.measuresRx1 != NULL) &&
            (!cfr_measure->ownCFR))
        {
          VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "CFR " MAC_PRINTF_FORMAT "; MIMO Ind %d\n", MAC_PRINTF_DATA(cfr_measure->MAC), cfr_measure->measure.mimoMeas);
        }
      }
    }

    // Print RXG_COMPENSATIONS
    if (bgnMeasure->measuresRx1 != NULL)
    {
      VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "bgn rx1 %d;", bgnMeasure->rxg1Compensation);

      if (bgnMeasure->mimoInd == TRUE)
      {
        VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "bgn rx2 %d;", bgnMeasure->rxg2Compensation);
      }

      for (measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures; measure_idx++ )
      {
        cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

        if (cfr_measure->ownCFR)
        {
          VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "cfr own rx1 %d;", cfr_measure->measure.rxg1Compensation);

          if(bgnMeasure->mimoInd == TRUE)
          {
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "cfr own rx2 %d;", cfr_measure->measure.rxg2Compensation);
          }

          break;
        }
      }

      for (measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures; measure_idx ++ )
      {
        cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

        if (!cfr_measure->ownCFR)
        {
          VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "cfr xtalk rx1 %d;", cfr_measure->measure.rxg1Compensation);

          if(bgnMeasure->mimoInd == TRUE)
          {
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "cfr xtalk rx2 %d;", cfr_measure->measure.rxg2Compensation);
          }
        }
      }

      VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "\n");
    }

    // Print measures header
    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "Measures\n");

    if (bgnMeasure->mimoInd == TRUE)
    {
      VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "[SNRFull Calculated/Rx1;Rx2];[SNRLow Calculated/Rx1;Rx2];[BackGround Noise/Rx1;Rx2];");
    }
    else
    {
      VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "SNRFull Calculated;SNRLow Calculated;BackGround Noise;");
    }

    if (cfrMeasureList->numCrossMeasures > 0)
    {
      for (measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures;  measure_idx++ )
      {
        cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

        if ((cfr_measure->measure.measuresRx1 != NULL) &&
            (cfr_measure->ownCFR))
        {
          if(bgnMeasure->mimoInd == TRUE)
          {
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "[CFR " MAC_PRINTF_FORMAT " / h11; h12; h22; h21];", MAC_PRINTF_DATA(cfr_measure->MAC));
          }
          else
          {
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "CFR " MAC_PRINTF_FORMAT ";", MAC_PRINTF_DATA(cfr_measure->MAC));
          }

          break;
        }
      }

      for (measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures; measure_idx++ )
      {
        cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

        if ((cfr_measure->measure.measuresRx1 != NULL) &&
            (!cfr_measure->ownCFR))
        {
          if(bgnMeasure->mimoInd == TRUE)
          {
            // Node measurer is MIMO
            if(cfr_measure->measure.mimoMeas == TRUE)
            {
              // Node measured is MIMO
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "[CFR " MAC_PRINTF_FORMAT " / h11; h12; h22; h21];", MAC_PRINTF_DATA(cfr_measure->MAC));
            }
            else
            {
              // Node measured is SISO
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "[CFR " MAC_PRINTF_FORMAT " / h11+h12; h21+h22];", MAC_PRINTF_DATA(cfr_measure->MAC));
            }
          }
          else
          {
            // Node measured is SISO
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "CFR " MAC_PRINTF_FORMAT ";", MAC_PRINTF_DATA(cfr_measure->MAC));
          }
        }
      }
    }

    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "\n");

    // Print DATA ENTRIES
    if (bgnMeasure->measuresRx1 != NULL)
    {
      if ((bgnMeasure->numMeasures > 0) && (snrCalculated != NULL))
      {
        for (actual_carrier = 0 ; actual_carrier < snrCalculated->numMeasures ; actual_carrier++)
        {
          snr_idx = actual_carrier;

          if (snrCalculated->measuresRx1 != NULL)
          {
            temp_float = VB_MEASURE_TO_FLOAT(snrCalculated->measuresRx1[snr_idx]);
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
          }
          else
          {
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
          }

          if (bgnMeasure->mimoInd == TRUE)
          {
            if (snrCalculated->measuresRx2 != NULL)
            {
              temp_float = VB_MEASURE_TO_FLOAT(snrCalculated->measuresRx2[snr_idx]);
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
            }
            else
            {
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
            }
          }

          if (snrCalculatedLow->measuresRx1 != NULL)
          {
            temp_float = VB_MEASURE_TO_FLOAT(snrCalculatedLow->measuresRx1[snr_idx]);
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
          }
          else
          {
            VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
          }

          if (bgnMeasure->mimoInd == TRUE)
          {
            if (snrCalculatedLow->measuresRx2 != NULL)
            {
              temp_float = VB_MEASURE_TO_FLOAT(snrCalculatedLow->measuresRx2[snr_idx]);
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
            }
            else
            {
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
            }
          }


          bgn_idx = (bgnMeasure->mimoInd == TRUE)?(actual_carrier<<1):actual_carrier;

          temp_float = (VB_MEASURE_TO_FLOAT(bgnMeasure->measuresRx1[bgn_idx]) - bgnMeasure->rxg1Compensation);
          VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);

          if (bgnMeasure->mimoInd == TRUE)
          {
            if (bgnMeasure->measuresRx2 != NULL)
            {
              temp_float = (VB_MEASURE_TO_FLOAT(bgnMeasure->measuresRx2[bgn_idx]) - bgnMeasure->rxg2Compensation);
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
            }
            else
            {
              VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
            }
          }

          for (measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures; measure_idx++ )
          {
            cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

            if ((cfr_measure->measure.measuresRx1 != NULL) &&
                (cfr_measure->ownCFR))
            {
              if((bgnMeasure->mimoInd == TRUE))
              {
                // MIMO Mode
                cfr_idx = actual_carrier<<1;
              }
              else
              {
                cfr_idx = actual_carrier;
              }

              if (cfr_measure->measure.measuresRx1 != NULL)
              {
                temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx1[cfr_idx]) - cfr_measure->measure.rxg1Compensation);
                VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);

                if (cfr_measure->measure.mimoInd == TRUE)
                {
                  temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx1[cfr_idx+1]) - cfr_measure->measure.rxg1Compensation);
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
                }
              }
              else
              {
                if (bgnMeasure->mimoInd == TRUE)
                {
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";;");
                }
                else
                {
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
                }
              }

              if (bgnMeasure->mimoInd == TRUE)
              {
                if (cfr_measure->measure.measuresRx2 != NULL)
                {
                  temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx2[cfr_idx]) - cfr_measure->measure.rxg2Compensation);
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);

                  temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx2[cfr_idx+1]) - cfr_measure->measure.rxg2Compensation);
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
                }
                else
                {
                  if (bgnMeasure->mimoInd == TRUE)
                  {
                    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";;");
                  }
                  else
                  {
                    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
                  }
                }
              }
              break;
            }
          }

          for (measure_idx = 0; measure_idx < cfrMeasureList->numCrossMeasures; measure_idx++)
          {
            cfr_measure = &cfrMeasureList->crossMeasureArray[measure_idx];

            if ((cfr_measure->measure.measuresRx1 != NULL) &&
                (!cfr_measure->ownCFR))
            {
              if ((bgnMeasure->mimoInd == TRUE))
              {
                // MIMO Mode
                cfr_idx = actual_carrier<<1;
              }
              else
              {
                cfr_idx = actual_carrier;
              }

              if (cfr_measure->measure.measuresRx1 != NULL)
              {
                temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx1[cfr_idx]) - cfr_measure->measure.rxg1Compensation);
                VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);

                if (cfr_measure->measure.mimoMeas == TRUE)
                {
                  temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx1[cfr_idx+1]) - cfr_measure->measure.rxg1Compensation);
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
                }
              }
              else
              {
                if(bgnMeasure->mimoInd == TRUE)
                {
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";;");
                }
                else
                {
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
                }
              }

              if (bgnMeasure->mimoInd == TRUE)
              {
                if (cfr_measure->measure.measuresRx2 != NULL)
                {
                  temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx2[cfr_idx]) - cfr_measure->measure.rxg2Compensation);
                  VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);

                  if (cfr_measure->measure.mimoMeas == TRUE)
                  {
                    temp_float = (VB_MEASURE_TO_FLOAT(cfr_measure->measure.measuresRx2[cfr_idx+1]) - cfr_measure->measure.rxg2Compensation);
                    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "%05.2f;", temp_float);
                  }
                }
                else
                {
                  if (bgnMeasure->mimoInd == TRUE)
                  {
                    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";;");
                  }
                  else
                  {
                    VbUtilStringToBuffer(&ptr_to_write, &remaining_size, ";");
                  }
                }
              }
            }
          }
          VbUtilStringToBuffer(&ptr_to_write, &remaining_size, "\n");
        }
      }
    }

    if (buffer != NULL)
    {
      // Save to file
      VbLogSaveBufferToTextFile(file_path, "w+", VB_MEASURE_BUFFER_TO_FILE_SIZE, buffer);
    }
  }

  if (file_path != NULL)
  {
    free(file_path);
  }

  if (buffer != NULL)
  {
    free(buffer);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessSnrProbesSaveMeasureSequenceHelper ( const char *folder,
    const  t_processMeasure *snrProbesMeasure, const INT8U *macMeasurer, const char *driverId, INT32S clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U actual_carrier;
  CHAR                *file_path = NULL;
  float temp_float_rx1;
  float temp_float_rx2;

  if((snrProbesMeasure == NULL) || (macMeasurer == NULL) || (folder == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate file path
    file_path = (char *)calloc(1, strlen(folder) + VB_MEASURE_FILE_NAME_SIZE + 2);

    if (file_path == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      sprintf(file_path, "%s/Measures_Dev_MAC_%02X_%02X_%02X_%02X_%02X_%02X.csv", folder, macMeasurer[0], macMeasurer[1], macMeasurer[2], macMeasurer[3], macMeasurer[4], macMeasurer[5]);

      VbLogSaveStringToTextFile(file_path, "w+", "MAC Measurer: " MAC_PRINTF_FORMAT "\n", MAC_PRINTF_DATA(macMeasurer));
      VbLogSaveStringToTextFile(file_path, "a+", "Driver Id: %s\n", driverId);
      VbLogSaveStringToTextFile(file_path, "a+", "Cluster Id: %d\n", clusterId);
      VbLogSaveStringToTextFile(file_path, "a+", "First Carriers: %d\n",snrProbesMeasure->firstCarrier);
      VbLogSaveStringToTextFile(file_path, "a+", "Spacing: %d\n",snrProbesMeasure->spacing);
      VbLogSaveStringToTextFile(file_path, "a+", "Flags: %d\n",snrProbesMeasure->flags);
      VbLogSaveStringToTextFile(file_path, "a+", "rxg1 compensation:\n%d\n",snrProbesMeasure->rxg1Compensation);
      VbLogSaveStringToTextFile(file_path, "a+", "rxg2 compensation:\n%d\n",snrProbesMeasure->rxg2Compensation);
      VbLogSaveStringToTextFile(file_path, "a+", "MIMO Ind:\n%d\n",snrProbesMeasure->mimoInd);
      VbLogSaveStringToTextFile(file_path, "a+", "Measures\n");
      VbLogSaveStringToTextFile(file_path, "a+", "SNR Probes;\n");

      // Print DATA ENTRIES
      //
      if(snrProbesMeasure->measuresRx1 != NULL)
      {
        if(snrProbesMeasure->numMeasures > 0)
        {
          if(snrProbesMeasure->mimoInd == FALSE)
          {
            for(actual_carrier = 0 ; actual_carrier < snrProbesMeasure->numMeasures ; actual_carrier++)
            {
              temp_float_rx1 = (((float)(snrProbesMeasure->measuresRx1[actual_carrier]))/4) - snrProbesMeasure->rxg1Compensation;
              VbLogSaveStringToTextFile(file_path, "a+", "%f;\n", temp_float_rx1);
            }
          }
          else
          {
            for(actual_carrier = 0 ; actual_carrier < snrProbesMeasure->numMeasures ; actual_carrier+=2)
            {
              temp_float_rx1 = (((float)(snrProbesMeasure->measuresRx1[actual_carrier]))/4) - snrProbesMeasure->rxg1Compensation;
              temp_float_rx2 = (((float)(snrProbesMeasure->measuresRx2[actual_carrier]))/4) - snrProbesMeasure->rxg2Compensation;
              VbLogSaveStringToTextFile(file_path, "a+", "%f;%f;\n", temp_float_rx1, temp_float_rx2);
            }
          }
        }
        else
        {
          VbLogPrint(VB_LOG_ERROR, "No measures!!!!");
        }
      }
    }

    if (file_path != NULL)
    {
      free(file_path);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMeasureSaveLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  CHAR                *file_path = (CHAR *)args;

  if ((node == NULL) || (driver == NULL) || (file_path == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineMeasureSaveHelper(file_path,
        &(node->measures.BGNMeasure),
        &(node->measures.CFRMeasureList),
        node->MAC,
        &(node->measures.snrFullXtalk),
        &(node->measures.snrLowXtalk),
        driver->vbDriverID,
        driver->clusterId);
  }

  return ret;
}

/*******************************************************************/

static t_processMeasure* VbEngineMeasureSnrProbesPtrGet(INT8U* macMeasurer, BOOLEAN *found,  t_VBDriver *thisDriver )
{
  INT32U i, j;
  t_domain   *domain = NULL;
  t_node     *domainEp = NULL;
  t_processMeasure *measurePtr = NULL;
  t_domainsList *domainsList;
  *found = FALSE;

  if (thisDriver != NULL)
  {
    domainsList = &(thisDriver->domainsList);

    for(i= 0; i < domainsList->numDomains; i++)
    {
      domain = &(domainsList->domainsArray[i]);
      if(memcmp(domain->dm.MAC, macMeasurer, 6) == 0)
      {
        *found = TRUE;
        measurePtr = &(domain->dm.measures.SNRProbesMeasure);
        break;
      }
      else
      {
        for(j= 0; j < domain->eps.numEPs; j++)
        {
          domainEp = &(domain->eps.epsArray[j]);
          if(memcmp(domainEp->MAC, macMeasurer, ETH_ALEN) == 0)
          {
            *found = TRUE;
            measurePtr = &(domainEp->measures.SNRProbesMeasure);
            break;
          }
        }
      }
    }
  }

  return measurePtr;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessMeasurePlanBuild(INT32U clusterId)
{
  t_VB_engineErrorCode  result;
  t_measconfdata       *measure_conf_data = NULL;
  t_reqMeasurement     *measurement_curr_meas = NULL;
  struct timespec      initial_ts_own_clock;
  t_vbEngineNumNodes    num_nodes;

  VbCounterIncrease(VB_ENGINE_COUNTER_MEASURE_PLAN_REQUESTED);

  // Check previous allocated memory
  result = VbEngineMeasurePlanClusterResourcesFree(clusterId);

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineMeasurePlanClusterResourcesAlloc(clusterId, &measurement_curr_meas);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Get a future sequence number
    result = VbEngineClockFutureTSGet(NULL, clusterId, &(measurement_curr_meas->initialSeqNumber), &initial_ts_own_clock);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Increment plan Id
    measurement_curr_meas->planId = vbMeasurePlanId++;
    if(measurement_curr_meas->planId == 0)
    {
      measurement_curr_meas->planId++;
    }

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Measure Plan Request");

    // Get number of nodes
    result = VbEngineDataModelNumNodesInClusterXGet(clusterId, &num_nodes);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Get measure plan settings
    result = VbEngineConfMeasurePlanGet(&measure_conf_data);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineConfVbInUpstreamGet() == TRUE)
    {
      measurement_curr_meas->measurePlan.msgLen = MEASURE_PLAN_SIZE_WITHDMS(num_nodes.numCompleteLines * 2);
    }
    else
    {
      measurement_curr_meas->measurePlan.msgLen = MEASURE_PLAN_SIZE_WITHOUTDMS(num_nodes.numEps);
    }

    measurement_curr_meas->measurePlan.msg = (INT8U *)calloc(1, measurement_curr_meas->measurePlan.msgLen);
    if (measurement_curr_meas->measurePlan.msg == NULL)
    {
      result = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineMeasurementPlanCreate(measurement_curr_meas->planId,
                                           measurement_curr_meas->initialSeqNumber,
                                           measure_conf_data,
                                           measurement_curr_meas->measurePlan.msg,
                                           num_nodes,
                                           &measurement_curr_meas->endSeqNumber,
                                           clusterId);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Calculate end time
    INT32U duration;
    INT32S err;
    INT16U interval_seq_num;

    // Number of MAC cycles
    interval_seq_num = DIF16(measurement_curr_meas->endSeqNumber, measurement_curr_meas->initialSeqNumber);

    // Duration in ms
    duration = interval_seq_num * MAC_CYCLE_DURATION;

    // Add msec to starting time
    err = VbUtilTimespecMsecAdd(&initial_ts_own_clock, duration, &(measurement_curr_meas->endTime));

    if (err != 0)
    {
      result = VB_ENGINE_ERROR_CLOCK;
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    CHAR start_time_str[TIMESPEC_STR_LEN];
    CHAR end_time_str[TIMESPEC_STR_LEN];

    // Get start and end time strings
    VbUtilTimespecToString(start_time_str, initial_ts_own_clock);
    VbUtilTimespecToString(end_time_str, measurement_curr_meas->endTime);

    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Measure plan built : planId %u; startTime %s; endTime %s, seq initial %u",
        measurement_curr_meas->planId,
        start_time_str,
        end_time_str, measurement_curr_meas->initialSeqNumber);
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineMeasureRespProcess(INT8U *payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEAMeasurePlanRsp *measure_rsp_payload;

  measure_rsp_payload = (t_vbEAMeasurePlanRsp *)payload;

  if ((measure_rsp_payload == NULL) || (driver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    INT8U plan_id;

    vb_err = VbEngineMeasurePlanIdGet(driver->clusterId, &plan_id);
    if(vb_err == VB_ENGINE_ERROR_NONE)
    {
      if (measure_rsp_payload->planID != plan_id)
      {
        // Measure response from different Plan Id, ignore it
        VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "EAMeasurePlan.rsp received with error : Invalid Plan Id %u; expected  %u",
            measure_rsp_payload->planID, plan_id);

        vb_err = VB_ENGINE_ERROR_INVALID_PLANID;
      }
      else if (measure_rsp_payload->errorCode != VB_EA_MEAS_RSP_ERR_NONE)
      {
        // Some error detected, measure plan rejected by driver
        VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error %d reported by driver while processing measure plan Id %u",
            measure_rsp_payload->errorCode, measure_rsp_payload->planID);

        VbCounterIncrease(VB_ENGINE_COUNTER_MEASURE_PLAN_REJECTED);
        vb_err = VB_ENGINE_ERROR_MEASURE_PLAN;
      }
      else
      {
        // Measure plan accepted Ok
        vb_err = VB_ENGINE_ERROR_NONE;
      }
    }
    else
    {
      vb_err = VB_ENGINE_ERROR_INVALID_PLANID;
    }
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineMeasureCancelRespProcess(INT8U* payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode        vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEAMeasurePlanCancelCnf *measure_cancel_payload;

  measure_cancel_payload = (t_vbEAMeasurePlanCancelCnf *)payload;

  if ((measure_cancel_payload == NULL) || (driver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  VbCounterIncrease(VB_ENGINE_COUNTER_MEASURE_PLAN_CANCELLED);

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    INT8U plan_id;

    VbEngineMeasurePlanIdGet(driver->clusterId, &plan_id);
    if (plan_id != measure_cancel_payload->planID)
    {
      // Measure cancel rsp received from different Plan Id, ignore it

      VbLogPrintExt(VB_LOG_WARNING, driver->vbDriverID, "EAMeasurePlanCancel.cnf received with error : Invalid Plan Id %u; expected  %u",
          measure_cancel_payload->planID, plan_id);

      vb_err = VB_ENGINE_ERROR_INVALID_PLANID;
    }
    else
    {
      VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Measure Plan %u cancelled with status %d",
          measure_cancel_payload->planID, measure_cancel_payload->status);
    }
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineMeasureErrorNotifyProcess(INT8U* payload, INT32U length, t_VBDriver *driver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEAMeasureEndTrg *measure_end_trg_payload;

  measure_end_trg_payload = (t_vbEAMeasureEndTrg *)payload;

  if ((measure_end_trg_payload == NULL) || (driver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    INT8U plan_id;

    vb_err = VbEngineMeasurePlanIdGet(driver->clusterId, &plan_id);
    if(vb_err == VB_ENGINE_ERROR_NONE)
    {
      if (plan_id != measure_end_trg_payload->planID)
      {
        // Measure notify received from different Plan Id, ignore it

        VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "EAMeasureEnd.trg received with error : Invalid Plan Id %u; expected  %u",
            measure_end_trg_payload->planID, plan_id);

        vb_err = VB_ENGINE_ERROR_INVALID_PLANID;
      }
      else
      {
        if (measure_end_trg_payload->errorCode != VB_EA_MEAS_RSP_ERR_NONE)
        {
          VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error (%d) Notified in Measure plan (%u)", measure_end_trg_payload->errorCode, measure_end_trg_payload->planID);
        }
        else
        {
          VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error Notified in Measure plan but plan Id (%u) ok and Err = 0", measure_end_trg_payload->planID);
        }

        VbCounterIncrease(VB_ENGINE_COUNTER_MEASURE_PLAN_FAILED);
      }
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "EAMeasureEnd.trg received with error : No plan Id for clusterId %u; err %l",
          driver->clusterId, vb_err);
    }
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessMeasurePlanRequestSend(t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  t_reqMeasurement *measurement_curr_meas;

  if (thisDriver == NULL)
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    t_VBCluster *cluster;

    result =  VbEngineClusterByIdGet(thisDriver->clusterId, &cluster);
    if(result == VB_ENGINE_ERROR_NONE)
    {
      measurement_curr_meas = (t_reqMeasurement *)cluster->measurePlanData;
      if ((measurement_curr_meas->measurePlan.msg == NULL) ||
          (measurement_curr_meas->measurePlan.msgLen == 0))
      {
        result = VB_ENGINE_ERROR_MEASURE_PLAN;
      }
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Error %d", result);
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_MEASURE_PLAN_REQ,
                                          measurement_curr_meas->measurePlan.msg,
                                          measurement_curr_meas->measurePlan.msgLen,
                                          thisDriver);
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessMeasurePlanCancelRequestSend(t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  t_vbEAMeasurePlanCancelReq *measure_plan_cancel;
  INT8U                      *payload;

  payload = (INT8U*)calloc(1, sizeof(t_vbEAMeasurePlanCancelReq));

  if(payload == NULL)
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(result == VB_ENGINE_ERROR_NONE)
  {
    INT8U planId;
    measure_plan_cancel = ( t_vbEAMeasurePlanCancelReq *)payload;

    result = VbEngineMeasurePlanIdGet(thisDriver->clusterId, &planId);
    if(result == VB_ENGINE_ERROR_NONE)
    {
      // Send measures request to EPs (DMs are the measured nodes)
      measure_plan_cancel->planID = planId;
      result = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_MEASURE_PLAN_CANCEL_REQ, payload, VB_EA_MEASURE_PLAN_CANCEL_REQ_SIZE, thisDriver);
    }

    free(payload);
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessCollectMeasuresStart(INT32U clusterId)
{
  t_VB_engineErrorCode result;
  t_nodesMacList       all_nodes_mac_list;

  // Collect all MACs
  result = VbEngineDatamodelClusterXAllNodesMacGet(&all_nodes_mac_list, TRUE, clusterId);

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Measure Collect Start cluster %d", clusterId);

  if (result == VB_ENGINE_ERROR_NONE)
  {
    /*
     * Allocate memory to store Crosstalk & direct DMs measures and
     * fill memory crosstalk list with MACs to be measured
     */
    result = VbEngineDatamodelClusterXAllNodesLoop(VbEngineCfrMeasureListCreateCb, clusterId, (void *)&all_nodes_mac_list);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Send measures request to each driver
    result = VbEngineDatamodelClusterXDriversLoop(VbEngineMeasCollectRequestPerDriverCb, clusterId, (void*)&all_nodes_mac_list);
  }

  // Always release memory
  VbEngineDatamodelMacListRelease(&all_nodes_mac_list);

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasCollectReqSend( INT8U planId, t_macsInfo *dmsMeasurer,
    t_macsInfo *epsMeasurer, t_macsInfo *dmsMeasured, t_macsInfo *epsMeasured, t_VBDriver *thisDriver )
{
  t_VB_engineErrorCode      result = VB_ENGINE_ERROR_NONE;
  INT8U                    *buffer = NULL;
  INT32U                    alloc_size;
  INT32U                    num_measurer;
  INT32U                    dm_measurer_size;
  INT32U                    ep_measurer_size;
  t_measconfdata           *measure_conf_data = NULL;

  if ((planId == 0) || (thisDriver == NULL) ||
      (dmsMeasurer == NULL) || (epsMeasurer == NULL) ||
      (dmsMeasured == NULL) || (epsMeasured == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    result = VbEngineConfMeasurePlanGet(&measure_conf_data);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Always: EPs are measurer (DMs are the measured nodes)
    num_measurer = epsMeasurer->numNodes;

    // Calculate memory for EP measurer nodes (common part + list of measured MACs)
    ep_measurer_size = epsMeasurer->numNodes * (VB_EA_MEAS_COLLECT_REQ_NODE_SIZE + (dmsMeasured->numNodes * ETH_ALEN));

    if (VbEngineConfVbInUpstreamGet() == TRUE)
    {
      // Only when VbEngineConfVbInUpstreamGet == TRUE: DMs are measurer (EPs are the measured nodes)
      num_measurer += dmsMeasurer->numNodes;

      // Calculate memory for DM measurer nodes (common part + list of measured MACs)
      dm_measurer_size = dmsMeasurer->numNodes * (VB_EA_MEAS_COLLECT_REQ_NODE_SIZE + (epsMeasured->numNodes * ETH_ALEN));
    }
    else
    {
      dm_measurer_size = 0;
    }

    // Allocate memory for payload (header + measurer size)
    alloc_size = VB_EA_MEAS_COLLECT_REQ_HDR_SIZE + ep_measurer_size + dm_measurer_size;

    buffer = calloc(1, alloc_size);

    if (buffer == NULL)
    {
      result = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    t_vbEAMeasCollectReqHdr  *meas_collect_req_hdr;
    t_vbEAMeasCollectReqNode *meas_collect_req_node;
    INT32U                    measurer_idx;
    INT32U                    num_measured;
    INT8U                    *measurer_mac_ptr;
    INT8U                    *measured_mac_ptr;
    INT8U                    *payload_ptr;

    payload_ptr = buffer;
    meas_collect_req_hdr = (t_vbEAMeasCollectReqHdr *)payload_ptr;
    meas_collect_req_hdr->planID = planId;
    meas_collect_req_hdr->dataType = measure_conf_data->MeasureDataType;
    meas_collect_req_hdr->formatType = measure_conf_data->MeasureDataFormat;
    meas_collect_req_hdr->numMACsMeasurer = _htons(num_measurer);

    // Advance to measurer section
    payload_ptr += VB_EA_MEAS_COLLECT_REQ_HDR_SIZE;

    for (measurer_idx = 0; measurer_idx < num_measurer; measurer_idx++)
    {
      // Calculate measurer MAC
      if (measurer_idx < epsMeasurer->numNodes)
      {
        // Get next EP measurer MAC
        measurer_mac_ptr = epsMeasurer->ptr + (ETH_ALEN * measurer_idx);

        // Get list of measured MACs (measurer is EP, so measured are DMs)
        measured_mac_ptr = dmsMeasured->ptr;

        // Number of measured devices is equal to number of DMs
        num_measured = dmsMeasured->numNodes;
      }
      else if (VbEngineConfVbInUpstreamGet() == TRUE)
      {
        // This is only possible when VB in Upstream is enabled

        // Get next DM measurer MAC
        measurer_mac_ptr = dmsMeasurer->ptr + (ETH_ALEN * (measurer_idx - epsMeasurer->numNodes));

        // Get list of measured MACs (measurer is DM, so measured are EPs)
        measured_mac_ptr = epsMeasured->ptr;

        // Number of measured devices is equal to number of EPs
        num_measured = epsMeasured->numNodes;
      }
      else
      {
        VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Inconsistent nodes at database when building MeasureCollect.req frame");
        result = VB_ENGINE_ERROR_DATA_MODEL;
      }

      if (result == VB_ENGINE_ERROR_NONE)
      {
        // Fill measurer section
        meas_collect_req_node = (t_vbEAMeasCollectReqNode *)payload_ptr;

        // Copy measurer MAC
        MACAddrClone(meas_collect_req_node->macMeasurer, measurer_mac_ptr);

        // Update number of measured MACs
        meas_collect_req_node->numMACsMeasured = _htons(num_measured);

        // Advance to measured MAC list
        payload_ptr += VB_EA_MEAS_COLLECT_REQ_NODE_SIZE;

        // Copy measured MAC list
        memcpy(payload_ptr, measured_mac_ptr, (num_measured * ETH_ALEN));

        // Advance to next measurer node
        payload_ptr += num_measured * ETH_ALEN;
      }

      if (result != VB_ENGINE_ERROR_NONE)
      {
        break;
      }
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    // Send frame
    result = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_MEAS_COLLECT_REQ, buffer, alloc_size, thisDriver);
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, thisDriver->vbDriverID, "Sent EAMeasCollect.req");
  }

  if (buffer != NULL)
  {
    free(buffer);
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineProcessMeasurePlanTimeToFinishGet(INT32U clusterId, INT32U *timeToFinish)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  struct timespec     current_ts;

  if (timeToFinish == NULL)
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
    INT64S      time_to_finish;
    t_VBCluster *cluster;

    result = VbEngineClusterByIdGet(clusterId, &cluster);
    if(result == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if(cluster->measurePlanData == NULL)
      {
        result = VB_ENGINE_ERROR_MEASURE_PLAN;
      }
    }

    if(result == VB_ENGINE_ERROR_NONE)
    {
      t_reqMeasurement *measurement_curr_meas = (t_reqMeasurement *)cluster->measurePlanData;

      // Calculate time to finish
      time_to_finish = VbUtilElapsetimeTimespecMs(current_ts, measurement_curr_meas->endTime);

      *timeToFinish = (INT32U)time_to_finish + VB_ENGINE_EXTRA_MEASURE_PLAN_DURATION;
    }
  }

  return result;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineCFRRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEACFRMeasure *cfrRsp;
  t_processMeasure measure;
  INT8U* pld_carriers_info;
  INT32U i;

  if(payload != NULL)
  {
    cfrRsp = ( t_vbEACFRMeasure *)payload;

    measure.measuresRx1 = NULL;
    measure.measuresRx2 = NULL;
    measure.errorCode = cfrRsp->ErrorCode;

    if (measure.errorCode  == VB_MEAS_ERRCODE_VALID)
    {
      measure.numMeasures = _ntohs(cfrRsp->numCarriers);
      measure.rxg1Compensation = cfrRsp->rxg1Compensation;
      measure.rxg2Compensation = cfrRsp->rxg2Compensation;
      measure.mimoInd = cfrRsp->mimoInd;
      measure.mimoMeas = cfrRsp->mimoMeas;
      measure.firstCarrier = _ntohs(cfrRsp->firstCarrier);
      measure.errorCode = cfrRsp->ErrorCode;
      measure.spacing = cfrRsp->spacing;
      measure.flags = cfrRsp->flags;
      measure.planID = cfrRsp->planId;
      measure.type = VB_MEAS_TYPE_CFR;

      pld_carriers_info = (INT8U *)(payload + sizeof(t_vbEACFRMeasure));

      if(measure.mimoInd == FALSE)
      {
        // Measurer node Mode is SISO, all measure info are related to Rx1
        measure.measuresRx1 = calloc(1, measure.numMeasures);
        measure.measuresRx2 = NULL;

        if(measure.measuresRx1)
        {
          memcpy(measure.measuresRx1, pld_carriers_info, measure.numMeasures);
        }
      }
      else
      {
        // Measurer node Mode is MIMO
        // Split received info in Rx1 and Rx2

        // Alloc memory for Rx1
        measure.measuresRx1 = calloc(1, measure.numMeasures/2);
        if(measure.measuresRx1 == NULL)
        {
          vb_err = VB_ENGINE_ERROR_MALLOC;
        }

        // Alloc memory for Rx2
        if(vb_err == VB_ENGINE_ERROR_NONE)
        {
          measure.measuresRx2 = calloc(1, measure.numMeasures/2);
          if(measure.measuresRx2 == NULL)
          {
            vb_err = VB_ENGINE_ERROR_MALLOC;
          }
        }

        if(vb_err == VB_ENGINE_ERROR_NONE)
        {
          if(measure.mimoMeas)
          {
            // Node Measured was also in MIMO
            // | | = 1 byte
            //        *       ci      *      ci+2      *
            // Change |h11|h21|h12|h22| ...
            // To:
            //              *  ci   *  ci+2 *
            //      - Rx1 : |h11|h12|...
            //      - Rx2 : |h22|h21|...
            for(i=0; i < measure.numMeasures; i+=4)
            {
              measure.measuresRx1[(i>>1)+0] = pld_carriers_info[i];
              measure.measuresRx2[(i>>1)+1] = pld_carriers_info[i+1];
              measure.measuresRx1[(i>>1)+1] = pld_carriers_info[i+2];
              measure.measuresRx2[(i>>1)+0] = pld_carriers_info[i+3];
            }

            // There are now numMeasures/2 in each Rxi
            measure.numMeasures >>=1;
          }
          else
          {
            // Node Measured was in SISO
            // | | = 1 byte
            //        *       ci      *     ci+1      *
            // Change |h11+h12|h21+h22| ...
            // To:
            //              *  ci   *  ci+1 *
            //      - Rx1 : |h11+h12|...
            //      - Rx2 : |h21+h22|...
            for(i=0; i < measure.numMeasures; i+=2)
            {
              measure.measuresRx1[i>>1] = pld_carriers_info[i];
              measure.measuresRx2[i>>1] = pld_carriers_info[i+1];
            }

            // There are now numMeasures/2 in each Rxi
            measure.numMeasures >>=1;
          }
        }
      }

      vb_err = VbEngineMeasureCfrSet(cfrRsp->MACMeasurer, cfrRsp->MACMeasured, thisDriver, &measure);

      if (vb_err == VB_ENGINE_ERROR_NOT_FOUND)
      {
        VbDatamodelNodeProcessMeasureDestroy(&measure);
      }
    }
    else
    {
      vb_err = VB_ENGINE_ERROR_MEASURE_NODE;
    }
  }
  else
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineBngNoiseRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEABGNMeasure    *bngRsp;
  t_processMeasure     measure;
  INT8U               *pld_carriers_info;
  INT32U               i;

  if(payload != NULL)
  {
    bngRsp = (t_vbEABGNMeasure *)payload;

    measure.measuresRx1 = NULL;
    measure.measuresRx2 = NULL;
    measure.errorCode = bngRsp->ErrorCode;

    if (measure.errorCode  == VB_MEAS_ERRCODE_VALID)
    {
      measure.numMeasures = _ntohs(bngRsp->numCarriers);
      measure.rxg1Compensation = bngRsp->rxg1Compensation;
      measure.rxg2Compensation = bngRsp->rxg2Compensation;
      measure.mimoInd = bngRsp->mimoInd;
      measure.mimoMeas = bngRsp->mimoMeas;
      measure.firstCarrier = _ntohs(bngRsp->firstCarrier);
      measure.spacing = bngRsp->spacing;
      measure.flags = bngRsp->flags;
      measure.planID = bngRsp->planId;
      measure.type = VB_MEAS_TYPE_BGN;

      pld_carriers_info = (INT8U *)(payload + sizeof(t_vbEABGNMeasure));

      if (measure.mimoInd == FALSE)
      {
        // Node measurer Mode is SISO
        measure.measuresRx1 = calloc(1, measure.numMeasures);

        if(measure.measuresRx1)
        {
          measure.measuresRx2 = NULL;
          memcpy(measure.measuresRx1, pld_carriers_info,  measure.numMeasures);

          vb_err = VbEngineMeasureBgnNoiseSet(bngRsp->MAC, thisDriver, &measure);

          if(vb_err == VB_ENGINE_ERROR_NOT_FOUND)
          {
            VbDatamodelNodeProcessMeasureDestroy(&measure);
          }
        }
        else
        {
          vb_err = VB_ENGINE_ERROR_MALLOC;
        }
      }
      else
      {
        // Node measurer Mode is MIMO
        measure.measuresRx1 = calloc(1, measure.numMeasures/2);

        if (measure.measuresRx1 == NULL)
        {
          vb_err = VB_ENGINE_ERROR_MALLOC;
        }

        if (vb_err == VB_ENGINE_ERROR_NONE)
        {
          measure.measuresRx2 = calloc(1, measure.numMeasures/2);

          if (measure.measuresRx2 == NULL)
          {
            vb_err = VB_ENGINE_ERROR_MALLOC;
          }
        }

        if (vb_err == VB_ENGINE_ERROR_NONE)
        {
          // Split noise info in corresponding reception path
          // Change |Noise Rx1 ci|Noise Rx2 ci|Noise Rx1 ci+1|Noise Rx2 ci+1|.....
          // To:
          //      - Rx1 ->  |Noise Rx1 ci|Noise Rx1 ci+1|....
          //      - Rx2 ->  |Noise Rx2 ci|Noise Rx2 ci+1|....
          for (i=0; i < measure.numMeasures; i+=2)
          {
            measure.measuresRx1[(i>>1)] = pld_carriers_info[i];
            measure.measuresRx2[(i>>1)] = pld_carriers_info[i+1];
          }

          // There are now numMeasures/2 in each Rxi
          measure.numMeasures >>=1;

          vb_err = VbEngineMeasureBgnNoiseSet(bngRsp->MAC, thisDriver, &measure);

          if (vb_err == VB_ENGINE_ERROR_NOT_FOUND)
          {
            VbDatamodelNodeProcessMeasureDestroy(&measure);
          }
        }
      }
    }
    else
    {
      vb_err = VB_ENGINE_ERROR_MEASURE_NODE;
    }
  }
  else
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineSnrProbesRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEASNRMeasure *snrRsp;
  BOOLEAN found = FALSE;
  t_processMeasure *measurePtr = NULL;
  INT8U* pld_carriers_info;
  INT32U spacing_temp;
  INT32U i;

  if ((payload != NULL) && (thisDriver != NULL))
  {
    snrRsp = (t_vbEASNRMeasure *)payload;
    measurePtr = VbEngineMeasureSnrProbesPtrGet(snrRsp->MAC, &found, thisDriver);
    if(found == TRUE)
    {
      if(measurePtr != NULL)
      {
        if(measurePtr->measuresRx1)
        {
          free(measurePtr->measuresRx1);
          measurePtr->measuresRx1 = NULL;
        }
      }

      if(measurePtr != NULL)
      {
        measurePtr->errorCode = snrRsp->ErrorCode;

        if(measurePtr->errorCode == VB_MEAS_ERRCODE_VALID)
        {
          measurePtr->numMeasures = _ntohs(snrRsp->numCarriers);
          measurePtr->rxg1Compensation = snrRsp->rxg1Compensation;
          measurePtr->rxg2Compensation = snrRsp->rxg2Compensation;
          measurePtr->firstCarrier = _ntohs(snrRsp->firstCarrier);
          measurePtr->mimoInd = snrRsp->mimoInd;
          spacing_temp = (_ntohl(*(INT32U *)(&snrRsp->spacing)));
          measurePtr->spacing = *((float *)(&spacing_temp));
          measurePtr->flags = snrRsp->flags;
          measurePtr->type = VB_MEAS_TYPE_SNRPROBE;

          pld_carriers_info = (INT8U *)(payload + sizeof(t_vbEASNRMeasure));
          if(measurePtr->mimoInd == FALSE)
          {
            measurePtr->measuresRx1 = calloc(1, measurePtr->numMeasures);
            if(measurePtr->measuresRx1)
            {
              memcpy(measurePtr->measuresRx1, pld_carriers_info,  measurePtr->numMeasures);
            }
          }
          else
          {
            measurePtr->numMeasures >>=1;
            measurePtr->measuresRx1 = calloc(1, measurePtr->numMeasures);
            if(measurePtr->measuresRx1)
            {
              measurePtr->measuresRx2 = calloc(1, measurePtr->numMeasures);
              if(measurePtr->measuresRx2)
              {
                for(i=0; i<measurePtr->numMeasures; i++)
                {
                  measurePtr->measuresRx1[i] = pld_carriers_info[2*i];
                  measurePtr->measuresRx2[i] = pld_carriers_info[2*i+1];
                }
              }
            }
          }
        }
        else
        {
          vb_err = VB_ENGINE_ERROR_MEASURE_NODE;
        }
      }
    }
    else
    {
      vb_err = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }
  else
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(vb_err == VB_ENGINE_ERROR_NONE)
  {
    CHAR            *file_path = NULL;

    // Allocate file path
    file_path = (char *)calloc(1, strlen(VB_MEASURE_PROBES_FOLDER) + TIMESPEC_FILE_NAME_STR_LEN + 1);
    if (file_path == NULL)
    {
      vb_err = VB_ENGINE_ERROR_MALLOC;
    }

    if (vb_err == VB_ENGINE_ERROR_NONE)
    {
      CHAR            *file_path_ptr = file_path;
      struct timespec current_ts;

      // Copy root measure folder to path (not including the terminating /0)
      strncpy(file_path_ptr, VB_MEASURE_PROBES_FOLDER, strlen(VB_MEASURE_PROBES_FOLDER));
      file_path_ptr +=  strlen(VB_MEASURE_PROBES_FOLDER);

      // Add '/'
      *file_path_ptr = '/';
      file_path_ptr++;

      // Get current time and append it to file path
      clock_gettime(CLOCK_REALTIME, &current_ts);
      VbUtilTimespecToFileName(file_path_ptr, current_ts);

      // Loop through all nodes to dump measures
      VbEngineProcessSnrProbesSaveMeasureSequenceHelper(file_path, measurePtr, snrRsp->MAC, thisDriver->vbDriverID, thisDriver->clusterId);
    }

    if (file_path != NULL)
    {
      free(file_path);
    }
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEnginePSDRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineMeasCollectionEndProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEAMeasureCollectEnd *measDone = (t_vbEAMeasureCollectEnd *)payload;

  if (measDone == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT8U plan_id;

    ret = VbEngineMeasurePlanIdGet(thisDriver->clusterId, &plan_id);
    if (ret == VB_ENGINE_ERROR_NONE)
    {
      if(plan_id != measDone->PlanID)
      {
        VbCounterIncrease(VB_ENGINE_COUNTER_MEASURE_PLAN_FAILED);
        ret = VB_ENGINE_ERROR_MEASURE_PLAN;
      }
      else
      {
        VbCounterIncrease(VB_ENGINE_COUNTER_MEASURE_PLAN_SUCCESS);
      }
    }
  }

  return ret;
}

/*******************************************************************/

void VbEngineMeasureConfDump(t_writeFun writeFun)
{
  t_measconfdata *meas_conf_data;

  VbEngineConfMeasurePlanGet(&meas_conf_data);

  writeFun("=====================================================\n");
  writeFun("|           Parameter          |        Value       |\n");
  writeFun("=====================================================\n");
  writeFun("| %28s | %18u |\n", "NumCycles",          meas_conf_data->NumCycles);
  writeFun("| %28s | %18u |\n", "StorageType",        meas_conf_data->StorageType);
  writeFun("| %28s | %18u |\n", "SymbolsNumber",      meas_conf_data->SymbolsNumber);
  writeFun("| %28s | %18u |\n", "TimeAveraging",      meas_conf_data->TimeAveraging);
  writeFun("| %28s | %18u |\n", "FrecuencyAveraging", meas_conf_data->FrecuencyAveraging);
  writeFun("| %28s | %18u |\n", "Offset",             meas_conf_data->Offset);
  writeFun("| %28s | %18u |\n", "Duration",           meas_conf_data->Duration);
  writeFun("| %28s | %18u |\n", "CFRMeasureType",     meas_conf_data->CFRMeasureType);
  writeFun("| %28s | %18u |\n", "MeasureDataType",    meas_conf_data->MeasureDataType);
  writeFun("| %28s | %18u |\n", "MeasureDataFormat",  meas_conf_data->MeasureDataFormat);
  writeFun("=====================================================\n");
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasureConfigurationParse( ezxml_t measure_conf )
{
  t_VB_engineErrorCode err_code;
  ezxml_t              ez_temp;
  t_measconfdata       *measure_conf_data = NULL;

  err_code = VbEngineConfMeasurePlanGet(&measure_conf_data);

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    if (measure_conf == NULL)
    {
      err_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "NumCycles");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->NumCycles = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid NumCycles value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "StorageType");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->StorageType = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid StorageType value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "SymbolsNumber");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->SymbolsNumber = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid SymbolsNumber value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "TimeAveraging");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->TimeAveraging = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid TimeAveraging value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "FrecuencyAveraging");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->FrecuencyAveraging = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid FrecuencyAveraging value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "Offset");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->Offset = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Offset value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "Duration");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->Duration = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Duration value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "CFRMeasureType");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->CFRMeasureType = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid CFRMeasureType value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "MeasureDataType");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->MeasureDataType = (t_MeasureDataType)strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid MeasureDataType value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(measure_conf, "MeasureDataFormat");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      measure_conf_data->MeasureDataFormat = (t_MeasureDataFormat)strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid MeasureDataFormat value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  return err_code;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasurePlanDump(t_writeFun writeFun, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_reqMeasurement    *measurement_curr_meas;

  if (writeFun == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_VBCluster *cluster;

    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if(cluster->measurePlanData == NULL)
      {
        ret = VB_ENGINE_ERROR_MEASURE_PLAN;
      }
    }

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      measurement_curr_meas = (t_reqMeasurement *)cluster->measurePlanData;
      if (measurement_curr_meas->measurePlan.msg == NULL)
      {
        writeFun("Measure Plan empty.\n");
        ret = VB_ENGINE_ERROR_MEASURE_PLAN;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U                         node_idx;
    t_MeasureConfiguration        *meas_conf;
    t_MeasureCFRDevice            *cfr_node;
    t_GeneralMeasureConfiguration *bgn_node;
    INT8U                         *ptr = measurement_curr_meas->measurePlan.msg;
    CHAR                           mac_str[MAC_STR_LEN];

    writeFun("=====================================================\n");
    writeFun("|           Parameter          |        Value       |\n");
    writeFun("=====================================================\n");

    // ParameterId
    writeFun("| %28s | %18u |\n", "HGF ParameterId",          *ptr);
    ptr++;

    // PlanId
    writeFun("| %28s | %18u |\n", "Plan Id",                  *ptr);
    ptr++;
    writeFun("-----------------------------------------------------\n");

    //////////////////////////////////
    // CFR measure configuration
    //////////////////////////////////
    writeFun("----------------- CFR Meas Conf ---------------------\n");
    meas_conf = (t_MeasureConfiguration *)ptr;
    writeFun("| %28s | %18u |\n", "Measure Type",              meas_conf->measureType);
    writeFun("| %28s | %18u |\n", "Num Measures",              _ntohs_ghn(meas_conf->numMeasure));
    writeFun("| %28s | %18u |\n", "Storage Type",              meas_conf->storageType);
    writeFun("| %28s | %18u |\n", "Symbols Number",            meas_conf->symbolsNumber);
    writeFun("| %28s | %18u |\n", "Time Avg",                  meas_conf->time);
    writeFun("| %28s | %18u |\n", "Frequency Avg",             meas_conf->frequency);
    writeFun("| %28s | %18u |\n", "Offset",                    _ntohl_ghn(meas_conf->offset));
    writeFun("| %28s | %18u |\n", "Duration",                  _ntohl_ghn(meas_conf->duration));
    writeFun("-----------------------------------------------------\n");
    ptr += MEASURE_CONF_SIZE;

    //////////////////////////////////
    // CFR nodes plan
    //////////////////////////////////
    writeFun("----------------- CFR Nodes Conf --------------------\n");
    for (node_idx = 0; node_idx < _ntohs_ghn(meas_conf->numMeasure); node_idx++)
    {
      cfr_node = (t_MeasureCFRDevice *)(ptr);
      writeFun("| %28s | %18u |\n", "Start SeqNum",            _ntohs_ghn(cfr_node->GeneralMeasureConfiguration.startSeqNumber));
      writeFun("| %28s | %18u |\n", "End SeqNum",              _ntohs_ghn(cfr_node->GeneralMeasureConfiguration.endSeqNumber));
      writeFun("| %28s | %18u |\n", "Device Type",             cfr_node->GeneralMeasureConfiguration.deviceType);
      writeFun("| %28s | %18u |\n", "Device Id",               cfr_node->nodeBasicInfo.devID);
      writeFun("| %28s | %18u |\n", "Ext Seed",                _ntohs_ghn(cfr_node->nodeBasicInfo.extSeed));

      MACAddrMem2str(mac_str, cfr_node->nodeBasicInfo.measureDeviceMAC);
      writeFun("| %28s | %18s |\n", "MAC Addr",                mac_str);
      writeFun("-----------------------------------------------------\n");
      ptr += MEASURE_CFR_DEV_PLAN_SIZE;
    }

    //////////////////////////////////
    // BGN measure configuration
    //////////////////////////////////
    writeFun("----------------- BGN Meas Conf ---------------------\n");
    meas_conf = (t_MeasureConfiguration *)ptr;
    writeFun("| %28s | %18u |\n", "Measure Type",              meas_conf->measureType);
    writeFun("| %28s | %18u |\n", "Num Measures",              _ntohs_ghn(meas_conf->numMeasure));
    writeFun("| %28s | %18u |\n", "Storage Type",              meas_conf->storageType);
    writeFun("| %28s | %18u |\n", "Symbols Number",            meas_conf->symbolsNumber);
    writeFun("| %28s | %18u |\n", "Time Avg",                  meas_conf->time);
    writeFun("| %28s | %18u |\n", "Frequency Avg",             meas_conf->frequency);
    writeFun("| %28s | %18u |\n", "Offset",                    _ntohl_ghn(meas_conf->offset));
    writeFun("| %28s | %18u |\n", "Duration",                  _ntohl_ghn(meas_conf->duration));
    writeFun("-----------------------------------------------------\n");
    ptr += MEASURE_CONF_SIZE;

    //////////////////////////////////
    // BGN nodes plan
    //////////////////////////////////
    writeFun("----------------- BGN Nodes Conf --------------------\n");
    bgn_node = (t_GeneralMeasureConfiguration *)ptr;
    writeFun("| %28s | %18u |\n", "Start SeqNum",            _ntohs_ghn(bgn_node->startSeqNumber));
    writeFun("| %28s | %18u |\n", "End SeqNum",              _ntohs_ghn(bgn_node->endSeqNumber));
    writeFun("| %28s | %18u |\n", "Device Type",             bgn_node->deviceType);
    writeFun("-----------------------------------------------------\n");

    if (_ntohs_ghn(meas_conf->numMeasure) > 1)
    {
      writeFun("| %28s | %18u |\n", "Start SeqNum",            _ntohs_ghn(bgn_node->startSeqNumber));
      writeFun("| %28s | %18u |\n", "End SeqNum",              _ntohs_ghn(bgn_node->endSeqNumber));
      writeFun("| %28s | %18u |\n", "Device Type",             bgn_node->deviceType);
      writeFun("-----------------------------------------------------\n");
    }

    writeFun("=====================================================\n");
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasurePlanIdGet(INT32U clusterId, INT8U *planId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster          *cluster;

  if(planId == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      // Check previous allocated memory
      if(cluster->measurePlanData == NULL)
      {
        ret = VB_ENGINE_ERROR_MEASURE_PLAN;
      }
    }

    if(ret == VB_ENGINE_ERROR_NONE)
    {
      t_reqMeasurement *measurement_curr_meas;

      measurement_curr_meas = (t_reqMeasurement *)cluster->measurePlanData;
      *planId = measurement_curr_meas->planId;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasurePlanInit(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

   vbMeasurePlanId = 0;
   return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasureSave(INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  CHAR                *file_path = NULL;

  // Allocate file path
  file_path = (char *)calloc(1, strlen(VB_MEASURE_ROOT_FOLDER) + VB_MEASURE_CLUSTER_ID_LEN + TIMESPEC_FILE_NAME_STR_LEN + 1);

  if (file_path == NULL)
  {
    ret = VB_ENGINE_ERROR_MALLOC;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    struct timespec current_ts;
    CHAR            *file_path_ptr = file_path;

    // Copy root measure folder to path (not including the terminating /0)
    strncpy(file_path_ptr, VB_MEASURE_ROOT_FOLDER, strlen(VB_MEASURE_ROOT_FOLDER));
    file_path_ptr +=  strlen(VB_MEASURE_ROOT_FOLDER);

    // Add '/'
    *file_path_ptr = '/';
    file_path_ptr++;

    // Add cluster Id
    snprintf(file_path_ptr, VB_MEASURE_CLUSTER_ID_LEN, VB_MEASURE_CLUSTER_ID_FMT, clusterId);
    file_path_ptr += VB_MEASURE_CLUSTER_ID_LEN - 1;

    // Get current time and append it to file path
    clock_gettime(CLOCK_REALTIME, &current_ts);
    VbUtilTimespecToFileName(file_path_ptr, current_ts);
  }

  // Loop through all nodes to dump measures
  ret = VbEngineDatamodelClusterXAllNodesLoop(VbEngineMeasureSaveLoopCb, clusterId, file_path);

  if (file_path != NULL)
  {
    free(file_path);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasurePlanClusterResourcesFree(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  t_VBCluster          *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Check previous allocated memory
    if (cluster->measurePlanData != NULL)
    {
      t_reqMeasurement       *measurement_curr_meas = (t_reqMeasurement *)cluster->measurePlanData;

      // Release previous memory
      if(measurement_curr_meas->measurePlan.msg != NULL)
      {
        free(measurement_curr_meas->measurePlan.msg);
        measurement_curr_meas->measurePlan.msg = NULL;
        measurement_curr_meas->measurePlan.msgLen = 0;
      }

      free(cluster->measurePlanData);
      cluster->measurePlanData = NULL;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineMeasurePlanClusterResourcesAlloc(INT32U clusterId, t_reqMeasurement **measurementReq)
{
  t_VB_engineErrorCode ret;
  t_VBCluster          *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    // Check previous allocated memory
    cluster->measurePlanData = (INT8U *)calloc(1, sizeof(t_reqMeasurement));
    if(cluster->measurePlanData == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    *measurementReq = (t_reqMeasurement *)cluster->measurePlanData;
    (*measurementReq)->measurePlan.msg = NULL;
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
