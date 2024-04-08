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
 * @file vb_engine_dataModel.c
 * @brief Implements engine data model
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
#include <stdio.h>
#include <errno.h>
#include <pthread.h>

#include "vb_engine_datamodel.h"
#include "vb_engine_conf.h"
#include "vb_log.h"
#include "vb_engine_process.h"
#include "vb_engine_measure.h"
#include "vb_priorities.h"
#include "vb_counters.h"
#include "vb_util.h"
#include "vb_timer.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_drivers_list.h"

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

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct s_loopArgs
{
  INT8U              *MAC;
  t_vbEngineBoostMode boostMode;
  BOOLEAN             found;
} t_loopArgs;

typedef struct s_seedArgs
{
  INT8U              *MAC;
  INT16U              seedIdx;
  INT16U              did;
} t_seedArgs;

typedef struct s_domainByMacSearchParams
{
  INT8U               MAC[ETH_ALEN];
  t_domain           *domain;
} t_domainByMacSearchParams;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static const CHAR *qosRateString[VB_ENGINE_QOS_RATE_LAST] =
{
  "0/100",
  "10/90",
  "20/80",
  "30/70",
  "40/60",
  "50/50",
  "60/40",
  "70/30",
  "80/20",
  "90/10",
};

/// Timers mutex
static pthread_mutex_t vbEngineTimersMutex;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/**
 * @brief This function destroys
 *
**/
static void VbDatamodelListEpsDestroy( t_endPointList *epsLists );

static void VbDatamodelNodeMeasuresDestroy( t_nodeMeasures *nodeMeasures );

static void VbDatamodelNodePsdDestroy( t_PSD *Psd );

static t_domain* VbEngineDatamodelDomainFind(INT8U *macDM, t_domainsList *domainsList);

static t_VB_engineErrorCode VbEngineInternalTimeoutStop(t_TimeoutCnf *timer);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VbEngineDatamodelNodeDestroy(t_node *node)
{
  if (node != NULL)
  {
    VbDatamodelNodeMeasuresDestroy(&(node->measures));
  }
}

/*******************************************************************/

static void VbEngineDatamodelDomainDestroy(t_domain *domain)
{
  if (domain != NULL)
  {
    VbDatamodelListEpsDestroy(&(domain->eps));
    VbEngineDatamodelNodeDestroy(&(domain->dm));
  }
  else
  {
    VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Unable to release NULL domain");
  }
}

/*******************************************************************/

static void VbDatamodelNodePsdDestroy( t_PSD *psd )
{
  if (psd != NULL)
  {
    if (psd->PSDStepsList.PSD != NULL)
    {
      free(psd->PSDStepsList.PSD);
      psd->PSDStepsList.PSD = NULL;
    }
    psd->PSDStepsList.NumPSDs = 0;

    if (psd->MSBsList.MSB != NULL)
    {
      free(psd->MSBsList.MSB);
      psd->MSBsList.MSB = NULL;
    }
    psd->MSBsList.NumMSBs = 0;
  }
}

/*******************************************************************/

static void VbDatamodelNodeMeasuresDestroy( t_nodeMeasures *nodeMeasures )
{
  VbDatamodelNodeProcessMeasureDestroy(&nodeMeasures->BGNMeasure);
  VbDatamodelNodeProcessMeasureDestroy(&nodeMeasures->SNRProbesMeasure);
  VbEngineDatamodelNodeCrossMeasureListDestroy(&nodeMeasures->CFRMeasureList);
  VbDatamodelNodeProcessMeasureDestroy(&nodeMeasures->snrFullXtalk);
  VbDatamodelNodeProcessMeasureDestroy(&nodeMeasures->snrLowXtalk);
  VbDatamodelNodePsdDestroy(&nodeMeasures->Psd);
}

/*******************************************************************/

static void VbDatamodelListEpsDestroy( t_endPointList *epsLists )
{
  INT8U i;

  if (epsLists != NULL)
  {
    if (epsLists->epsArray != NULL)
    {
      for (i = 0; i < epsLists->numEPs; i++)
      {
        VbEngineDatamodelNodeDestroy(&(epsLists->epsArray[i]));
      }

      free(epsLists->epsArray);
      epsLists->epsArray = NULL;
      epsLists->numEPs = 0;
    }
  }
}

/*******************************************************************/

static t_domain* VbEngineDatamodelDomainFind(INT8U *macDM, t_domainsList *domainsList)
{
  INT32U      i;
  t_domain   *domain_found = NULL;
  t_domain   *domain = NULL;

  if ((macDM != NULL) && (domainsList != NULL))
  {
    for (i = 0; i < domainsList->numDomains; i++)
    {
      domain = &(domainsList->domainsArray[i]);

      if (memcmp(domain->dm.MAC, macDM, ETH_ALEN) == 0)
      {
        domain_found = domain;
        break;
      }
    }
  }

  return domain_found;
}

/*******************************************************************/

static t_node* VbEngineDatamodelEpFind(INT8U *macEP, t_endPointList *epsList)
{
  INT32U      i;
  t_node     *ep_found = NULL;
  t_node     *ep = NULL;

  if ((macEP != NULL) && (epsList != NULL) && (epsList->epsArray != NULL))
  {
    for (i = 0; i < epsList->numEPs; i++)
    {
      ep = &(epsList->epsArray[i]);

      if (memcmp(ep->MAC, macEP, ETH_ALEN) == 0)
      {
        ep_found = ep;
        break;
      }
    }
  }

  return ep_found;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineBoostModeByMacSetLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_loopArgs            *loop_args = (t_loopArgs *)args;

  if (node == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (memcmp(loop_args->MAC, node->MAC, ETH_ALEN) == 0)
    {
      // Node found
      loop_args->found = TRUE;
      node->channelSettings.boostInfo.mode = loop_args->boostMode;

      // Return this error code to abort loop
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineSeedByMacSetLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_seedArgs            *loop_args = (t_seedArgs *)args;

  if (node == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (memcmp(loop_args->MAC, node->MAC, ETH_ALEN) == 0)
    {
      // Node found
      node->addInfo1.extSeed = loop_args->seedIdx;
      node->devID = loop_args->did;


      // Return this error code to abort loop
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineLineStatusUpdateLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if ((domain == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    domain->dm.state = *((t_vb_DevState *)args);
  }

  return ret;
}

/*******************************************************************/

static void VbEngineTimeOutCb(sigval_t sigval)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  t_TimeoutCnf         *timer = (t_TimeoutCnf *)sigval.sival_ptr;

  if (timer == NULL)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Timeout error");
  }

  if ((err == VB_ENGINE_ERROR_NONE) &&
      (VbEngineProcessThreadRunningGet() == TRUE))
  {
    pthread_mutex_lock(&vbEngineTimersMutex);

    if (timer->periodic == FALSE)
    {
      // Release the timer
      VbEngineInternalTimeoutStop(timer);
    }

    if (timer->driver == NULL)
    {
      if(timer->clusterCast.numCLuster == 0)
      {
        // Send timeout event to engine to all drivers
        err = VbEngineProcessAllDriversEvSend(timer->event, NULL);
      }
      else
      {
        // Send timeout event to engine to a specific cluster
        err = VbEngineProcessClusterXDriversEvSend(timer->event, NULL, timer->clusterCast.list[0]);
      }

      if (err != VB_ENGINE_ERROR_NONE)
      {
        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d while sending event %s to engine",
            err, FSMEvToStrGet(timer->event));
      }
    }
    else
    {
      // Send timeout event to driver
      err = VbEngineProcessEvSend(timer->driver, timer->event, NULL);

      if (err != VB_ENGINE_ERROR_NONE)
      {
        VbLogPrintExt(VB_LOG_ERROR, timer->driver->vbDriverID, "Error %d while sending event %s",
                   err, FSMEvToStrGet(timer->event));
      }
    }

    pthread_mutex_unlock(&vbEngineTimersMutex);
  }
}

/*******************************************************************/

static t_VB_engineErrorCode NumNodesLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_vbEngineNumNodes    *count_nodes = (t_vbEngineNumNodes *)args;

  if (domain == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    count_nodes->numDms++;
    if (VbEngineDatamodelDomainIsComplete(domain) == TRUE)
    {
      count_nodes->numEps++;
      count_nodes->numCompleteLines++;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode NodeMACListInsert(t_node *node, t_nodesMacList *macList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((node == NULL) || (macList == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT8U *ptr;

    if (node->type == VB_NODE_DOMAIN_MASTER)
    {
      if (macList->dmsMacs.ptr == NULL)
      {
        ret = VB_ENGINE_ERROR_MALLOC;
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {

        ptr = macList->dmsMacs.ptr + (macList->dmsMacs.numNodes * ETH_ALEN);
        MACAddrClone(ptr, node->MAC);
        macList->dmsMacs.numNodes++;
      }
    }
    else
    {
      if (macList->epsMacs.ptr == NULL)
      {
        ret = VB_ENGINE_ERROR_MALLOC;
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        ptr = macList->epsMacs.ptr + (macList->epsMacs.numNodes * ETH_ALEN);
        MACAddrClone(ptr, node->MAC);
        macList->epsMacs.numNodes++;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode NodesMACListCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_nodesMacList      *mac_list =  (t_nodesMacList *)args;

  if ((node == NULL) || (mac_list == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Insert MAC in list
    ret = NodeMACListInsert(node, mac_list);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode CompleteLinesMACListCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_nodesMacList      *mac_list =  (t_nodesMacList *)args;

  if ((node == NULL) || (domain == NULL) || (mac_list == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineDatamodelDomainIsComplete(domain) == TRUE)
    {
      // Insert MAC in list
      ret = NodeMACListInsert(node, mac_list);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode DomainByMacSearchLoopCb(t_VBDriver *driver, t_domain *domain, void *args)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;
  t_domainByMacSearchParams *search_params = (t_domainByMacSearchParams *)args;

  if ((domain == NULL) || (search_params == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (MACAddrQuickCmp(domain->dm.MAC, search_params->MAC) == TRUE)
    {
      // Node found
      search_params->domain = domain;
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineInternalTimeoutStop(t_TimeoutCnf *timer)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  INT32S                 err;

  if (timer == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (timer->running == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_STARTED;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      err = TimerTaskDelete(timer->id, timer->name);

      if (err != 0)
      {
        if (timer->driver == NULL)
        {
          VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d stopping timer %s", err, timer->name);
        }
        else
        {
          VbLogPrintExt(VB_LOG_ERROR, timer->driver->vbDriverID, "Error %d stopping timer %s", err, timer->name);
        }

        ret = VB_ENGINE_ERROR_TIMERS;
      }
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      timer->id = 0;
      timer->running = FALSE;
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

t_VB_engineErrorCode VbEngineDatamodelCreateCluster(t_VBCluster **vbCluster)
{
  t_VB_engineErrorCode vb_engine_error_code = VB_ENGINE_ERROR_NONE;
  t_VBCluster *new_cluster;

  if(vbCluster == NULL)
  {
    vb_engine_error_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_engine_error_code == VB_ENGINE_ERROR_NONE)
  {
    new_cluster = (t_VBCluster*)calloc(1, sizeof(t_VBCluster));

    if(new_cluster == NULL)
    {
      vb_engine_error_code = VB_ENGINE_ERROR_MALLOC;
    }
    else
    {
      new_cluster->cdtaData = NULL;
      new_cluster->measurePlanData = NULL;
      new_cluster->confReqBuffer = NULL;
      new_cluster->confReqBufferLen = 0;
      new_cluster->clusterInfo.clusterId = 0;
      new_cluster->clusterInfo.numLines = 0;
      new_cluster->clusterInfo.alignRef = 0;
      new_cluster->timeoutCnf.driver = NULL;

      *vbCluster = new_cluster;
    }
  }

  return vb_engine_error_code;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClusterDel(t_VBCluster **vbCluster)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((vbCluster == NULL) || (*vbCluster == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelClusterMemFree(*vbCluster);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Release the cluster itself
    free(*vbCluster);
    *vbCluster = NULL;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClusterMemFree(t_VBCluster *vbCluster)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (vbCluster == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if(vbCluster->cdtaData != NULL)
    {
      free(vbCluster->cdtaData);
      vbCluster->cdtaData = NULL;
    }

    if(vbCluster->measurePlanData != NULL)
    {
      t_reqMeasurement       *measurement_curr_meas = (t_reqMeasurement *)vbCluster->measurePlanData;

      // Release previous memory
      if(measurement_curr_meas->measurePlan.msg != NULL)
      {
        free(measurement_curr_meas->measurePlan.msg);
        measurement_curr_meas->measurePlan.msg = NULL;
      }

      free(vbCluster->measurePlanData);
      vbCluster->measurePlanData = NULL;
    }

    if(vbCluster->confReqBuffer != NULL)
    {
      free(vbCluster->confReqBuffer);
      vbCluster->confReqBuffer = NULL;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbEngineTimeoutStop(&(vbCluster->timeoutCnf));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelCreateDriver(CHAR *driverId, t_VBDriver **vbDriver)
{
  t_VB_engineErrorCode vb_engine_error_code = VB_ENGINE_ERROR_NONE;
  t_VBDriver *new_driver;

  if(vbDriver == NULL)
  {
    vb_engine_error_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_engine_error_code == VB_ENGINE_ERROR_NONE)
  {
    new_driver = (t_VBDriver*)calloc(1, sizeof(t_VBDriver));

    if(new_driver == NULL)
    {
      vb_engine_error_code = VB_ENGINE_ERROR_MALLOC;
    }
    else
    {
      new_driver->FSMState = ENGINE_STT_DISCONNECTED;
      new_driver->domainsList.numDomains = 0;
      new_driver->domainsList.domainsArray = NULL;
      new_driver->clusterId = 0;
      pthread_mutex_init(&(new_driver->domainsMutex), NULL);
      pthread_mutex_init(&(new_driver->time.mutex), NULL);
      new_driver->timeoutCnf.driver = new_driver;
      new_driver->timeoutCnf.clusterCast.numCLuster = 0;
      if (driverId != NULL)
      {
        vb_engine_error_code = VbEngineDatamodelDriverIdSet(driverId, new_driver, FALSE);
      }

      *vbDriver = new_driver;
    }
  }

  return vb_engine_error_code;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelDriverIdSet(CHAR *driverId, t_VBDriver *driver, BOOLEAN definitiveDriverId)
{
  t_VB_engineErrorCode error= VB_ENGINE_ERROR_NONE;

  if(driver == NULL || driverId == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    if(strlen(driverId) == 0)
    {
      error = VB_ENGINE_ERROR_PARAMS;
    }
    else
    {
      strncpy(driver->vbDriverID, driverId, VB_EA_DRIVER_ID_MAX_SIZE);

      // Force the null byte in last position
      driver->vbDriverID[VB_EA_DRIVER_ID_MAX_SIZE - 1] = '\0';

      // Update version file name
      snprintf(driver->versionFileName, VB_ENGINE_MAX_FILE_NAME_SIZE, "%s_%s", VB_DRIVER_VERSION_FILE, (char *)driver->vbDriverID);
      // Force the null byte in last position
      driver->versionFileName[VB_ENGINE_MAX_FILE_NAME_SIZE - 1] = '\0';

      // Update FSM state file name
      snprintf(driver->fsmStateFileName, VB_ENGINE_MAX_FILE_NAME_SIZE, "%s_%s", VB_ENGINE_STATE_FILE, (char *)driver->vbDriverID);
      // Force the null byte in last position
      driver->fsmStateFileName[VB_ENGINE_MAX_FILE_NAME_SIZE - 1] = '\0';

      // Update remote state file name
      snprintf(driver->remoteStateFileName, VB_ENGINE_MAX_FILE_NAME_SIZE, "%s_%s", VB_DRIVER_STATE_FILE, (char *)driver->vbDriverID);
      // Force the null byte in last position
      driver->remoteStateFileName[VB_ENGINE_MAX_FILE_NAME_SIZE - 1] = '\0';

      driver->definitiveDriverId = definitiveDriverId;
    }
  }

  return error;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelDriverMemRelease(t_VBDriver *vbDriver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (vbDriver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbEngineDatamodelListDomainsDestroy(vbDriver);

    // Release pending timers
    VbEngineDriverTimeoutStop(vbDriver);

    pthread_mutex_destroy(&(vbDriver->time.mutex));
    pthread_mutex_destroy(&(vbDriver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelDriverDel(t_VBDriver **vbDriver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((vbDriver == NULL) || (*vbDriver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelDriverMemRelease(*vbDriver);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Release the driver itself
    free(*vbDriver);
    *vbDriver = NULL;
  }

  return ret;
}

/*******************************************************************/

BOOLEAN VbEngineDatamodelNodeIsReadyToBoost(t_node *node)
{
  BOOLEAN is_ready = FALSE;

  if (node != NULL)
  {
    is_ready = (node->channelSettings.boostInfo.valid == TRUE) &&
               (node->trafficReports.reportsReceived == TRUE);
  }

  return is_ready;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDataModelChannelCapacityFirstCarrierSet(t_node *node, INT16U firstCarrier)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;

  if (node != NULL)
  {
    node->channelSettings.firstValidCarrier = firstCarrier;
  }
  else
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  return result;
}

/*******************************************************************/

void VbEngineMainQEvSend(t_VB_Process_Event event, mqd_t vbQueue, t_VBDriver *thisDriver)
{
  t_VBMainMsg vb_msg;

  vb_msg.vbEvent = event;
  vb_msg.thisDriver = thisDriver;

  if (0 != (mq_send(vbQueue, ((const char *)(&vb_msg)), sizeof(t_VBMainMsg), VB_THREADMSG_PRIORITY)))
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error posting msg to Main queue. errno %s", strerror(errno));
  }
}

/*******************************************************************/

void VbDatamodelNodeProcessMeasureDestroy( t_processMeasure *measure )
{

  if (measure != NULL)
  {
    if (measure->measuresRx1 != NULL)
    {
      free(measure->measuresRx1);
      measure->measuresRx1 = NULL;
    }

    if (measure->measuresRx2 != NULL)
    {
      free(measure->measuresRx2);
      measure->measuresRx2 = NULL;
    }
  }
}

/*******************************************************************/

void VbEngineDatamodelListDmDestroy(t_VBDriver *driver, t_domainsList *domainsList)
{
  INT16U actual_domain_pointer;
  t_domain *this_domain;

  if (domainsList != NULL)
  {
    if (driver != NULL)
    {
      VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Clean domain list");
    }

    if (domainsList->domainsArray != NULL)
    {
      for (actual_domain_pointer = 0 ; actual_domain_pointer < domainsList->numDomains ; actual_domain_pointer++)
      {
        this_domain = (t_domain *)(&(domainsList->domainsArray[actual_domain_pointer]));
        VbEngineDatamodelDomainDestroy(this_domain);
      }

      free(domainsList->domainsArray);
      domainsList->domainsArray = NULL;
    }
  }
}

/*******************************************************************/

void VbEngineDatamodelListDomainsDestroy( t_VBDriver *thisDriver )
{
  if (thisDriver != NULL)
  {
    pthread_mutex_lock(&(thisDriver->domainsMutex));

    VbEngineDatamodelListDmDestroy(thisDriver, &(thisDriver->domainsList));

    pthread_mutex_unlock(&(thisDriver->domainsMutex));
  }
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelListDomainsLostCheck(t_VBDriver *driver, t_domainsList *newDomainsList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               old_domain_index;
  t_domain            *old_domain;
  t_domain            *new_domain;

  if ((driver == NULL) || (newDomainsList == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Check lost lines");

    pthread_mutex_lock(&(driver->domainsMutex));

    if ((newDomainsList->domainsArray != NULL) && (newDomainsList->numDomains > 0) &&
        (driver->domainsList.domainsArray != NULL) && (driver->domainsList.numDomains > 0))
    {
      // New domains discovered and old domains list is not empty

      for (old_domain_index = 0; old_domain_index < driver->domainsList.numDomains; old_domain_index++)
      {
        old_domain = (t_domain *)(&(driver->domainsList.domainsArray[old_domain_index]));

        new_domain = VbEngineDatamodelDomainFind(old_domain->dm.MAC, newDomainsList);

        if (new_domain == NULL)
        {
          // Old domain not found in new list
          if (VbEngineDatamodelDomainIsComplete(old_domain) == TRUE)
          {
            // Complete line not found in new list

            VbCounterIncrease(VB_ENGINE_COUNTER_LINE_LOST);

            VbMetricsReportDeviceEvent(VB_METRICS_EVENT_LINE_LOST, old_domain->dm.MAC, TRUE, driver, 0, 0);

            VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : line lost (DM and EP not found)",
                old_domain->dm.MACStr,
                old_domain->eps.epsArray[0].MACStr);
          }
        }
        else
        {
          // Old domain found in new list
          if ((VbEngineDatamodelDomainIsComplete(old_domain) == TRUE) &&
              (VbEngineDatamodelDomainIsComplete(new_domain) == FALSE))
          {
            // Complete line not found in new list

            VbCounterIncrease(VB_ENGINE_COUNTER_LINE_LOST);

            VbMetricsReportDeviceEvent(VB_METRICS_EVENT_LINE_LOST, old_domain->dm.MAC, TRUE, driver, 0, 0);

            VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : line lost (EP not found)",
                old_domain->dm.MACStr,
                old_domain->eps.epsArray[0].MACStr);
          }
        }
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelListDomainsRecoverData( t_VBDriver *driver,  t_domainsList *newDomainsList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U    new_domain_index;
  t_domain *this_old_domain;
  t_domain *this_new_domain;
  t_node   *ep_old = NULL;

  if ((driver == NULL) || (newDomainsList == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Recover data from this driver domain list");

    pthread_mutex_lock(&(driver->domainsMutex));

    if ((newDomainsList->domainsArray != NULL) && (newDomainsList->numDomains > 0))
    {
      // New domains discovered

      if ((driver->domainsList.domainsArray != NULL) && (driver->domainsList.numDomains > 0))
      {
        // Old domains list is not empty

        for (new_domain_index = 0; new_domain_index < newDomainsList->numDomains; new_domain_index++)
        {
          this_new_domain = (t_domain *)(&(newDomainsList->domainsArray[new_domain_index]));

          this_old_domain = VbEngineDatamodelDomainFind(this_new_domain->dm.MAC, &(driver->domainsList));

          if (this_old_domain != NULL)
          {
            // Domain found in previous domains list

            if (VbEngineDatamodelDomainIsComplete(this_new_domain) == TRUE)
            {
              // Search for new EP in old domain
              ep_old = VbEngineDatamodelEpFind(this_new_domain->eps.epsArray[0].MAC, &(this_old_domain->eps));

              if (ep_old != NULL)
              {
                // Ep found in previous domains list

                // Release previously allocated memory
                VbDatamodelListEpsDestroy(&(this_new_domain->eps));

                // Copy info from old domains list
                this_new_domain->eps = this_old_domain->eps;

                /*
                 * Clear node info from old domains list.
                 * Do not release the allocated memory, as this node is now present in the new domains list.
                 */
                memset(&(this_old_domain->eps), 0, sizeof(this_old_domain->eps));
              }
            }

            // Remove previously allocated memory
            VbEngineDatamodelNodeDestroy(&(this_new_domain->dm));

            // Copy old domain info to new domains list
            this_new_domain->dm = this_old_domain->dm;

            // Copy align info to new domains list
            this_new_domain->alignInfo = this_old_domain->alignInfo;

            // Update linkedNode ptr
            if (VbEngineDatamodelDomainIsComplete(this_new_domain) == TRUE)
            {
              this_new_domain->eps.epsArray[0].linkedNode = &(this_new_domain->dm);
              this_new_domain->dm.linkedNode = &(this_new_domain->eps.epsArray[0]);
            }
            else
            {
              this_new_domain->dm.linkedNode = NULL;
            }

            /*
             * Clear node info from old domains list.
             * Do not release the allocated memory, as this node is now present in the new domains list.
             */
            memset(&(this_old_domain->dm), 0, sizeof(this_old_domain->dm));

            // Update device state
            if (VbEngineDatamodelDomainIsComplete(this_new_domain) == TRUE)
            {
              if (ep_old != NULL)
              {
                newDomainsList->domainsArray[new_domain_index].dm.state = VB_DEV_PRESENT;
                newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].state = VB_DEV_PRESENT;

                VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : already present",
                    newDomainsList->domainsArray[new_domain_index].dm.MACStr,
                    newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].MACStr);
              }
              else
              {
                VbCounterIncrease(VB_ENGINE_COUNTER_NEW_LINE);

#if VB_ENGINE_METRICS_ENABLED
                VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                    this_new_domain->dm.MAC, TRUE, driver, 0, 0);
                VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                    this_new_domain->eps.epsArray[0].MAC, FALSE, driver, 0, 0);
#endif

                newDomainsList->domainsArray[new_domain_index].dm.state = VB_DEV_PRESENT;
                newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].state = VB_DEV_NEW;

                VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : new EP",
                    newDomainsList->domainsArray[new_domain_index].dm.MACStr,
                    newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].MACStr);
              }
            }
            else
            {
              newDomainsList->domainsArray[new_domain_index].dm.state = VB_DEV_PRESENT;

#if VB_ENGINE_METRICS_ENABLED
              VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                  this_new_domain->dm.MAC, TRUE, driver, 0, 0);
#endif

              VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : DM already present",
                  newDomainsList->domainsArray[new_domain_index].dm.MACStr, "--:--:--:--:--:--");
            }
          }
          else
          {
            if (VbEngineDatamodelDomainIsComplete(this_new_domain) == TRUE)
            {
              VbCounterIncrease(VB_ENGINE_COUNTER_NEW_LINE);

#if VB_ENGINE_METRICS_ENABLED
              VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                  this_new_domain->dm.MAC, TRUE, driver, 0, 0);
              VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                  this_new_domain->eps.epsArray[0].MAC, FALSE, driver, 0, 0);
#endif

              newDomainsList->domainsArray[new_domain_index].dm.state = VB_DEV_NEW;
              newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].state = VB_DEV_NEW;

              VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : new line",
                  newDomainsList->domainsArray[new_domain_index].dm.MACStr,
                  newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].MACStr);
            }
            else
            {
              newDomainsList->domainsArray[new_domain_index].dm.state = VB_DEV_NEW;

              VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : new DM",
                  newDomainsList->domainsArray[new_domain_index].dm.MACStr, "--:--:--:--:--:--");
            }
          }
        }
      }
      else
      {
        // Old domains list is empty

        for (new_domain_index = 0 ; new_domain_index < newDomainsList->numDomains ; new_domain_index++)
        {
          if (VbEngineDatamodelDomainIsComplete(&(newDomainsList->domainsArray[new_domain_index])) == TRUE)
          {
            VbCounterIncrease(VB_ENGINE_COUNTER_NEW_LINE);

#if VB_ENGINE_METRICS_ENABLED
            VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                newDomainsList->domainsArray[new_domain_index].dm.MAC,
                TRUE, driver, 0, 0);
            VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].MAC, FALSE, driver, 0, 0);
#endif

            newDomainsList->domainsArray[new_domain_index].dm.state = VB_DEV_NEW;
            newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].state = VB_DEV_NEW;

            VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : new line",
                newDomainsList->domainsArray[new_domain_index].dm.MACStr,
                newDomainsList->domainsArray[new_domain_index].eps.epsArray[0].MACStr);
          }
          else
          {
            newDomainsList->domainsArray[new_domain_index].dm.state = VB_DEV_NEW;

            VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain DM %18s <-> EP %18s : new DM",
                newDomainsList->domainsArray[new_domain_index].dm.MACStr, "--:--:--:--:--:--");
          }
        }
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelNodeFind(t_VBDriver *driver, const INT8U *mac, t_node **node)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_domainsList       *domains_list = NULL;
  INT32U               domain_idx = 0;
  INT32U               ep_idx = 0;
  t_node              *ep = NULL;
  t_domain            *domain = NULL;

  if ((driver == NULL) || (node == NULL) || (mac == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VB_ENGINE_ERROR_NOT_FOUND;

    domains_list = &(driver->domainsList);

    for (domain_idx = 0; domain_idx < domains_list->numDomains; domain_idx++)
    {
      domain = &(domains_list->domainsArray[domain_idx]);

      if(domain != NULL)
      {
        if (memcmp(domain->dm.MAC, mac, ETH_ALEN) == 0)
        {
          // Node found
          *node = &(domain->dm);

          ret = VB_ENGINE_ERROR_NONE;
        }
        else
        {
          for (ep_idx = 0; ep_idx < domain->eps.numEPs; ep_idx++)
          {
            ep = &(domain->eps.epsArray[ep_idx]);

            if (memcmp(ep->MAC, mac, ETH_ALEN) == 0)
            {
              // Node found
              *node = ep;

              ret = VB_ENGINE_ERROR_NONE;
              break;
            }
          }
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Node was found
        break;
      }
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineBoostModeByMacSet(INT8U *nodeMac, t_vbEngineBoostMode boostMode)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_loopArgs             loop_args;

  if ((nodeMac == NULL) ||
      (boostMode >= VB_ENGINE_BOOST_MODE_LAST))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args.MAC = nodeMac;
    loop_args.boostMode = boostMode;
    loop_args.found = FALSE;
    ret = VbEngineDatamodelAllNodesLoop(VbEngineBoostModeByMacSetLoopCb, &loop_args);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (loop_args.found == FALSE)
    {
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineReqCapacityRatioCalc(INT16U needed, INT16U reference, INT32U *reqCap)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if ( (reqCap == NULL) || (reference == 0) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *reqCap = IN_PERC(needed, reference);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDriverTimeoutStart(t_VBDriver *driver, t_VB_Comm_Event eventId, INT32U timeoutMs, CHAR *name)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (eventId >= ENGINE_EV_LAST) || (timeoutMs == 0) || (name == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineTimeoutStart(&(driver->timeoutCnf), eventId, timeoutMs, FALSE, name);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDriverTimeoutStop(t_VBDriver *driver)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineTimeoutStop(&(driver->timeoutCnf));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineTimeoutStart(t_TimeoutCnf *timer, t_VB_Comm_Event eventId, INT32U timeoutMs, BOOLEAN periodic, CHAR *name)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32S               err;

  if ((timer == NULL) || (eventId >= ENGINE_EV_LAST) || (timeoutMs == 0) || (name == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&vbEngineTimersMutex);

    if (timer->running == TRUE)
    {
      if (timer->driver == NULL)
      {
        VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "Warning: timer %s already enabled with event %s; new event %s",
            timer->name, FSMEvToStrGet(timer->event), FSMEvToStrGet(eventId));
      }
      else
      {
        VbLogPrintExt(VB_LOG_WARNING, timer->driver->vbDriverID, "Warning: timer %s already enabled with event %s; new event %s",
            timer->name, FSMEvToStrGet(timer->event), FSMEvToStrGet(eventId));
      }

      ret = VB_ENGINE_ERROR_ALREADY_STARTED;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      timer->event = eventId;
      timer->name = name;
      timer->periodic = periodic;

      if (timer->periodic == FALSE)
      {
        err = TimerOneShotTaskSet(timer->name, timeoutMs,
            VbEngineTimeOutCb, timer, &(timer->id));
      }
      else
      {
        err = TimerPeriodicTaskSet(timer->name, timeoutMs,
            VbEngineTimeOutCb, timer, &(timer->id));
      }

      if (err == -1)
      {
        if (timer->driver == NULL)
        {
          VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error starting timeout timer %s (event %s)",
              timer->name, FSMEvToStrGet(timer->event));
        }
        else
        {
          VbLogPrintExt(VB_LOG_ERROR, timer->driver->vbDriverID, "Error starting timeout timer %s (event %s)",
                      timer->name, FSMEvToStrGet(timer->event));
        }

        ret = VB_ENGINE_ERROR_TIMERS;
      }
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      timer->running = TRUE;
    }

    pthread_mutex_unlock(&vbEngineTimersMutex);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Configured timer: %s (timeout %u; event %s; periodic %u)",
        timer->name, timeoutMs, FSMEvToStrGet(timer->event), timer->periodic);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineTimeoutStop(t_TimeoutCnf *timer)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (timer == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&vbEngineTimersMutex);

    ret = VbEngineInternalTimeoutStop(timer);

    pthread_mutex_unlock(&vbEngineTimersMutex);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelAlignInfoSet(t_VBDriver *driver, INT8U *mac, t_alignInfo *alignInfo)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (mac == NULL) || (alignInfo == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_domain *target_domain = NULL;

    pthread_mutex_lock(&(driver->domainsMutex));

    // Search given domain
    target_domain = VbEngineDatamodelDomainFind(mac, &(driver->domainsList));

    if (target_domain == NULL)
    {
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      INT32U idx;
      t_alignInfo prev_info = target_domain->alignInfo;

      // Copy new info
      target_domain->alignInfo = *alignInfo;

      // Maintain previous value of "allowed" field
      for (idx = 0; idx < VB_ALIGN_GHN_MAX_TX_NODES; idx++)
      {
        // If detected device Id is the same, maintain previous "allowed" value
        if (prev_info.syncDetsInfo[idx].detDid == alignInfo->syncDetsInfo[idx].detDid)
        {
          target_domain->alignInfo.syncDetsInfo[idx].allowed = prev_info.syncDetsInfo[idx].allowed;
        }
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Align query info from %s : macClock %u; seqNum %u;",
          target_domain->dm.MACStr, alignInfo->macClock, alignInfo->seqNum);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDataModelNumNodesGet(t_vbEngineNumNodes *numNodes)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (numNodes == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    bzero(numNodes, sizeof(*numNodes));

    ret = VbEngineDatamodelAllDomainsLoop(NumNodesLoopCb, numNodes);
  }

  return ret;
}


/*******************************************************************/

t_VB_engineErrorCode VbEngineDataModelNumNodesInClusterXGet(INT32U clusterId, t_vbEngineNumNodes *numNodes)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (numNodes == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    bzero(numNodes, sizeof(*numNodes));

    ret = VbEngineDatamodelClusterXAllDomainsLoop(NumNodesLoopCb, clusterId, numNodes);
  }

  return ret;
}


/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelNumNodesInDriverGet(t_VBDriver *driver, t_vbEngineNumNodes *numNodes)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (numNodes == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    bzero(numNodes, sizeof(*numNodes));

    // Loop through domains in driver and count complete lines
    ret = VbEngineDatamodelDomainsLoop(driver, NumNodesLoopCb, numNodes);
  }

  return ret;
}

/*******************************************************************/

BOOLEAN VbEngineDatamodelDomainIsComplete(t_domain *domain)
{
  BOOLEAN ret = FALSE;

  if ((domain != NULL) && (domain->eps.epsArray != NULL) && (domain->eps.numEPs > 0))
  {
    ret = TRUE;
  }

  return ret;
}

/*******************************************************************/

BOOLEAN VbEngineDatamodelDomainIsNew(t_domain *domain)
{
  BOOLEAN ret = TRUE;

  if ((VbEngineDatamodelDomainIsComplete(domain) == TRUE) &&
      (domain->dm.state == VB_DEV_PRESENT))
  {
    ret = FALSE;
  }

  return ret;
}

/*******************************************************************/

INT32U VbEngineDatamodelNumDomainsByDriverGet(t_VBDriver *driver)
{
  INT32U num_domains = 0;

  if (driver != NULL)
  {
    pthread_mutex_lock(&(driver->domainsMutex));

    num_domains = driver->domainsList.numDomains;

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return num_domains;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelNodesMacGet(t_VBDriver *driver, t_nodesMacList *macList, BOOLEAN onlyCompleteLines)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               num_domains;
  INT8U               *dm_macs = NULL;
  INT8U               *ep_macs = NULL;

  if (macList == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    // Init fields
    macList->dmsMacs.numNodes = 0;
    macList->epsMacs.numNodes = 0;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get number of nodes
    num_domains = VbEngineDatamodelNumDomainsByDriverGet(driver);

    // Allocate memory for DM MACs
    dm_macs = (INT8U*)calloc(num_domains, ETH_ALEN);

    if (dm_macs == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for EP MACs
    ep_macs = (INT8U*)calloc(num_domains, ETH_ALEN);

    if (ep_macs == NULL)
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init fields
    macList->dmsMacs.ptr = dm_macs;
    macList->epsMacs.ptr = ep_macs;

    if (onlyCompleteLines == TRUE)
    {
      // Loop through complete lines and get MACs
      ret = VbEngineDatamodelNodesLoop(driver, CompleteLinesMACListCb, (void *)macList);
    }
    else
    {
      // Loop through all nodes in given driver and get MACs
      ret = VbEngineDatamodelNodesLoop(driver, NodesMACListCb, (void *)macList);
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    if (dm_macs != NULL)
    {
      free(dm_macs);
    }

    if (ep_macs != NULL)
    {
      free(ep_macs);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelAllNodesMacGet(t_nodesMacList *macList, BOOLEAN onlyCompleteLines)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEngineNumNodes   num_nodes;
  INT8U               *dm_macs = NULL;
  INT8U               *ep_macs = NULL;

  bzero(&num_nodes, sizeof(num_nodes));

  if (macList == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    // Init fields
    macList->dmsMacs.numNodes = 0;
    macList->epsMacs.numNodes = 0;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get number of nodes
    ret = VbEngineDataModelNumNodesGet(&num_nodes);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for DM MACs
    if (onlyCompleteLines == TRUE)
    {
      if (num_nodes.numCompleteLines > 0)
      {
        dm_macs = (INT8U*)calloc(num_nodes.numCompleteLines, ETH_ALEN);
        if (dm_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
    else
    {
      if (num_nodes.numDms > 0)
      {
        dm_macs = (INT8U*)calloc(num_nodes.numDms, ETH_ALEN);
        if (dm_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for EP MACs
    if (onlyCompleteLines == TRUE)
    {
      if (num_nodes.numCompleteLines > 0)
      {
        ep_macs = (INT8U*)calloc(num_nodes.numCompleteLines, ETH_ALEN);
        if (ep_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
    else
    {
      if (num_nodes.numEps > 0)
      {
        ep_macs = (INT8U*)calloc(num_nodes.numEps, ETH_ALEN);
        if (ep_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init fields
    macList->dmsMacs.ptr = dm_macs;
    macList->epsMacs.ptr = ep_macs;

    if ((macList->dmsMacs.ptr != NULL) || (macList->epsMacs.ptr != NULL))
    {
      if (onlyCompleteLines == TRUE)
      {
        // Loop through complete lines and get MACs
        ret = VbEngineDatamodelAllNodesLoop(CompleteLinesMACListCb, (void *)macList);
      }
      else
      {
        // Loop through all nodes and get MACs
        ret = VbEngineDatamodelAllNodesLoop(NodesMACListCb, (void *)macList);
      }
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    if (dm_macs != NULL)
    {
      free(dm_macs);
    }

    if (ep_macs != NULL)
    {
      free(ep_macs);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClusterXAllNodesMacGet(t_nodesMacList *macList, BOOLEAN onlyCompleteLines, INT32U clusterId)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEngineNumNodes   num_nodes;
  INT8U               *dm_macs = NULL;
  INT8U               *ep_macs = NULL;

  bzero(&num_nodes, sizeof(num_nodes));

  if (macList == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    // Init fields
    macList->dmsMacs.numNodes = 0;
    macList->epsMacs.numNodes = 0;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get number of nodes
    ret = VbEngineDataModelNumNodesInClusterXGet(clusterId, &num_nodes);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for DM MACs
    if (onlyCompleteLines == TRUE)
    {
      if (num_nodes.numCompleteLines > 0)
      {
        dm_macs = (INT8U*)calloc(num_nodes.numCompleteLines, ETH_ALEN);
        if (dm_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
    else
    {
      if (num_nodes.numDms > 0)
      {
        dm_macs = (INT8U*)calloc(num_nodes.numDms, ETH_ALEN);
        if (dm_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Allocate memory for EP MACs
    if (onlyCompleteLines == TRUE)
    {
      if (num_nodes.numCompleteLines > 0)
      {
        ep_macs = (INT8U*)calloc(num_nodes.numCompleteLines, ETH_ALEN);
        if (ep_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
    else
    {
      if (num_nodes.numEps > 0)
      {
        ep_macs = (INT8U*)calloc(num_nodes.numEps, ETH_ALEN);
        if (ep_macs == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Init fields
    macList->dmsMacs.ptr = dm_macs;
    macList->epsMacs.ptr = ep_macs;

    if ((macList->dmsMacs.ptr != NULL) || (macList->epsMacs.ptr != NULL))
    {
      if (onlyCompleteLines == TRUE)
      {
        // Loop through complete lines and get MACs
        ret = VbEngineDatamodelClusterXAllNodesLoop(CompleteLinesMACListCb, clusterId, (void *)macList);
      }
      else
      {
        // Loop through all nodes and get MACs
        ret = VbEngineDatamodelClusterXAllNodesLoop(NodesMACListCb, clusterId, (void *)macList);
      }
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    if (dm_macs != NULL)
    {
      free(dm_macs);
    }

    if (ep_macs != NULL)
    {
      free(ep_macs);
    }
  }

  return ret;
}

/*******************************************************************/

void VbEngineDatamodelMacListRelease(t_nodesMacList *macList)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

  if (macList == NULL)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    if (macList->dmsMacs.ptr != NULL)
    {
      free(macList->dmsMacs.ptr);
      macList->dmsMacs.ptr = NULL;
      macList->dmsMacs.numNodes = 0;
    }

    if (macList->epsMacs.ptr != NULL)
    {
      free(macList->epsMacs.ptr);
      macList->epsMacs.ptr = NULL;
      macList->epsMacs.numNodes = 0;
    }
  }
}

/*******************************************************************/

void VbEngineDatamodelNodeCrossMeasureListDestroy( t_crossMeasureList *crossMeasureList )
{
  INT16U numCrossMeasures;

  if (crossMeasureList != NULL )
  {
    if (crossMeasureList->crossMeasureArray != NULL)
    {
      for (numCrossMeasures = 0; numCrossMeasures < crossMeasureList->numCrossMeasures; numCrossMeasures++)
      {
        VbDatamodelNodeProcessMeasureDestroy(&(crossMeasureList->crossMeasureArray[numCrossMeasures].measure));
      }

      free(crossMeasureList->crossMeasureArray);
      crossMeasureList->crossMeasureArray = NULL;

      crossMeasureList->numCrossMeasures = 0;
    }
  }
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelListDomainsDMsAdd(t_VBDriver *driver, INT32U numAddedDms, t_vbEADomainDiffRspDMAdded *dmsInfo)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               new_dm_idx;
  INT32U               num_new_domains = 0;
  INT32U               num_old_domains = 0;
  t_domain            *new_dm_domain;
  INT8U               *list_of_domains_idx_to_be_added = NULL;

  if ((driver == NULL) || (dmsInfo == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {

    pthread_mutex_lock(&(driver->domainsMutex));

    list_of_domains_idx_to_be_added = malloc(numAddedDms);
    if(list_of_domains_idx_to_be_added != NULL)
    {
      num_old_domains = driver->domainsList.numDomains;
      for(new_dm_idx=0; new_dm_idx<numAddedDms; new_dm_idx++)
      {
        new_dm_domain = VbEngineDatamodelDomainFind(dmsInfo[new_dm_idx].dmMAC, &driver->domainsList);
        if (new_dm_domain == NULL)
        {
          // new DM Mac to be added
          list_of_domains_idx_to_be_added[num_new_domains] = new_dm_idx;
          num_new_domains++;
        }
      }

      if(num_new_domains > 0)
      {
        INT8U      *domain_ptr;
        INT32U      idx;
        t_domain   *new_domains = NULL;

        // Allocate current size + size for new domains
        domain_ptr = calloc(1, (driver->domainsList.numDomains+num_new_domains)*sizeof(t_domain));
        if(driver->domainsList.numDomains > 0)
        {
          memcpy(domain_ptr, driver->domainsList.domainsArray, driver->domainsList.numDomains*sizeof(t_domain));
        }

        new_domains = (t_domain *)(domain_ptr + driver->domainsList.numDomains*sizeof(t_domain));
        driver->domainsList.numDomains += num_new_domains;

        for(new_dm_idx=0; new_dm_idx<num_new_domains; new_dm_idx++)
        {
          idx = list_of_domains_idx_to_be_added[new_dm_idx];
          memcpy(new_domains[new_dm_idx].dm.MAC, dmsInfo[idx].dmMAC, ETH_ALEN);
          MACAddrMem2str(new_domains[new_dm_idx].dm.MACStr, dmsInfo[idx].dmMAC);
          new_domains[new_dm_idx].dm.devID            = dmsInfo[idx].dmDevId;
          new_domains[new_dm_idx].dm.type             = VB_NODE_DOMAIN_MASTER;
          new_domains[new_dm_idx].dm.linkedNode       = NULL;

          new_domains[new_dm_idx].dm.state            = VB_DEV_NEW;
          new_domains[new_dm_idx].dm.addInfo1.extSeed = dmsInfo[idx].extSeed;
          new_domains[new_dm_idx].dm.addInfo1.qosRate = dmsInfo[idx].qosRate;
          new_domains[new_dm_idx].dm.addInfo1.maxLengthTxop = dmsInfo[idx].maxLengthTxop;
          memcpy(new_domains[new_dm_idx].dm.addInfo1.fwVersion, dmsInfo[idx].fwVersion, VB_FW_VERSION_LENGTH);
          new_domains[new_dm_idx].dm.addInfo1.fwVersion[VB_FW_VERSION_LENGTH-1] = '\0';

          new_domains[new_dm_idx].eps.numEPs          = dmsInfo[idx].numEps;
          new_domains[new_dm_idx].eps.epsArray        = NULL;

          new_domains[new_dm_idx].dm.measures.BGNMeasure.measuresRx1           = NULL;
          new_domains[new_dm_idx].dm.measures.BGNMeasure.measuresRx2           = NULL;
          new_domains[new_dm_idx].dm.measures.SNRProbesMeasure.measuresRx1     = NULL;
          new_domains[new_dm_idx].dm.measures.SNRProbesMeasure.measuresRx2     = NULL;
          new_domains[new_dm_idx].dm.measures.CFRMeasureList.numCrossMeasures  = 0;
          new_domains[new_dm_idx].dm.measures.CFRMeasureList.crossMeasureArray = NULL;

          new_domains[new_dm_idx].dm.measures.Psd.MSBsList.NumMSBs      = 0;
          new_domains[new_dm_idx].dm.measures.Psd.MSBsList.MSB          = NULL;
          new_domains[new_dm_idx].dm.measures.Psd.PSDStepsList.NumPSDs  = 0;
          new_domains[new_dm_idx].dm.measures.Psd.PSDStepsList.PSD      = NULL;
          new_domains[new_dm_idx].dm.measures.snrFullXtalk.measuresRx1  = NULL;
          new_domains[new_dm_idx].dm.measures.snrFullXtalk.measuresRx2  = NULL;
          new_domains[new_dm_idx].dm.measures.snrLowXtalk.measuresRx1   = NULL;
          new_domains[new_dm_idx].dm.measures.snrLowXtalk.measuresRx2   = NULL;

          memset(&new_domains[new_dm_idx].dm.channelSettings.boostInfo, 0, sizeof(t_boostInfo));
          new_domains[new_dm_idx].dm.channelSettings.interferenceDetectionCounter = 0;

          memset(&new_domains[new_dm_idx].dm.cdtaInfo, 0, sizeof(t_nodeCdtaInfo));
          new_domains[new_dm_idx].dm.channelSettings.boostInfo.forcedBandsBitmap = (1 | (((VbEngineConfVdslCoexGet() == TRUE)?1:0)<<1));
          new_domains[new_dm_idx].dm.channelSettings.boostInfo.lastLevel = 1;

          VbEngineConfProfileGet(dmsInfo[idx].dmMAC,
                                 &(new_domains[new_dm_idx].dm.cdtaInfo.profile.userSLA),
                                 &(new_domains[new_dm_idx].dm.cdtaInfo.profile.slaWeight),
                                 &(new_domains[new_dm_idx].dm.cdtaInfo.profile.userWeight) );

          // Init node Align info
          memset(&(new_domains[new_dm_idx].dm.nodeAlignInfo), 0, sizeof(new_domains[new_dm_idx].dm.nodeAlignInfo));

#if VB_ENGINE_METRICS_ENABLED
          VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                                     new_domains[new_dm_idx].dm.MAC, TRUE, driver, 0, 0);
#endif
        }

        if(driver->domainsList.domainsArray != NULL)
        {
          free(driver->domainsList.domainsArray);
        }

        driver->domainsList.domainsArray = (t_domain*)domain_ptr;
      }

      if(list_of_domains_idx_to_be_added != NULL)
      {
        free(list_of_domains_idx_to_be_added);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        INT32U idx;
        for(idx =0; idx<num_old_domains; idx++)
        {
          t_domain   *domain = &(driver->domainsList.domainsArray[idx]);
          if(domain->eps.numEPs > 0)
          {
            domain->eps.epsArray[0].linkedNode = &(domain->dm);
          }
        }
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelListDomainsEPsAdd(t_VBDriver *driver, INT32U numAddedEps, t_vbEADomainDiffRspEPAdded *epsInfo)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               new_ep_idx;
  t_domain            *new_dm_domain;
  t_node              *node;

  if ((driver == NULL) || (epsInfo == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&(driver->domainsMutex));

    if(driver->domainsList.domainsArray != NULL)
    {
      for(new_ep_idx=0; new_ep_idx<numAddedEps; new_ep_idx++)
      {
        new_dm_domain = VbEngineDatamodelDomainFind(epsInfo[new_ep_idx].dmMAC, &driver->domainsList);
        if (new_dm_domain != NULL)
        {
          node = VbEngineDatamodelEpFind(epsInfo[new_ep_idx].epMAC, &new_dm_domain->eps);
          if(node == NULL)
          {
            node = calloc(1, sizeof(t_node));
            if(node != NULL)
            {
              if(new_dm_domain->eps.epsArray == NULL)
              {
                new_dm_domain->eps.numEPs = 1;
                new_dm_domain->eps.epsArray = node;
                memcpy(node->MAC, epsInfo[new_ep_idx].epMAC, ETH_ALEN);
                MACAddrMem2str(node->MACStr, epsInfo[new_ep_idx].epMAC);

                node->devID                                     = epsInfo[new_ep_idx].epDevId;
                node->state                                     = VB_DEV_NEW;
                node->type                                      = VB_NODE_END_POINT;
                node->linkedNode                                = &(new_dm_domain->dm);
                new_dm_domain->dm.linkedNode                    = node;
                node->measures.BGNMeasure.measuresRx1           = NULL;
                node->measures.BGNMeasure.measuresRx2           = NULL;
                node->measures.SNRProbesMeasure.measuresRx1     = NULL;
                node->measures.SNRProbesMeasure.measuresRx2     = NULL;
                node->measures.CFRMeasureList.numCrossMeasures  = 0;
                node->measures.CFRMeasureList.crossMeasureArray = NULL;

                node->measures.Psd.MSBsList.MSB         = NULL;
                node->measures.Psd.MSBsList.NumMSBs     = 0;
                node->measures.Psd.PSDStepsList.PSD     = NULL;
                node->measures.Psd.PSDStepsList.NumPSDs = 0;
                node->measures.snrFullXtalk.measuresRx1 = NULL;
                node->measures.snrFullXtalk.measuresRx2 = NULL;
                node->measures.snrLowXtalk.measuresRx1  = NULL;
                node->measures.snrLowXtalk.measuresRx2  = NULL;

                node->channelSettings.psdShape.numPSDBands = 0;
                node->channelSettings.psdShape.sendUpdate = FALSE;

                memset(&node->channelSettings.boostInfo, 0, sizeof(t_boostInfo));
                node->channelSettings.interferenceDetectionCounter = 0;

                memset(&node->cdtaInfo, 0, sizeof(t_nodeCdtaInfo));
                node->channelSettings.boostInfo.forcedBandsBitmap = (1 | (((VbEngineConfVdslCoexGet() == TRUE)?1:0)<<1));
                node->channelSettings.boostInfo.lastLevel = 1;

                memcpy(node->addInfo1.fwVersion, epsInfo[new_ep_idx].fwVersion, VB_FW_VERSION_LENGTH);
                node->addInfo1.fwVersion[VB_FW_VERSION_LENGTH-1] = '\0';

                VbEngineConfProfileGet(epsInfo[new_ep_idx].epMAC,
                                       &(node->cdtaInfo.profile.userSLA),
                                       &(node->cdtaInfo.profile.slaWeight),
                                       &(node->cdtaInfo.profile.userWeight) );

                // Init Traffic report
                memset(&node->trafficReports, 0, sizeof(t_trafficReport));

                // Init node Align info
                memset(&(node->nodeAlignInfo), 0, sizeof(node->nodeAlignInfo));

#if VB_ENGINE_METRICS_ENABLED
                VbMetricsReportDeviceEvent(VB_METRICS_EVENT_NEW_LINE,
                                           node->MAC, TRUE, driver, 0, 0);
#endif
              }
            }
            else
            {
              ret = VB_ENGINE_ERROR_MALLOC;
              break;
            }
          }
        }
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelListDomainsDMsRem(t_VBDriver *driver, INT32U numRemDms, INT8U *remMacdms, BOOL  *refOrRrelay)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               new_dm_idx;
  INT32U               num_rem_domains = 0;
  INT32U               i,j;
  INT16U               num_new_domain;
  t_domain            *domain;
  INT8U               *list_of_domains_idx_to_be_removed = NULL;
  INT8U               *new_domain_ptr;
  t_domain            *new_domains;
  BOOL                 do_not_copy;

  if ((driver == NULL) || (remMacdms == NULL) || (refOrRrelay == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&(driver->domainsMutex));

    list_of_domains_idx_to_be_removed = malloc(numRemDms);
    if(list_of_domains_idx_to_be_removed != NULL)
    {
      if(driver->domainsList.domainsArray != NULL)
      {
        for(new_dm_idx=0; new_dm_idx<numRemDms; new_dm_idx++)
        {
          for (i = 0; i < driver->domainsList.numDomains; i++)
          {
            domain = &(driver->domainsList.domainsArray[i]);
            if (memcmp(domain->dm.MAC, remMacdms, ETH_ALEN) == 0)
            {
              // DM to be removed found in current list

              if( (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_REF) ||
                  (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY_CANDIDATE) ||
                  (domain->dm.nodeAlignInfo.role == VB_ALIGN_ROLE_RELAY) )
              {
                *refOrRrelay = TRUE;
              }

              // Free related memory (Measures...)
              VbEngineDatamodelDomainDestroy(domain);
#if VB_ENGINE_METRICS_ENABLED
              VbMetricsReportDeviceEvent(VB_METRICS_EVENT_LINE_LOST, domain->dm.MAC, TRUE, driver, 0, 0);
#endif

              list_of_domains_idx_to_be_removed[num_rem_domains] = i;
              num_rem_domains++;
              break;
            }
          }

          remMacdms += ETH_ALEN;
        }

        if(numRemDms < driver->domainsList.numDomains)
        {
          num_new_domain = driver->domainsList.numDomains - numRemDms;
          new_domains = calloc(1, sizeof(t_domain)*(driver->domainsList.numDomains-numRemDms));
          if(new_domains != NULL)
          {
            new_domain_ptr = (INT8U *)new_domains;
            for (i = 0; i < driver->domainsList.numDomains; i++)
            {
              // Copy domains that are not going to be removed into new list
              do_not_copy = FALSE;
              for(j=0; j<num_rem_domains; j++)
              {
                if (list_of_domains_idx_to_be_removed[j] == i)
                {
                  // This one is going to be removed, do not copy
                  do_not_copy = TRUE;
                  break;
                }
              }

              if(do_not_copy == FALSE)
              {
                // This domain is not part of the removal list
                // Copy it into the new list
                domain = &(driver->domainsList.domainsArray[i]);

                memcpy(new_domain_ptr, (INT8U *)domain, sizeof(t_domain));
                new_domain_ptr += sizeof(t_domain);
              }
            }

            driver->domainsList.numDomains = num_new_domain;
            // Update pointer to new list
            driver->domainsList.domainsArray = new_domains;
          }
          else
          {
            ret = VB_ENGINE_ERROR_MALLOC;
          }
        }
        else
        {
          driver->domainsList.numDomains = 0;
          if(driver->domainsList.domainsArray != NULL)
          {
            VbEngineDatamodelListDmDestroy(driver, &driver->domainsList);
          }
          driver->domainsList.domainsArray = NULL;
        }
      }

      free(list_of_domains_idx_to_be_removed);
    }
    else
    {
      ret = VB_ENGINE_ERROR_MALLOC;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      INT32U idx;
      for(idx =0; idx<driver->domainsList.numDomains; idx++)
      {
        t_domain   *domain = &(driver->domainsList.domainsArray[idx]);
        if(domain->eps.numEPs > 0)
        {
          domain->eps.epsArray[0].linkedNode = &(domain->dm);
        }
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelListDomainsEPsRem(t_VBDriver *driver, INT32U numRemEps, INT8U *remMacEps)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               rem_ep_idx;
  INT32U               i;
  t_domain            *domain;

  if ((driver == NULL) || (remMacEps == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&(driver->domainsMutex));

    if(driver->domainsList.domainsArray != NULL)
    {
      for(rem_ep_idx=0; rem_ep_idx<numRemEps; rem_ep_idx++)
      {
        for (i = 0; i < driver->domainsList.numDomains; i++)
        {
          domain = &(driver->domainsList.domainsArray[i]);
          if(domain != NULL)
          {
            if ( (domain->eps.numEPs > 0) &&
                 (memcmp(domain->eps.epsArray[0].MAC, remMacEps, ETH_ALEN) == 0)
               )
            {
              // EP to be removed found in current list

#if VB_ENGINE_METRICS_ENABLED
              VbMetricsReportDeviceEvent(VB_METRICS_EVENT_LINE_LOST, domain->eps.epsArray[0].MAC, TRUE, driver, 0, 0);
#endif

              // Free related memory (Measures...)
              VbEngineDatamodelNodeDestroy(&domain->eps.epsArray[0]);
              free(domain->eps.epsArray);
              domain->eps.epsArray = NULL;
              domain->eps.numEPs = 0;
              domain->dm.linkedNode = NULL;

              break;
            }
          }
        }

        remMacEps +=ETH_ALEN;
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

t_vbEngineCountersIndex VbEngineDatamodelEvToCounter(t_VB_Comm_Event event)
{
  t_vbEngineCountersIndex counter = VB_ENGINE_COUNTER_EV_KILL_ALL;

  if (event >= ENGINE_EV_LAST)
  {
    VbLogPrint(VB_LOG_ERROR, "Invalid event to translate (%u)", event);
  }
  else
  {
    // Counters are located in same positions as defined in t_driverEvent
    counter = (t_vbEngineCountersIndex)event;
  }

  return counter;
}

/*******************************************************************/

const CHAR *VbEngineQosRateToStrGet(t_vbEngineQosRate rate)
{
  const CHAR *rate_str = "UNKNOWN";

  if (rate < VB_ENGINE_QOS_RATE_LAST)
  {
    rate_str = qosRateString[rate];
  }

  return rate_str;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineSeedIndexByMacSet(INT8U *nodeMac, INT16U seedIndex, INT16U did)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;
  t_seedArgs         loop_args;

  if (nodeMac == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args.MAC = nodeMac;
    loop_args.seedIdx = seedIndex;
    loop_args.did = did;
    ret = VbEngineDatamodelAllNodesLoop(VbEngineSeedByMacSetLoopCb, &loop_args);
  }

  return ret;
}


/*******************************************************************/

t_VB_engineErrorCode VbEngineLineStatusUpdate(INT32U clusterId, t_vb_DevState state)
{
  t_VB_engineErrorCode   ret = VB_ENGINE_ERROR_NONE;

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelClusterXAllDomainsLoop(VbEngineLineStatusUpdateLoopCb, clusterId,  &state);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClusterXDomainByMacGet(INT32U clusterId, INT8U *mac, t_domain **domain)
{
  t_VB_engineErrorCode       ret = VB_ENGINE_ERROR_NONE;
  t_domainByMacSearchParams  search_params;

  if ((mac == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    MACAddrClone(search_params.MAC, mac);
    search_params.domain = NULL;

    ret = VbEngineDatamodelClusterXAllDomainsLoop(DomainByMacSearchLoopCb, clusterId, &search_params);

    if ((ret == VB_ENGINE_ERROR_NONE) && (search_params.domain != NULL))
    {
      // Domain found
      *domain = search_params.domain;
      ret = VB_ENGINE_ERROR_NONE;
    }
    else
    {
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  return ret;
}

/*******************************************************************/

void VbEngineDatamodelStart(void)
{
  pthread_mutex_init(&(vbEngineTimersMutex), NULL);
}

/*******************************************************************/

void VbEngineDatamodelStop(void)
{
  pthread_mutex_destroy(&(vbEngineTimersMutex));
}

/*******************************************************************/

/**
 * @}
 **/
