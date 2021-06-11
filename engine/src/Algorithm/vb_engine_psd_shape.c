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
 * @file vb_engine_psd_shape.c
 * @brief Implements engine PSD shape functions
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016/06/16
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

#include "vb_log.h"
#include "vb_util.h"
#include "vb_engine_datamodel.h"
#include "vb_engine_EA_interface.h"
#include "vb_engine_clock.h"
#include "vb_counters.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_psd_shape.h"
#include "vb_engine_conf.h"

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



/*******************************************************************/

static t_VB_engineErrorCode VbEngineProcessSendPsdShapeLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  BOOLEAN             *send_psdshape = (BOOLEAN *)args;

  if ((node == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (node->channelSettings.psdShape.sendUpdate == TRUE)
    {
      *send_psdshape = TRUE;

      // Break the loop with VB_ENGINE_ERROR_EXIT_LOOP_OK error code
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEnginePsdShapeCnfNotifyLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
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

static t_VB_engineErrorCode VbEnginePsdShapeCnfCheckLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (node == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if ((node->channelSettings.psdShape.sendUpdate == TRUE) ||
        (node->channelSettings.boostInfo.levelCnf != node->channelSettings.boostInfo.level))
    {
      ret = VB_ENGINE_ERROR_NOT_READY;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEnginePSDShapeDriverCb(t_VBDriver *driver, void *args)
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
    if ((driver->FSMState == ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY) ||
        (num_nodes.numCompleteLines == 0))
    {
      // Skip drivers in "ENGINE_STT_DISCOVER_DOMAIN_WAIT_EMPTY" state or with no complete lines

      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "No complete lines detected -> Skip sending EAPsdShape.req");

      ret = VB_ENGINE_ERROR_SKIP;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Add room for header
    shape_args.length = VB_EA_PSD_SHAPE_REQ_HDR_SIZE;

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
    VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Send PSD shape to %u nodes", shape_args.numPsds);

    // Update pointer to start writing PSDs
    shape_args.ptrToWrite = shape_args.payload + VB_EA_PSD_SHAPE_REQ_HDR_SIZE;

    // Loop through all nodes to build the message
    ret = VbEngineDatamodelNodesLoop(driver, VbEnginePSDShapeBuildCb, &shape_args);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_vbEAPSDShapeHdr *psd_hdr;
    INT16U             seq_num = *((INT16U*)args);

    // Fill PSD Shape header
    psd_hdr = (t_vbEAPSDShapeHdr *)shape_args.payload;
    psd_hdr->numNodes = shape_args.numPsds;
    psd_hdr->seqNumber = _htons(seq_num);

    // Send EA message
    ret = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_PSD_SHAPE_CFG,
        shape_args.payload,
        shape_args.length,
        driver);
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
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Error %d sending new PSD shape", ret);
    }
  }
  else
  {
    VbCounterIncrease(VB_ENGINE_COUNTER_BOOSTING_SEND_PSD_SHAPE);
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

t_VB_engineErrorCode VbEnginePSDShapeConfigureAll(INT32U clusterId)
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
    // Loop through all drivers to build and send PSD Shape message
    ret = VbEngineDatamodelClusterXDriversLoop(VbEnginePSDShapeDriverCb, clusterId, &apply_seq_num);
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

t_VB_engineErrorCode    VbEnginePSDShapeRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;
  t_vbEAPsdShapingCnf *psd_shaping = (t_vbEAPsdShapingCnf *)payload;

  if ((psd_shaping == NULL) || (thisDriver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    if (psd_shaping->status == VB_EA_PSD_SHAPE_STATUS_OK)
    {
      // Notify PSD Shape confirmation
      VbEnginePsdShapeCnfNotify(thisDriver);
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "PSD shape confirmation error");
    }
  }

  return vb_err;
}

/*******************************************************************/

BOOLEAN VbEnginePsdShapeSendCheck(INT32U clusterId)
{
  t_VB_engineErrorCode err;
  BOOLEAN              send_psdshape = FALSE;

  // Loop through all nodes to detect PSD changes
  err = VbEngineDatamodelClusterXAllNodesLoop(VbEngineProcessSendPsdShapeLoopCb, clusterId, &send_psdshape);

  if (err != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d searching for PSD changes", err);
  }
  else
  {
    VbLogPrintExt(VB_LOG_DEBUG, VB_ENGINE_ALL_DRIVERS_STR, "Send new PSD shape to nodes -> %s", send_psdshape?"YES":"NO");
  }

  return send_psdshape;
}

/*******************************************************************/

t_VB_engineErrorCode VbEnginePsdShapeCnfNotify(t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (thisDriver != NULL)
  {
    // Loop through all nodes of given driver to notify Psd Shape confirmation
    ret = VbEngineDatamodelNodesLoop(thisDriver, VbEnginePsdShapeCnfNotifyLoopCb, NULL);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Error %d notifying for PSD shape confirmation", ret);
    }
  }
  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEnginePsdShapeCnfCheck(INT32U clusterId)
{
  t_VB_engineErrorCode ret;

  // Loop through all nodes to notify Psd Shape confirmation
  ret = VbEngineDatamodelClusterXAllNodesLoop(VbEnginePsdShapeCnfCheckLoopCb, clusterId, NULL);

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d checking for PSD shape confirmation", ret);
  }

  return ret;
}

/*******************************************************************/


t_VB_engineErrorCode VbEnginePSDShapeLengthCalcCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_psdShapeArgs      *shape_args = (t_psdShapeArgs *)args;

  if ((node == NULL) || (driver == NULL) || (domain == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineDatamodelDomainIsComplete(domain) == FALSE)
    {
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Domain is not complete for node %s. Skip it from PSD shaping process", node->MACStr);
    }
    else if (node->channelSettings.boostInfo.level != 0)
    {
      shape_args->length += VB_EA_PSD_SHAPE_REQ_STEP_HDR_SIZE + (node->channelSettings.psdShape.numPSDBands * VB_EA_PSD_SHAPE_REQ_STEP_SIZE);
      (shape_args->numPsds)++;
    }
    else if (node->trafficReports.reportsReceived == FALSE)
    {
      VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "No traffic reports received from %s, wait to send new PSD", node->MACStr);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEnginePSDShapeBuildCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_psdShapeArgs      *shape_args = (t_psdShapeArgs *)args;
  INT32U               i = 0;
  INT8U               *pld = NULL;

  if ((driver == NULL) ||
      (domain == NULL) ||
      (node == NULL) ||
      (shape_args == NULL) ||
      (shape_args->payload == NULL) ||
      (shape_args->ptrToWrite == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if ((ret == VB_ENGINE_ERROR_NONE) &&
      (VbEngineDatamodelDomainIsComplete(domain) == TRUE) &&
      (node->channelSettings.boostInfo.level != 0))
  {
    INT32U band_idx;
    t_psdBandAllocation *psd_band_allocation = VbEngineConfPSDBandAllocationGet();

    // Get pointer to write
    pld = shape_args->ptrToWrite;

    memcpy(((t_vbEAPSDShapeStepHdr *)pld)->MAC, node->MAC, ETH_ALEN);
    ((t_vbEAPSDShapeStepHdr *)pld)->numPSDSteps = _htons(node->channelSettings.psdShape.numPSDBands);

    // Calculate Upstream and Downstream phy rates
    if (node->type == VB_NODE_DOMAIN_MASTER)
    {
      if (domain->eps.epsArray != NULL)
      {
        ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateDown =
            node->cdtaInfo.bandCapacities[0][VB_ENGINE_QOS_RATE_80_20].capacityAllBoosted;

        ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateUp =
            domain->eps.epsArray[0].cdtaInfo.bandCapacities[0][VB_ENGINE_QOS_RATE_20_80].capacityAllBoosted;

        for(band_idx = 1; band_idx < psd_band_allocation->numBands200Mhz; band_idx++)
        {
          ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateDown +=
              node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_80_20].capacity1Boosted;
          ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateUp +=
              domain->eps.epsArray[0].cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_20_80].capacity1Boosted;
        }
      }
    }
    else
    {
      ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateDown =
          domain->dm.cdtaInfo.bandCapacities[0][VB_ENGINE_QOS_RATE_80_20].capacityAllBoosted;

      ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateUp =
          node->cdtaInfo.bandCapacities[0][VB_ENGINE_QOS_RATE_20_80].capacityAllBoosted;

      for(band_idx = 1; band_idx < psd_band_allocation->numBands200Mhz; band_idx++)
      {
        ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateDown +=
            domain->dm.cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_80_20].capacity1Boosted;
        ((t_vbEAPSDShapeStepHdr *)pld)->maxPhyRateUp +=
            node->cdtaInfo.bandCapacities[band_idx][VB_ENGINE_QOS_RATE_20_80].capacity1Boosted;
      }
    }

    pld += VB_EA_PSD_SHAPE_REQ_STEP_HDR_SIZE;

    VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "%s MAC %s; numBands %u; level %u",
        VbNodeTypeToStr(node->type),
        node->MACStr,
        node->channelSettings.psdShape.numPSDBands,
        node->channelSettings.boostInfo.level);

    for (i = 0; i < node->channelSettings.psdShape.numPSDBands; i++)
    {
      ((t_vbEAPSDShapeStep*)pld)->stopCarrier = _htons(node->channelSettings.psdShape.psdBandLevel[i].stopCarrier);
      ((t_vbEAPSDShapeStep*)pld)->attPSD = node->channelSettings.psdShape.psdBandLevel[i].attLevel;
      pld += VB_EA_PSD_SHAPE_REQ_STEP_SIZE;
    }

    // Update ptr to write for future nodes
    shape_args->ptrToWrite = pld;
  }

  return ret;
}
/**
 * @}
 **/
