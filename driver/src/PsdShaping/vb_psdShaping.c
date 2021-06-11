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
 * @file vb_psdShaping.c
 * @brief Psd Shaping
 *
 * @internal
 *
 * @author 
 * @date 27/04/2015
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

#include "vb_LCMP_com.h"
#include "vb_log.h"
#include "vb_LCMP_socket.h"
#include "vb_main.h"
#include "vb_psdShaping.h"
#include "vb_thread.h"
#include "vb_priorities.h"

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

#define PSD_SHAPING_THREAD_NAME       ("PSDShaping")

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

struct PACKMEMBER _vectorboostPsdShapeDmHeaderConf
{
  INT8U           macDm[6];         ///<
  INT16U          numBands;         ///<
};
typedef struct _vectorboostPsdShapeDmHeaderConf TYPE_ALIGNED32(t_vectorboostPsdShapeDmHeaderConf);

struct PACKMEMBER _vectorboostPsdShapeDmBandConf
{

  INT32U          stopCarrier:16;      ///< Control type: set to VB_PSD_SHAP
  INT32U          attLevel:8;       ///< Used to trigger traffic reports depending on ingress traffic and channel capacity
};
typedef struct _vectorboostPsdShapeDmBandConf TYPE_ALIGNED32(t_vectorboostPsdShapeDmBandConf);

struct PACKMEMBER _vectorboostTlvPsdShapeHeaderConf
{
  INT32U          ctrlType:8;          ///< Control type: set to VB_PSD_SHAPE
  INT32U          seqNumber:16;
  INT32U          numDm:8;       ///< Used to trigger traffic reports depending on ingress traffic and channel capacity
};
typedef struct _vectorboostTlvPsdShapeHeaderConf TYPE_ALIGNED32(t_vectorboostTlvPsdShapeHeaderConf);

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_t vbPsdShapeThread = 0;
static BOOL      vbPsdShapeThreadRunning = FALSE;

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

static t_vbPsdShapeError VBPsdShapeConf(t_Transmision transmisionType, INT8U *dstMac, INT8U *pld)
{
  t_vbPsdShapeError                  ret = VB_PSDSHAPE_ERR_NONE;
  t_VB_comErrorCode                  com_err = VB_COM_ERROR_NONE;
  t_HGF_LCMP_ErrorCode               lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams                    lcmp_params;
  t_ValuesArray                     *control_values_array = NULL;
  t_vbEAPSDShapeHdr                 *psd_shape_hdr;
  t_vbEAPSDShapeStepHdr             *psd_shape_step_hdr;
  t_vbEAPSDShapeStep                *psd_shape_step;
  t_vectorboostPsdShapeDmHeaderConf *lcmp_psdshape_dm_header_conf_tlv;
  t_vectorboostPsdShapeDmBandConf   *vbPsdShapeDmBandConf_tlv;
  INT8U                             *tlv;
  INT8U                             *pld_aux;
  INT8U                             *tlv_aux;
  INT32U                             lcmp_length = 0;
  INT32U                             i;
  INT32U                             j;
  INT16U                             num_bands;
  INT16U                             stopCarrier;

  if (((dstMac == NULL) && (transmisionType == UNICAST)) || (pld == NULL))
  {
    ret = VB_PSDSHAPE_ERR_BAD_ARGS;
    VbLogPrint(VB_LOG_ERROR, "Bad arguments");
  }

  if (ret == VB_PSDSHAPE_ERR_NONE)
  {
    lcmp_length        += sizeof(t_vectorboostTlvPsdShapeHeaderConf);
    psd_shape_hdr       = (t_vbEAPSDShapeHdr *)pld;
    pld_aux             = pld + VB_EA_PSD_SHAPE_REQ_HDR_SIZE;

    for (i = 0; i < psd_shape_hdr->numNodes; i++)
    {
      psd_shape_step_hdr  = (t_vbEAPSDShapeStepHdr *)(pld_aux);
      num_bands           = _ntohs(psd_shape_step_hdr->numPSDSteps);
      lcmp_length        += sizeof(t_vectorboostPsdShapeDmHeaderConf)+ (num_bands * sizeof(t_vectorboostPsdShapeDmBandConf));
      pld_aux            += VB_EA_PSD_SHAPE_REQ_STEP_HDR_SIZE + (num_bands * VB_EA_PSD_SHAPE_REQ_STEP_SIZE);
    }

    VbLogPrint(VB_LOG_DEBUG, "Psd tlv length: %d", lcmp_length);
    tlv = calloc(1, lcmp_length);
    if(tlv != NULL)
    {
      ((t_vectorboostTlvPsdShapeHeaderConf*)tlv)->ctrlType = VB_PSD_SHAPE;
      ((t_vectorboostTlvPsdShapeHeaderConf*)tlv)->seqNumber = _htons_ghn(_ntohs(psd_shape_hdr->seqNumber));
      ((t_vectorboostTlvPsdShapeHeaderConf*)tlv)->numDm = psd_shape_hdr->numNodes;

      pld_aux = pld + VB_EA_PSD_SHAPE_REQ_HDR_SIZE;
      tlv_aux = &tlv[sizeof(t_vectorboostTlvPsdShapeHeaderConf)];

      for (i = 0; i < psd_shape_hdr->numNodes; i++)
      {
        psd_shape_step_hdr = (t_vbEAPSDShapeStepHdr *)pld_aux;
        lcmp_psdshape_dm_header_conf_tlv = (t_vectorboostPsdShapeDmHeaderConf *)tlv_aux;

        MACAddrClone(lcmp_psdshape_dm_header_conf_tlv->macDm, psd_shape_step_hdr->MAC);
        num_bands = _ntohs(psd_shape_step_hdr->numPSDSteps);
        lcmp_psdshape_dm_header_conf_tlv->numBands = _htons_ghn(num_bands);

        pld_aux += VB_EA_PSD_SHAPE_REQ_STEP_HDR_SIZE;
        tlv_aux += sizeof(t_vectorboostPsdShapeDmHeaderConf);

        for (j = 0; j < num_bands; j++)
        {
          psd_shape_step = (t_vbEAPSDShapeStep *)pld_aux;
          vbPsdShapeDmBandConf_tlv = (t_vectorboostPsdShapeDmBandConf *)tlv_aux;

          vbPsdShapeDmBandConf_tlv->attLevel = psd_shape_step->attPSD;
          stopCarrier = _ntohs(psd_shape_step->stopCarrier);
          vbPsdShapeDmBandConf_tlv->stopCarrier = _htons_ghn(stopCarrier);

          pld_aux += VB_EA_PSD_SHAPE_REQ_STEP_SIZE;
          tlv_aux += sizeof(t_vectorboostPsdShapeDmBandConf);
        }
      }

      com_err = VbDatamodelValueToArrayAdd(lcmp_length, tlv, &control_values_array);

      free(tlv);
      tlv = NULL;
    }

    if (com_err != VB_COM_ERROR_NONE)
    {
      ret = VB_PSDSHAPE_ERR_COM;
      VbLogPrint(VB_LOG_ERROR, "VbDatamodelValueToArrayAdd; error %d",com_err);
    }
  }
  else
  {
    ret = VB_PSDSHAPE_ERR_NO_MEMORY;
  }

  if (ret == VB_PSDSHAPE_ERR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      ret = VB_PSDSHAPE_ERR_COM;
    }
  }

  if (ret == VB_PSDSHAPE_ERR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = transmisionType;
    lcmp_params.dstMac          = dstMac;
    lcmp_params.paramIdReq      = VB_PSD_SHAPE;
    lcmp_params.reqValues       = control_values_array;

    // Send control frame to configure PSD Shape
    lcmp_err = VbLcmpWrite(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending PsdShapeWrite.req", lcmp_err);

      ret = VB_PSDSHAPE_ERR_COM;
    }
  }

  // Always release memory
  VbDatamodelHTLVsArrayDestroy(&control_values_array);

  return ret;
}

/*******************************************************************/

static void *VBPsdShapeProcess(void *arg)
{
   t_vbPsdShapeError       err = VB_PSDSHAPE_ERR_NONE;
   INT8U                  *psd_shape = (INT8U *)arg;
   mqd_t                   vb_main_queue = -1;

   if (psd_shape == NULL)
   {
     err = VB_PSDSHAPE_ERR_BAD_ARGS;
   }

   if (err == VB_PSDSHAPE_ERR_NONE)
   {
     vb_main_queue = mq_open(VBQUEUENAME, O_WRONLY);

     if (vb_main_queue == -1)
     {
       VbLogPrint(VB_LOG_ERROR, "Error opening main queue : %s", strerror(errno));
       err = VB_PSDSHAPE_ERR_QUEUE;
     }
   }

   if (err == VB_PSDSHAPE_ERR_NONE)
   {
     // Always send PSD changes to Multicast to advise all nodes in domain network
     err = VBPsdShapeConf(MULTICAST, NULL, psd_shape);

     if (err == VB_PSDSHAPE_ERR_NONE)
     {
       VbMainQEvSend(DRIVER_EV_PSD_SHAPE_END_OK, vb_main_queue, NULL);
     }
     else
     {
       VbMainQEvSend(DRIVER_EV_PSD_SHAPE_END_KO, vb_main_queue, NULL);
     }
   }

   if(arg != NULL)
   {
     // Release buffer
     free(arg);
   }

   if (vb_main_queue != -1)
   {
     mq_close(vb_main_queue);
   }

   return NULL;
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

void VbPsdShapeWriteRun(INT8U *payload, INT16U length)
{
  INT8U *payload_copy = NULL;

  if (payload != NULL)
  {
    VbPsdShapeStop();

    // Allocate memory for given payload
    payload_copy = (INT8U *)calloc(1, length);

    if (payload_copy == NULL)
    {
      VbLogPrint( VB_LOG_ERROR, "No memory!");
    }
    else
    {
      memcpy(payload_copy, payload, length);

      VbLogPrint(VB_LOG_INFO, "Starting %s thread", PSD_SHAPING_THREAD_NAME);

      vbPsdShapeThreadRunning = TRUE;
      if (FALSE == VbThreadCreate(PSD_SHAPING_THREAD_NAME, VBPsdShapeProcess, payload_copy, VB_DRIVER_PSD_THREAD_PRIORITY, &vbPsdShapeThread))
      {
        VbLogPrint(VB_LOG_ERROR,"Can't create %s thread", PSD_SHAPING_THREAD_NAME);
        free(payload_copy);
        vbPsdShapeThreadRunning = FALSE;
      }
    }
  }
}

/*******************************************************************/

void VbPsdShapeStop(void)
{
  if (vbPsdShapeThreadRunning == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", PSD_SHAPING_THREAD_NAME);

    vbPsdShapeThreadRunning = FALSE;
    VbThreadJoin(vbPsdShapeThread, PSD_SHAPING_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", PSD_SHAPING_THREAD_NAME);
  }
}

/*******************************************************************/

void VbPsdShapeInit(void)
{
  vbPsdShapeThread = 0;
  vbPsdShapeThreadRunning = FALSE;
}

/*******************************************************************/

/**
 * @}
 **/
