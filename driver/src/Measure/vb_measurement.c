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
 * @file vb_mweasurement.c
 * @brief Measurement of vector boost system
 *
 * @internal
 *
 * @author
 * @date 19/01/2015
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
#include "vb_measurement.h"
#include "vb_measure_utils.h"
#include "vb_EA_interface.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_counters.h"
#include "vb_driver_conf.h"
#include "vb_main.h"
#include "ezxml.h"

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

#define MEASURE_PLAN_THREAD_NAME                   ("MeasurePlan")
#define MEASURE_COLLECT_THREAD_NAME                ("MeasCollect")
#define SNRPROBE_THREAD_NAME                       ("SNRProbe")

#define MEASURE_MAX_THREADS_COLLECTION             (100)

#define TIMEOUT_READ_SNR_PROBES_MEASUREMENT        (VbDriverConfLcmpDefaultTimeoutGet() + 100)//ms
#define TIMEOUT_READ_MEASUREMENT                   (VbDriverConfLcmpDefaultTimeoutGet())//ms

#define VB_MEASURE_BGN_IND_MEASURE_OFFSET          (sizeof(t_BGNMeasureIND))
#define VB_MEASURE_CFR_IND_MEASURE_OFFSET          (sizeof(t_CFRMeasureIND))
#define VB_MEASURE_SNR_IND_MEASURE_OFFSET          (sizeof(t_SNRMeasureIND))

#define VB_MEASURE_BGN_C_SIZE                      (sizeof(t_BGNMeasureC))
#define VB_MEASURE_CFR_C_SIZE                      (sizeof(t_CFRMeasureC))
#define VB_MEASURE_SNR_C_SIZE                      (sizeof(t_SNRMeasureC))

#define VB_MEASUREMENT_N_RETRY                     (5)

#define MEASURE_TRANSACTION_ID_OFFSET              (1000)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct __attribute__ ((packed))
{
  INT16U numCarriers;
  INT8U nextMeasureCarrierPosition;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U validMeasure : 1 ;
  INT8U phyMedium : 3 ;
  INT8U mimoMeas : 1 ;
  INT8U mimoInd : 1 ;
  INT8U :2 ;
#else
  INT8U :2 ;
  INT8U mimoInd : 1 ;
  INT8U mimoMeas : 1 ;
  INT8U phyMedium : 3 ;
  INT8U validMeasure : 1 ;
#endif
  INT8S rxg1Compensation;
  INT8S rxg2Compensation;
  INT16U firstCarrier;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U typeFormat : 4 ;
  INT8U dataFormat : 4 ;
#else
  INT8U dataFormat : 4 ;
  INT8U typeFormat : 4 ;
#endif
  INT8U outFormatSize;
  INT8U outFormatIntegerSize;
} t_ConfMeasureIND;

typedef struct __attribute__ ((packed))
{
  INT8U ParamID;
  INT8U MACMeasurer[ETH_ALEN];
  INT8U MACMeasured[ETH_ALEN];
  t_ConfMeasureIND  ConfMeasureIND;
} t_CFRMeasureIND;

typedef struct __attribute__ ((packed))
{
  INT8U ParamID;
  INT8U MACMeasurer[ETH_ALEN];
  t_ConfMeasureIND  ConfMeasureIND;
} t_BGNMeasureIND;

typedef struct __attribute__ ((packed))
{
  INT8U ParamID;
  INT8U MACMeasurer[ETH_ALEN];
  INT16U numCarriers;
  INT8U nextMeasureCarrierPosition;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U validMeasure : 1 ;
  INT8U phyMedium : 3 ;
  INT8U mimoMeas : 1 ;
  INT8U mimoInd : 1 ;
  INT8U :2 ;
#else
  INT8U :2 ;
  INT8U mimoInd : 1 ;
  INT8U mimoMeas : 1 ;
  INT8U phyMedium : 3 ;
  INT8U validMeasure : 1 ;
#endif
  INT8S rxg1Compensation;
  INT8S rxg2Compensation;
  INT16U firstCarrier;
  INT8U outFormatSize;
  INT8U outFormatIntegerSize;
} t_SNRMeasureIND;

typedef struct __attribute__ ((packed))
{
  INT8U ParamID;
  INT8U planID;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U type : 4 ;
  INT8U data : 4 ;
#else
  INT8U data : 4 ;
  INT8U type : 4 ;
#endif
} t_BGNMeasureC;

typedef struct __attribute__ ((packed))
{
  INT8U ParamID;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U type : 4 ;
  INT8U freqavg : 4 ;
  INT8U timeavg : 4 ;
  INT8U :4 ;
#else
  INT8U freqavg : 4 ;
  INT8U type : 4 ;
  INT8U :4 ;
  INT8U timeavg : 4 ;
#endif
} t_SNRMeasureC;

typedef struct __attribute__ ((packed))
{
  INT8U ParamID;
  INT8U planID;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U type : 4 ;
  INT8U data : 4 ;
#else
  INT8U data : 4 ;
  INT8U type : 4 ;
#endif
  INT8U MACMeasured[ETH_ALEN];
} t_CFRMeasureC;

typedef struct
{
  pthread_t          threadId;
  BOOL               threadRunning;
  INT8U              planId;
  INT8U              dataType;
  INT8U              formatType;
  INT8U              macMeasurer[ETH_ALEN];
  INT32U             numMacsMeasured;
  INT8U             *macsMeasuredList;
} t_measCollectInfo;

typedef struct
{
  INT32U             numThreads;
  t_measCollectInfo  collectInfo[MEASURE_MAX_THREADS_COLLECTION];
} t_measCollectInfoList;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_t              vbMeasurePlanThread = 0;
static BOOL                   vbMeasurePlanThreadRunning = FALSE;
static t_vbMsg                vbMeasurePlanReqCpy;
static t_vbMsg                vbMeasureSNRProbeReqCpy;
static INT8U                  vbMeasurePlanId;
static t_measCollectInfoList  collectInfoList;
static pthread_t              vbMeasureSNRProbeThread = 0;
static BOOL                   vbMeasureSNRProbeThreadRunning = FALSE;
static t_vbEAMeasRspErrorCode vbMeasureCollectEndError;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/**
 * @brief Send the cancellation of measurement plan to DMs and EPs
 * @param[in] planID Measurement plan identification
 * @return error code
**/
static t_VB_comErrorCode VbMeasurementPlanCancel( INT8U planId);

static t_VB_comErrorCode VbMeasurementPlanLCMPRequest( const INT8U *value, INT16U valueLength );

static t_VB_comErrorCode VbMeasurementBgnMeasureLCMPRead( const INT8U *nodeMac, INT8U planId,
                                                          t_MeasureDataType measureDataType, t_MeasureDataFormat measureDataFormat,
                                                          t_measCollectInfo *measCollectInfo, t_processMeasure **bgnMeasure);

static t_VB_comErrorCode VbMeasurementCfrMeasureNodeLCMPRead( const INT8U *nodeMac, const INT8U *nodeMacMeasured, INT8U planId,
    t_MeasureDataType measureDataType, t_MeasureDataFormat measureDataFormat, t_measCollectInfo *measCollectInfo, t_processMeasure **cfrMeasure);

static void *VBMeasurementPlanReqProcess(void *args);

static void *VBMeasurementMeasCollectProcess(void *args);

static void VbMeasureMeasuresCollectAllStop(void);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_VB_comErrorCode VbMeasurementPlanCancel( INT8U planId )
{
  t_VB_comErrorCode    result;
  t_ValuesArray       *cancels_plan_array = NULL;
  t_HGF_LCMP_ErrorCode lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams      lcmp_params;
  INT8U                value[2];
  INT16U               value_length;

  value[0] = VB_MEASPLAN_CANCEL;
  value[1] = planId;
  value_length = 2;
  result = VbDatamodelValueToArrayAdd(value_length, value, &cancels_plan_array);

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
    lcmp_params.transmisionType = MULTICAST;
    lcmp_params.paramIdReq      = VB_MEASPLAN_CANCEL;
    lcmp_params.reqValues       = cancels_plan_array;

    lcmp_err = VbLcmpControl(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending MeasPlanCancel.req", lcmp_err);
      result = VB_COM_ERROR_LCMP_CONTROL;
    }
  }

  VbDatamodelHTLVsArrayDestroy(&cancels_plan_array);

  return result;
}

/************************************************************************/

static t_VB_comErrorCode VbMeasurementPlanLCMPRequest( const INT8U *value, INT16U valueLength )
{
  t_VB_comErrorCode    result = VB_COM_ERROR_NONE;
  t_ValuesArray       *measurements_plan_array = NULL;
  t_HGF_LCMP_ErrorCode lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams      lcmp_params;

  if (value != NULL)
  {
    result = VbDatamodelValueToArrayAdd(valueLength, value, &measurements_plan_array);

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
      lcmp_params.transmisionType = MULTICAST;
      lcmp_params.paramIdReq      = VB_MEASPLAN;
      lcmp_params.reqValues       = measurements_plan_array;

      lcmp_err = VbLcmpWrite(&lcmp_params);

      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d sending MeasPlan.req", lcmp_err);

        result = VB_COM_ERROR_LCMP_WRITE;
      }
    }

    VbDatamodelHTLVsArrayDestroy(&measurements_plan_array);
  }
  else
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }

  return result;
}


/*******************************************************************/

static t_VB_comErrorCode VbMeasurementBgnMeasureLCMPRead( const INT8U *nodeMac, INT8U planId,
                                                          t_MeasureDataType measureDataType, t_MeasureDataFormat measureDataFormat,
                                                          t_measCollectInfo *measCollectInfo, t_processMeasure **bgnMeasure)
{
  t_VB_comErrorCode    result = VB_COM_ERROR_NONE;
  INT32U               n_retry;
  t_ValuesArray       *control_tlvs_array = NULL;
  t_ValuesArray       *read_measurement_array = NULL;
  t_HGF_LCMP_ErrorCode lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams      lcmp_params;
  t_BGNMeasureC        bgn_Measure_C;
  t_BGNMeasureIND     *measure;
  INT8U               *measure_data;
  INT16U               i;
  CHAR                 mac_str[MAC_STR_LEN];
  CHAR                 mac_rx_str[MAC_STR_LEN];

  if ((nodeMac != NULL) && (measCollectInfo != NULL))
  {
    // Get MAC string to be used in debug logs
    MACAddrMem2str(mac_str, nodeMac);

    if ((*bgnMeasure) != NULL)
    {
      VBDestroyProcessMeasure(bgnMeasure);
    }

    bgn_Measure_C.ParamID = VB_MEASURE_BGN_C;
    bgn_Measure_C.planID = planId;
    bgn_Measure_C.type = measureDataType;
    bgn_Measure_C.data = measureDataFormat;

    result = VbDatamodelValueToArrayAdd(VB_MEASURE_BGN_C_SIZE, (INT8U*)&bgn_Measure_C, &control_tlvs_array);

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
      n_retry = 0;

      // Configure LCMP params
      lcmp_params.transmisionType = UNICAST;
      lcmp_params.dstMac          = nodeMac;
      lcmp_params.paramIdReq      = VB_MEASURE_BGN_C;
      lcmp_params.paramIdRsp      = VB_MEASURE_BGN_IND;
      lcmp_params.reqValues       = control_tlvs_array;
      lcmp_params.notifyValues    = &read_measurement_array;

      do
      {
        lcmp_params.timeoutMs       = TIMEOUT_READ_MEASUREMENT * (n_retry + 1);

        lcmp_err = VbLcmpControlWaitNotify(&lcmp_params);

        if ((lcmp_err != HGF_LCMP_ERROR_NONE) || (read_measurement_array == NULL))
        {
          VbLogPrint(VB_LOG_ERROR, "Read BGN Error %d; BGN measure 0x%p:\tDeviceMAC: %s\tPlanID: %d - Retry %u", lcmp_err, read_measurement_array, mac_str, planId, n_retry);

          result = VB_COM_ERROR_LCMP_WAITNOTIFY;
        }

        if (result == VB_COM_ERROR_NONE)
        {
          measure = (t_BGNMeasureIND *)read_measurement_array->values[0].Value;

          if (measure->ParamID != VB_MEASURE_BGN_IND)
          {
            VbLogPrint(VB_LOG_ERROR, "Read BGN Error; OPCODE error; OPCODE expected: %X; OPCODE received: %X; DeviceMAC %s; Retry %u",
                              VB_MEASURE_BGN_IND, measure->ParamID, mac_str, n_retry);

            result = VB_COM_ERROR_LCMP_WAITNOTIFY;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          if (memcmp(nodeMac, measure->MACMeasurer, ETH_ALEN) != 0)
          {
            MACAddrMem2str(mac_rx_str, measure->MACMeasurer);
            VbLogPrint(VB_LOG_ERROR, "Read BGN Error; response MAC error; expected %s; received %s;  Retry %u",
                mac_str, mac_rx_str, n_retry);

            result = VB_COM_ERROR_LCMP_WAITNOTIFY;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          (*bgnMeasure) =  (t_processMeasure *)calloc(1, sizeof(t_processMeasure));

          if ((*bgnMeasure) == NULL)
          {
            VbLogPrint(VB_LOG_ERROR, "Read BGN Error; no memory for MAC %s!", mac_str);

            result = VB_COM_ERROR_MALLOC;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          (*bgnMeasure)->planID = planId;
          (*bgnMeasure)->spacing = measure->ConfMeasureIND.nextMeasureCarrierPosition;

          if ((measure->ConfMeasureIND.validMeasure && 0x01) == 1)
          {
            (*bgnMeasure)->errorCode = VB_MEAS_ERRCODE_VALID;
          }
          else
          {
            (*bgnMeasure)->errorCode = VB_MEAS_ERRCODE_INVALID;
          }

          (*bgnMeasure)->measures = (INT8U *)calloc(1, measure->ConfMeasureIND.numCarriers);

          if ((*bgnMeasure)->measures == NULL)
          {
            VbLogPrint(VB_LOG_ERROR, "Read BGN Error; no memory for MAC %s!", mac_str);

            result = VB_COM_ERROR_MALLOC;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          (*bgnMeasure)->type         = VB_MEAS_TYPE_BGN;
          (*bgnMeasure)->flags        = (measure->ConfMeasureIND.dataFormat & 0x01);
          (*bgnMeasure)->firstCarrier = _ntohs_ghn(measure->ConfMeasureIND.firstCarrier);
          (*bgnMeasure)->numMeasures  = _ntohs_ghn(measure->ConfMeasureIND.numCarriers);
          (*bgnMeasure)->rxg1Compensation = measure->ConfMeasureIND.rxg1Compensation;
          (*bgnMeasure)->rxg2Compensation = measure->ConfMeasureIND.rxg2Compensation;
          (*bgnMeasure)->mimoInd  =  (BOOLEAN)measure->ConfMeasureIND.mimoInd;
          (*bgnMeasure)->mimoMeas  =  (BOOLEAN)measure->ConfMeasureIND.mimoMeas;

          if (((*bgnMeasure)->numMeasures > 0) &&
              (read_measurement_array->values[0].ValueLength >= (VB_MEASURE_BGN_IND_MEASURE_OFFSET + (*bgnMeasure)->numMeasures)))
          {
            measure_data = &read_measurement_array->values[0].Value[VB_MEASURE_BGN_IND_MEASURE_OFFSET];

            if((measure->ConfMeasureIND.dataFormat  & 0x01) == 0)//db
                {
              for( i = 0 ; i < (*bgnMeasure)->numMeasures; i++ )
              {
                (*bgnMeasure)->measures[i] = measure_data[i];
              }
                }
            else
            {
              VbLogPrint(VB_LOG_ERROR,"Read BGN Error; invalid data type for MAC %s!", mac_str);

              result = VB_COM_ERROR_MEASURE_DATA_TYPE;
            }
          }
          else
          {
            VbLogPrint(VB_LOG_ERROR,"Read BGN Error; invalid measure for MAC %s!", mac_str);

            result = VB_COM_ERROR_COLLECT_MEASURE;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          VbLogPrint(VB_LOG_INFO,"Read BGN Ok; MAC measurer %s", mac_str);
        }

        if (result == VB_COM_ERROR_LCMP_WAITNOTIFY)
        {
          VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_BGN_RETRIES);
        }

        n_retry++;
      } while ((n_retry < VB_MEASUREMENT_N_RETRY) && (result == VB_COM_ERROR_LCMP_WAITNOTIFY) && (measCollectInfo->threadRunning == TRUE));
    }

    VbDatamodelHTLVsArrayDestroy(&read_measurement_array);
    VbDatamodelHTLVsArrayDestroy(&control_tlvs_array);
  }
  else
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_BGN_SUCCESS);
  }
  else
  {
    VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_BGN_ERROR);
  }

  return result;
}

/*******************************************************************/

static t_VB_comErrorCode VbMeasurementSnrProbesMeasureLCMPRead( const INT8U *nodeMAC, t_MeasureDataType measureDataType, t_MeasureDataFormat measureDataFormat, INT16U transactionId,
                                                         t_processMeasure **snrProbesMeasure)
{
  t_VB_comErrorCode     result = VB_COM_ERROR_NONE;
  t_ValuesArray        *control_tlvs_array = NULL;
  t_ValuesArray        *read_measurement_array = NULL;
  t_HGF_LCMP_ErrorCode  lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams       lcmp_params;
  t_SNRMeasureC         snr_measure_C;
  t_SNRMeasureIND      *measure = NULL;
  INT8U                *measure_data;
  INT16U                i;
  CHAR                  mac_str[MAC_STR_LEN];

  if (nodeMAC != NULL)
  {
    // Get MAC strings to be used in debug logs
    MACAddrMem2str(mac_str, nodeMAC);

    if ((*snrProbesMeasure) != NULL)
    {
      VBDestroyProcessMeasure(snrProbesMeasure);
    }

    snr_measure_C.ParamID = VB_MEASURE_SNR_C;
    snr_measure_C.type = measureDataType;
    snr_measure_C.freqavg = 0;
    snr_measure_C.timeavg = 0;

    result = VbDatamodelValueToArrayAdd(VB_MEASURE_SNR_C_SIZE, (INT8U*)&snr_measure_C, &control_tlvs_array);

    if (result != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Read SNR Error %d building request for MAC %s", result, mac_str);
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

    if (result ==  VB_COM_ERROR_NONE)
    {
      // Configure LCMP params
      lcmp_params.transmisionType = UNICAST;
      lcmp_params.dstMac          = nodeMAC;
      lcmp_params.paramIdReq      = VB_MEASURE_SNR_C;
      lcmp_params.paramIdRsp      = VB_MEASURE_SNR_IND;
      lcmp_params.reqValues       = control_tlvs_array;
      lcmp_params.notifyValues    = &read_measurement_array;
//      lcmp_params.transactionId   = transactionId;
      lcmp_params.timeoutMs       = TIMEOUT_READ_SNR_PROBES_MEASUREMENT;

      lcmp_err = VbLcmpControlWaitNotify(&lcmp_params);

      if ((lcmp_err != HGF_LCMP_ERROR_NONE) || (read_measurement_array == NULL))
      {
        VbLogPrint(VB_LOG_ERROR, "Read SNR Error %d;\tDeviceMAC: %s", lcmp_err, mac_str);
        result = VB_COM_ERROR_LCMP_WAITNOTIFY;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      measure = (t_SNRMeasureIND *)read_measurement_array->values[0].Value;

      if(measure->ParamID != VB_MEASURE_SNR_IND)
      {
        VbLogPrint(VB_LOG_ERROR, "Read SNR Error; OPCODE error; OPCODE expected: %X; OPCODE received: %X; DeviceMAC %s",
            VB_MEASURE_SNR_IND, measure->ParamID, mac_str);

        result = VB_COM_ERROR_LCMP_WAITNOTIFY;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      if (memcmp(nodeMAC, measure->MACMeasurer, ETH_ALEN) != 0 )
      {
        VbLogPrint(VB_LOG_ERROR, "Read SNR Error; response MAC error; expected MAC measurer %s", mac_str);

        MACAddrMem2str(mac_str, measure->MACMeasurer);
        VbLogPrint(VB_LOG_ERROR, "Read SNR Error; response MAC error; received MAC measurer %s", mac_str);

        result = VB_COM_ERROR_LCMP_WAITNOTIFY;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      (*snrProbesMeasure) =  (t_processMeasure *)calloc(1, sizeof(t_processMeasure));

      if ((*snrProbesMeasure) == NULL)
      {
        VbLogPrint(VB_LOG_ERROR, "Read SNR Error; no memory for MAC %s!", mac_str);

        result = VB_COM_ERROR_MALLOC;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      (*snrProbesMeasure)->planID = 0;
      (*snrProbesMeasure)->spacing = measure->nextMeasureCarrierPosition;

      if ((measure->validMeasure && 0x01) == 1)
      {
        (*snrProbesMeasure)->errorCode = VB_MEAS_ERRCODE_VALID;
      }
      else
      {
        (*snrProbesMeasure)->errorCode = VB_MEAS_ERRCODE_INVALID;
      }

      (*snrProbesMeasure)->measures = (INT8U *)calloc(1, measure->numCarriers);

      if ((*snrProbesMeasure)->measures == NULL)
      {
        VbLogPrint(VB_LOG_ERROR, "Read SNR Error; no memory for MAC %s!", mac_str);

        result = VB_COM_ERROR_MALLOC;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      (*snrProbesMeasure)->type         = VB_MEAS_TYPE_SNRPROBE;
      (*snrProbesMeasure)->flags        = 0x01;
      (*snrProbesMeasure)->firstCarrier = _ntohs_ghn(measure->firstCarrier);
      (*snrProbesMeasure)->numMeasures  = _ntohs_ghn(measure->numCarriers);
      (*snrProbesMeasure)->rxg1Compensation = measure->rxg1Compensation;
      (*snrProbesMeasure)->rxg2Compensation = measure->rxg2Compensation;
      (*snrProbesMeasure)->mimoInd = measure->mimoInd;

      if ((*snrProbesMeasure)->numMeasures > 0)
      {
        measure_data = &read_measurement_array->values[0].Value[VB_MEASURE_SNR_IND_MEASURE_OFFSET];

        for( i = 0 ; i < (*snrProbesMeasure)->numMeasures; i++ )
        {
          (*snrProbesMeasure)->measures[i] = measure_data[i];
        }
      }
      else
      {
        VbLogPrint(VB_LOG_ERROR,"Read SNR Error; invalid measure for MAC %s!", mac_str);

        result = VB_COM_ERROR_COLLECT_MEASURE;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_INFO,"Read SNR Probe Ok; MAC measurer %s", mac_str);
    }

    VbDatamodelHTLVsArrayDestroy(&read_measurement_array);
    VbDatamodelHTLVsArrayDestroy(&control_tlvs_array);
  }
  else
  {
    result = VB_COM_ERROR_BAD_ARGS;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_SNR_SUCCESS);
  }
  else
  {
    VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_SNR_ERROR);
  }

  return result;
}

/*******************************************************************/

static t_VB_comErrorCode VbSnrProbesMeasures( INT8U *macMeasurer, t_processMeasure **snrProbesMeasured)
{

  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
   t_VB_comErrorCode end_result;
   INT16U num_domains_check;
   INT16U num_eps_check;
   INT16U num_domains;
   INT16U num_eps;
   t_DomainData *domain_data = NULL;
   t_EpData *ep_data = NULL;
   INT8U *this_dm_mac = NULL;
   INT8U *this_ep_mac = NULL;
//   static INT16U transaction_id = 0;
   BOOLEAN           found = FALSE;

   //Get data of DMs
   end_result = VbDatamodelDomainsDataGet(&num_domains,&domain_data, FALSE, FALSE);

   if ((end_result == VB_COM_ERROR_NONE) && (domain_data != NULL))
   {
     //For each DM with EP
     for(num_domains_check = 0 ; num_domains_check < num_domains ; num_domains_check++)
     {
       this_dm_mac = domain_data[num_domains_check].mac;

       //If DM are measured in measures plan
       if(memcmp(this_dm_mac, macMeasurer, ETH_ALEN) == 0)
       {
         found = TRUE;
         //Request SNR probes from node
         result = VbMeasurementSnrProbesMeasureLCMPRead(this_dm_mac, 0, 0, 0/*++transaction_id*/, snrProbesMeasured);

         if(result != VB_COM_ERROR_NONE)
         {
           VbLogPrint(VB_LOG_ERROR,"DM %02X:%02X:%02X:%02X:%02X:%02X SNR probes failure",
                          this_dm_mac[0], this_dm_mac[1], this_dm_mac[2],
                          this_dm_mac[3], this_dm_mac[4], this_dm_mac[5]);
         }

         break;
       }

       if(found == FALSE)
       {
         //Get list of EPs data
         result = VbDatamodelEpsDataGet(this_dm_mac, &num_eps, &ep_data);

         if ((num_eps > 0) &&
             (ep_data != NULL) &&
             (end_result == VB_COM_ERROR_NONE) &&
             (result == VB_COM_ERROR_NONE))
         {
           //For each EP of domain
           for(num_eps_check = 0 ; num_eps_check < num_eps; num_eps_check++ )
           {
             this_ep_mac = ep_data[num_eps_check].mac;

             //If DM are measured in measures plan
             if(memcmp(this_ep_mac, macMeasurer, ETH_ALEN) == 0)
             {
               //Get SNR probes of EP
               result = VbMeasurementSnrProbesMeasureLCMPRead(this_ep_mac, 0, 0, 0, snrProbesMeasured);
               break;
             }
           }
         }
         else
         {
           end_result = VB_COM_ERROR_COLLECT_MEASURE;
         }
         if(ep_data != NULL)
         {
           free(ep_data);
           ep_data = NULL;
         }
       }
     }
   }

   if(domain_data != NULL)
   {
     free(domain_data);
     domain_data = NULL;
   }

   return end_result;
}

/*******************************************************************/

static t_VB_comErrorCode VbMeasurementCfrMeasureNodeLCMPRead( const INT8U *nodeMac, const INT8U *nodeMacMeasured, INT8U planId,
    t_MeasureDataType measureDataType, t_MeasureDataFormat measureDataFormat, t_measCollectInfo *measCollectInfo, t_processMeasure **cfrMeasure)
{

  t_VB_comErrorCode    result = VB_COM_ERROR_NONE;
  t_ValuesArray       *control_tlvs_array = NULL;
  t_ValuesArray       *read_measurement_array = NULL;
  t_HGF_LCMP_ErrorCode lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams      lcmp_params;
  INT32U               n_retry;
  INT8U               *measure_data;
  t_CFRMeasureIND     *measure;
  INT16U               i;
  t_CFRMeasureC        cfr_measure_c;
  CHAR                 mac_str[MAC_STR_LEN];
  CHAR                 mac_measured_str[MAC_STR_LEN];

  if ((nodeMac != NULL) && (nodeMacMeasured != NULL) && (measCollectInfo != NULL))
  {
    // Get MAC strings to be used in debug logs
    MACAddrMem2str(mac_str, nodeMac);
    MACAddrMem2str(mac_measured_str, nodeMacMeasured);

    if ((*cfrMeasure) != NULL)
    {
      VBDestroyProcessMeasure(cfrMeasure);
    }

    cfr_measure_c.ParamID = VB_MEASURE_CFR_AMP_C;
    cfr_measure_c.planID = planId;
    cfr_measure_c.type = measureDataType ;
    cfr_measure_c.data = measureDataFormat;
    MACAddrClone(cfr_measure_c.MACMeasured, nodeMacMeasured);

    result = VbDatamodelValueToArrayAdd(VB_MEASURE_CFR_C_SIZE, (INT8U *)&cfr_measure_c, &control_tlvs_array);

    if (result != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Read CFR Error %d building request for MAC %s", result, mac_str);
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
      n_retry = 0;

      // Configure LCMP params
      lcmp_params.transmisionType = UNICAST;
      lcmp_params.dstMac          = nodeMac;
      lcmp_params.paramIdReq      = VB_MEASURE_CFR_AMP_C;
      lcmp_params.paramIdRsp      = VB_MEASURE_CFR_AMP_IND;
      lcmp_params.reqValues       = control_tlvs_array;
      lcmp_params.notifyValues    = &read_measurement_array;

      do
      {
        lcmp_params.timeoutMs       = TIMEOUT_READ_MEASUREMENT * (n_retry + 1);

        lcmp_err = VbLcmpControlWaitNotify(&lcmp_params);

        if (lcmp_err == HGF_LCMP_ERROR_CNF_INVALID)
        {
          VbLogPrint(VB_LOG_INFO, "CFR measure not found; CFR measure:\tDeviceMAC: %s\tDeviceMACMeasured: %s\tPlanID: %d - Retry %u",
                     mac_str,
                     mac_measured_str,
                     planId,
                     n_retry);

          result = VB_COM_ERROR_NOT_FOUND;
        }
        else if ((lcmp_err != HGF_LCMP_ERROR_NONE) || (read_measurement_array == NULL))
        {
          VbLogPrint(VB_LOG_ERROR, "Read CFR Error %d; CFR measure:\tDeviceMAC: %s\tDeviceMACMeasured: %s\tPlanID: %d - Retry %u",
                     lcmp_err,
                     mac_str,
                     mac_measured_str,
                     planId,
                     n_retry);

          result = VB_COM_ERROR_LCMP_WAITNOTIFY;
        }

        if (result == VB_COM_ERROR_NONE)
        {
          measure = (t_CFRMeasureIND *)read_measurement_array->values[0].Value;

          if (measure->ParamID != VB_MEASURE_CFR_AMP_IND)
          {
            VbLogPrint(VB_LOG_ERROR, "Read CFR Error; OPCODE error; OPCODE expected: %X; OPCODE received: %X; DeviceMAC %s; Retry %u",
                       VB_MEASURE_CFR_AMP_IND, measure->ParamID, mac_str, n_retry);

            result = VB_COM_ERROR_LCMP_WAITNOTIFY;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          if ((memcmp(nodeMac, measure->MACMeasurer, ETH_ALEN) != 0) ||
              (memcmp(nodeMacMeasured, measure->MACMeasured, ETH_ALEN) != 0))
          {
            VbLogPrint(VB_LOG_ERROR, "Read CFR Error; response MAC error; expected MAC measurer %s; MAC measured %s",
                mac_str, mac_measured_str);

            MACAddrMem2str(mac_str, measure->MACMeasurer);
            MACAddrMem2str(mac_measured_str, measure->MACMeasured);
            VbLogPrint(VB_LOG_ERROR, "Read CFR Error; response MAC error; received MAC measurer %s; MAC measured %s; Retry %u",
                mac_str, mac_measured_str, n_retry);

            result = VB_COM_ERROR_LCMP_WAITNOTIFY;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          (*cfrMeasure) =  (t_processMeasure *)calloc(1, sizeof(t_processMeasure));

          if ((*cfrMeasure) == NULL)
          {
            VbLogPrint(VB_LOG_ERROR, "Read CFR Error; no memory for MAC %s!", mac_str);

            result = VB_COM_ERROR_MALLOC;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          (*cfrMeasure)->planID = planId;
          (*cfrMeasure)->spacing = measure->ConfMeasureIND.nextMeasureCarrierPosition;

          if ((measure->ConfMeasureIND.validMeasure && 0x01) == 1)
          {
            (*cfrMeasure)->errorCode = VB_MEAS_ERRCODE_VALID;
          }
          else
          {
            (*cfrMeasure)->errorCode = VB_MEAS_ERRCODE_INVALID;
          }

          (*cfrMeasure)->measures = (INT8U *)calloc(1, measure->ConfMeasureIND.numCarriers);

          if ((*cfrMeasure)->measures == NULL)
          {
            VbLogPrint(VB_LOG_ERROR, "Read CFR Error; no memory for MAC %s!", mac_str);

            result = VB_COM_ERROR_MALLOC;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          (*cfrMeasure)->type         = VB_MEAS_TYPE_CFR;
          (*cfrMeasure)->flags        = (measure->ConfMeasureIND.dataFormat & 0x01);
          (*cfrMeasure)->firstCarrier = _ntohs_ghn(measure->ConfMeasureIND.firstCarrier);
          (*cfrMeasure)->numMeasures  = _ntohs_ghn(measure->ConfMeasureIND.numCarriers);
          (*cfrMeasure)->mimoInd  =  (BOOLEAN)measure->ConfMeasureIND.mimoInd;
          (*cfrMeasure)->mimoMeas  =  (BOOLEAN)measure->ConfMeasureIND.mimoMeas;
          (*cfrMeasure)->rxg1Compensation  = measure->ConfMeasureIND.rxg1Compensation;
          (*cfrMeasure)->rxg2Compensation  = measure->ConfMeasureIND.rxg2Compensation;

          if (((*cfrMeasure)->numMeasures > 0) &&
              (read_measurement_array->values[0].ValueLength >= (VB_MEASURE_CFR_IND_MEASURE_OFFSET + (*cfrMeasure)->numMeasures)))
          {
            measure_data = &read_measurement_array->values[0].Value[VB_MEASURE_CFR_IND_MEASURE_OFFSET];

            if ((measure->ConfMeasureIND.dataFormat  & 0x01) == 0)//db
            {
              for( i = 0 ; i < (*cfrMeasure)->numMeasures; i++ )
              {
                (*cfrMeasure)->measures[i] = measure_data[i];
              }
            }
            else
            {
              VbLogPrint(VB_LOG_ERROR,"Read CFR Error; invalid data type for MAC %s!", mac_str);

              result = VB_COM_ERROR_MEASURE_DATA_TYPE;
            }
          }
          else
          {
            VbLogPrint(VB_LOG_ERROR,"Read CFR Error; invalid measure for MAC %s!", mac_str);

            result = VB_COM_ERROR_COLLECT_MEASURE;
          }
        }

        if (result == VB_COM_ERROR_NONE)
        {
          VbLogPrint(VB_LOG_INFO,"Read CFR Ok; MAC measurer %s; MAC measured %s", mac_str, mac_measured_str);
        }

        if (result == VB_COM_ERROR_LCMP_WAITNOTIFY)
        {
          VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_CFR_RETRIES);
        }

        n_retry++;
      } while((n_retry < VB_MEASUREMENT_N_RETRY) && (result == VB_COM_ERROR_LCMP_WAITNOTIFY) && (measCollectInfo->threadRunning == TRUE));
    }

    VbDatamodelHTLVsArrayDestroy(&read_measurement_array);
    VbDatamodelHTLVsArrayDestroy(&control_tlvs_array);
  }
  else
  {
    result = VB_COM_ERROR_BAD_ARGS;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_CFR_SUCCESS);
  }
  else
  {
    VbCounterIncrease(VB_DRIVER_COUNTER_MEASURE_CFR_ERROR);
  }

  return result;
}

/*******************************************************************/

static void *VBMeasurementPlanReqProcess(void *args)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbMsg               *planReqMsg = (t_vbMsg *)args;
  INT16U                 length;
  INT8U                  *msg;
  mqd_t                  vb_main_queue;

  vb_main_queue = mq_open(VBQUEUENAME, O_WRONLY);

  if (vb_main_queue == -1)
  {
    VbLogPrint(VB_LOG_ERROR, "Error opening main queue : %s", strerror(errno));
    ret = VB_COM_ERROR_QUEUE;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if(planReqMsg == NULL)
    {
      ret = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    length = planReqMsg->msgLen;
    msg = planReqMsg->msg;

    if (msg == NULL)
    {
      VbLogPrint(VB_LOG_ERROR, "Invalid measure plan request");
      ret = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_INFO, "Measure plan request");

    // Get measure plan id from msg
    vbMeasurePlanId = msg[1];

    // Send LCMP Measure Plan Request
    ret = VbMeasurementPlanLCMPRequest(msg, length);

    if ((ret != VB_COM_ERROR_NONE) &&
        (vbMeasurePlanThreadRunning == TRUE))
    {
      VbLogPrint(VB_LOG_ERROR, "Measure execution error %d", ret);

      // Measure pLan has not been sent successfully
      // Send Plan Failed Message to Driver
      VbMainQEvSend(DRIVER_EV_PLAN_FAILED, vb_main_queue, (void *)VB_EA_MEAS_RSP_ERR_PROCESS);
    }
  }

  if ((ret == VB_COM_ERROR_NONE) &&
      (vbMeasurePlanThreadRunning == TRUE))
  {
    // Measure plan has been sent successfully
    VbMainQEvSend(DRIVER_EV_PLAN_CNF_OK, vb_main_queue, NULL);
  }

  if ((vbMeasurePlanThreadRunning == TRUE) && (planReqMsg != NULL) &&
       (planReqMsg->msg != NULL))
  {
    // Free inital msg received on TCP socket from the engine
    free(planReqMsg->msg);
    planReqMsg->msg = NULL;
  }

  if (vb_main_queue != -1)
  {
    // Close queue
    mq_close(vb_main_queue);
  }

  return NULL;
}

/*******************************************************************/

static void *VBMeasurementSNRProbeProcess(void *args)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbMsg                *snr_probe_msg = (t_vbMsg *)args;
  INT32U                 i;
  INT8U                  *pld;
  INT8U                  *mac;
  INT8U                  n_macs;
  t_processMeasure       *measure = NULL;

  if(args == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if(ret == VB_COM_ERROR_NONE)
  {
    pld = snr_probe_msg->msg;

    n_macs = pld[0];
    mac = &pld[1];

    for (i=0; i<n_macs; i++)
    {
      // Request SNR Probe through LCMP
      ret = VbSnrProbesMeasures(mac, &measure);

      if (ret == VB_COM_ERROR_NONE)
      {
        // Pass it to the engine
        ret = VbEASnrProbesRspSend(mac, measure);
      }
      else
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d requesting SNR Probe measures through LCMP", ret);
      }

      mac += ETH_ALEN;

      if (measure != NULL)
      {
        VBDestroyProcessMeasure(&measure);
        measure = NULL;
      }
    }

    if(snr_probe_msg->msg)
    {
      free(snr_probe_msg->msg);
      snr_probe_msg->msg = NULL;
    }
  }

  return NULL;
}

/*******************************************************************/

static void *VBMeasurementMeasCollectProcess(void *args)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  INT16U                 num_nodes_measured;
  INT16U                 plan_id;
  INT16U                 i;
  INT8U                  *mac_measurer;
  INT8U                  *mac_measured;
  t_measCollectInfo     *meas_collect_info = (t_measCollectInfo *)args;
  t_processMeasure      *measure = NULL;
  mqd_t                  vb_main_queue = -1;

  if (meas_collect_info == NULL)
  {
    VbLogPrint(VB_LOG_ERROR, "Error: measure collect info is null");
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    vb_main_queue = mq_open(VBQUEUENAME, O_WRONLY);

    if (vb_main_queue == -1)
    {
      VbLogPrint(VB_LOG_ERROR, "Error opening main queue : %s", strerror(errno));
      ret = VB_COM_ERROR_QUEUE;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    CHAR mac_measurer_str[MAC_STR_LEN];

    // Get measurer MAC
    MACAddrMem2str(mac_measurer_str, meas_collect_info->macMeasurer);
    VbLogPrint(VB_LOG_INFO, "Measures Collect thread for node %s", mac_measurer_str);

    // Send LCMP Measure CFR Request
    mac_measurer = meas_collect_info->macMeasurer;
    num_nodes_measured = meas_collect_info->numMacsMeasured;
    plan_id = meas_collect_info->planId;
    mac_measured = meas_collect_info->macsMeasuredList;

    // Request CFRs Direct & Crosstalk
    for (i = 0; (i < num_nodes_measured) && (meas_collect_info->threadRunning == TRUE); i++)
    {
      // Get CFR from G.hn node
      ret = VbMeasurementCfrMeasureNodeLCMPRead(mac_measurer, mac_measured, plan_id,
          meas_collect_info->dataType , meas_collect_info->formatType, meas_collect_info, &measure);

      if (ret != VB_COM_ERROR_NONE)
      {
        /*
         * CFR Measure not found, continue with next one.
         * If own CFR is missing, engine will detect it later during SNR calculation
         */
        ret = VB_COM_ERROR_NONE;
      }
      else
      {
        // CFR Measure found, pass it to the engine
        ret = VbEACfrRspSend(mac_measurer, mac_measured, plan_id, measure);
      }

      if (measure != NULL)
      {
        VBDestroyProcessMeasure(&measure);
      }

      if (ret != VB_COM_ERROR_NONE)
      {
        break;
      }

      mac_measured += ETH_ALEN;
    }

    if ((ret == VB_COM_ERROR_NONE) && (meas_collect_info->threadRunning == TRUE))
    {
      // Request BNG noise from G.hn node
      ret = VbMeasurementBgnMeasureLCMPRead(mac_measurer, plan_id,
          meas_collect_info->dataType , meas_collect_info->formatType, meas_collect_info, &measure);

      if (ret == VB_COM_ERROR_NONE)
      {
        // Pass it to the engine
        ret = VbEABgnRspSend(mac_measurer, plan_id, measure);
      }

      if (measure != NULL)
      {
        VBDestroyProcessMeasure(&measure);
      }
    }

    if (ret != VB_COM_ERROR_NONE)
    {
      // Update error
      vbMeasureCollectEndError = VB_EA_MEAS_RSP_ERR_PROCESS;
    }

    if (meas_collect_info->threadRunning == TRUE)
    {
      // Send event to the main
      VbMainQEvSend(DRIVER_EV_MEAS_COLLECT_NODE_END, vb_main_queue, meas_collect_info);
    }

    if (meas_collect_info->macsMeasuredList != NULL)
    {
      free(meas_collect_info->macsMeasuredList);
      meas_collect_info->macsMeasuredList = NULL;
    }
  }

  if (vb_main_queue != -1)
  {
    // Close queue
    mq_close(vb_main_queue);
  }

  return NULL;
}

/*******************************************************************/

static void VbMeasureMeasuresCollectAllStop(void)
{
  INT32U    i;
  INT32U    num_threads_to_join = 0;
  INT32U    threads_to_join[MEASURE_MAX_THREADS_COLLECTION];

  for (i = 0; i < MEASURE_MAX_THREADS_COLLECTION; i++)
  {
    if (collectInfoList.collectInfo[i].threadRunning == TRUE)
    {
      VbLogPrint(VB_LOG_INFO, "Stopping %s thread for node " MAC_PRINTF_FORMAT "...",
          MEASURE_COLLECT_THREAD_NAME, MAC_PRINTF_DATA(collectInfoList.collectInfo[i].macMeasurer));

      collectInfoList.collectInfo[i].threadRunning = FALSE;

      threads_to_join[num_threads_to_join] = i;
      num_threads_to_join++;
    }
  }

  for (i = 0; i < num_threads_to_join; i++)
  {
    INT32U             thread_info_idx = threads_to_join[i];
    t_measCollectInfo *meas_collect_info;

    if (thread_info_idx < MEASURE_MAX_THREADS_COLLECTION)
    {
      meas_collect_info = &(collectInfoList.collectInfo[thread_info_idx]);

      VbThreadJoin(meas_collect_info->threadId, MEASURE_COLLECT_THREAD_NAME);

      VbLogPrint(VB_LOG_INFO, "Stopped %s thread for node " MAC_PRINTF_FORMAT "!",
          MEASURE_COLLECT_THREAD_NAME, MAC_PRINTF_DATA(meas_collect_info->macMeasurer));

      if (meas_collect_info->macsMeasuredList != NULL)
      {
        free(meas_collect_info->macsMeasuredList);
        meas_collect_info->macsMeasuredList = NULL;
      }
    }
  }

  collectInfoList.numThreads = 0;
}

/*******************************************************************/

static t_VB_comErrorCode VbMeasureMeasuresCollectThreadNew(INT8U planId, INT8U dataType,
    INT8U formatType, INT8U *macMeasurer, INT32U numMacsMeasured, INT8U *macsMeasuredList)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_measCollectInfo     *meas_collect_info = NULL;

  if ((macMeasurer == NULL) || (macsMeasuredList == NULL) || (numMacsMeasured == 0))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    INT32U time_between_threads = VbDriverConfMeasCollectThreadIntGet();

    if (time_between_threads > 0)
    {
      VbThreadSleep(time_between_threads);
    }

    if (collectInfoList.numThreads >= MEASURE_MAX_THREADS_COLLECTION)
    {
      VbLogPrint(VB_LOG_ERROR, "Unable to create new thread. Max number reached (%u)", MEASURE_MAX_THREADS_COLLECTION);
      ret = VB_COM_ERROR_MEASURE_THREAD_MAX;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    meas_collect_info = &collectInfoList.collectInfo[collectInfoList.numThreads];

    // Allocate memory for MAC measured list
    meas_collect_info->macsMeasuredList = (INT8U *)calloc(1, ETH_ALEN * numMacsMeasured);

    if (meas_collect_info->macsMeasuredList == NULL)
    {
      VbLogPrint(VB_LOG_ERROR, "No memory!");
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    BOOLEAN running;
    CHAR    mac_measurer_str[MAC_STR_LEN];

    // Get measurer MAC
    MACAddrMem2str(mac_measurer_str, macMeasurer);

    VbLogPrint(VB_LOG_INFO, "Starting %s thread #%u for node %s", MEASURE_COLLECT_THREAD_NAME, collectInfoList.numThreads, mac_measurer_str);

    // Init collect info parameters
    meas_collect_info->planId = planId;
    meas_collect_info->dataType = dataType;
    meas_collect_info->formatType = formatType;
    meas_collect_info->numMacsMeasured = numMacsMeasured;
    MACAddrClone(meas_collect_info->macMeasurer, macMeasurer);
    memcpy(meas_collect_info->macsMeasuredList, macsMeasuredList, ETH_ALEN * numMacsMeasured);
    meas_collect_info->threadRunning = TRUE;

    // Launch thread
    running =  VbThreadCreate(MEASURE_COLLECT_THREAD_NAME, VBMeasurementMeasCollectProcess,
    		meas_collect_info, VB_DRIVER_MEAS_COLLECT_THREAD_PRIORITY, &meas_collect_info->threadId);

    if (running == FALSE)
    {
      VbLogPrint(VB_LOG_ERROR, "Can't create %s thread", MEASURE_COLLECT_THREAD_NAME);

      meas_collect_info->threadRunning = FALSE;
      ret = VB_COM_ERROR_MEASURE_THREAD_ABORT;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Increase the number of threads
    collectInfoList.numThreads++;
  }

  if (ret != VB_COM_ERROR_NONE)
  {
    // Release allocated memory
    if ((meas_collect_info != NULL) && (meas_collect_info->macsMeasuredList != NULL))
    {
      free(meas_collect_info->macsMeasuredList);
      meas_collect_info->macsMeasuredList = NULL;
    }
  }

  return ret;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/*******************************************************************/

t_VB_comErrorCode VBDestroyProcessMeasure(t_processMeasure **ProcessMeasure)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;

  if((*ProcessMeasure)!= NULL)
  {
    if(((*ProcessMeasure)->measures)!= NULL)
    {
      free(((*ProcessMeasure)->measures));
      ((*ProcessMeasure)->measures) = NULL;
    }
    free((*ProcessMeasure));
    (*ProcessMeasure) = NULL;
  }
  return result;
}

/*******************************************************************/

t_VB_comErrorCode VbMeasurementInit(void)
{
  t_VB_comErrorCode err_code = VB_COM_ERROR_NONE;

  memset((INT8U*)&collectInfoList, 0x00, sizeof(collectInfoList));

  return err_code;
}

/*******************************************************************/

t_VB_comErrorCode VbMeasurePlanReqRun(t_vbMsg planReqMsg)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (vbMeasurePlanThreadRunning == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Previous Measure plan still running");
    ret = VB_COM_ERROR_ALREADY_RUNNING;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate memory for given payload
    vbMeasurePlanReqCpy.msg = (INT8U *)calloc(1, planReqMsg.msgLen);

    if (vbMeasurePlanReqCpy.msg == NULL)
    {
      VbLogPrint(VB_LOG_ERROR, "No memory!");
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_INFO, "Starting %s thread", MEASURE_PLAN_THREAD_NAME);

    memcpy(vbMeasurePlanReqCpy.msg, planReqMsg.msg, planReqMsg.msgLen);
    vbMeasurePlanReqCpy.msgLen = planReqMsg.msgLen;

    vbMeasurePlanThreadRunning = TRUE;

    if (FALSE == VbThreadCreate(MEASURE_PLAN_THREAD_NAME, VBMeasurementPlanReqProcess, (void *)&vbMeasurePlanReqCpy, VB_DRIVER_MEASUREMENT_THREAD_PRIORITY, &vbMeasurePlanThread))
    {
      VbLogPrint(VB_LOG_ERROR,"Can't create %s thread", MEASURE_PLAN_THREAD_NAME);
      free(vbMeasurePlanReqCpy.msg);
      vbMeasurePlanReqCpy.msg = NULL;
      vbMeasurePlanThreadRunning = FALSE;

      ret = VB_COM_ERROR_MEASURE_THREAD_ABORT;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbMeasureSNRProbeReqRun(t_vbMsg snrProbeMsg)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  // Allocate memory for given payload
  vbMeasureSNRProbeReqCpy.msg = (INT8U *)calloc(1, snrProbeMsg.msgLen);
  if (vbMeasureSNRProbeReqCpy.msg == NULL)
  {
    VbLogPrint(VB_LOG_ERROR, "No memory!");
    ret = VB_COM_ERROR_MALLOC;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_INFO, "Starting %s thread", SNRPROBE_THREAD_NAME);

    memcpy(vbMeasureSNRProbeReqCpy.msg, snrProbeMsg.msg, snrProbeMsg.msgLen);
    vbMeasureSNRProbeReqCpy.msgLen = snrProbeMsg.msgLen;

    vbMeasureSNRProbeThreadRunning = TRUE;

    if (FALSE == VbThreadCreate(SNRPROBE_THREAD_NAME, VBMeasurementSNRProbeProcess, (void *)&vbMeasureSNRProbeReqCpy, VB_DRIVER_SNRPROBE_THREAD_PRIORITY, &vbMeasureSNRProbeThread))
    {
      VbLogPrint(VB_LOG_ERROR,"Can't create %s thread", SNRPROBE_THREAD_NAME);
      vbMeasureSNRProbeThreadRunning = FALSE;
      free(vbMeasureSNRProbeReqCpy.msg);
      vbMeasureSNRProbeReqCpy.msg = NULL;
      ret = VB_COM_ERROR_MEASURE_THREAD_ABORT;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbMeasureMeasuresCollectRun(INT8U *measCollectReqMsg, INT16U msgLength)
{
  t_VB_comErrorCode         ret = VB_COM_ERROR_NONE;
  t_vbEAMeasCollectReqHdr  *meas_collect_req_hdr;
  INT32U                    num_macs_measurer;

  if ((measCollectReqMsg == NULL) || (msgLength < VB_EA_MEAS_COLLECT_REQ_HDR_SIZE))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    meas_collect_req_hdr = (t_vbEAMeasCollectReqHdr *)measCollectReqMsg;

    // Get number of measure collect threads to run
    num_macs_measurer = _ntohs(meas_collect_req_hdr->numMACsMeasurer);

    if (num_macs_measurer == 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Invalid EAMeasCollect.req message (no measurer device)");
      ret = VB_COM_ERROR_NODEVICE;
    }
    else if ((collectInfoList.numThreads + num_macs_measurer) > MEASURE_MAX_THREADS_COLLECTION)
    {
      VbLogPrint(VB_LOG_ERROR, "Unable to create new thread. Max number reached (%u)", MEASURE_MAX_THREADS_COLLECTION);
      ret = VB_COM_ERROR_MEASURE_THREAD_MAX;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEAMeasCollectReqNode *collect_req_node;
    INT32U                    measurer_idx;
    INT8U                    *macs_measured_list;
    INT8U                    *payload_ptr;
    INT16U                    num_macs_measured;

    // Init error
    vbMeasureCollectEndError = VB_EA_MEAS_RSP_ERR_NONE;

    // Advance to measurer section
    payload_ptr = measCollectReqMsg + VB_EA_MEAS_COLLECT_REQ_HDR_SIZE;

    for (measurer_idx = 0; (measurer_idx < num_macs_measurer) && (ret == VB_COM_ERROR_NONE); measurer_idx++)
    {
      // Get measurer section
      collect_req_node = (t_vbEAMeasCollectReqNode *)payload_ptr;

      // Advance to measured MAC list
      payload_ptr += VB_EA_MEAS_COLLECT_REQ_NODE_SIZE;

      // Get measured MAC list
      macs_measured_list = payload_ptr;

      // Get number of measured MACs
      num_macs_measured = _ntohs(collect_req_node->numMACsMeasured);

      // Create a new thread
      ret = VbMeasureMeasuresCollectThreadNew(meas_collect_req_hdr->planID,
          meas_collect_req_hdr->dataType,
          meas_collect_req_hdr->formatType,
          collect_req_node->macMeasurer,
          num_macs_measured,
          macs_measured_list);

      // Advance to next measurer node
      payload_ptr += num_macs_measured * ETH_ALEN;
    }
  }

  return ret;
}

/*******************************************************************/

void VbMeasurePlanStop(void)
{
  if (vbMeasurePlanThreadRunning == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", MEASURE_PLAN_THREAD_NAME);

    vbMeasurePlanThreadRunning = FALSE;
    VbThreadJoin(vbMeasurePlanThread, MEASURE_PLAN_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", MEASURE_PLAN_THREAD_NAME);
  }

  // Release previously allocated memory
  if (vbMeasurePlanReqCpy.msg != NULL)
  {
    free(vbMeasurePlanReqCpy.msg);
    vbMeasurePlanReqCpy.msg = NULL;
  }
}

/*******************************************************************/

t_VB_comErrorCode VbMeasureMeasuresCollectStop(void *data)
{
  t_VB_comErrorCode  ret = VB_COM_ERROR_NONE;
  t_measCollectInfo *meas_collect_info = (t_measCollectInfo *)data;

  if (meas_collect_info == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (meas_collect_info->threadRunning == TRUE)
    {
      CHAR    mac_measurer_str[MAC_STR_LEN];

      // Get measurer MAC
      MACAddrMem2str(mac_measurer_str, meas_collect_info->macMeasurer);

      VbLogPrint(VB_LOG_INFO, "Stopping %s thread for node %s...", MEASURE_COLLECT_THREAD_NAME, mac_measurer_str);

      meas_collect_info->threadRunning = FALSE;
      VbThreadJoin(meas_collect_info->threadId, MEASURE_COLLECT_THREAD_NAME);
      VbLogPrint(VB_LOG_INFO, "Stopped %s thread for node %s!", MEASURE_COLLECT_THREAD_NAME, mac_measurer_str);

      if (meas_collect_info->macsMeasuredList != NULL)
      {
        free(meas_collect_info->macsMeasuredList);
        meas_collect_info->macsMeasuredList = NULL;
      }

      collectInfoList.numThreads--;
    }
  }

  return ret;
}

/*******************************************************************/

void VbMeasureSNRProbeStop(void)
{
  if (vbMeasureSNRProbeThreadRunning == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", SNRPROBE_THREAD_NAME);

    vbMeasureSNRProbeThreadRunning = FALSE;
    VbThreadJoin(vbMeasureSNRProbeThread, SNRPROBE_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", SNRPROBE_THREAD_NAME);
  }

  // Release previously allocated memory
  if (vbMeasureSNRProbeReqCpy.msg != NULL)
  {
    free(vbMeasureSNRProbeReqCpy.msg);
    vbMeasureSNRProbeReqCpy.msg = NULL;
  }
}


/*******************************************************************/

void VbMeasurementStop(void)
{
  VbMeasurePlanStop();
  VbMeasureMeasuresCollectAllStop();
  VbMeasureSNRProbeStop();

  memset((INT8U*)&collectInfoList, 0x00, sizeof(collectInfoList));
}

/*******************************************************************/

t_VB_comErrorCode VbMeasurementCancel(INT8U *measCancelReqMsg)
{
  t_VB_comErrorCode           ret = VB_COM_ERROR_NONE;
  t_vbEAMeasurePlanCancelReq *measure_plan_cancel_req;
  INT8U                       plan_id;
  t_vbEAMeasRspErrorCode      rsp_err_code;

  if(measCancelReqMsg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if(ret == VB_COM_ERROR_NONE)
  {
    measure_plan_cancel_req = (t_vbEAMeasurePlanCancelReq *)measCancelReqMsg;
    plan_id = measure_plan_cancel_req->planID;

    VbLogPrint(VB_LOG_INFO, "Cancel Measure plan %u (%u)", plan_id, vbMeasurePlanId);

    if(plan_id == vbMeasurePlanId)
    {
      ret = VbMeasurementPlanCancel(plan_id);
    }
    else
    {
      ret = VB_COM_ERROR_BAD_PLAN_ID;
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_NONE;
    }
    else if (ret == VB_COM_ERROR_BAD_PLAN_ID)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_PLANID;
    }
    else
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_UNKNOWN;
    }

    // Pass it to the engine
    ret = VbEAMeasurePlanCancelCnfSend(plan_id, rsp_err_code);

  }

  return ret;
}

/*******************************************************************/

BOOLEAN VbMeasurementCollectThreadsRunning(void)
{
  INT32U  i;
  BOOLEAN running = FALSE;

  for (i = 0; i < MEASURE_MAX_THREADS_COLLECTION; i++)
  {
    if (collectInfoList.collectInfo[i].threadRunning == TRUE)
    {
      // At least one thread running
      running = TRUE;
      break;
    }
  }

  return running;
}

/*******************************************************************/

INT8U VBMeasurementPlanIDGet( void )
{
  return vbMeasurePlanId;
}

/*******************************************************************/

t_vbEAMeasRspErrorCode VBMeasurementCollectEndErrorGet(void)
{
  return vbMeasureCollectEndError;
}

/*******************************************************************/

INT8U VBMeasurementPlanIDFromMsgGet(INT8U *msg)
{
  INT8U meas_plan = 0;

  if (msg != NULL)
  {
    meas_plan = msg[1];
  }

  return meas_plan;
}

/*******************************************************************/

/**
 * @}
 **/
