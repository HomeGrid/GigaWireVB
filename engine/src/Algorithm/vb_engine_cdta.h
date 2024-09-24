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
 * @file vb_engine_cdta.h
 * @brief Interface of C-DTA feature
 *
 * @internal
 *
 * @author Y. Raoul
 * @date 2017/08/21
 *
 **/

#ifndef VB_ENGINE_CDTA_H_
#define VB_ENGINE_CDTA_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */
#include "ezxml.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */
#define NUMBANDS 10
#define VB_ENGINE_STXOP_LOSS_EFF                           (7)
#define VB_ENGINE_NUM_HIST_DATA                            (75)
/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef enum
{
  VB_ENGINE_BPS_BAND_NOT_AVAILABLE = 0,
  VB_ENGINE_BPS_BAND_VALID,
  VB_ENGINE_BPS_BAND_INVALID,
}t_vbEngineBpsBandStatus;


typedef struct s_cdtaHistData
{
  struct timespec   timeStamp;
  INT32U            metrics[VB_ENGINE_QOS_RATE_LAST];
  INT8U             rate;
}t_cdtaHistData;

typedef struct s_cdtaHist
{
  pthread_mutex_t    mutex;
  INT32U             wPointer;
  t_cdtaHistData     data[VB_ENGINE_NUM_HIST_DATA];
}t_cdtaHist;

typedef struct s_cdtaConf
{
  BOOLEAN enabled;
  INT8U   downUpWeight;
  INT8U   minDownUpRate;
  INT8U   maxDownUpRate;
  INT8U   defaultDownUpRate;
  INT8U   percMetricChange;
}t_cdtaConf;

typedef struct s_cdtaStats
{
  INT16U  nDowns;
  INT16U  nUps;
  INT32U  aggregatedDowns;
  INT32U  maxDown;
  INT32U  aggregatedUps;
  INT32U  maxUp;
  INT32U  bpsValidBandsBitmap;
  INT8U   nBoostedDownLines[NUMBANDS][VB_ENGINE_QOS_RATE_LAST];
  INT8U   nBoostedUpLines[NUMBANDS][VB_ENGINE_QOS_RATE_LAST];
}t_cdtaStats;

typedef struct s_cdtaMetrics
{
  INT32U      rateMetricsDown[VB_ENGINE_QOS_RATE_LAST];
  INT32U      rateMetricsUp[VB_ENGINE_QOS_RATE_LAST];
  INT32U      rateMetrics[VB_ENGINE_QOS_RATE_LAST];
  INT32U      sumWeightsDown[VB_ENGINE_QOS_RATE_LAST];
  INT32U      sumWeightsUp[VB_ENGINE_QOS_RATE_LAST];
}t_cdtaMetrics;

typedef struct s_cdtaWeights
{
  INT32U            downPref;
  INT32U            upPref;
  INT32U            downUpMultFactor; ///< 2 ^ val
}t_cdtaWeights;

typedef struct s_clusterCdtaInfo
{
  t_cdtaStats       stats;
  t_cdtaMetrics     metrics;
  t_cdtaHist        hist;
  INT32U            selectedMetrics;
  INT8U             finalQosRate;
}t_clusterCdtaInfo;

typedef struct s_cdtaGlobalConf
{
  t_cdtaWeights     weights;
  INT32U            minQosRate;
  INT32U            maxQosRate;
  INT8U             defaultQosRate;
  INT8U             percMetricChange;
}t_cdtaGlobalConf;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */


/**
 * @brief Get current Qos Rate set
 * @param[in] clusterId cluster Id of requested qos rate
 * @return @ref t_VB_engineErrorCode
 **/
t_vbEngineQosRate VbCdtaQosRateGet(INT32U clusterId);

/**
 * @brief Get current Qos Rate set
 * @param[in] clusterId cluster Id of qos rate to be set
 * @param[in] qosRate qos rate to be set
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbCdtaQosRateForce(INT32U clusterId, INT8U qosRate);

/**
 * @brief Get default Qos Rate set
 * @return @ref t_VB_engineErrorCode
 **/
t_vbEngineQosRate VbCdtaDefaultQosRateGet(void);

/**
 * @brief Dump CDTA global info
 * @param[out] writeFun function to dump info
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbCdtaGlobalInfoDump(t_writeFun writeFun, INT32U clusterId);

/**
 * @brief Dump CDTA hist global info
 * @param[out] writeFun function to dump info
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbCdtaGlobalHistInfoDump(t_writeFun writeFun, INT32U clusterId);

/**
 * @brief Parse CDTA info in vb_engine.ini file
 * @param[in] cdta_conf processed cdta info fom ini file
 * @return @ref t_VB_engineErrorCodee
 **/
t_VB_engineErrorCode VbEngineCDTAConfigurationParse( ezxml_t cdta_conf );

/**
 * @brief Run CDTA analysis
 * @param[out] qosRate qosRate to be applied
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbCdtaAnalyseRun(t_vbEngineQosRate *qosRate, INT32U clusterId);

/**
 * @brief Build CDTA message
 * @param[out] timeInfo time info of when the changes should be completed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCDTAConfigureAll(BOOLEAN forceChannelEstimation, t_vbEngineQosRate nextQosRate, INT32U clusterId);

/**
 * @brief Process CDTA response message
 * @param[in] payload payload of resp message
 * @param[in] length length of resp message
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCdtaRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Set CDTA weight Down Up multiplication factor
 * @param[in] multFactor down up mult factor
 **/
void VbCdtaWeightDownUpMultFactorSet(INT16U multFactor);

/**
 * @brief Update BPS info from traffic report
 * @param[in] node node related info
 * @param[in] trafficReport traffic report info
 * @param[in] nodeType node type
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineCdtaReportedBpsUpdate(t_node *node, t_trafficReport *trafficReport, t_nodeType nodeType);

/**
 * @brief Alloc CDTA resources related to a specific cluster
 * @param[in] clusterId cluster for which memory has to be allocated
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCdtaClusterResourcesAlloc(INT32U clusterId);

/**
 * @brief Free CDTA resources related to a specific cluster
 * @param[in] clusterId cluster for which memory has to be freed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCdtaFreeClusterResources(INT32U clusterId);


#endif /* VB_ENGINE_CDTA_H_ */

/**
 * @}
**/
