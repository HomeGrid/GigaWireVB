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

#include "ezxml.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define MEASURE_PLAN_ID_OFFSET                (0)
#define MEASURE_PLAN_ID_SIZE                  (1)
#define MEASURE_CONF_SIZE                     (sizeof(t_MeasureConfiguration))

#define MEASURE_SNR_DEV_PLAN_SIZE             (sizeof(t_GeneralMeasureConfiguration))
#define MEASURE_CFR_DEV_PLAN_SIZE             (sizeof(t_MeasureCFRDevice))
#define MEASURE_CFR_CONF_OFFSET               (MEASURE_PLAN_ID_OFFSET + MEASURE_PLAN_ID_SIZE)

#define MEASURE_PLAN_SIZE_WITHOUTDMS(NUMDEVS) (PARAMETER_TYPE_SIZE + MEASURE_PLAN_ID_SIZE +\
                                              (2*MEASURE_CONF_SIZE) + (MEASURE_SNR_DEV_PLAN_SIZE) +\
                                              (MEASURE_CFR_DEV_PLAN_SIZE*NUMDEVS))

#define MEASURE_PLAN_SIZE_WITHDMS(NUMDEVS)    (MEASURE_PLAN_SIZE_WITHOUTDMS(NUMDEVS) + MEASURE_SNR_DEV_PLAN_SIZE)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef struct
{
  INT8U                planId;
  INT16U               endSeqNumber;
  INT16U               initialSeqNumber;
  struct timespec      endTime;
  t_vbMsg              measurePlan;
} t_reqMeasurement;

typedef struct __attribute__ ((packed))
{
  INT8U devID;
  INT16U extSeed;
  INT8U measureDeviceMAC[ETH_ALEN];
} t_MeasureNodeBasicInfo;

typedef struct __attribute__ ((packed))
{
  INT8U measureType;
  INT16U numMeasure;
  INT8U storageType;
  INT8U symbolsNumber;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U time : 4 ;
  INT8U frequency : 4 ;
#else
  INT8U frequency : 4 ;
  INT8U time : 4 ;
#endif
  INT32U offset;
  INT32U duration;
} t_MeasureConfiguration;

typedef struct __attribute__ ((packed))
{
  INT16U startSeqNumber;
  INT16U endSeqNumber;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  INT8U deviceType : 1 ;
  INT8U :7 ;
#else
  INT8U :7 ;
  INT8U deviceType : 1 ;
#endif
} t_GeneralMeasureConfiguration;

typedef struct __attribute__ ((packed))
{
  t_GeneralMeasureConfiguration GeneralMeasureConfiguration;
  t_MeasureNodeBasicInfo nodeBasicInfo;
} t_MeasureCFRDevice;

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
* @brief This function builds the measure plan packet
**/
t_VB_engineErrorCode VbEngineProcessMeasurePlanBuild(INT32U clusterId);

/**
* @brief This function sends to the driver the measure plan
* param[in] thisDriver Pointer to vbDriver data struct
**/
t_VB_engineErrorCode VbEngineProcessMeasurePlanRequestSend(t_VBDriver *thisDriver);

/**
 * @brief Process the response from the driver to the measure plan request
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] driver Pointer to sender driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineMeasureRespProcess(INT8U *payload, INT32U length, t_VBDriver *driver);

/**
 * @brief Process the response from the driver to the measure plan cancel order
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] driver Pointer to sender driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineMeasureCancelRespProcess(INT8U* payload, INT32U length, t_VBDriver *driver);

/**
 * @brief Get the time the measure plan is going to finish
 * @param[in]  clusterId cluster to measure
 * @param[out] timeToFinish time the plan will finish
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessMeasurePlanTimeToFinishGet(INT32U clusterId, INT32U *timeToFinish);

/**
 * @brief Dumps the last measure plan created
 * @param[in] writeFun Callback used to print
 * @return @ref t_VB_engineErrorCode
 **/
void VbEngineMeasureConfDump(t_writeFun writeFun);

/**
 * @brief Parse the measure plan configuration bit in the vb_engine.ini
 * @param[in] measureConf measure configuration to parse
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineMeasureConfigurationParse( ezxml_t measureConf );

/**
 * @brief Notify the driver of an error occurred during the measure plan
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] driver Pointer to sender driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineMeasureErrorNotifyProcess(INT8U* payload, INT32U length, t_VBDriver *driver);

/**
 * @brief Start the measure collection process
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessCollectMeasuresStart(INT32U clusterId);

/**
 * @brief
 * @param[in] planId plan id in consideration
 * @param[in] numMacsMeasurer Number of measurer nodes
 * @param[in] macsMeasurer MACs of the measurer nodes
 * @param[in] numMacsMeasured Number of nodes measured
 * @param[in] macsMeasured MACs of the nodes measured
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineMeasCollectReqSend( INT8U planId, t_macsInfo *dmsMeasurer,
    t_macsInfo *epsMeasurer, t_macsInfo *dmsMeasured, t_macsInfo *epsMeasured, t_VBDriver *thisDriver );

/**
 * @brief Process CFR response
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCFRRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief End of collection measures
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineMeasCollectionEndProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Process background noise response
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineBngNoiseRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Dumps the last measure plan created
 * @param[in] writeFun Callback used to print
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineMeasurePlanDump(t_writeFun writeFun, INT32U clusterId);

/**
* @brief Request the cancelation of the ongoing measure plan
* param[in] this_driver Pointer to vbDriver data struct
**/
t_VB_engineErrorCode VbEngineProcessMeasurePlanCancelRequestSend(t_VBDriver *thisDriver);

/**
 * @brief Get current Measure Plan Id
 **/
t_VB_engineErrorCode VbEngineMeasurePlanIdGet(INT32U clusterId, INT8U *planId);

/**
 * @brief Save measures to disk
 **/
t_VB_engineErrorCode VbEngineMeasureSave(INT32U clusterId);

/**
 * @brief Process background SNR Probe response
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineSnrProbesRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Process PSD measure response
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEnginePSDRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Allocate measurement memory for cluster
 * @param[in] clusterId cluster Id to allocate memory to
 * @param[out] measurementReq measurment request allocated memory
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineMeasurePlanClusterResourcesAlloc(INT32U clusterId, t_reqMeasurement **measurementReq);

/**
 * @brief Free Allocated measurement memory for cluster
 * @param[in] clusterId  cluster Id of related cluster
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineMeasurePlanClusterResourcesFree(INT32U clusterId);


/**
 * @brief Init measure plan related data
 */
t_VB_engineErrorCode VbEngineMeasurePlanInit(void);

/**
 * @}
 **/
