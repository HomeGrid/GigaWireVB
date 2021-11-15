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
 * @file vb_engine_alignment.h
 * @brief VB Drivers alignment
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-10
 *
 **/

#ifndef VB_ENGINE_ALIGNMENT_H_
#define VB_ENGINE_ALIGNMENT_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_engine_datamodel.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef struct
{
  t_domain           *txNodes[VB_EA_ALIGN_GHN_MAX_TX_NODES];
  t_domain           *relays[VB_EA_ALIGN_GHN_MAX_RELAYS];
  t_domain           *relaysCandidate[VB_EA_ALIGN_GHN_MAX_RELAYS];
  t_domain           *ref;
  INT32U              numRelays;
  INT32U              numRelaysCandidate;
  INT32U              numTxNodes;
} t_vbAlignGhnTxNodesInfo;

typedef struct s_vbAlignClusterBuildInfo
{
  t_vbAlignGhnTxNodesInfo txNodesInfo;
  INT32U                  clusterId;
  INT32U                  numLinesInCluster;
  INT32U                  numLinesSyncedInCluster;
  INT32U                  numLinesSyncedExclusivelyWithRelayCandidate;
}t_vbAlignClusterBuildInfo;

typedef enum
{
  VB_ENGINE_ALIGNMENT_NEXT_NONE    = 0,
  VB_ENGINE_ALIGNMENT_NEXT_DONE_EVT    = 1,
  VB_ENGINE_ALIGNMENT_NEXT_RESTART_EVT = 2
}t_vbAlignNextStep;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Launch a EA_CycQuery.req message to all drivers to request alignment information
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineAlignCycQueryRequest(INT32U clusterId);

/**
 * @brief Parses the EA_CycQuery.rsp message from driver and gets the error code
 * @param[in] payload Pointer to payload
 * @param[in] errCod Pointer to error code
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineAlignCycQueryRspErrGet(INT8U *payload, t_vbEACycQueryRspErrorCode *errCod, INT32U clusterId);

/**
 * @brief Parses the EA_CycQuery.rsp message from driver and stores alignment information
 * @param[in] payload Pointer to payload
 * @param[in] length Length in bytes of the message
 * @param[in] driver Driver to store the alignment information
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineAlignCycQueryRspProcess(INT8U* payload, INT32U length, t_VBDriver *driver);

/**
 * @brief Parses the EA_AlignSyncLost.trg message from driver
 * @param[in] payload Pointer to payload
 * @param[in] length Length in bytes of the message
 * @param[in] driver Driver to store the alignment information
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineAlignSyncLostTrgProcess(INT8U* payload, INT32U length, t_VBDriver *driver);

/**
 * @brief Parses the EA_CycChange.rsp message from driver and gets the error code
 * @param[in] payload Pointer to payload
 * @param[in] errCod Pointer to error code
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineAlignCycChangeRspErrGet(INT8U *payload, t_vbEACycChangeRspErrorCode *errCod);

/**
 * @brief Parses the EA_AlignMode.rsp message from driver and gets the error code
 * @param[in] payload Pointer to payload
 * @param[in] errCod Pointer to error code
 * @param[in] driver pointer to driver sending the response
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineAlignModeRspErrGet(INT8U *payload, t_vbEAAlignModeRspErrorCode *errCod, t_VBDriver *driver);

/**
 * @brief Parses the EA_ClusterStop.rsp message from driver and gets the error code
 * @param[in] payload Pointer to payload
 * @param[in] length  length of payload
 * @param[in] driver pointer to driver sending the response
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEngineAlignClusterStopRspProcess(INT8U* payload, INT32U length, t_VBDriver *driver);

/**
 * @brief Loops through all drivers and check the correct alignment
 * @return TRUE: all drivers are aligned as expected; FALSE: otherwise
 **/
BOOLEAN VbEngineAlignCheck(INT32U clusterId);

/**
 * @brief Loops through all drivers and chooses the best MAC Clock and Sequence numbers.
 * They will be used as reference to align the rest of domains.
 * @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignRefCalc(INT32U clusterId);

/**
 * @brief Gets the last reference MAC Clock and Sequence numbers
 * @return Alignment info to use as reference
 **/
t_alignInfo VbEngineAlignRefGet(void);

/**
 * @brief Builds and send a EA_CycChange.req message to align domains in given driver
 * @param[in] driver Pointer to driver to align
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignCycChangeSend(t_VBDriver *driver);

/**
 * @brief Notifies that alignment process has finished ok and stores a timestamp
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignDoneNotify(INT32U clusterId);

/**
 * @brief Forces an alignment check
 * @param[in] clusterId Cluster Id
 **/
void VbEngineAlignPeriodicCheckForce(INT32U clusterId);

/**
 * @brief Shows if a new alignment check is needed
 * @param[in] lastCheckTS Last alignment check timestamp
 * @param[in] clusterId Cluster Id
 * @return TRUE: a new alignment check is needed; FALSE: otherwise
 **/
BOOLEAN VbEngineAlignPeriodicCheckNeeded(struct timespec *lastCheckTS, INT32U clusterId);

/**
 * @brief Gets last alignment check timestamp
 * @param[out] lastCheckTS Last alignment check timestamp
 * @param[in]  clusterId cluster Id
 *
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignLastCheckTSGet(struct timespec *lastCheckTS, INT32U clusterId);

/**
 * @brief Builds and send a EA_EngineConf.req message to configure the remote drivers
 * @param[in] driver Pointer to driver to align
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEngineConfSend(t_VBDriver *driver);

/**
 * @brief Tries next node as reference or relay node
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignGhnNextTry(t_vbAlignNextStep *next, t_vbAlignClusterBuildInfo  *clusterBuildInfo, INT32U clusterId);

/**
 * @brief Builds the engine configuration message to select alignment method, inform default Qos Rate, PSD to apply at boot up
 * @param[in] clusterId Cluster Id
 * @param[in] forceRelayCandidates TRUE: all relays are moved to relay candidates
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEngineConfReqBuild(INT32U clusterId, BOOLEAN forceRelayCandidates);

/**
 * @brief Gets the name of given alignment role
 * @param[in] role Alignment role
 * @return Role name
 **/
const CHAR *VbEngineAlignRoleStringGet(t_alignRole role);

/**
 * @brief Gets the name of given alignment mode
 * @param[in] mode Alignment mode
 * @return Alignment mode name
 **/
const CHAR *VbEngineAlignModeStringGet(t_VBAlignmentMode mode);

/**
 * @brief Gets ADC out RMS level in floating point format
 * @param[in] adcOutRms ADC level in HW format
 * @return ADC out RMS level (in dB)
 **/
FP32 VbEngineAlignAdcOutToFPGet(INT32U adcOutRms);

/**
 * @brief Gets ADC out RMS level
 * @param[in] domain Pointer to domain structure
 * @return ADC out RMS level (in dB) or 0 if adc out level not reported
 **/
FP32 VbEngineAlignAdcOutRmsGet(t_domain *domain);

/**
 * @brief Add cluster to cluster list
 * @param[in]  currentClusterId current cluster Id
 * @param[out] newClusterId new cluster Id created
 * @param[in] clusterBuildInfo cluster info
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignClusterAdd(INT32U currentClusterId, INT32U *newClusterId, t_vbAlignClusterBuildInfo *clusterBuildInfo);

/**
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineOldClusterAdd(INT32U oldClusterId);

/**
 * @brief Update cluster info
 * @param[in] clusterId cluster Id of cluster to be updated
 * @param[in] clusterBuildInfo pointer onto cluster info to be updated
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignClusterUpdate(INT32U clusterId, t_vbAlignClusterBuildInfo *clusterBuildInfo);

/**
 * @brief Remove existing cluster
 * @param[in] clusterId cluster Id of cluster to be removed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignClusterRemove(INT32U clusterId);

/**
 * @brief Tag all drivers with cluster Id passed as parameter
 * @param[in] clusterId cluster Id of drivers to be tagged with
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignAllDriversClusterTag(INT32U clusterId);

/**
 * @brief Return whether this cluster shall be stopped or continue (based on its number of lines)
 * @param[in] clusterId cluster Id of cluster
 * @param[out] stopTx pointer on stop variable telling if transmission in the cluster must be stopped or not
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignClusterStopGet(INT32U clusterId, BOOL *stopTx);

/**
 * @brief Send cluster stop message to driver
 * @param[in] driver pointer to driver to be stopped
 * @param[in] stopDriverTx flag to indicate f
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignStopClusterSend(t_VBDriver *driver, BOOLEAN stopTx);

/**
 * @brief Tag synced lines in cluster X with cluster Y
 * @param[in] currentClusterId current cluster Id
 * @param[in] newClusterId     new cluster to be applied onto the current cluster Id
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignClusterXSyncedTagY(INT32U currentClusterId, INT32U newClusterId);

/**
 * @brief Get alignment info of cluster Id
 * @param[out] clusterInfoBuild
 * @param[in]  clusterId Cluster Id of align info to be retrieved
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignClusterAlignInfoGet(t_vbAlignClusterBuildInfo *clusterInfoBuild, INT32U clusterId);

/**
 * @brief Clear roles of nodes in cluster
 * @param[in] clusterId Cluster Id to be reset
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignClusterRoleReset(INT32U clusterId);

/**
 * @brief Add cluster 0 to cluster list
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignCluster0Add(void);

/**
 * @brief Release seed resources
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignReleaseResources(void);

/**
 * @brief Get best detected Did
 * @param[in] domain Pointer to domain structure
 * @return @ref t_VB_engineErrorCode
 **/
INT8U VbEngineAlignGhnBestDetectedDidGet(t_domain *domain);

/**
 * @brief Get power of best dectected Did
 * @param[in] domain Pointer to domain structure
 * @return @ref t_VB_engineErrorCode
 **/
INT32U VbEngineAlignGhnBestDetectedPowGet(t_domain *domain);

/**
 * @brief Get align Id of last engine conf message sent
 * @param[in] clusterId cluster Id related to the engine conf
 * @param[in] cluster Pointer to cluster structure. If this argument is != NULL, then clusterId is not used.
 * @param[out] alignId alignId of last engine conf sent
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignmentIdGet(INT32U clusterId, t_VBCluster *cluster, INT8U *alignId);

/**
 * @brief Reset hasBeenCandidate flag of all nodes in cluster
 * @param[in] clusterId Cluster Id
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignmentHasBeenCandidateReset(INT32U clusterId);

/**
 * @brief Tag specified driver with different cluster id
 * @param[in] driver Pointer to driver structure.
 * @param[in] clusterId Cluster Id
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignDriverXSyncedTagY(t_VBDriver *driver, INT32U clusterId);

#endif /* VB_ENGINE_ALIGNMENT_H_ */

/**
 * @}
**/
