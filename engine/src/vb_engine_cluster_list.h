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
 * @file vb_engine_clusters_list.h
 * @brief Implements clusters list functionality
 *
 * @internal
 *
 * @author
 * @date 29/05/2017
 *
 **/

#include "vb_engine_datamodel.h"

#ifndef VB_ENGINE_CLUSTERS_LIST_H_
#define VB_ENGINE_CLUSTERS_LIST_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */


/*
 ************************************************************************
 ** Public Typedefs
 ************************************************************************
 */

typedef struct s_VBClustersList{
  INT16U numVBClusters;
  INT16U nextCLusterId;
  t_VBCluster *vbClustersArray;
} t_VBClustersList;


typedef t_VB_engineErrorCode (*t_clusterLoopCb)(t_VBCluster *cluster, void *args);

/*
 ************************************************************************
 ** Public functions
 ************************************************************************
 */

/**
 * @brief This function destroys the engine clusters list component and free memory
**/
void VbEngineCltListClustersDestroy(void);

/**
 * @brief This function destroys the engine clusters list component and free memory
**/
void VbEngineCltListClustersNonZeroDestroy(void);

/**
 * @brief Creates a new t_VBCluster struct and appends it to the cluster list
 * @param[in] cluster
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCltListClusterAdd(t_VBCluster *cluster);

/**
 * @brief Removes the cluster from the clusters list and frees the memory
 * @param[in] cluster cluster to be removed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCltListClusterRemove(t_VBCluster *cluster);


/**
 * @brief Returns the number of clusters in the clusters list
 * @return number of clusters
 **/
INT16U VbEngineDataModelNumClustersGet(void);

/**
 * @brief Returns the Id of the next cluster Id to be created
 * @return new cluster Id to be applied
 **/
INT32U VbEngineDataModelNextClusterIdGet(void);

/**
 * @brief Returns the cluster Id of the biggest cluster in the clusters list
 * @return Cluster Id of the biggest cluster
 **/
INT32U VbEngineDataModelBiggestClusterIdGet(void);

/**
 * @brief Returns the cluster info of cluster Id passed as parameter
 * @param[in] clusterId cluster Id
 * @param[in] cluster cluster information retrieved
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClusterByIdGet(INT32U clusterId, t_VBCluster **cluster);

/**
 * @brief Remove cluster data from clusters list
 * @param[in] cluster cluster information  to be removed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineCltListRemoveCluster(t_VBCluster *cluster);

/**
 * @brief Loop on each cluster and execute callback
 * @param[in] loopCb callback to be executed for each cluster
 * @param[in] args parameters to be passed to callback
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDatamodelClustersLoop(t_clusterLoopCb loopCb, void *args);

/**
 * @brief Loop on each driver in clusterId to get the max min time from the drivers
 * @param[in] clusterId cluster Id to be looked at
 * @param[out] maxRespTime maximum time out time of the drivers
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClusterMinTimeDriverGet(INT32U clusterId, INT32U *maxRespTime);

/**
 * @brief Stop timers of all clusters
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClusterStopTimers(void);

/**
 * @brief Save the time related information of cluster Id
 * @param[in] clusterId cluster Id of related cluster
 * @param[in] seqNum    sequence number
 * @param[in] applyOwnTS Time Stamp of engine
 * @param[in] applyDrvTS Time Stamp of driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClusterTimeApplyUpdate(INT32U clusterId, INT16U seqNum, struct timespec applyOwnTS, struct timespec applyDrvTS);

/**
 * @brief Save the time related information of cluster Id
 * @param[in] clusterId cluster Id of related cluster
 * @param[in] flag value to set the sync lost flag with
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClusterSyncLostSet(INT32U clusterId, BOOLEAN flag);

/**
 * @brief Save the time related information of cluster Id
 * @param[in] clusterId cluster Id of related cluster
 * @param[in] flag pointer to wirte the sync lost status
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClusterSyncLostGet(INT32U clusterId, BOOLEAN *flag);

/**
 * @brief Get driver time related information of cluster Id
 * @param[in] clusterId cluster Id of related cluster
 * @param[out] applyTS time satmp of driver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClusterTimeApplyDrvGet(INT32U clusterId, struct timespec *applyTS);

#endif /* VB_ENGINE_CLUSTERS_LIST_H_ */
/**
 * @}
 **/
