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
 * @file vb_engine_clusters_list.c
 * @brief Implements cluster list functionality
 *
 * @internal
 *
 * @author
 * @date 29/05/2017
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

#include "vb_engine_cluster_list.h"
#include "vb_engine_drivers_list.h"
#include "vb_linked_list.h"
#include "vb_engine_conf.h"
#include "vb_log.h"

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

static t_VBClustersList vbEngineDatamodelClustersList = {0, 0, NULL};
static pthread_mutex_t vbEngineDatamodelClustersListMutex = PTHREAD_MUTEX_INITIALIZER;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/*******************************************************************/


/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/*******************************************************************/

static t_VB_engineErrorCode ClustersLoop(t_clusterLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster = NULL;
  t_linkedElement     *elem;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

    if (vbEngineDatamodelClustersList.vbClustersArray != NULL)
    {
      for (elem = (t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray;
          (elem != NULL) && (ret == VB_ENGINE_ERROR_NONE); (elem) = (elem)->next)
      {
        cluster = (t_VBCluster *)elem;

        if (cluster != NULL)
        {
          ret = loopCb(cluster, args);
        }
      }
    }

    pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineMaxRespTimePerDriverCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if((driver == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U max_resp_time = *((INT32U *)args);

    if(driver->transactionMinTime > max_resp_time)
    {
      *((INT32U *)args) = driver->transactionMinTime;
    }
  }

  return ret;
}

/*******************************************************************/

void VbEngineCltListClustersDestroy(void)
{
  t_VBCluster *this_cluster;
  t_linkedElement *elem;

  pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

  if(vbEngineDatamodelClustersList.vbClustersArray != NULL)
  {
    LIST_FOREACH((t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray, elem)
    {
      this_cluster = (t_VBCluster *)elem;
      VbEngineDatamodelClusterMemFree(this_cluster);
    }

    ClearList((t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray);
    vbEngineDatamodelClustersList.vbClustersArray = NULL;
  }

  vbEngineDatamodelClustersList.numVBClusters = 0;

  pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );

}


/*******************************************************************/

void VbEngineCltListClustersNonZeroDestroy(void)
{
  t_VBCluster *this_cluster;
  t_linkedElement *elem;

  pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

  if(vbEngineDatamodelClustersList.vbClustersArray != NULL)
  {
    LIST_FOREACH((t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray, elem)
    {
      this_cluster = (t_VBCluster *)elem;
      if(this_cluster->clusterInfo.clusterId != 0)
      {
        RemoveElement((t_linkedElement **)&vbEngineDatamodelClustersList.vbClustersArray, (t_linkedElement *)this_cluster);
        VbEngineDatamodelClusterDel(&this_cluster);
        // Go back to first element of the list (The current element have been removed)
        elem = (t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray;
      }
    }
  }

  vbEngineDatamodelClustersList.numVBClusters = 1;

  pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );

}
/*******************************************************************/

t_VB_engineErrorCode VbEngineCltListClusterAdd(t_VBCluster *vbCluster)
{
  t_VB_engineErrorCode vb_engine_error_code = VB_ENGINE_ERROR_NONE;

  if (vbCluster == NULL)
  {
    vb_engine_error_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_engine_error_code == VB_ENGINE_ERROR_NONE)
  {
     pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

     //Add to the list
     AppendElement((t_linkedElement **)&vbEngineDatamodelClustersList.vbClustersArray, (t_linkedElement *) vbCluster);
     vbEngineDatamodelClustersList.numVBClusters++;

     VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "ADD New cluster in list, numClusters %d", vbEngineDatamodelClustersList.numVBClusters);

     pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );
  }

  return vb_engine_error_code;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineCltListRemoveCluster(t_VBCluster *cluster)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (cluster == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

    if (vbEngineDatamodelClustersList.vbClustersArray != NULL)
    {
      RemoveElement((t_linkedElement **)&vbEngineDatamodelClustersList.vbClustersArray, (t_linkedElement *)cluster);
      vbEngineDatamodelClustersList.numVBClusters--;
    }

    pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );
  }

  return ret;
}

/*******************************************************************/

INT16U VbEngineDataModelNumClustersGet(void)
{
  INT16U num_clusters = 0;
  t_linkedElement *elem;

  pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

  LIST_COUNT((t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray, elem, num_clusters);

  pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );

  return num_clusters;
}

/*******************************************************************/

INT32U VbEngineDataModelNextClusterIdGet(void)
{
  t_VBAlignmentMode  alignment_mode;
  pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

  VbEngineConfAlignmentModeGet(&alignment_mode);
  if(alignment_mode == VB_ALIGN_MODE_GHN)
  {
    vbEngineDatamodelClustersList.nextCLusterId++;
    if(vbEngineDatamodelClustersList.nextCLusterId == 0)
    {
      vbEngineDatamodelClustersList.nextCLusterId++;
    }
  }
  else
  {
    vbEngineDatamodelClustersList.nextCLusterId = 1;
  }

  pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );

  return vbEngineDatamodelClustersList.nextCLusterId;
}

/*******************************************************************/

INT32U VbEngineDataModelBiggestClusterIdGet(void)
{
  t_VBCluster *this_cluster;
  t_linkedElement *elem;
  INT32U max_num_lines = 0;
  INT32U biggest_cluster_id = 0;

  pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

  if(vbEngineDatamodelClustersList.vbClustersArray != NULL)
  {
    LIST_FOREACH((t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray, elem)
    {
      this_cluster = (t_VBCluster *)elem;
      if(this_cluster->clusterInfo.clusterId != 0)
      {
        if(max_num_lines < this_cluster->clusterInfo.numLines)
        {
          biggest_cluster_id = this_cluster->clusterInfo.clusterId;
          max_num_lines = this_cluster->clusterInfo.numLines;
        }
        else if ( (max_num_lines == this_cluster->clusterInfo.numLines) &&
                  (biggest_cluster_id > this_cluster->clusterInfo.clusterId)
                )
        {
          biggest_cluster_id = this_cluster->clusterInfo.clusterId;
        }
      }
    }
  }

  pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );

  return biggest_cluster_id;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClusterByIdGet(INT32U clusterId, t_VBCluster **cluster)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster_ptr = NULL;
  t_linkedElement     *elem;

  if(cluster == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VB_ENGINE_ERROR_NOT_FOUND;

    pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

    if (vbEngineDatamodelClustersList.vbClustersArray != NULL)
    {
      LIST_FOREACH((t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray, elem)
      {
        cluster_ptr = (t_VBCluster *)elem;

        if (clusterId == cluster_ptr->clusterInfo.clusterId)
        {
          // Cluster found
          ret = VB_ENGINE_ERROR_NONE;
          *cluster = cluster_ptr;
          break;
        }
      }
    }

    pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClusterStopTimers(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster          *cluster_ptr = NULL;
  t_linkedElement     *elem;

  pthread_mutex_lock( &vbEngineDatamodelClustersListMutex );

  if (vbEngineDatamodelClustersList.vbClustersArray != NULL)
  {
    LIST_FOREACH((t_linkedElement *)vbEngineDatamodelClustersList.vbClustersArray, elem)
    {
      cluster_ptr = (t_VBCluster *)elem;
      ret = VbEngineTimeoutStop(&cluster_ptr->timeoutCnf);
    }
  }

  pthread_mutex_unlock( &vbEngineDatamodelClustersListMutex );

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClusterTimeApplyUpdate(INT32U clusterId, INT16U seqNum, struct timespec applyOwnTS, struct timespec applyDrvTS)
{
  t_VB_engineErrorCode ret;
  t_VBCluster         *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    cluster->applySeqNum = seqNum;
    cluster->applyOwnTs = applyOwnTS;
    cluster->applyDrvTs = applyDrvTS;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClusterSyncLostSet(INT32U clusterId, BOOLEAN flag)
{
  t_VB_engineErrorCode ret;
  t_VBCluster         *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    cluster->syncLostFlag = flag;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClusterSyncLostGet(INT32U clusterId, BOOLEAN *flag)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  if(flag == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      *flag = cluster->syncLostFlag;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClusterTimeApplyDrvGet(INT32U clusterId, struct timespec *applyTS)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBCluster         *cluster;

  if(applyTS == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineClusterByIdGet(clusterId, &cluster);
    if(ret == VB_ENGINE_ERROR_NONE)
    {
      *applyTS = cluster->applyDrvTs;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClusterMinTimeDriverGet(INT32U clusterId, INT32U *maxRespTime)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if(maxRespTime == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    *maxRespTime = 0;
    ret = VbEngineDatamodelClusterXDriversLoop(VbEngineMaxRespTimePerDriverCb, clusterId, (void*)maxRespTime);
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClustersLoop(t_clusterLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = ClustersLoop(loopCb, args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}


/**
 * @}
 **/
