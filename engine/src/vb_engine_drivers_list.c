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
 * @file vb_engine_drivers_list.c
 * @brief Implements driver list functionality
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

#include "vb_engine_drivers_list.h"
#include "vb_linked_list.h"
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

typedef struct s_loopAllDomains
{
  t_domainLoopCb      domainsCb;
  void               *domainsArgs;
} t_loopAllDomains;

typedef struct s_loopAllNodes
{
  t_nodeLoopCb        callback;
  void               *args;
} t_loopAllNodes;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_VBDriversList vbEngineDatamodelDriversList = {0, NULL};
static pthread_mutex_t vbEngineDatamodelDriversListMutex = PTHREAD_MUTEX_INITIALIZER;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/*******************************************************************/

static t_VB_engineErrorCode NodesLoop(t_VBDriver *driver, t_nodeLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               domain_idx = 0;
  INT32U               ep_idx = 0;
  t_domain            *domain = NULL;

  if ((driver == NULL) || (loopCb == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&(driver->domainsMutex));

    if (driver->domainsList.domainsArray != NULL)
    {
      for (domain_idx = 0; (domain_idx < driver->domainsList.numDomains) && (ret == VB_ENGINE_ERROR_NONE); domain_idx++)
      {
        domain = &(driver->domainsList.domainsArray[domain_idx]);

        if (domain != NULL)
        {
          // Call callback for DM
          ret = loopCb(driver, domain, &(domain->dm), args);

          if ((ret == VB_ENGINE_ERROR_NONE) && (domain->eps.epsArray != NULL))
          {
            for (ep_idx = 0; (ep_idx < domain->eps.numEPs) && (ret == VB_ENGINE_ERROR_NONE); ep_idx++)
            {
              // Call callback for EPs
              ret = loopCb(driver, domain, &(domain->eps.epsArray[ep_idx]), args);
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

static t_VB_engineErrorCode DomainsLoop(t_VBDriver *driver, t_domainLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               domain_idx = 0;
  t_domain            *domain = NULL;

  if ((driver == NULL) || (loopCb == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&(driver->domainsMutex));

    if (driver->domainsList.domainsArray != NULL)
    {
      for (domain_idx = 0; (domain_idx < driver->domainsList.numDomains) && (ret == VB_ENGINE_ERROR_NONE); domain_idx++)
      {
        domain = &(driver->domainsList.domainsArray[domain_idx]);

        if (domain != NULL)
        {
          ret = loopCb(driver, domain, args);
        }
      }
    }

    pthread_mutex_unlock(&(driver->domainsMutex));
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode DriversLoop(t_driverLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBDriver          *driver = NULL;
  t_linkedElement     *elem;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );

    if (vbEngineDatamodelDriversList.vbDriversArray != NULL)
    {
      for (elem = (t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray;
          (elem != NULL) && (ret == VB_ENGINE_ERROR_NONE); (elem) = (elem)->next)
      {
        driver = (t_VBDriver *)elem;

        if (driver != NULL)
        {
          ret = loopCb(driver, args);
        }
      }
    }

    pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClusterXLoop(t_driverLoopCb loopCb, INT32U clusterId, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBDriver          *driver = NULL;
  t_linkedElement     *elem;
  BOOL                 found = FALSE;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );

    if (vbEngineDatamodelDriversList.vbDriversArray != NULL)
    {
      for (elem = (t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray;
          (elem != NULL) && (ret == VB_ENGINE_ERROR_NONE); (elem) = (elem)->next)
      {
        driver = (t_VBDriver *)elem;
        if ((driver != NULL) && (driver->clusterId == clusterId))
        {
          ret = loopCb(driver, args);
          found = TRUE;
        }
      }
    }

    pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );
  }

  if((found == FALSE) && (clusterId > 0))
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Cluster %d not found", clusterId);
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AllDomainsLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_loopAllDomains    *loop_args = (t_loopAllDomains *)args;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = DomainsLoop(driver, loop_args->domainsCb, loop_args->domainsArgs);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode AllNodesLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_loopAllNodes      *loop_args = (t_loopAllNodes *)args;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = NodesLoop(driver, loop_args->callback, loop_args->args);
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

void VbEngineDrvListDriversDestroy(void)
{
  t_VBDriver *this_driver;
  t_linkedElement *elem;

  pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );

  if(vbEngineDatamodelDriversList.vbDriversArray != NULL)
  {
    LIST_FOREACH((t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray, elem)
    {
      this_driver = (t_VBDriver *)elem;
      VbEngineDatamodelDriverMemRelease(this_driver);
    }

    ClearList((t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray);
    vbEngineDatamodelDriversList.vbDriversArray = NULL;
  }

  vbEngineDatamodelDriversList.numVBDrivers = 0;

  pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );

}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDrvListDriverAdd(t_VBDriver *vbDriver)
{
  t_VB_engineErrorCode vb_engine_error_code = VB_ENGINE_ERROR_NONE;

  if (vbDriver == NULL)
  {
    vb_engine_error_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_engine_error_code == VB_ENGINE_ERROR_NONE)
  {
     VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "ADD New driver");

     pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );
     //Add to the list
     AppendElement((t_linkedElement **)&vbEngineDatamodelDriversList.vbDriversArray, (t_linkedElement *) vbDriver);

     pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );
  }

  return vb_engine_error_code;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDrvListRemoveDriver(t_VBDriver *driver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );

    if (vbEngineDatamodelDriversList.vbDriversArray != NULL)
    {
      RemoveElement((t_linkedElement **)&vbEngineDatamodelDriversList.vbDriversArray, (t_linkedElement *)driver);
    }

    pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );
  }

  return ret;
}


/*******************************************************************/

INT16U VbEngineDataModelNumDriversGet(void)
{
  INT16U num_drivers = 0;
  t_linkedElement *elem;

  pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );

  LIST_COUNT((t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray, elem, num_drivers);

  pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );

  return num_drivers;
}

/*******************************************************************/

INT16U VbEngineDataModelNumDriversInCLusterXGet(INT32U clusterId)
{
  INT16U num_drivers = 0;
  t_VBDriver *this_driver;
  t_linkedElement *elem;

  pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );

  LIST_FOREACH((t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray, elem)
  {
    this_driver = (t_VBDriver *)elem;
    if(this_driver->clusterId == clusterId)
    {
      num_drivers++;
    }
  }

  pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );

  return num_drivers;
}


/*******************************************************************/

INT16U VbEngineDataModelNumDomainsDriverGet(INT16U numDriver)
{
  INT16U num_domains = 0;
  t_linkedElement *elem;
  t_VBDriver *driver;

  pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );
  if(vbEngineDatamodelDriversList.vbDriversArray != NULL)
  {
    if(GetElementByIndex((t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray, &elem, numDriver) == TRUE)
    {
      driver = (t_VBDriver *)elem;

      pthread_mutex_lock(&(driver->domainsMutex));
      num_domains = driver->domainsList.numDomains;
      pthread_mutex_unlock(&(driver->domainsMutex));
    }
  }
  pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );

  return num_domains;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDriverByIdGet(char *driverId, t_VBDriver **driver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_VBDriver          *driver_ptr = NULL;
  t_linkedElement     *elem;

  if ((driver == NULL) || (driverId == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VB_ENGINE_ERROR_NOT_FOUND;

    pthread_mutex_lock( &vbEngineDatamodelDriversListMutex );

    if (vbEngineDatamodelDriversList.vbDriversArray != NULL)
    {
      LIST_FOREACH((t_linkedElement *)vbEngineDatamodelDriversList.vbDriversArray, elem)
      {
        driver_ptr = (t_VBDriver *)elem;

        if (!strcmp(driverId, driver_ptr->vbDriverID))
        {
          // Driver found
          ret = VB_ENGINE_ERROR_NONE;
          *driver = driver_ptr;
          break;
        }
      }
    }

    pthread_mutex_unlock( &vbEngineDatamodelDriversListMutex );
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelDriversLoop(t_driverLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = DriversLoop(loopCb, args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClusterXDriversLoop(t_driverLoopCb loopCb, INT32U clusterId, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = ClusterXLoop(loopCb, clusterId, args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}


/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelDomainsLoop(t_VBDriver *driver, t_domainLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (loopCb == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = DomainsLoop(driver, loopCb, args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelAllDomainsLoop(t_domainLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_loopAllDomains     loop_args;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args.domainsArgs = args;
    loop_args.domainsCb = loopCb;

    ret = DriversLoop(AllDomainsLoopCb, &loop_args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClusterXAllDomainsLoop(t_domainLoopCb loopCb, INT32U clusterId, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_loopAllDomains     loop_args;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args.domainsArgs = args;
    loop_args.domainsCb = loopCb;

    ret = ClusterXLoop(AllDomainsLoopCb, clusterId, &loop_args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelNodesLoop(t_VBDriver *driver, t_nodeLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (loopCb == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = NodesLoop(driver, loopCb, args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelAllNodesLoop(t_nodeLoopCb loopCb, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_loopAllNodes       loop_args;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args.args     = args;
    loop_args.callback = loopCb;

    ret = DriversLoop(AllNodesLoopCb, &loop_args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineDatamodelClusterXAllNodesLoop(t_nodeLoopCb loopCb, INT32U clusterId, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_loopAllNodes       loop_args;

  if (loopCb == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    loop_args.args     = args;
    loop_args.callback = loopCb;

    ret = ClusterXLoop(AllNodesLoopCb, clusterId, &loop_args);
  }

  if (ret == VB_ENGINE_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_ENGINE_ERROR_NONE
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
