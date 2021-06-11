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
 * @file vb_engine_drivers_list.h
 * @brief Implements driver list functionality
 *
 * @internal
 *
 * @author
 * @date 29/05/2017
 *
 **/

#include "vb_engine_datamodel.h"

#ifndef VB_ENGINE_DRIVERS_LIST_H_
#define VB_ENGINE_DRIVERS_LIST_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#define VB_ENGINE_DEFAULT_DRIVER_ID             ("driver_%u")

/*
 ************************************************************************
 ** Public Typedefs
 ************************************************************************
 */

typedef struct s_VBDriversList{
  INT16U numVBDrivers;
  t_VBDriver *vbDriversArray;
} t_VBDriversList;

typedef t_VB_engineErrorCode (*t_driverLoopCb)(t_VBDriver *driver, void *args);
typedef t_VB_engineErrorCode (*t_domainLoopCb)(t_VBDriver *driver, t_domain *domain, void *args);
typedef t_VB_engineErrorCode (*t_nodeLoopCb)(t_VBDriver *driver, t_domain *domain, t_node *node, void *args);

/*
 ************************************************************************
 ** Public functions
 ************************************************************************
 */

/**
 * @brief This function destroys the engine drivers list component and free memory
**/
void VbEngineDrvListDriversDestroy(void);

/**
 * @brief Creates a new t_VBDriver struct and appends it to the drivers list
 * @param[in] driverId string with the id of the driver
 * @param[in] vbDriver
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDrvListDriverAdd(t_VBDriver *vbDriver);

/**
 * @brief Removes the driver from the drivers list and frees the memory
 * @param[in] driver driver to be removed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineDrvListRemoveDriver(t_VBDriver *driver);

/**
 * @brief Search a driver by Id and returns a pointer to driver structure
 * @param[in] driverId Driver ID to search
 * @param[out] driver Pointer to driver
 * @return @ref t_VB_engineErrorCode
 * @remarks Drivers list mutex is grabbed inside this function
 **/
t_VB_engineErrorCode VbEngineDriverByIdGet(char *driverId, t_VBDriver **driver);

/**
 * @brief Loops through all VB registered drivers and executes given callback
 * @param[in] loopCb Callback to execute for each VB driver
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Drivers list mutex is grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelDriversLoop(t_driverLoopCb loopCb, void *args);

/**
 * @brief Loops through all VB registered drivers matching the cluster Id passed as paramter and executes given callback
 * @param[in] loopCb Callback to execute for each VB driver
 * @param[in] clusterId cluster id to loop through
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Drivers list mutex is grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelClusterXDriversLoop(t_driverLoopCb loopCb, INT32U clusterId, void *args);

/**
 * @brief Loops through all domains registered in given VB driver and executes given callback
 * @param[in] loopCb Callback to execute for each domain
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains list mutex is grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelDomainsLoop(t_VBDriver *driver, t_domainLoopCb loopCb, void *args);

/**
 * @brief Loops through all domains registered in VB engine and executes given callback
 * @param[in] loopCb Callback to execute for each domain
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains list mutex and Driver list mutex are grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelAllDomainsLoop(t_domainLoopCb loopCb, void *args);

/**
 * @brief Loops through all domains registered in VB engine and executes given callback
 * @param[in] loopCb Callback to execute for each domain
 * @param[in] clusterId cluster id to loop through
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains list mutex and Driver list mutex are grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelClusterXAllDomainsLoop(t_domainLoopCb loopCb, INT32U clusterId, void *args);

/**
 * @brief Loops through all nodes of a given VB driver and executes a callback
 * @param[in] loopCb Callback to execute for each node
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains list mutex is grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelNodesLoop(t_VBDriver *driver, t_nodeLoopCb loopCb, void *args);

/**
 * @brief Loops through all nodes of all drivers and executes given callback
 * @param[in] loopCb Callback to execute for each node
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains list and drivers list mutex are grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelAllNodesLoop(t_nodeLoopCb loopCb, void *args);

/**
 * @brief Loops through all nodes of all drivers and executes given callback
 * @param[in] loopCb Callback to execute for each node
 * @param[in] clusterId cluster Id to loop through
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_engineErrorCode
 * @remarks Domains list and drivers list mutex are grabbed before calling callback
 **/
t_VB_engineErrorCode VbEngineDatamodelClusterXAllNodesLoop(t_nodeLoopCb loopCb, INT32U clusterId, void *args);

/**
 * @brief Returns the number of drivers in the drivers list
 * @return number of drivers
 **/
INT16U VbEngineDataModelNumDriversGet(void);

/**
 * @brief Returns the number of drivers in the drivers list tagged with clusterId
 * @param[in] clusterId cluster Id to be counted
 * @return number of drivers in cluster clusterId
 **/
INT16U VbEngineDataModelNumDriversInCLusterXGet(INT32U clusterId);

/**
 * @brief Returns the number of domains in a specified driver
 * @param[in] numDriver driver number
 * @return number of domains in driver numDriver
 **/
INT16U VbEngineDataModelNumDomainsDriverGet(INT16U numDriver);

#endif /* VB_ENGINE_DRIVERS_LIST_H_ */
/**
 * @}
 **/
