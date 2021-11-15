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
 * @file vb_engine_process.h
 * @brief Implements engine process
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
 *
 **/

#ifndef VB_ENGINE_PROCESS_H_
#define VB_ENGINE_PROCESS_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

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


/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Get current thread running flag status
**/
BOOL VbEngineProcessThreadRunningGet(void);

/**
 * @brief Set current thread running flag status
**/
void VbEngineProcessThreadRunningSet(BOOL value);

/**
 * @brief This function stops the engine process thread per driver
**/
void VbEngineProcessProtocolThreadStop(void);

/**
 * @brief Initializes Engine process thread variables
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessThreadInit(void);

/**
 * @brief This function executes the engine process thread per driver
**/
t_VB_engineErrorCode VbEngineProcessThreadExecute(void);

/**
 * @brief Sends an event to engine process thread
 * @param[in] driver Driver related to given event
 * @param[in] event Event to send
 * @param[in] args Generic pointer to arguments. They will be passed to FSM transition functions.
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineProcessEvSend(t_VBDriver *driver, t_VB_Comm_Event event, void *args);

/**
 * @brief Process a received frame on the TCP Socket (From the driver) and insert a new message into the process queue
 * @param[in] msg EA message received
 * @param[in] thisDriver driver that received the frame
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineProcessEAFrameRx(t_vbEAMsg *msg, t_VBDriver *thisDriver);

/**
 * @brief Sends an event related to all drivers
 * @param[in] event Event to send
 * @param[in] args Generic pointer to arguments. They will be passed to FSM transition functions.
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineProcessAllDriversEvSend(t_VB_Comm_Event event, void *args);

/**
 * @brief Sends an event related to all drivers belonging to clusterId
 * @param[in] event Event to send
 * @param[in] args Generic pointer to arguments. They will be passed to FSM transition functions.
 * @param[in] clusterId Targeted cluster Id
 * @return @ref t_VB_engineErrorCode
**/
t_VB_engineErrorCode VbEngineProcessClusterXDriversEvSend(t_VB_Comm_Event event, void *args, INT32U clusterId);

/**
* @brief Get a string defining a given FSM state
*
**/
const CHAR *FSMSttToStrGet(t_vbEngineProcessFSMState state);

/**
* @brief Generate a new execute step event
*
**/
const CHAR *FSMEvToStrGet(t_VB_Comm_Event event);

/**
 * @brief Gets engine process queue name
 * @return Engine process queue name
 **/
CHAR *VbEngineProcessQueueNameGet(void);

/**
 * @brief Find if there is associated ClusterId and role from DMs history of MAC addresses and return one when found
 * @param[in] DMsMAC DMs MAC
 * @param[out] clusterId found cluster Id
 * @param[out] role previous role associated with this DMs
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbeFindDMsClusterIdByMAC(INT8U *DMsMAC, INT32U *clusterId, t_alignRole *role);

/**
 * @brief Update history entry for specified DMs MAC
 * @param[in] MAC this DMs MAC address
 * @param[in] clusterId found cluster Id associated with this DM
 * @param[in] role previouis role associated with this DM
 * @param[in] isAdding is this DMs being added or removed from operation
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbeUpdateDMsHistory(INT8U *MAC, INT32U clusterId, t_alignRole role, BOOL isAdding);

#endif /* VB_ENGINE_PROCESS_H_ */
/**
 * @}
**/
