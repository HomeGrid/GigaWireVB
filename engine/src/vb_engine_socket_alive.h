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
 * @file vb_engine_socket_alive.h
 * @brief VB Drivers socket alive mechanism
 *
 * @internal
 *
 * @author Y. Raoul
 * @date 2019.11.11
 *
 **/

#ifndef VB_ENGINE_SOCKET_ALIVE_H_
#define VB_ENGINE_SOCKET_ALIVE_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_engine_datamodel.h"
#include "ezxml.h"

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
typedef struct s_socketAlive
{
  BOOL            enable;
  INT32U          period;
  INT32U          nLostMsgThr;
} t_socketAlive;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Request system socket alive (EASocketAlive.req frame) to given driver
 * @param[in] thisDriver Driver to launch the request to
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineSocketAliveRequest( t_VBDriver *thisDriver );

/**
 * @brief Process the EASocketAlive.rsp received frame to extract system socket alive and sequence number from given driver
 * @param[in] payload Pointer to frame payload
 * @param[in] length Length in bytes of message
 * @param[in] thisDriver Driver related to received frame
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineSocketAliveRspProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Launch a EASocketAlive.req frame to all drivers to update system socket alive
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineSocketAliveCheckReq(void);


/**
 * @brief Initializes the socket alive monitor task
 **/
void VbEngineSocketAliveMonitorInit(void);

/**
 * @brief Stops the socket alive monitor task
 **/
void VbEngineSocketAliveMonitorStop(void);

/**
 * @brief Starts the socket alive monitor task
 **/
BOOLEAN VbEngineSocketAliveMonitorRun(void);

/**
 * @brief Parse the socket alive in the vb_engine.ini
 * @param[in] socketAliveConf socket alive configuration to parse
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineSocketAliveConfigurationParse(ezxml_t socketAliveConf);



#endif /* VB_ENGINE_SOCKET_ALIVE_H_ */

/**
 * @}
**/
