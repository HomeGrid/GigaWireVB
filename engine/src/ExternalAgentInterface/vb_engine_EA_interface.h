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
 * @file vb_EA_interface.h
 * @brief External Agent interface
 *
 * @internal
 *
 * @author
 * @date 30/01/2015
 *
 **/

#ifndef VB_ENGINE_EA_INTERFACE_H_
#define VB_ENGINE_EA_INTERFACE_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief This function executes the external agent protocol thread for given driver
 * @param[in] vbDriver Driver associated
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEAProtocolDriverThreadStart( t_VBDriver *vbDriver );

/**
 * @brief This function stops the external agent protocol thread for given driver
 * @param[in] vbDriver Driver associated
 * @param[in] args Generic argument pointer
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEAProtocolDriverThreadStop(t_VBDriver *vbDriver, void *args);

/**
 * @brief Stop external agent thread for all drivers
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEAProtocolAllDriversThreadStop(void);

/**
 * @brief This function executes the external agent protocol thread for server mode
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEAProtocolServerThreadStart(void);

/**
 * @brief This function stops the external agent protocol thread for server mode
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEAProtocolServerThreadStop(void);

/**
 * @brief Initializes External Agent Interface
 * @param[in] vbDriverIp VB driver IP address
 * @param[in] vbDriverPort VB driver port to connect
 * @param[in] thisDriver Pointer to driver structure
 * @param[in] type Type of connection
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineEAInterfaceInit( const char *ifeth, struct in6_addr vbDriverIp,
    INT16U port, t_VBDriver *thisDriver, t_vbEAType type);

/**
 * @brief This function sends a frame to External Agent interface socket
 * @param[in] opcode Frame's opcode
 * @param[in] buffer Pointer to payload data
 * return error code
**/
t_VB_engineErrorCode VbEngineEAInterfaceSendFrame(
    t_vbEAOpcode opcode, const INT8U *payload, INT16U payloadLength, t_VBDriver *thisDriver);

#endif /* VB_ENGINE_EA_INTERFACE_H_ */

/**
 * @}
**/
