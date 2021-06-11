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
 * @file vb_engine_communication.h
 * @brief Implements engine communication
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
 *
 **/

#ifndef VB_ENGINE_COMMUNITACION_H_
#define VB_ENGINE_COMMUNITACION_H_

/*
 ************************************************************************
 ** Included files
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
 * @brief This function sends a measure plan request frame to VectorBoos Driver connected
 * @param[in] thisDriver Pointer to driver structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessMeasurePlanRequest( t_VBDriver *thisDriver );

/**
 * @brief This function sends a snr probes request frame to VectorBoos Driver connected
 * @param[in] nMacs Number of nodes to request SNR Probes measure
 * @param[in] macs MAC address list measurer nodes
 * @param[in] thisDriver Pointer to driver structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessMeasureSnrProbesRequest(INT8U nMacs, const INT8U *macs, t_VBDriver *thisDriver );

/**
 * @brief This function sends a CFR request frame to VectorBoos Driver connected
 * @param[in] planId Measure plan identifier of back ground noise measure
 * @param[in] macMeasurer Hardware direction of node measurer
 * @param[in] macMeasured Hardware direction of node measured
 * @param[in] thisDriver Pointer to driver structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessCfrRequest( INT8U planId,
    const INT8U *macMeasurer, INT16U numNodes, const INT8U *macMeasured, t_VBDriver *thisDriver );

/**
 * @brief This function sends a PSD request frame to VectorBoost Driver connected
 * @param[in] thisDriver Pointer to driver structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessPsdRequest( t_VBDriver *thisDriver );

/**
 * @brief This function sends a VectorBoost Driver version request frame to VectorBoos Driver connected
 * @param[in] thisDriver Pointer to driver structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessVersionRequest( t_VBDriver *thisDriver );

/**
 * @brief Process the received message from driver
 * @param[in] processMsg Message to be processed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineFrameRxProcess(t_VBProcessMsg *processMsg);

/**
 * @brief Request the current state to a given driver
 * @param[in] thisDriver Pointer to driver structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineProcessDriverStateRequest( t_VBDriver *thisDriver );

/**
 * @brief Gets the number of domains present in given EADomain.rsp frame
 * @param[in] payload Pointer to EADomain.rsp frame
 * @return Number of domains
 **/
INT32U VbEngineDomainsRespNumDomainsGet(INT8U *payload);

#endif /* VB_ENGINE_COMMUNITACION_H_ */

/**
 * @}
**/
