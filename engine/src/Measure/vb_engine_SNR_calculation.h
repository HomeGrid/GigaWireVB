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
 * @file vb_SNR_calculation.h
 * @brief SNR calculation
 *
 * @internal
 *
 * @author
 * @date 22/12/2014
 *
 **/

#ifndef VB_SNR_CALCULATION_H_
#define VB_SNR_CALCULATION_H_

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
 * @brief Calculate the SNR & Capacity for each device of domains list
 * @param[in] clusterId clusterId related to the SNR calculations
 * @return @ref t_VB_engineErrorCode
 *
**/
t_VB_engineErrorCode VbCalculateSNRAndCapacityRun(INT32U clusterId);

/**
 * @brief Stops the SNR & Capacity calculation thread related to cluster Id
 * @param[in] clusterId cluster Id to be stopped
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbCalculateSNRAndCapacityStop(INT32U clusterId);

/**
 * @brief Stops the SNR & Capacity calculation threads
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbCalculateAllSNRAndCapacityStop(void);


/**
 * @brief Requests SNR Probe measures for a node or a group of nodes of a given driver
 * @param[in] driver Driver to launch the request
 * @param[in] mac MAC address of the node to request the SNR Probe measure. If it is NULL,
 * the request will be sent to all nodes inside given driver.
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineSNRProbeForceRequest(t_VBDriver *driver, INT8U *mac);


/*******************************************************************/
#endif /* VB_SNR_CALCULATION_H_ */


/**
 * @}
**/
