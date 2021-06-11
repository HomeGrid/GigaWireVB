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
 * @file vb_engine_alignment_metrics.h
 * @brief VB Drivers alignment metrics
 *
 * @internal
 *
 * @author V.Grau
 * @date 2019-11-13
 *
 **/

#ifndef VB_ENGINE_ALIGNMENT_METRICS_H_
#define VB_ENGINE_ALIGNMENT_METRICS_H_

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
 * @brief Starts alignment metrics
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineAlignMetricsStart(void);

/**
 * @brief Resets alignment metrics
 **/
void VbEngineAlignMetricsReset(void);

/**
 * @brief Sends a metrics event related to alignment process
 * @param[in] event Event to send
 * @param[in] driverId Driver Id
 * @param[in] clusterId Cluster Id
 * @param[in] step Step name
 * @param[in] fmt String format
 **/
void VbEngineAlignMetricsEvReport(t_VBMetricsEventType event, const char *driverId, INT32U clusterId, const char *step, const char *fmt, ...);

/**
 * @brief Inserts a report of all domain master nodes in given cluster Id
 * @param[in] clusterId Cluster Id
 **/
void VbEngineAlignMetricsNodeInfoReport(INT32U clusterId);

#endif /* VB_ENGINE_ALIGNMENT_METRICS_H_ */

/**
 * @}
**/
