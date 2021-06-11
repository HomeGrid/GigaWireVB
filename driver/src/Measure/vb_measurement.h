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
 * @file vb_measurement.h
 * @brief Measurement of vector boost system
 *
 * @internal
 *
 * @author
 * @date 19/01/2015
 *
 **/

#ifndef VB_MEASUREMENT_H_
#define VB_MEASUREMENT_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <stdio.h>

#include "vb_console.h"
#include "vb_ea_communication.h"

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
 * @brief Stops the task which executes the measure plan process
**/
void VbMeasurePlanStop(void);

/**
 * @brief Stops all threads related with measures: measure plan, measures collect and snr probes.
 * It also clears the collect info list.
 **/
void VbMeasurementStop(void);

/**
 * @brief Executes the measure plan received from VB engine
 * @param[in] planReqMsg Measure plan message
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMeasurePlanReqRun(t_vbMsg planReqMsg);

/**
 * @brief Return last planID
 * @return planID
**/
INT8U VBMeasurementPlanIDGet( void );

/**
 * @brief Gets measure collect end error.
 * It is only valid when all measure collect threads have finished.
 * @return @ref t_vbEAMeasRspErrorCode
 **/
t_vbEAMeasRspErrorCode VBMeasurementCollectEndErrorGet(void);

/**
 * @brief Extracts Measure Plan Id from given EA message
 * @param[in] msg Pointer to message
 * @return Measure Plan Id
 **/
INT8U VBMeasurementPlanIDFromMsgGet(INT8U *msg);

/**
 * @brief Cancels the stated measure plan in all G.hn devices and
 * sends a response to VB engine.
 * @param[in] measCancelReqMsg Measure plan cancel message
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMeasurementCancel(INT8U *measCancelReqMsg);

/**
 * @brief Destroy a process measure structure and free memory
 * @param[in,out] ProcessMeasure Pointer to process measure in float measure
 * @return error code
**/
t_VB_comErrorCode VBDestroyProcessMeasure(t_processMeasure **ProcessMeasure);

/**
 * @brief This function initiates the measurement component to default values
**/
t_VB_comErrorCode VbMeasurementInit( void );

/**
 * @brief Executes the measure collection process from stated G.hn devices.
 * @param[in] msg Measure collect message
 * @param[in] length Length of the message in bytes
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMeasureMeasuresCollectRun(INT8U *msg, INT16U length);

/**
 * @brief Stops the given measure collect thread
 * @param[in] data Pointer to measure collect thread to stop
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMeasureMeasuresCollectStop(void *data);

/**
 * @brief Shows if there is a measure collect thread running.
 * @return TRUE: at least one thread running; FALSE: otherwise
 **/
BOOLEAN VbMeasurementCollectThreadsRunning(void);

/**
 * @brief Extracts Measure Plan Id from given EA message
 * @param[in] msg Pointer to message
 * @return Measure Plan Id
 **/
t_VB_comErrorCode VbMeasureSNRProbeReqRun(t_vbMsg snrProbeMsg);

#endif /* VB_MEASUREMENT_H_ */

/**
 * @}
**/
