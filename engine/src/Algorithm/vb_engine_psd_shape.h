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
 * @file vb_engine_psd_shape.h
 * @brief Interface of engine PSD Shape feature
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017/06/16
 *
 **/

#ifndef VB_ENGINE_PSD_SHAPE_H_
#define VB_ENGINE_PSD_SHAPE_H_

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
typedef struct s_psdShapeArgs
{
  INT32U length;
  INT32U numPsds;
  INT8U *payload;
  INT8U *ptrToWrite;
} t_psdShapeArgs;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Checks if a new PSD Shape message shall be sent
 * @return TRUE: A new PSD Shape shall be sent; FALSE: otherwise
 **/
BOOLEAN VbEnginePsdShapeSendCheck(INT32U clusterId);

/**
 * @brief Notifies the reception of a PSD Shape confirmation message.
 * @param[in] thisDriver Pointer to VB driver structure
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEnginePsdShapeCnfNotify(t_VBDriver *thisDriver);

/**
 * @brief Checks that PSD shape has been confirmed for all nodes
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEnginePsdShapeCnfCheck(INT32U clusterId);

/**
 * @brief Builds and send PSD Shape message to all drivers
 * @param[out] timeInfo time info of when the changes will be completed
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEnginePSDShapeConfigureAll(INT32U clusterId);

/**
 * @brief Gets the time until the PSD shape update will be applied
 * @param[in] timeToFinish Time to apply PSD shape in msecs
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEnginePSDShapeTimeToFinishGet(INT32U *timeToFinish);

/**
 * @brief Process PSD Shape response
 * @param[in] payload payload of the packet
 * @param[in] length length of the packet
 * @param[in] thisDriver Pointer to vbDriver data struct
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode    VbEnginePSDShapeRespProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Calculate length of the PSD shape info
 * @param[in] driver driver the node belongs to
 * @param[in] domain domain the node belongs to
 * @param[in] node   node under study
 * @param[in] args   arguments passed to the function
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEnginePSDShapeLengthCalcCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args);

/**
 * @brief Build PSD info msg
 * @param[in] driver driver the node belongs to
 * @param[in] domain domain the node belongs to
 * @param[in] node   node under study
 * @param[in] args   arguments passed to the function
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEnginePSDShapeBuildCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args);

#endif /* VB_ENGINE_PSD_SHAPE_H_ */

/**
 * @}
**/
