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

#ifndef VB_EA_INTERFACE_H_
#define VB_EA_INTERFACE_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

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
 * @brief This function stops the external agent protocol thread
**/
void VbEAStop( void );

/**
 * @brief This function executes the external agent protocol thread
**/
void VbEAStart( void );

/**
 * @brief This function initiates the external agent interface component to default values
**/
void VbEAInit( const char *ifeth, INT16U eaiPort, CHAR *serverIp, BOOL serverMode, INT16U family);

/**
 * @brief Processes frame received from engine
 * @param[in] msg Pointer to message from engine
 **/
void VbEAFrameRxProcess(t_vbEAMsg *msg);

/**
 * @brief Sends a EAMeasurePlan.rsp frame to engine with given plan Id and error code
 * @param[in] planId Plan Id to send
 * @param[in] errorCode Error code to send
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEAMeasurePlanRspSend(INT8U planId, INT8U errorCode);

/**
 * @brief Sends a EAMeasPlanCancel.rsp frame to engine with given plan Id and error code
 * @param[in] planId Plan Id to send
 * @param[in] errorCode Error code to send
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEAMeasurePlanCancelCnfSend(INT8U planId, INT8U errorCode);

/**
 * @brief Sends a EAMeasureErr.trg frame to engine with given plan Id and error code
 * @param[in] planId Plan Id to send
 * @param[in] errorCode Error code to send
 **/
t_VB_comErrorCode VbEAMeasurePlanErrorSend(INT8U planId, INT8U errorCode);

/**
 * @brief Allocates memory for a traffic awareness multiframe
 * @param[in] numReports Number of reports
 * @param[in] numBands Number of bands
 * @param[in] msg pointer to the new msg
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEATrafficAwarenessMultiFrameCreate(INT32U numReports, INT32U numBands, t_vbEAMsg **msg);

/**
 * @brief Fills the content of the traffic report message
 * @param[in] macAddr MAC of the device
 * @param[in] report Traffic data
 * @param[in] msg Pointer to the buffer space to write the report data
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEATrafficAwarenessFillMsg(const INT8U *macAddr, t_IngressTraffic *report, INT8U *msg);

/**
 * @brief Sends Network change.trg message to VB engine
 * @param[in] msg Pointer to the message to be sent
 **/
void VbEAMsgTrafficAwarenessTrgSend(t_vbEAMsg **msg);

/**
 * @brief Sends PSD Shaping confirmation message to VB engine
 * @param[in] writeStatus write status of the PSD shaping
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEAPsdShapeCnfSend(INT8U writeStatus);

/**
 * @brief Sends PSD Shaping & Qos Rate confirmation message to VB engine
 * @param[in] writeStatus write status of the PSD shaping
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEACdtaCnfSend(INT8U writeStatus);

/**
 * @brief Sends Network change.trg message to VB engine
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEANetworkChangeReportTrgSend(void);

/**
 * @brief Sends a EAMeasCollectEnd.trg frame to engine with given plan Id
 * @param[in] planId Plan Id to send
 **/
t_VB_comErrorCode VbEAMeasCollectionEndSend(INT8U planId);

/**
 * @brief Sends a EAVbDriverState.trg frame to engine with given state
 * @param[in] stateTEXT Vb driver state
 **/
void VbEAStateTrgSend(const char *stateTEXT);

/**
 * @brief Gets client connection status
 * @return TRUE: client connected; FALSE: otherwise
 **/
BOOL VbEAEngineIsConnected(void);

/**
 * @brief Sends given message to Engine
 * @param[in] msg EA frame to send
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEADriverFrameSend(t_vbEAMsg *msg);

/**
 * @brief This function mount the frame of external agent protocol read CFR.
 * @param[in] MACMeasurer MAC of measurer device
 * @param[in] MACMeasured MAC of measured device
 * @param[in] planID ID of measure plan to read
**/
t_VB_comErrorCode VbEACfrRspSend(const INT8U *macMeasurer, const INT8U *macMeasured, INT8U planId, t_processMeasure *processMeasure);

/**
 * @brief This function mount the frame of external agent protocol read BGN.
 * @param[in] mac MAC of measurer device
 * @param[in] planID ID of measure plan to read
**/
t_VB_comErrorCode VbEABgnRspSend(const INT8U *mac, INT8U planId, t_processMeasure *processMeasure);

/**
 * @brief This function mount the frame of external agent protocol read BGN.
 * @param[in] mac MAC of measurer device
 * @param[in] planID ID of measure plan to read
**/
t_VB_comErrorCode VbEASnrProbesRspSend(const INT8U *mac, t_processMeasure *processMeasure);

/**
 * @brief Sends EAAlignSyncLost.trg frame to VB Engine
 * @param[in] reporterMac Reporter MAC address
 * @param[in] syncDid Device Id lost
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbEAAlignSyncLostSend(const INT8U *reporterMac, INT8U syncDid);

/**
 * @brief Dumps EA debug table with TX/RX frames
 * @param[in] writeFun Callback to function to dump
 * @remarks EA internal mutex is grabbed inside this function
 **/
void VbEAConnDbgMsgDump(t_writeFun writeFun);

/**
 * @brief Resets EA debug table with TX/RX frames
 * @remarks EA internal mutex is grabbed inside this function
 **/
void VbEAConnDbgMsgReset(void);

/**
 * @brief Free socket descriptor memory used by the driver
 **/
void VbEADestroy();

#endif /* VB_EA_INTERFACE_H_ */
/**
 * @}
**/
