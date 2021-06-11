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
 * @file vb_alignment.h
 * @brief Alignment
 *
 * @internal
 *
 * @author
 * @date 22/12/2014
 *
 **/

#ifndef VB_ALIGNMENT_H_
#define VB_ALIGNMENT_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_DataModel.h"

/*
 ************************************************************************
 ** Public constantsdeviceMAC
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
 * @brief Sends the CycQuery frame and waits for response from G.hn nodes
 * @return @ref t_VB_comErrorCode
**/
t_VB_comErrorCode VbAlignmentCycQuerySend(void);

/**
 * @brief Parses the EA_CycQuery.req message from Engine and sends
 * proper CycQuery LCMP frames to G.hn nodes to get alignment info.
 * @param[in] msg EA_CycQuery.req message payload
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbAlignmentEACycQueryReqProcess(INT8U *msg);

/**
 * @brief Stops alignment check thread
 **/
void VbAlignmentCheckProcessStop(void);

/**
 * @brief Parses the EA_CycChange.req message from Engine and sends
 * proper CycChange LCMP frames to G.hn nodes to correct alignment.
 * @param[in] msg EA_CycChange.req message payload
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbAlignmentEACycChangeReqProcess(INT8U *msg);

/**
 * @brief Stops alignment change thread
 **/
void VbAlignmentChangeProcessStop(void);

/**
 * @brief Enables monitor thread to receive AlignSyncLost.ind
 * @return TRUE: thread is running; FALSE: otherwise
 **/
BOOLEAN VbAlignmentSyncLostMonitorRun(void);

/**
 * @brief Stops sync lost monitoring thread
 **/
void VbAlignmentSyncLostMonitorStop(void);

/**
 * @brief Gets current sequence number from G.hn nodes
 * @param[out] seqNum Sequence number
 * @param[out] valid TRUE: sequence number is valid; FALSE: otherwise
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbAlignmentSeqNumGet(INT16U *seqNum, BOOLEAN *valid);

/**
 * @brief Parses EA_AlignMode.req message from Engine and sends
 * a multicast message to all dms configuring the alignment mode
 * @param[in] msg EA_AlignMode.req message payload
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbAlignmentModeProcess(INT8U *msg);

/**
 * @brief Parses EA_AlignClusterStop.req message from Engine and sends
 * a multicast message to all dms requesting them to go back to listening state
 * @param[in] msg EA_AlignClusterStop.req message payload
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbAlignmentClusterStopProcess(INT8U *msg);

/**
 * @brief Send   LCMP AlignClusterStop.req message from driver to DM
 * @param[in] clusterStopFlag flag indicating if the cluster has to be stopped or not
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbAlignmentLCMPClusterStopSend(BOOLEAN clusterStopFlag);

#endif /* VB_ALIGNMENT_H_ */
/**
 * @}
**/
