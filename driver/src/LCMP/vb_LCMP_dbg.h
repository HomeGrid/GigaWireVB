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
 * @file vb_LCMP_dbg.h
 * @brief Debug LCMP communications
 *
 * @internal
 *
 * @author
 * @date 2016/09/20
 *
 **/

#ifndef VB_LCMP_DBG_H_
#define VB_LCMP_DBG_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_console.h"

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
 * @brief Resets LCMP debug table with TX/RX frames
 **/
void LcmpDbgMsgReset(void);

/**
 * @brief Adds a new frame to LCMP debug tables
 * @param[in] mac MAC address of sender or receiver
 * @param[in] txNrx TRUE: Tx frame; FALSE: Rx frame
 * @param[in] proc Only valid for Rx frames; TRUE: frame was processed by a callback; FALSE: otherwise
 * @param[in] lcmpOpcode LCMP Opcode
 * @param[in] paramId Vectorboost LCMP parameter Id
 * @return @ref t_HGF_LCMP_ErrorCode
 **/
t_HGF_LCMP_ErrorCode LcmpDbgMsgAdd(const INT8U *mac, BOOLEAN txNrx, BOOLEAN proc, t_LCMP_OPCODE lcmpOpcode, INT8U paramId);

/**
 * @brief Adds a new frame (no VB) to LCMP debug tables
 * @param[in] mac MAC addres of sender
 * @return @ref t_HGF_LCMP_ErrorCode
 **/
t_HGF_LCMP_ErrorCode LcmpDbgNoVbMsgAdd(const INT8U *mac);

/**
 * @brief Dumps LCMP debug tables with TX/RX frames
 * @param[in] writeFun Function to dump info
 **/
void LcmpDbgMsgDump(t_writeFun writeFun);

/**
 * @brief Console command to dump or reset LCMP debug table
 * @param[in] arg Generic argument pointer
 * @param[in] writeFun Function to dump info
 * @param[in] cmd List of command arguments
 * @return TRUE: if Ok; FALSE: otherwise
 **/
BOOL VbLcmpConsoleCmd(void *arg, t_writeFun writeFun, char **cmd);

#endif /* VB_LCMP_DBG_H_ */

/**
 * @}
**/
