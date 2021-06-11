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
 * @file vb_main_timer.h
 * @brief Driver main timer interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-12
 *
 **/

#ifndef _VB_MAIN_TIMER_H_
#define _VB_MAIN_TIMER_H_

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
 * @brief Starts main timer thread
 * @return TRUE if OK; FALSE otherwise
 **/
BOOL VbMainTimerRun(void);

/**
 * @brief Starts socket alive timer thread
 * @param[in] desc socket descriptor
 * @param[in] period period of alive sending
 * @param[in] nMsgLostThr number of messages to be lost to detect socket as closed
 * @return TRUE if OK; FALSE otherwise
 **/
BOOL VbMainSocketAliveTimerRun(t_vbEADesc *desc, INT32U period, INT32U nMsgLostThr);

/**
 * @brief Stops socket alive timer thread
 **/
void VbMainSocketAliveTimerStop(void);

/**
 * @brief Get status of socket alive mechanism
 **/
BOOL VbMainSocketAliveTimerStatusGet(void);

/**
 * @brief Stops main timer thread
 **/
void VbMainTimerStop(void);

t_VB_comErrorCode VbMainKeepAliveSend( void );


#endif /* _VB_MAIN_TIMER_H_ */

/**
 * @}
**/


