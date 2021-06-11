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
 * @file vb_timer.h
 * @brief Common timers and clock functions interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-12
 *
 **/

#ifndef _VB_TIMER_H_
#define _VB_TIMER_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <time.h>
#include <signal.h>

#include "vb_console.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define TIMER_SIGNAL_NUM       (SIGALRM)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef void (*t_timerTaskCb)(sigval_t sigval);

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Initializes timer module, including mutex and internal memory.
 * @return 0 if OK
 **/
INT32S VbTimerInit(void);

/**
 * @brief Configures a periodic task to be executed each "periodMs".
 * @param[in] name Task name
 * @param[in] periodMs Task period in msecs.
 * @param[in] handler Function to execute.
 * @param[in] args Generic arguments to pass to the function.
 * @param[out] timerId Timer ID. Useful to delete this periodic task.
 * @return 0 if OK; -1 otherwise
 **/
INT32S TimerPeriodicTaskSet(const CHAR *name, INT32U periodMs, t_timerTaskCb handler, void *args, timer_t *timerId);

/**
 * @brief Configures a one-shot task to be executed after given timeout in msecs.
 * @param[in] name Task name
 * @param[in] timeoutMs Timeout in msecs to execute the handler.
 * @param[in] handler Function to execute.
 * @param[in] args Generic arguments to pass to the function.
 * @param[out] timerId Timer ID. Useful to delete this periodic task.
 * @return 0 if OK; -1 otherwise
 **/
INT32S TimerOneShotTaskSet(const CHAR *name, INT32U timeoutMs, t_timerTaskCb handler, void *args, timer_t *timerId);

/**
 * @brief Deletes a period task
 * @param[in] timerId Timer ID to delete
 * @param[in] name Task name
 * @return 0 if OK; -1 otherwise
 **/
INT32S TimerTaskDelete(timer_t timerId, const CHAR *name);

/**
 * @brief Dumps timed tasks info
 * @param[in] writeFun Function to write to
 **/
void VbTimerListTaskDump(t_writeFun writeFun);

#endif /* _VB_TIMER_H_ */

/**
 * @}
**/


