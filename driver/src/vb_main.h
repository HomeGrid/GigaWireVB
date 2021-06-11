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
 * @file vb_main.h
 * @brief Main interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016-10-04
 *
 **/

#ifndef VB_MAIN_H_
#define VB_MAIN_H_

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

typedef enum
{
  DRIVER_STT_IDLE = 0,
  DRIVER_STT_DISCONNECTED,
  DRIVER_STT_PRECONNECTED,
  DRIVER_STT_CONNECTED,
  DRIVER_STT_ALIGNCHECK,
  DRIVER_STT_ALIGNCHANGE,
  DRIVER_STT_MEASURING,
  DRIVER_STT_MEASCOLLECT,
  DRIVER_STT_PSDSHAPING,
  DRIVER_STT_LAST,
} t_driverState;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Gets current FSM state name
 * @return FSM state name
 **/
const CHAR* VbMainStateStrGet();

/**
 * @brief Moves FSM to IDLE state
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMainIdleModeSet(void);

/**
 * @brief Moves FSM to STAND-BY state
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMainStart(void);

/**
 * @brief Kills VB driver in a proper way
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMainKill(void);

/**
 * @brief This function posts an event in main queue
 * @param[in] event Event to post
 * @param[in] vbQueue Queue id
 * @param[in] args Pointer to generic extra arguments
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMainQEvSend(t_driverEvent event, mqd_t vbQueue, void *args);

/**
 * @brief Posts an event to main queue attaching given EA frame
 * @param[in] msg Pointer to EA frame from engine
 * @param[in] vbQueue Main queue descriptor
 * @param[in] args Generic args pointer
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbMainQEAFrameEvSend(t_vbEAMsg *msg, mqd_t vbQueue, void *args);

#endif /* VB_MAIN_H_ */

/**
 * @}
 **/
