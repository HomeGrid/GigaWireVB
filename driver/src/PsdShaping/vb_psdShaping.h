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
 * @file vb_traffic.h
 * @brief Traffic awareness interface
 *
 * @internal
 *
 * @author
 * @date 27/04/2015
 *
 **/

#ifndef VB_PSDSHAPE_H_
#define VB_PSDSHAPE_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

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

typedef enum
{
  VB_PSDSHAPE_ERR_NONE = 0,
  VB_PSDSHAPE_ERR_NOT_SUPPORTED = -1,               ///< Requested action not supported (ex.: dummy package, ...)
  VB_PSDSHAPE_ERR_NOT_INITIALIZED = -2,             ///< Component not initialized
  VB_PSDSHAPE_ERR_NOT_STARTED = -3,                 ///< Component not started
  VB_PSDSHAPE_ERR_BAD_ARGS = -4,                    ///< Wrong parameter(s)
  VB_PSDSHAPE_ERR_NO_MEMORY = -5,                   ///< Not enough memory
  VB_PSDSHAPE_ERR_CONFIG_LAYER = -6,                ///< Config Layer error
  VB_PSDSHAPE_ERR_NOT_FOUND = -7,                   ///< Param not found
  // Error codes from 0 to -31 inclusive are reserved to be common errors to all components
  VB_PSDSHAPE_ERR_OTHER = -31,                      ///< Unknown error
  // Errors specific to the component are defined here starting at -32
  VB_PSDSHAPE_ERR_COM   = -32,                      ///< Communications related error
  VB_PSDSHAPE_ERR_TIMEOUT = -33,                    ///< Timeout
  VB_PSDSHAPE_ERR_SEND  = -34,                      ///< Error sending frame
  VB_PSDSHAPE_ERR_EA  = -35,                        ///< EA related error
  VB_PSDSHAPE_ERR_QUEUE = -36,                      ///< Posix queue error

} t_vbPsdShapeError;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Stops the task which executes the PSD shaping process
**/
void VbPsdShapeStop(void);

/**
 * @brief Executes the PSD shape write process
 * @param[in] payload PSD Shape message to process
 * @param[in] length Length in bytes of the PSD Shape message
**/
void VbPsdShapeWriteRun(INT8U *payload, INT16U length);

/**
 * @brief This function initiates the PSD shaping feature to default values
**/
void VbPsdShapeInit(void);

#endif /* VB_PSDSHAPE_H_ */

/**
 * @}
**/
