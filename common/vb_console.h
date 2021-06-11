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
 * @file vb_console.h
 * @brief Console to test the Vector Boost
 *
 * @internal
 *
 * @author
 * @date 22/12/2014
 *
 **/

#ifndef _VB_CONSOLE_H_
#define _VB_CONSOLE_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public type definition
 ************************************************************************
 */

typedef void (*t_writeFun)(const char *fmt, ...);

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */
#define VB_CONSOLE_PARAMS_MAXNUM      (10)
#define VB_CONSOLE_PARAMS_MAX_LENGHT  (40)
#define VB_CONSOLE_BUFFER_SIZE        (20 + VB_CONSOLE_PARAMS_MAXNUM * VB_CONSOLE_PARAMS_MAX_LENGHT) // Command name (20) + args

/**
 * @brief Initializes console component.
 * Clears all registered commands and configures socket.
 * @param[in] console_port Console TCP port.
 * @param[in] name String to dump in console prompt.
**/
void VbConsoleInit(int console_port, const char* name);

/**
 * @brief Starts console thread
**/
void VbConsoleStart(void);

/**
 * @brief Stops console thread
**/
void VbConsoleStop( void );

/**
 * @brief Releases all reserved memory and clears registered commands.
**/
void VbConsoleDestroy( void );

/**
 * @brief Register a callback function to a specific console command
 *
 * @param[in] command_name  String representing the console command that will
 *                          trigger the callback
 *
 * @param[in] f             Callback function.
 *                          When executed it will be called with the following
 *                          arguments:
 *                            - arg: The same pointer you specify as the third
 *                                   argument to this function.
 *                            - write_fun: A printf-like function you must use
 *                                   inside the callback function to send text
 *                                   back to the user.
 *                            - cmd: A null-terminated list of strings containing
 *                                   each of the words that make up the actual
 *                                   command that triggered the callback
 *                                   (see example below). This buffer shall not be
 *                                   freed by given callback function.
 *
 * @param[in] arg           Pointer that will be passed to the callback
 *                          function when executed
 *
 * @notes
 * Let's see an example:
 * Imagine you want to register a callback for command "algorithm".
 * You would then call "VbConsoleCommandRegister("algorithm", algorithm_callback, NULL);"
 *
 * At runtime, if someone calls "algorithm mode fast enable", the callback will be executed with the following arguments:
 *
 *  arg = NULL
 *  write_fun = <whatever the system decides>
 *  arg[0] = "algorithm"
 *  arg[1] = "mode"
 *  arg[2] = "fast"
 *  arg[3] = "enable"
 *  arg[4] = NULL
 **/
void VbConsoleCommandRegister(const CHAR* command_name, BOOL (*f)(void *arg, t_writeFun writeFun, char **cmd), void *arg);

#endif /* _VB_CONSOLE_H_ */
/**
 * @}
**/
