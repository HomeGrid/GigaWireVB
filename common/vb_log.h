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
 * @file vb_log.h
 * @brief  Implements log feature
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
 *
 **/

#ifndef _VB_LOG_H_
#define _VB_LOG_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <sys/time.h>
#include "vb_console.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define VB_LOG_DEFAULT_MAX_BUFFER_LEN     (500)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef enum {
  VB_LOG_ALWAYS = 0,
  VB_LOG_ERROR,
  VB_LOG_WARNING,
  VB_LOG_INFO,
  VB_LOG_DEBUG,
  VB_LOG_LAST,
} t_vbLogLevel;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Gets the verbose level name
 * @param[in] verboseLevel Verbose level
 * @return Verbose level name
 **/
static inline CHAR * VbVerboseLevelToStr(t_vbLogLevel verboseLevel)
{
  static CHAR *verboseLevelStr[VB_LOG_LAST] = {"ALWAYS", "ERROR", "WARNING", "INFO", "DEBUG"};
  CHAR *ret = "-";

  if (verboseLevel < VB_LOG_LAST)
  {
    ret = verboseLevelStr[verboseLevel];
  }

  return ret;
}

/**
 * @brief Initialize the Log subsystem structures
 *
 * @param[in] queueName       Text string that identifies the POSIX queue that
 *                            will be used to send messages to the Log subsystem.
 *                            This name must be unique... This means that if you
 *                            are running more than one Log subsystems on the same
 *                            PC (this might happen when running both the vector
 *                            boost engine and driver on the same computer) you
 *                            need to asign them different names.
 *
 * @param[in] verboseLevel    Only messages which are as important as this level
 *                            (or more) will be processed/displayed.
 *
 * @param[in] outputFolder    Messages saved to disk files (using
 *                            "VbLogSaveToTextFile()") will be placed inside this
 *                            folder.
 *
 * @param[in] persLogNumLines Number of lines for persistent log buffer.
 *
 * @param[in] persLogVerbose  Minimum log level to store message in persistent log
 *                            buffer.
 *
 * @param[in] circular        TRUE: configures persistent log as a circular buffer.
 *
 * @return 0 if OK; -1 if error
 **/
INT32S VbLogInit(const char *queueName, t_vbLogLevel verboseLevel, CHAR *outputFolder, INT32U persLogNumLines, t_vbLogLevel persLogVerbose, BOOLEAN circular);

/**
 * @brief Start the Log subsystem.
**/
BOOLEAN VbLogRun(void);

/**
 * @brief Stop the Log subsystem.
**/
void VbLogStop();

/**
 * @brief Send message to the Log subsystem
 *
 * Call like this: VbLogPrint(<verboseLevel>, fmt_str, ...).
 *
 * Example:
 *
 *   VbLogPrint(VB_LOG_WARNING, "My favorite number: %d\n", 7);
 *
 **/
#define VbLogPrint(verboseLevel, args...) VbLogPrint_helper (__FILE__, __LINE__, __FUNCTION__, verboseLevel, args)
void VbLogPrint_helper(const char *current_file_name, INT16U current_line_number, const char *current_function_name, t_vbLogLevel verboseLevel, const char *fmt, ...);

/**
 * @brief Send message to the Log subsystem
 *
 * Call like this: VbLogPrint(<verboseLevel>, <driverId>, fmt_str, ...).
 *
 * Example:
 *
 *   VbLogPrint(VB_LOG_WARNING, driver->vbDriverID, "My favorite number: %d\n", 7);
 *
 **/
#define VbLogPrintExt(verboseLevel, driverId, args...) VbLogPrintExt_helper (__FILE__, __LINE__, __FUNCTION__, verboseLevel, driverId, args)
void VbLogPrintExt_helper(const char *current_file_name, INT16U current_line_number, const char *current_function_name, t_vbLogLevel verboseLevel, const char *driverId, const char *fmt, ...);

/**
 * @brief Save text to a disk file
 *
 * @param[in] file_name    File where text is going to be saved.
 *
 * @param[in] access_mode  "w+" if you want to overwrite the file in case it
 *                         already existed.
 *                         "a+" if you want to append the provided text to the
 *                         file in case it already existed.
 *
 * @param[in] maxLen       Maximum length of buffer to dump to file.
 *
 * @param[in] fmt          Format string (see 'printf()')
 *
 **/
void VbLogSaveToTextFile(const char *file_name, const char *access_mode, int maxLen, const char *fmt, ...);

/**
 * @brief Save buffer to a disk file
 *
 * @param[in] file_name    File where text is going to be saved.
 *
 * @param[in] access_mode  "w+" if you want to overwrite the file in case it
 *                         already existed.
 *                         "a+" if you want to append the provided text to the
 *                         file in case it already existed.
 *
 * @param[in] maxLen       Maximum length of buffer to dump to file.
 *
 * @param[in] buffer       Buffer to dump
 *
 **/
#define VbLogSaveBufferToTextFile(file_name, access_mode, maxLen, buffer) VbLogSaveToTextFile(file_name, access_mode, maxLen, "%s", buffer)

/**
 * @brief Save string (with a maximum length of 500 bytes) to a disk file
 *
 * @param[in] file_name    File where text is going to be saved.
 *
 * @param[in] access_mode  "w+" if you want to overwrite the file in case it
 *                         already existed.
 *                         "a+" if you want to append the provided text to the
 *                         file in case it already existed.
 *
 * @param[in] args         Variable number of arguments, including format string (see 'printf()')
 *
 **/
#define VbLogSaveStringToTextFile(file_name, access_mode, args...)   VbLogSaveToTextFile(file_name, access_mode, VB_LOG_DEFAULT_MAX_BUFFER_LEN, args)

/**
 * @brief Gets current verbose level
 * @return Current verbose level
 **/
t_vbLogLevel VbLogVerboseLevelGet(void);

/**
 * @brief Resets persistent log buffer
 **/
void VbLogPersistentReset(void);

/**
 * @brief Dumps persistent log buffer
 * @param[in] writeFun Callback to call to dump strings
 **/
void VbLogPersistentDump(t_writeFun writeFun);

/**
 * @brief Log console command
 * @param[in] arg Generic argument pointer
 * @param[in] writeFun A printf-like callback
 * @param[in] cmd A null-terminated list of strings containing each of the words that make up the actual
 *                command that triggered the callback.
 * @return TRUE if OK; FALSE otherwise
 **/
BOOL VbLogConsoleCmd(void *arg, t_writeFun writeFun, char **cmd);

#endif /* _VB_LOG_H_ */

/**
 * @}
**/
