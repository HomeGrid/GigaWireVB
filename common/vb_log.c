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
 * @file vb_log.c
 * @brief Implements log feature
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <syslog.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <libgen.h>

#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_log.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_util.h"
#include "vb_ea_communication.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

// Single line log limits
//
#define VB_LOG_MAX_MODE_LEN     7
#define VB_LOG_MAX_FILE_LEN     30
#define VB_LOG_MAX_LINENUM_LEN  5
#define VB_LOG_MAX_FUNC_LEN     40
#define VB_LOG_MAX_EXTRA_LEN    24 // [][][][][xx:xx:xx xxxms]
#define VB_LOG_MAX_HDR_LEN      (VB_LOG_MAX_FILE_LEN + VB_LOG_MAX_LINENUM_LEN + \
                                 VB_LOG_MAX_FUNC_LEN + VB_LOG_MAX_MODE_LEN    + \
                                 VB_LOG_MAX_EXTRA_LEN)
#define VB_LOG_FMT              "[%7s][%30s][%5s][%40s][%02d:%02d:%02d %03dms]"

#define VB_LOG_MAX_DRIVER_HDR_LEN (VB_EA_DRIVER_ID_MAX_SIZE + 2) // Take into account [] (+2)
#define VB_LOG_MAX_EXT_HDR_LEN  (VB_LOG_MAX_HDR_LEN + VB_LOG_MAX_DRIVER_HDR_LEN)
#define VB_LOG_DRIVER_ID_FMT    "[%-20s]"

#define VB_LOG_MAX_LINE_LEN     200

#define VB_LOG_ENTRY_LEN        (VB_LOG_MAX_HDR_LEN + VB_LOG_MAX_LINE_LEN)
#define VB_LOG_EXT_ENTRY_LEN    (VB_LOG_MAX_EXT_HDR_LEN + VB_LOG_MAX_LINE_LEN)

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_vbLogLevel vbLogVerbose;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

// Print a message to stderr, used (internally) when the regular log mechanism
// cannot be used (ex: errors reported by the Log subsystem itself)
static void VbLogErrorPrint(const char *fmt, ...);

// Build the "log header" string (that is pre-pended to all messages sent to the
// default log output)
static void VbLogHdrBuild(CHAR *dst, t_vbLogLevel mode, const char *file, INT16U line, const char *function);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VbLogErrorPrint(const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);

  fprintf(stderr, "\n");
}

/*******************************************************************/

static void VbLogHdrBuild(CHAR *dst, t_vbLogLevel mode, const char *file, INT16U line, const char *function)
{
  CHAR              mode_str[VB_LOG_MAX_MODE_LEN+1];
  CHAR              file_name_str[VB_LOG_MAX_FILE_LEN+1];
  CHAR              linenum_name_str[VB_LOG_MAX_LINENUM_LEN+1];
  CHAR              func_name_str[VB_LOG_MAX_FUNC_LEN+1];

  time_t            t;
  struct            tm *tmu;
  struct            timeval tv;

  if ((dst != NULL) && (mode < VB_LOG_LAST) && (file != NULL) && (function != NULL))
  {
    t = time(NULL);
    tmu = localtime(&t);

    if (tmu != NULL)
    {
      gettimeofday(&tv, NULL);

      // Copy level string
      snprintf(mode_str, VB_LOG_MAX_MODE_LEN+1, "%7s", VbVerboseLevelToStr(mode));
      mode_str[VB_LOG_MAX_MODE_LEN] = '\0';

      // Copy file name
      strncpy(file_name_str, file, VB_LOG_MAX_FILE_LEN+1);
      file_name_str[VB_LOG_MAX_FILE_LEN] = '\0';

      // Copy line number
      snprintf(linenum_name_str, VB_LOG_MAX_LINENUM_LEN+1, "%05u", line);
      linenum_name_str[VB_LOG_MAX_LINENUM_LEN] = '\0';

      // Copy func name
      strncpy(func_name_str, function, VB_LOG_MAX_FUNC_LEN+1);
      func_name_str[VB_LOG_MAX_FUNC_LEN] = '\0';

      {
        // Limit msec to 999 to avoid a compilation warning of GCC-7
        INT32U msec = tv.tv_usec / 1000;

        if (msec >= 1000)
        {
          msec = 999;
        }

        // Build the whole header
        snprintf(dst, VB_LOG_MAX_HDR_LEN+1, VB_LOG_FMT,
            mode_str, file_name_str, linenum_name_str, func_name_str,
            tmu->tm_hour, tmu->tm_min, tmu->tm_sec, msec);
      }
    }
  }
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

INT32S VbLogInit(const char *queueName, t_vbLogLevel verboseLevel, CHAR *outputFolder, INT32U persLogNumLines, t_vbLogLevel persLogVerbose, BOOLEAN circular)
{
  vbLogVerbose = verboseLevel;

  return 0;
}

/*******************************************************************/

BOOLEAN VbLogRun()
{
  return 1;
}

/*******************************************************************/

void VbLogStop()
{
}

/*******************************************************************/

void VbLogPrint_helper(const char *current_file_name, INT16U current_line_number, const char *current_function_name, t_vbLogLevel verboseLevel, const char *fmt, ...)
{
  va_list           args;
  CHAR             *log_line = NULL;
  CHAR             *dst = NULL;

  if ((verboseLevel <= vbLogVerbose))
  {
    log_line = (CHAR *)calloc(1, VB_LOG_ENTRY_LEN+1);

    if (log_line == NULL)
    {
      VbLogErrorPrint("No memory to allocate log_line (%d bytes)", VB_LOG_ENTRY_LEN+1);
    }
    else
    {
      // Build header
      VbLogHdrBuild(log_line, verboseLevel, current_file_name, current_line_number, current_function_name);

      // Locate pointer after header
      dst = log_line + VB_LOG_MAX_HDR_LEN;

      va_start(args, fmt);
      vsnprintf(dst, VB_LOG_MAX_LINE_LEN, fmt, args);
      va_end(args);

      syslog(LOG_INFO, "%s", log_line);
    }
  }
}

/*******************************************************************/

void VbLogPrintExt_helper(const char *current_file_name, INT16U current_line_number, const char *current_function_name, t_vbLogLevel verboseLevel, const char *driverId, const char *fmt, ...)
{
  va_list           args;
  CHAR             *log_line = NULL;
  CHAR             *dst = NULL;
  CHAR              driver_id_str[VB_EA_DRIVER_ID_MAX_SIZE];

  if ((verboseLevel <= vbLogVerbose))
  {
    log_line = (CHAR *)calloc(1, VB_LOG_EXT_ENTRY_LEN+1);

    if (log_line == NULL)
    {
      VbLogErrorPrint("No memory to allocate log_line (%d bytes)", VB_LOG_EXT_ENTRY_LEN+1);
    }
    else
    {
      // Build header
      VbLogHdrBuild(log_line, verboseLevel, current_file_name, current_line_number, current_function_name);

      // Locate pointer after base header
      dst = log_line + VB_LOG_MAX_HDR_LEN;

      // Copy driver Id
      strncpy(driver_id_str, driverId, VB_EA_DRIVER_ID_MAX_SIZE);
      driver_id_str[VB_EA_DRIVER_ID_MAX_SIZE - 1] = '\0';

      // Dump DriverId
      snprintf(dst, VB_LOG_MAX_DRIVER_HDR_LEN, VB_LOG_DRIVER_ID_FMT, driver_id_str);

      // Locate pointer after driverId header
      dst += VB_LOG_MAX_DRIVER_HDR_LEN - 1;  // -1 to locate ptr just in '\0' character

      va_start(args, fmt);
      vsnprintf(dst, VB_LOG_MAX_LINE_LEN, fmt, args);
      va_end(args);

      syslog(LOG_INFO, "%s", log_line);
    }
  }
}

/*******************************************************************/

void VbLogSaveToTextFile(const char *file_name, const char *access_mode, int maxLen, const char *fmt, ...)
{
}

/*******************************************************************/

t_vbLogLevel VbLogVerboseLevelGet(void)
{
  return vbLogVerbose;
}

/*******************************************************************/

void VbLogPersistentDump(t_writeFun writeFun)
{
}

/*******************************************************************/

BOOL VbLogConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  return 0;
}

/*******************************************************************/

/**
 * @}
 **/
