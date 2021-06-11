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
 * @file vb_driver_conf.c
 * @brief Vector boost driver configuration functionality
 *
 * @internal
 *
 * @author
 * @date 2017-05-11
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"
#include <sys/socket.h>
#include <stdio.h>
#include <getopt.h>
#include <linux/if.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>

#include "vb_LCMP_com.h"
#include "vb_console.h"
#include "vb_log.h"
#include "vb_driver_conf.h"
#include "vb_driver_conf.h"
#include "ezxml.h"
#include "vb_measurement.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_DRIVER_CONF_DEFAULT_INI_FILE                ("vb_driver.ini")
#define VB_DRIVER_CONF_DEFAULT_EA_PORT                 (40011)
#define VB_DRIVER_CONF_DEFAULT_AUTO_ALIGN              (TRUE)
#define VB_DRIVER_CONF_DEFAULT_CONSOLE_PORT            (50000)
#define VB_DRIVER_CONF_DEFAULT_VERBOSE_LEVEL           (VB_LOG_ALWAYS)
#define VB_DRIVER_CONF_DEFAULT_TRAFFIC_REPORT_PERIOD   (1000)
#define VB_DRIVER_CONF_DEFAULT_TRAFFIC_REPORT_MEASWIN  (100)
#define VB_DRIVER_CONF_DEFAULT_TRAFFIC_REPORT_THR      (50)
#define VB_DRIVER_CONF_DEFAULT_MEAS_COLLECT_THREAD_INT (0)
#define VB_DRIVER_CONF_DEFAULT_LCMP_TIMEOUT            (200)
#define VB_DRIVER_CONF_DEFAULT_LCMP_N_ATTEMPT          (2)
#define VB_DRIVER_CONF_DEFAULT_PERSLOG_NUMLINES        (100)
#define VB_DRIVER_CONF_DEFAULT_PERSLOG_VERBOSE         (VB_LOG_ERROR)
#define VB_DRIVER_CONF_DEFAULT_PERSLOG_CIRCULAR        (TRUE)

#define MAX_FILE_NAME_LENGTH                           (150)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct s_persistentLog
{
  INT32U          numLines;
  t_vbLogLevel    verboseLevel;
  BOOLEAN         circular;
} t_persistentLog;

typedef struct s_vbDriverConf
{
  CHAR            driverId[VB_EA_DRIVER_ID_MAX_SIZE];     ///< External agent interface name
  CHAR            lcmpIf[IFNAMSIZ];                   ///< LCMP interface name
  CHAR            eaIf[IFNAMSIZ];                     ///< External agent interface name
  CHAR            outputPath[VB_PARSE_MAX_PATH_LEN];  ///< Debug info output path
  INT16U          eaPort;                             ///< External agent port number
  INT16U          consolePort;                        ///< VB Driver console port (0 to disable console)
  INT32U          trafficReportPeriod;                ///< Traffic report period to configure (in ms)
  INT32U          trafficReportMeasWin;               ///< Traffic report measure window (in ms)
  INT32U          trafficReportThreshold;             ///< Traffic report threshold (in Mbps)
  INT32U          measCollectThreadInt;               ///< Interval of time between thread creation for measure collect process (in ms)
  t_vbLogLevel    verboseLevel;                       ///< Verbose level
  BOOLEAN         serverMode;                         ///< Whether the driver works in server mode
  CHAR            eaRemoteIp[INET6_ADDRSTRLEN];       ///< Engine IP, in client mode
  INT16U          family;                             ///< Family address type (IPv4 or IPv6)
  INT32U          lcmpDefaultTimeout;                 ///< Default timeout for LCMP requests (in ms)
  INT32U          lcmpDefaultNAttempt;                ///< Number of attempt
  t_persistentLog persistentLog;                      ///< Persistent log parameters
} t_vbDriverConf;

/*
 ************************************************************************
 ** Private function prototypes
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_vbDriverConf vbDriverConf;

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_VB_comErrorCode VbDriverPersistentLogParse( ezxml_t persistentLogConf )
{
  t_VB_comErrorCode    ret = VB_COM_ERROR_NONE;
  ezxml_t              ez_temp;

  ez_temp = ezxml_child(persistentLogConf, "NumLines");

  if ((ez_temp != NULL) && (ez_temp->txt != NULL))
  {
    errno = 0;
    vbDriverConf.persistentLog.numLines = strtoul(ez_temp->txt, NULL, 0);

    if (errno != 0)
    {
      printf("ERROR (%d:%s) parsing .ini file: Invalid PersistentLog/NumLines value\n", errno, strerror(errno));
      ret = VB_COM_ERROR_INI_FILE;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(persistentLogConf, "VerboseLevel");

    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      vbDriverConf.persistentLog.verboseLevel = strtoul(ez_temp->txt, NULL, 0);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid PersistentLog/VerboseLevel value\n", errno, strerror(errno));
        ret = VB_COM_ERROR_INI_FILE;
      }
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(persistentLogConf, "Circular");

    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbDriverConf.persistentLog.circular = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode  VbDriverFileInit( const char *path )
{
  t_VB_comErrorCode         error = VB_COM_ERROR_NONE;
  ezxml_t                   driver = NULL;
  ezxml_t                   ez_temp;
  int                       ip_err;

  // Init default values
  vbDriverConf.driverId[0]                = '\0';
  vbDriverConf.lcmpIf[0]                  = '\0';
  vbDriverConf.eaIf[0]                    = '\0';
  vbDriverConf.outputPath[0]              = '\0';
  vbDriverConf.eaPort                     = VB_DRIVER_CONF_DEFAULT_EA_PORT;
  vbDriverConf.consolePort                = VB_DRIVER_CONF_DEFAULT_CONSOLE_PORT;
  vbDriverConf.trafficReportPeriod        = VB_DRIVER_CONF_DEFAULT_TRAFFIC_REPORT_PERIOD;
  vbDriverConf.trafficReportMeasWin       = VB_DRIVER_CONF_DEFAULT_TRAFFIC_REPORT_MEASWIN;
  vbDriverConf.trafficReportThreshold     = VB_DRIVER_CONF_DEFAULT_TRAFFIC_REPORT_THR;
  vbDriverConf.verboseLevel               = VB_DRIVER_CONF_DEFAULT_VERBOSE_LEVEL;
  vbDriverConf.serverMode                 = TRUE;
  vbDriverConf.measCollectThreadInt       = VB_DRIVER_CONF_DEFAULT_MEAS_COLLECT_THREAD_INT;
  vbDriverConf.family                     = PF_UNSPEC;
  vbDriverConf.lcmpDefaultTimeout         = VB_DRIVER_CONF_DEFAULT_LCMP_TIMEOUT;
  vbDriverConf.lcmpDefaultNAttempt        = VB_DRIVER_CONF_DEFAULT_LCMP_N_ATTEMPT;
  vbDriverConf.persistentLog.numLines     = VB_DRIVER_CONF_DEFAULT_PERSLOG_NUMLINES;
  vbDriverConf.persistentLog.verboseLevel = VB_DRIVER_CONF_DEFAULT_PERSLOG_VERBOSE;
  vbDriverConf.persistentLog.circular     = VB_DRIVER_CONF_DEFAULT_PERSLOG_CIRCULAR;

  if (path == NULL)
  {
    error = VB_COM_ERROR_BAD_ARGS;
  }

  if (error == VB_COM_ERROR_NONE)
  {
    driver = ezxml_parse_file(path);

    if (driver == NULL)
    {
      printf("Driver Conf: .ini file could not be opened\n");
      error = VB_COM_ERROR_NOT_FOUND;
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    // Read driver Id
    ez_temp = ezxml_child(driver, "DriverId");
    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      strncpy(vbDriverConf.driverId, ezxml_trimtxt(ez_temp), VB_EA_DRIVER_ID_MAX_SIZE);

      // Force the null byte in last position
      vbDriverConf.driverId[VB_EA_DRIVER_ID_MAX_SIZE - 1] = '\0';
    }
    else
    {
      printf("Driver Conf: Error reading DriverId parameter\n");
      error = VB_COM_ERROR_INI_FILE;
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    // Read ConnectionMode
    ez_temp = ezxml_child(driver, "ServerConnMode");
    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbDriverConf.serverMode = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
    else
    {
      printf("Driver Conf: Error reading ServerConnMode parameter\n");
      error = VB_COM_ERROR_INI_FILE;
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    // Read LCMP Interface
    ez_temp = ezxml_child(driver, "LcmpIf");
    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      strncpy(vbDriverConf.lcmpIf, ezxml_trimtxt(ez_temp), IFNAMSIZ);
      vbDriverConf.lcmpIf[IFNAMSIZ - 1] = '\0';
    }
    else
    {
      printf("Driver Conf: Error reading LcmpIf parameter\n");
      error = VB_COM_ERROR_INI_FILE;
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    // Read TCP Port
    ez_temp = ezxml_child(driver, "Port");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.eaPort = strtol(ezxml_txt(ez_temp), NULL, 0);

      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid Port value\n", errno, strerror(errno));
      }
    }
    else
    {
      printf("Driver Conf: Error reading Port parameter\n");
      error = VB_COM_ERROR_INI_FILE;
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    if (vbDriverConf.serverMode)
    {
      // Read External agent interface
      ez_temp = ezxml_child(driver, "VBEngineIf");
      if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
      {
        strncpy(vbDriverConf.eaIf, ezxml_trimtxt(ez_temp), IFNAMSIZ);
        vbDriverConf.eaIf[IFNAMSIZ - 1] = '\0';
      }
      else
      {
        printf("Driver Conf: Error reading VBEngineIf parameter\n");
        error = VB_COM_ERROR_INI_FILE;
      }
    }
    else
    {
      // Read External Agent IP. Only with driver in client mode
      ez_temp = ezxml_child(driver, "VBEngineIP");
      if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
      {
        CHAR buff[sizeof(struct in6_addr)];

        // Check if address is IPv6
        ip_err = inet_pton(AF_INET6, ezxml_trimtxt(ez_temp), buff);

        if (ip_err != 1)
        {
          // Check if address is IPv4
          ip_err = inet_pton(AF_INET, ezxml_trimtxt(ez_temp),buff);

          if (ip_err != 1)
          {
            printf("Engine Conf: Error %d reading Driver>IP parameter\n", ip_err);
            error = VB_COM_ERROR_INI_FILE;
          }
          else
          {
            vbDriverConf.family = AF_INET;
            strcpy(vbDriverConf.eaRemoteIp,ezxml_trimtxt(ez_temp));
          }
        }
        else
        {
          vbDriverConf.family = AF_INET6;
          strcpy(vbDriverConf.eaRemoteIp,ezxml_trimtxt(ez_temp));
        }
      }
      else
      {
        printf("Driver Conf: Error reading VBEngineIP parameter\n");
        error = VB_COM_ERROR_INI_FILE;
      }
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    // Read OutputPath
    ez_temp = ezxml_child(driver, "OutputPath");
    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      strncpy(vbDriverConf.outputPath, ezxml_trimtxt(ez_temp), VB_PARSE_MAX_PATH_LEN);
      vbDriverConf.outputPath[VB_PARSE_MAX_PATH_LEN - 1] = '\0';
    }
    else
    {
      printf("Driver Conf: Error reading OutputPath parameter\n");
      error = VB_COM_ERROR_INI_FILE;
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "VerboseLevel");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.verboseLevel = (t_vbLogLevel)strtol(ezxml_txt(ez_temp),NULL,10);
      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid VerboseLevel value\n", errno, strerror(errno));
      }
    }
    else
    {
      printf("Driver Conf: Error reading VerboseLevel parameter\n");
      error = VB_COM_ERROR_INI_FILE;
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "TrafficReportPeriod");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.trafficReportPeriod = (INT32U)strtol(ezxml_txt(ez_temp),NULL,10);
      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid traffic report period value\n", errno, strerror(errno));
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "TrafficReportMeasWin");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.trafficReportMeasWin = (INT32U)strtol(ezxml_txt(ez_temp),NULL,10);
      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid traffic report meas win value\n", errno, strerror(errno));
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "TrafficReportThreshold");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.trafficReportThreshold = (INT32U)strtol(ezxml_txt(ez_temp),NULL,10);
      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid traffic report threshold value\n", errno, strerror(errno));
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "MeasCollectThreadInt");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.measCollectThreadInt = (INT32U)strtol(ezxml_txt(ez_temp),NULL,10);
      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid measure collect thread interval value\n", errno, strerror(errno));
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "ConsolePort");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.consolePort = (INT16U)strtol(ezxml_txt(ez_temp), NULL, 0);
      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid ConsolePort value\n", errno, strerror(errno));
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "LcmpDefaultTimeout");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.lcmpDefaultTimeout = (INT32U)strtoul(ezxml_txt(ez_temp), NULL, 0);

      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid LcmpDefaultTimeout value\n", errno, strerror(errno));
      }
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "LcmpDefaultNretries");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbDriverConf.lcmpDefaultNAttempt = (INT32U)strtoul(ezxml_txt(ez_temp), NULL, 0);

      if (errno != 0)
      {
        error = VB_COM_ERROR_INI_FILE;
        printf("ERROR (%d:%s) parsing .ini file: Invalid LcmpDefaultTimeout value\n", errno, strerror(errno));
      }
    }
  }

  if (error == VB_COM_ERROR_NONE)
  {
    ez_temp = ezxml_child(driver, "PersistentLog");

    if (ez_temp != NULL)
    {
      error = VbDriverPersistentLogParse( ez_temp );
    }
  }

  if(driver != NULL)
  {
    ezxml_free(driver);
  }

  return error;
}

/*******************************************************************/

static t_VB_comErrorCode VbDriverConfFileRead(const char *path)
{
  t_VB_comErrorCode err_code = VB_COM_ERROR_NONE;

  if (path == NULL)
  {
    err_code = VB_COM_ERROR_BAD_ARGS;
  }
  else
  {
    err_code = VbDriverFileInit(path);
  }

  return err_code;
}

/************************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/************************************************************************/

t_VB_comErrorCode VbDriverConfParse(int argc, char **argv)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  int              opt;
  CHAR              driver_ini_file[MAX_FILE_NAME_LENGTH] = VB_DRIVER_CONF_DEFAULT_INI_FILE;

  while ((opt = getopt(argc, argv, "cf:h")) != -1)
  {
    switch (opt)
    {
      case ('f'):
          {
        strncpy(driver_ini_file, optarg, MAX_FILE_NAME_LENGTH);
        driver_ini_file[MAX_FILE_NAME_LENGTH - 1] = '\0';
        break;
          }
      default:
      {
        ret = VB_COM_ERROR_BAD_ARGS;
        break;
      }
    }
  }

  if (ret != VB_COM_ERROR_NONE)
  {
    printf("Command line:\n\tvector_boost_driver [-f PATHFILEINI] [-h] \n");
    printf("Where:\n");
    printf("\t-f\tThis option allows the user to select ini file (length max %d).\n\t\tPATHFILEINI has to be the entire path name\n", MAX_FILE_NAME_LENGTH);
    printf("\t-h\tShow this manual\n");
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VbDriverConfFileRead(driver_ini_file);

    if (ret != VB_COM_ERROR_NONE)
    {
      printf("Configuration error (%d) parsing file %s!\n", ret, driver_ini_file);
    }
  }

  return ret;
}

/*******************************************************************/

CHAR *VbDriverConfDriverIdGet(void)
{
  return vbDriverConf.driverId;
}

/*******************************************************************/

BOOL VbDriverConfServerModeGet(void)
{
  return vbDriverConf.serverMode;
}

/*******************************************************************/

CHAR *VbDriverConfRemoteIPGet(void)
{
  return vbDriverConf.eaRemoteIp;
}

/*******************************************************************/

INT16U VbDriverConfFamilyGet(void)
{
  return vbDriverConf.family;
}

/*******************************************************************/

CHAR *VbDriverConfLcmpIfGet(void)
{
  return vbDriverConf.lcmpIf;
}

/*******************************************************************/

CHAR *VbDriverConfEaIfGet(void)
{
  return vbDriverConf.eaIf;
}

/*******************************************************************/

CHAR *VbDriverConfOutputPathGet(void)
{
  return vbDriverConf.outputPath;
}

/*******************************************************************/

t_vbLogLevel VbDriverConfVerboseLevelGet(void)
{
  return vbDriverConf.verboseLevel;
}

/*******************************************************************/

INT16U VbDriverConfEaPortGet(void)
{
  return vbDriverConf.eaPort;
}

/*******************************************************************/

INT16U VbDriverConfConsolePortGet(void)
{
  return vbDriverConf.consolePort;
}

/*******************************************************************/

INT32U VbDriverConfTrafficReportPeriodGet(void)
{
  return vbDriverConf.trafficReportPeriod;
}

/*******************************************************************/

INT32U VbDriverConfTrafficReportMeasWinGet(void)
{
  return vbDriverConf.trafficReportMeasWin;
}

/*******************************************************************/

INT32U VbDriverConfTrafficReportThresholdGet(void)
{
  return vbDriverConf.trafficReportThreshold;
}

/*******************************************************************/

INT32U VbDriverConfMeasCollectThreadIntGet(void)
{
  return vbDriverConf.measCollectThreadInt;
}

/*******************************************************************/

INT32U VbDriverConfLcmpDefaultTimeoutGet(void)
{
  return vbDriverConf.lcmpDefaultTimeout;
}

/*******************************************************************/

INT32U VbDriverConfLcmpDefaultNAttemptGet(void)
{
  return vbDriverConf.lcmpDefaultNAttempt;
}
/*******************************************************************/

void VbDriverConfDump(t_writeFun writeFun)
{
  writeFun("=========================================================================\n");
  writeFun("|                     Parameter                    |        Value       |\n");
  writeFun("=========================================================================\n");
  writeFun("| %-48s |%20s|\n",        "Driver Id",                        vbDriverConf.driverId);
  writeFun("| %-48s | %18s |\n",      "Connection mode",                  vbDriverConf.serverMode ? "SERVER" : "CLIENT");
  writeFun("| %-48s | %18u |\n",      "VBEngine Port",                    vbDriverConf.eaPort);
  writeFun("| %-48s | %18s |\n",      "VBEngine Iface",                   vbDriverConf.eaIf);
  writeFun("| %-48s | %18s |\n",      "LCMP Iface",                       vbDriverConf.lcmpIf);
  writeFun("| %-48s | %18s |\n",      "Debug Output Path",                vbDriverConf.outputPath);
  writeFun("| %-48s | %18u |\n",      "Console Port",                     vbDriverConf.consolePort);
  writeFun("| %-48s | %18s |\n",      "Log Verbose Level",                VbVerboseLevelToStr(vbDriverConf.verboseLevel));
  writeFun("| %-48s | %15u ms |\n",   "Traffic report period",            vbDriverConf.trafficReportPeriod);
  writeFun("| %-48s | %15u ms |\n",   "Traffic report measure win",       vbDriverConf.trafficReportMeasWin);
  writeFun("| %-48s | %13u Mbps |\n", "Traffic report threshold",         vbDriverConf.trafficReportThreshold);
  writeFun("| %-48s | %15u ms |\n",   "Meas collect thread int",          vbDriverConf.measCollectThreadInt);
  writeFun("| %-48s | %15u ms |\n",   "LCMP default timeout",             vbDriverConf.lcmpDefaultTimeout);
  writeFun("| %-48s | %15u ms |\n",   "LCMP default N Attempt",           vbDriverConf.lcmpDefaultNAttempt);
  writeFun("| %-48s | %18u |\n",      "Persistent log - Number of lines", vbDriverConf.persistentLog.numLines);
  writeFun("| %-48s | %18s |\n",      "Persistent log - Verbose level",   VbVerboseLevelToStr(vbDriverConf.persistentLog.verboseLevel));
  writeFun("| %-48s | %18s |\n",      "Persistent log - Circular",        vbDriverConf.persistentLog.circular?"ENABLED":"DISABLED");
  writeFun("=========================================================================\n");
}

/*******************************************************************/

INT32U VbDriverConfPeristentLogNumLinesGet(void)
{
  return vbDriverConf.persistentLog.numLines;
}

/*******************************************************************/

t_vbLogLevel VbDriverConfPeristentLogVerboseLevelGet(void)
{
  return vbDriverConf.persistentLog.verboseLevel;
}

/*******************************************************************/

BOOLEAN VbDriverConfPersistentLogIsCircular(void)
{
  return vbDriverConf.persistentLog.circular;
}

/*******************************************************************/

/**
 * @}
 **/
