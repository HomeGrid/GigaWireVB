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
 * @addtogroup vector_boost_engine
 * @{
 **/

/**
 * @file vb_engine_sock_alive.c
 * @brief VB Drivers socket alive mechanism
 *
 * @internal
 *
 * @author Y. Raoul
 * @date 2019-11-11
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"
#include <time.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "vb_engine_socket_alive.h"
#include "vb_engine_EA_interface.h"
#include "vb_ea_communication.h"
#include "vb_util.h"
#include "vb_thread.h"
#include "vb_timer.h"
#include "vb_priorities.h"
#include "vb_log.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_process.h"
#include "vb_engine_conf.h"

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define SOCKET_ALIVE_WAIT_RSP                   (2000)  // In msec
#define SOCKET_ALIVE_TASK_NAME                  ("SocketAliveMonitor")
#define SOCKET_ALIVE_CHECK_INTERVAL             (10000) // In msec
#define SOCKET_ALIVE_DEAD                       (3)     // Number of consecutive unreplied message from the driver

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

static timer_t          vbEngineSocketAliveMonitorTimer;
static BOOLEAN          vbEngineSocketAliveMonitorRunning = FALSE;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */



/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */
/*******************************************************************/

static void VbEngineSocketAliveMonitorStateSet(BOOLEAN running)
{
  vbEngineSocketAliveMonitorRunning = running;
}

/*******************************************************************/

static BOOLEAN VbEngineSocketAliveMonitorStateGet(void)
{
  return vbEngineSocketAliveMonitorRunning;
}

/*******************************************************************/

static void VbEngineSocketAliveMonitor(sigval_t sigval)
{
  // Send an event to update the clock from all drivers
  VbEngineProcessAllDriversEvSend(ENGINE_EV_ALIVE_SOCK_CHECK, NULL);
}

/*******************************************************************/

static t_VB_engineErrorCode SocketAliveLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (driver->FSMState == ENGINE_STT_DISCONNECTED)
    {
      ret = VB_ENGINE_ERROR_SKIP;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineSocketAliveRequest(driver);
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current driver
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_VB_engineErrorCode VbEngineSocketAliveRequest( t_VBDriver *thisDriver )
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;

  if (thisDriver == NULL)
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(vb_err == VB_ENGINE_ERROR_NONE)
  {
    if(thisDriver->vbEAConnDesc.socketAliveCounter < SOCKET_ALIVE_DEAD)
    {
      t_vbEASocketAliveReq     socket_alive_req;

      socket_alive_req.enable   = VbEngineConfSocketAliveEnableGet();
      socket_alive_req.period   = _htonl(VbEngineConfSocketAlivePeriodGet());
      socket_alive_req.nLostMsg = _htonl(VbEngineConfSocketAliveNLostThrGet());

      thisDriver->vbEAConnDesc.socketAliveCounter++;
      vb_err = VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_SOCKET_ALIVE_REQUEST, (INT8U*)&socket_alive_req, VB_EA_SOCKET_ALIVE_REQ_SIZE, thisDriver);
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Unresponsive Driver -> Send Kill event");
      vb_err = VbEngineProcessClusterXDriversEvSend(ENGINE_EV_KILL, thisDriver, thisDriver->clusterId);
    }
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineSocketAliveRspProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;

  if ((payload == NULL) || (thisDriver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    thisDriver->vbEAConnDesc.socketAliveCounter = 0;
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineSocketAliveCheckReq(void)
{
  t_VB_engineErrorCode ret;

  ret = VbEngineDatamodelDriversLoop(SocketAliveLoopCb, NULL);

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VB_ENGINE_ERROR_SKIP_ALL_DRIVERS_LOOP;
  }

  return ret;
}

/*******************************************************************/

void VbEngineSocketAliveMonitorInit(void)
{
  VbEngineSocketAliveMonitorStateSet(FALSE);
}

/*******************************************************************/

void VbEngineSocketAliveMonitorStop(void)
{
  INT32S err;

  if (VbEngineSocketAliveMonitorStateGet() == TRUE)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopping Socket Alive timed task");

    err = TimerTaskDelete(vbEngineSocketAliveMonitorTimer, SOCKET_ALIVE_TASK_NAME);

    if (err == 0)
    {
      VbEngineSocketAliveMonitorStateSet(FALSE);
    }
  }
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineSocketAliveConfigurationParse(ezxml_t socketAliveConf )
{
  t_VB_engineErrorCode err_code;
  ezxml_t              ez_temp;
  t_socketAlive        *socket_alive_conf = NULL;

  err_code = VbEngineConfSocketAliveGet(&socket_alive_conf);

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    if (socket_alive_conf == NULL)
    {
      err_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(socketAliveConf, "Enable");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      socket_alive_conf->enable = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Enabled value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(socketAliveConf, "Period");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      socket_alive_conf->period = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Period value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(socketAliveConf, "Nlost");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      socket_alive_conf->nLostMsgThr = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Nlost value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  return err_code;
}

/*******************************************************************/

BOOLEAN VbEngineSocketAliveMonitorRun(void)
{
  INT32S err;

  VbEngineSocketAliveMonitorStop();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Starting Socket Alive timed task");

  // Create periodic task
  err = TimerPeriodicTaskSet(SOCKET_ALIVE_TASK_NAME, SOCKET_ALIVE_CHECK_INTERVAL, VbEngineSocketAliveMonitor, NULL, &vbEngineSocketAliveMonitorTimer);

  if (err == 0)
  {
    VbEngineSocketAliveMonitorStateSet(TRUE);
  }
  else
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Can't create Socket Alive monitor timed task!!");
    VbEngineSocketAliveMonitorStateSet(FALSE);
  }


  return VbEngineSocketAliveMonitorStateGet();
}



/*******************************************************************/

/**
 * @}
 **/
