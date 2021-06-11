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
 * @file vb_main_timer.c
 * @brief Driver main timer implementation
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-12
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <string.h>

#include "types.h"

#include "vb_timer.h"
#include "vb_DataModel.h"
#include "vb_main.h"
#include "vb_log.h"
#include "vb_LCMP_com.h"
#include "vb_EA_interface.h"
#include "vb_counters.h"
#include "vb_main_timer.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define MAIN_TIMER_NAME                              ("MainTimer")
#define SOCKET_TIMER_NAME                            ("SocketTimer")

#define VB_NUM_MACCYCLES_TO_KEEPALIVE                (100)
#define KEEPALIVE_SEND_PERIOD                        (MAC_CYCLE_DURATION * VB_NUM_MACCYCLES_TO_KEEPALIVE) // In ms

#define SOCKET_ALIVE_PERIOD_GUARD_TIME               (500)   // In ms

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct __attribute__ ((packed))
{
  INT8U Param;
  INT8U driverState;
  INT8U engineState;
  INT16U lowBanbInUse;
  INT16U period;
} t_VB_KeepAlive_Value;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static timer_t vbMainTimerId;
static timer_t vbMainSocketAliveTimerId;

static BOOLEAN vbMainTimerEnabled = FALSE;
static BOOLEAN vbMainSocketAliveTimerEnabled = FALSE;
static INT32U  vbMainSocketAliveNConsecLostThr = 0;

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/



/*******************************************************************/

static void VbMainTimerCb(sigval_t sigval)
{
  // Send Keep Alive
  VbMainKeepAliveSend();
}

/*******************************************************************/

static void VbMainSocketAliveTimerCb(sigval_t sigval)
{
  t_vbEADesc *desc = (t_vbEADesc *)sigval.sival_ptr;
  // Send Keep Alive

  if(desc != NULL)
  {
    desc->socketAliveCounter++;
    if(desc->socketAliveCounter > vbMainSocketAliveNConsecLostThr)
    {
      mqd_t             vb_queue;

      // Connect to main queue
      VbLogPrint(VB_LOG_ERROR, "Socket Alive kill %lu", desc->socketAliveCounter);

      vb_queue = mq_open(VBQUEUENAME, O_WRONLY);
      VbMainQEvSend(DRIVER_EV_KILL, vb_queue, NULL);
    }
  }
}
/*******************************************************************/

/*******************************************************************/

static t_VB_comErrorCode VbMainTermAliveSend( void )
{
  t_VB_comErrorCode    result = VB_COM_ERROR_NONE;
  t_ValuesArray       *tlv_notify_values = NULL;
  t_HGF_LCMP_ErrorCode lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams      lcmp_params;
  t_VB_KeepAlive_Value keep_alive_value;
  INT8U               *buffer;

  bzero(&keep_alive_value, sizeof(keep_alive_value));
  keep_alive_value.Param = VB_KEEP_ALIVE_IND;
  keep_alive_value.driverState = 0;
  keep_alive_value.engineState = 0;
  keep_alive_value.lowBanbInUse = 0;
  keep_alive_value.period = _htons_ghn(VB_NUM_MACCYCLES_TO_KEEPALIVE);

  buffer = (INT8U *)calloc(1,sizeof(keep_alive_value) + strlen(VERSION) + 1);

  if (buffer != NULL)
  {
    bzero(buffer, sizeof(keep_alive_value) + strlen(VERSION) + 1);
    memcpy(buffer, (INT8U *)&keep_alive_value, sizeof(t_VB_KeepAlive_Value));
    memcpy(&buffer[sizeof(t_VB_KeepAlive_Value)],VERSION,strlen(VERSION));

    result = VbDatamodelValueToArrayAdd( sizeof(keep_alive_value) + strlen(VERSION) + 1,buffer,
        &tlv_notify_values);

    if (result == VB_COM_ERROR_NONE)
    {
      // Init LCMP params
      lcmp_err = VbLcmpParamsInit(&lcmp_params);

      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
        result = VB_COM_ERROR_LCMP;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      // Configure LCMP params
      lcmp_params.transmisionType = MULTICAST_DMS;
      lcmp_params.paramIdReq      = VB_KEEP_ALIVE_IND;
      lcmp_params.reqValues       = tlv_notify_values;

      // Send Keep Alive message
      lcmp_err = VbLcmpNotify(&lcmp_params);

      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d sending Alive.ind", lcmp_err);
        result = VB_COM_ERROR_LCMP_WRITE;
      }
      else
      {
        VbCounterIncrease(VB_DRIVER_COUNTER_KEEP_ALIVE_TX);
      }
    }

    VbDatamodelHTLVsArrayDestroy( &tlv_notify_values );
    free(buffer);
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Error Malloc");
    result = VB_COM_ERROR_MALLOC;
  }

  return result;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

BOOL VbMainTimerRun(void)
{
  INT32S err;

  if (vbMainTimerEnabled == TRUE)
  {
    VbMainTimerStop();
  }

  VbLogPrint(VB_LOG_INFO, "Starting timer");

  // Create periodic task
  err = TimerPeriodicTaskSet(MAIN_TIMER_NAME, KEEPALIVE_SEND_PERIOD, VbMainTimerCb, NULL, &vbMainTimerId);

  if (err == 0)
  {
    vbMainTimerEnabled = TRUE;
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Can't create %s timer thread!!", MAIN_TIMER_NAME);
    vbMainTimerEnabled = FALSE;
  }

  return vbMainTimerEnabled;
}

/*******************************************************************/

BOOL VbMainSocketAliveTimerStatusGet(void)
{
  return vbMainSocketAliveTimerEnabled;
}

/*******************************************************************/

BOOL VbMainSocketAliveTimerRun(t_vbEADesc *desc, INT32U period, INT32U nMsgLostThr)
{
  INT32S err;

  if (vbMainSocketAliveTimerEnabled == TRUE)
  {
    VbMainSocketAliveTimerStop();
  }

  vbMainSocketAliveNConsecLostThr = nMsgLostThr;

  VbLogPrint(VB_LOG_INFO, "Starting timer, period %lu", period);

  // Create periodic task
  err = TimerPeriodicTaskSet(SOCKET_TIMER_NAME, (period+SOCKET_ALIVE_PERIOD_GUARD_TIME), VbMainSocketAliveTimerCb, desc, &vbMainSocketAliveTimerId);

  if (err == 0)
  {
    vbMainSocketAliveTimerEnabled = TRUE;
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Can't create %s timer thread!!", MAIN_TIMER_NAME);
    vbMainSocketAliveTimerEnabled = FALSE;
  }

  return vbMainSocketAliveTimerEnabled;
}

/*******************************************************************/

void VbMainSocketAliveTimerStop(void)
{
  INT32S err;

  if (vbMainSocketAliveTimerEnabled == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", SOCKET_TIMER_NAME);

    err = TimerTaskDelete(vbMainSocketAliveTimerId, SOCKET_TIMER_NAME);

    if (err == 0)
    {
      vbMainSocketAliveTimerEnabled = FALSE;
    }

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", SOCKET_TIMER_NAME);
  }
}

/*******************************************************************/

void VbMainTimerStop(void)
{
  INT32S err;

  if (vbMainTimerEnabled == TRUE)
  {
    // Inform node that the driver is going to shut down
    VbMainTermAliveSend();

    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", MAIN_TIMER_NAME);

    err = TimerTaskDelete(vbMainTimerId, MAIN_TIMER_NAME);

    if (err == 0)
    {
      vbMainTimerEnabled = FALSE;
    }

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", MAIN_TIMER_NAME);
  }
}

/*******************************************************************/

t_VB_comErrorCode VbMainKeepAliveSend( void )
{
  t_VB_comErrorCode    result = VB_COM_ERROR_NONE;
  t_ValuesArray       *tlv_notify_values = NULL;
  t_HGF_LCMP_ErrorCode lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams      lcmp_params;
  t_VB_KeepAlive_Value keep_alive_value;
  INT8U               *buffer;

  bzero(&keep_alive_value, sizeof(keep_alive_value));
  keep_alive_value.Param = VB_KEEP_ALIVE_IND;
  keep_alive_value.driverState = 1;
  keep_alive_value.engineState = VbEAEngineIsConnected();
  keep_alive_value.lowBanbInUse = 0;
  keep_alive_value.period = _htons_ghn(VB_NUM_MACCYCLES_TO_KEEPALIVE);

  buffer = (INT8U *)calloc(1,sizeof(keep_alive_value) + strlen(VERSION) + 1);

  if (buffer != NULL)
  {
    bzero(buffer, sizeof(keep_alive_value) + strlen(VERSION) + 1);
    memcpy(buffer, (INT8U *)&keep_alive_value, sizeof(t_VB_KeepAlive_Value));
    memcpy(&buffer[sizeof(t_VB_KeepAlive_Value)],VERSION,strlen(VERSION));

    result = VbDatamodelValueToArrayAdd( sizeof(keep_alive_value) + strlen(VERSION) + 1,buffer,
        &tlv_notify_values);

    if (result == VB_COM_ERROR_NONE)
    {
      // Init LCMP params
      lcmp_err = VbLcmpParamsInit(&lcmp_params);

      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
        result = VB_COM_ERROR_LCMP;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      // Configure LCMP params
      lcmp_params.transmisionType = MULTICAST_DMS;
      lcmp_params.paramIdReq      = VB_KEEP_ALIVE_IND;
      lcmp_params.reqValues       = tlv_notify_values;

      // Send Keep Alive message
      lcmp_err = VbLcmpNotify(&lcmp_params);

      if (lcmp_err != HGF_LCMP_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d sending Alive.ind", lcmp_err);
        result = VB_COM_ERROR_LCMP_WRITE;
      }
      else
      {
        VbCounterIncrease(VB_DRIVER_COUNTER_KEEP_ALIVE_TX);
      }
    }

    VbDatamodelHTLVsArrayDestroy( &tlv_notify_values );
    free(buffer);
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Error Malloc");
    result = VB_COM_ERROR_MALLOC;
  }

  return result;
}


/**
 * @}
 **/
