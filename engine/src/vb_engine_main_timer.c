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
 * @file vb_engine_main_timer.c
 * @brief Engine main timer implementation
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-25
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <stdio.h>

#include "types.h"

#include "vb_timer.h"
#include "vb_log.h"
#include "vb_thread.h"
#include "vb_engine_main_timer.h"
#include "vb_engine_drivers_list.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define MAIN_TIMER_NAME                              ("MainTimer")
#define MAIN_TIMER_PERIOD                            (3000) // In msecs
#define MAX_FILE_NAME_SIZE                           (100)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static timer_t vbMainTimerId;
static BOOLEAN vbMainTimerEnabled = FALSE;

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_VB_engineErrorCode StateInfoToFilesCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((driver == NULL) || (domain == NULL) || (node == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT16U               max_xput;
    INT16U               current_xput;

    max_xput = (node->channelSettings.boostInfo.maxCapacity * node->trafficReports.macEfficiency) / 100;
    current_xput = node->trafficReports.ingressTrafficP0 + node->trafficReports.ingressTrafficP1 +
        node->trafficReports.ingressTrafficP2 + node->trafficReports.ingressTrafficP3;

    VbLogSaveStringToTextFile(node->stateFileName, "w+",
        "LineStatus=%u (%u%);PayloadMax=%d;CurrentPayload=%d\n",
        node->channelSettings.boostInfo.level,
        node->channelSettings.boostInfo.perc,
        max_xput, current_xput);
  }

  return ret;
}

/*******************************************************************/

static void VbEngineMainTimerCb(sigval_t sigval)
{
  t_VB_engineErrorCode err;

  // Save channel info to disk
  err = VbEngineDatamodelAllNodesLoop(StateInfoToFilesCb, NULL);

  if (err != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error %d dumping drivers info to files", err);
  }
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

BOOL VbEngineMainTimerRun(void)
{
  INT32S err;

  if (vbMainTimerEnabled == TRUE)
  {
    VbEngineMainTimerStop();
  }

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Starting timer");

  // Create periodic task
  err = TimerPeriodicTaskSet(MAIN_TIMER_NAME, MAIN_TIMER_PERIOD, VbEngineMainTimerCb, NULL, &vbMainTimerId);

  if (err == 0)
  {
    vbMainTimerEnabled = TRUE;
  }
  else
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Can't create timer thread!!");
    vbMainTimerEnabled = FALSE;
  }

  return vbMainTimerEnabled;
}

/*******************************************************************/

void VbEngineMainTimerStop(void)
{
  INT32S err;

  if (vbMainTimerEnabled == TRUE)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopping timer");

    err = TimerTaskDelete(vbMainTimerId, MAIN_TIMER_NAME);

    if (err == 0)
    {
      vbMainTimerEnabled = FALSE;
    }
  }
}

/*******************************************************************/

/**
 * @}
 **/
