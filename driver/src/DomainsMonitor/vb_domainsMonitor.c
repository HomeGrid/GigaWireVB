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
 * @file vb_domainsMonitor.c
 * @brief Domains changes detection feature
 *
 * @internal
 *
 * @author 
 * @date 27/04/2015
 *
 **/

/*
 ************************************************************************ 
 ** Included files
 ************************************************************************
 */
#include <pthread.h>

#include "types.h"

#include "vb_LCMP_com.h"
#include "vb_log.h"
#include "vb_domainsMonitor.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_counters.h"
#include "vb_main.h"

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

#define VB_DOMAINS_MONITOR_THREAD_NAME                  ("DomainsMonitor")
#define VB_DOMAINS_MONITOR_PARAM_PERIOD                 (3000)   // In ms
#define VB_DOMAINS_MONITOR_PARAM_WAIT_SYNC              (20000)  // In ms
#define VB_DOMAINS_MONITOR_NO_WAIT                      (1)      // In ms
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

static pthread_t           vbDomainsMonitorThread = 0;
static BOOLEAN             vbDomainsMonitorThreadRunning = FALSE;
static BOOLEAN             vbDomainsMonitorFirstAtt = TRUE;
static pthread_mutex_t     vbDomainsMonitorMutex;
static pthread_cond_t      vbDomainsMonitorCond;
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

static void VbDomainsMonitorStateSet(BOOLEAN running)
{
  vbDomainsMonitorThreadRunning = running;
}

/*******************************************************************/

static void *VBDomainsMonitorProcess(void *arg)
{
  t_VB_comErrorCode       err = VB_COM_ERROR_NONE;
  BOOLEAN                 run = TRUE;
  BOOLEAN                 net_change;
  BOOLEAN                 send_report = FALSE;
  mqd_t                   vb_main_queue;
  INT32U                  num_domains;
  INT32U                  wait, time_elapsed;
  struct timespec         ts1, ts2;


  vb_main_queue = mq_open(VBQUEUENAME, O_WRONLY);

  vbDomainsMonitorFirstAtt = TRUE;

  while ((VbDomainsMonitorStateGet() == TRUE) && (run == TRUE))
  {
    net_change = FALSE;

    if (VbDomainsMonitorStateGet() == TRUE)
    {
      clock_gettime(CLOCK_MONOTONIC, &ts1);

      // Measurement plan has finished and has signalled us
      err =  VbDatamodelDomainDiscover(&net_change, &num_domains, 0, vbDomainsMonitorFirstAtt);
      if (err == VB_COM_ERROR_RECEIVE_TIMEOUT)
      {
        VbCounterIncrease(VB_DRIVER_COUNTER_NETWORK_DISCOVERY_ERROR);
      }


      if ((err == VB_COM_ERROR_NONE) || (err == VB_COM_ERROR_RECEIVE_TIMEOUT))
      {
        //if network has changed during measurement process this is erroneous
        if ((net_change == TRUE) || (vbDomainsMonitorFirstAtt == TRUE))
        {
          VbLogPrint(VB_LOG_INFO, "Network has changed. Sending event, num_domains %lu", num_domains);

          if (num_domains > 0)
          {
            // Retrieving additional info
            err = VbDatamodelAddInfo1Get();
          }

          send_report = TRUE;

          //Send Network changes Message to Driver
          VbMainQEvSend(DRIVER_EV_NETCHANGE, vb_main_queue, NULL);
        }

        vbDomainsMonitorFirstAtt = FALSE;
      }
    }

    if (VbDomainsMonitorStateGet() == TRUE)
    {
      if(send_report == TRUE)
      {
        // Wait sync from Tx change report
        VbThreadCondSleep(&vbDomainsMonitorMutex, &vbDomainsMonitorCond, VB_DOMAINS_MONITOR_PARAM_WAIT_SYNC);
        send_report = FALSE;
      }

      clock_gettime(CLOCK_MONOTONIC, &ts2);

      time_elapsed = SEC_TO_MS(ts2.tv_sec) + NS_TO_MS(ts2.tv_nsec) - (SEC_TO_MS(ts1.tv_sec) - NS_TO_MS(ts1.tv_nsec));
      wait = (time_elapsed > VB_DOMAINS_MONITOR_PARAM_PERIOD)? VB_DOMAINS_MONITOR_NO_WAIT:(VB_DOMAINS_MONITOR_PARAM_PERIOD - time_elapsed);

      VbLogPrint(VB_LOG_DEBUG, "Domain Monitor wait %lu", wait);
    }

    if (VbDomainsMonitorStateGet() == TRUE)
    {
      // Wait x sec to check for potential changes
      VbThreadCondSleep(&vbDomainsMonitorMutex, &vbDomainsMonitorCond, wait);
    }
  }

  mq_close(vb_main_queue);

  return NULL;
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

void VbDomainsMonitorRun(void)
{
  VbDomainsMonitorStop();

  VbLogPrint(VB_LOG_INFO, "Starting %s thread", VB_DOMAINS_MONITOR_THREAD_NAME);

  VbDomainsMonitorStateSet(TRUE);

  if (FALSE == VbThreadCreate(VB_DOMAINS_MONITOR_THREAD_NAME, VBDomainsMonitorProcess, NULL, VB_DRIVER_DOMAINS_MONITOR_THREAD_PRIORITY, &vbDomainsMonitorThread))
  {
    VbLogPrint(VB_LOG_ERROR,"Can't create %s thread", VB_DOMAINS_MONITOR_THREAD_NAME);

    VbDomainsMonitorStateSet(FALSE);
  }
}

/*******************************************************************/

void VbDomainsMonitorStop()
{
  if (VbDomainsMonitorStateGet() == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", VB_DOMAINS_MONITOR_THREAD_NAME);

    VbDomainsMonitorStateSet(FALSE);

    // Free memory potentially allocated for Domain Report
    VbDatamodelDomainsReportDataFree();

    // Wake up thread
    VbThreadCondWakeUp(&vbDomainsMonitorMutex, &vbDomainsMonitorCond);

    VbThreadJoin(vbDomainsMonitorThread, VB_DOMAINS_MONITOR_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", VB_DOMAINS_MONITOR_THREAD_NAME);
  }
}

/*******************************************************************/

void VbDomainsMonitorInit(void)
{
  INT32S     thr_err;

  vbDomainsMonitorThread = 0;
  VbDomainsMonitorStateSet(FALSE);

  thr_err = VbThreadCondSleepInit(&vbDomainsMonitorMutex, &vbDomainsMonitorCond);

  if (thr_err != 0)
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d initializing cond variable", thr_err);
  }
}

/*******************************************************************/

BOOLEAN VbDomainsMonitorStateGet(void)
{
  return vbDomainsMonitorThreadRunning;
}

/*******************************************************************/

BOOLEAN VbDomainsMonitorFirstAttemptGet(void)
{
  return vbDomainsMonitorFirstAtt;
}

/*******************************************************************/

void VbDomainsMonitorSignal(void)
{
  if (VbDomainsMonitorStateGet() == TRUE)
  {
    INT32S thr_err;

    thr_err = VbThreadCondWakeUp(&vbDomainsMonitorMutex, &vbDomainsMonitorCond);

    if (thr_err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d forcing a network discovery", thr_err);
    }
  }
}

/*******************************************************************/

/**
 * @}
 **/
