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
 * @file vb_engine_clock.c
 * @brief VB Drivers clock tracking
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-08
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"
#include <time.h>
#include <pthread.h>
#include <strings.h>

#include "vb_engine_clock.h"
#include "vb_engine_EA_interface.h"
#include "vb_util.h"
#include "vb_thread.h"
#include "vb_timer.h"
#include "vb_priorities.h"
#include "vb_log.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_engine_process.h"

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

#define CLOCK_TASK_NAME                  ("ClockMonitor")
#define CLOCK_FUTURE_FIXED_GUARD         (500)   // In msec
#define CLOCK_WAIT_RSP                   (2000)  // In msec
#define CLOCK_UPDATE_INTERVAL            (60000) // In msec
#define CLOCK_MAX_DEV_SPAN               (20)    // In msec

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  INT64S           maxDev;
  INT64S           minDev;
  INT64S           maxRtt;
} t_clockStats;

typedef struct
{
  struct timespec  futureClock;
  struct timespec  futureOwnClock;
  INT16U           futureSeqNum;
  BOOLEAN          found;
} t_clockFuture;

typedef struct
{
  t_clockStats     stats;
  INT64S           referenceDev;
  BOOLEAN          referenceFound;
} t_clockDevCheck;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static timer_t          vbEngineClockMonitorTimer;
static BOOLEAN          vbEngineClockMonitorRunning = FALSE;
static t_clockStats     vbEngineClockStats;
static pthread_mutex_t  vbEngineClockStatsMutex;

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

static void VbEngineClockMonitorStateSet(BOOLEAN running)
{
  vbEngineClockMonitorRunning = running;
}

/*******************************************************************/

static BOOLEAN VbEngineClockMonitorStateGet(void)
{
  return vbEngineClockMonitorRunning;
}

/*******************************************************************/

static void VbEngineClockMonitor(sigval_t sigval)
{
  // Send an event to update the clock from all drivers
  VbEngineProcessAllDriversEvSend(ENGINE_EV_CLOCK_REQ, NULL);

  // Wait to receive Clock responses
  VbThreadSleep(CLOCK_WAIT_RSP);

  // Check maximum deviation of Drivers time
  VbEngineClockAllDriversNTPDeviationCheck();
}

/*******************************************************************/

static t_VB_engineErrorCode CurrentDriverClockCalc(t_VBDriver *driver, struct timespec *ownCurrClock, struct timespec *driverCurrClock, INT16U *numSeq)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  struct timespec     own_clock_monotonic;
  INT64S               elapsed_time_in_us;

  if ((driver == NULL) || (ownCurrClock == NULL) || (driverCurrClock == NULL) || (numSeq == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check time elapsed since last clock measure

    // Get own time
    clock_gettime(CLOCK_MONOTONIC, &own_clock_monotonic);

    elapsed_time_in_us = VbUtilElapsetimeTimespecUs(&(driver->time.lastClockReqTS), &own_clock_monotonic);

    if (elapsed_time_in_us > (2 * MS_TO_US(CLOCK_UPDATE_INTERVAL)))
    {
      // Timings out-of-date
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "Clock measure out-of-date");
      ret = VB_ENGINE_ERROR_SKIP;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Check if sequence number is valid
    if (driver->time.lastSeqNumberValid == FALSE)
    {
      // When driver has no devices, sequence number is not valid, so do not select this driver as time reference
      ret = VB_ENGINE_ERROR_SKIP;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    struct timespec diff;
    INT64S           elapsed_time_in_ms;
    INT16U           elapsed_num_cycles;

    // Get own time
    clock_gettime(CLOCK_REALTIME, ownCurrClock);

    // Calculate elapsed time since last driver clock update
    VbUtilTimespecSubtract(&(driver->time.ownClock), ownCurrClock, &diff);

    // Calculate current driver clock (aprox)
    VbUtilTimespecAdd(&(driver->time.adjClock), &diff, driverCurrClock);

    // Calculate elapsed time from own clock
    elapsed_time_in_ms = VbUtilElapsetimeTimespecMs(driver->time.ownClock, *ownCurrClock);

    // Calculate number of MAC cycles
    elapsed_num_cycles = elapsed_time_in_ms / MAC_CYCLE_DURATION;

    // Calculate current MAC cycle
    *numSeq = driver->time.lastSeqNumberRx + elapsed_num_cycles;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode FutureClockDriverLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  struct timespec     own_curr_clock;
  struct timespec     driver_curr_clock;
  INT16U               driver_curr_seq = 0;

  if ((args == NULL) || (driver == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&(driver->time.mutex));

    if (driver->FSMState == ENGINE_STT_DISCONNECTED)
    {
      ret = VB_ENGINE_ERROR_SKIP;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      // Get current Driver time
      ret = CurrentDriverClockCalc(driver, &own_curr_clock, &driver_curr_clock, &driver_curr_seq);
    }

    pthread_mutex_unlock(&(driver->time.mutex));
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    t_clockFuture *future_params;
    INT32U         guard_time;

    // Get args
    future_params = (t_clockFuture *)args;

    pthread_mutex_lock(&vbEngineClockStatsMutex);
    // Take into account RTT to calculate guard time
    guard_time = (INT32U)(NS_TO_MS(vbEngineClockStats.maxRtt)) + CLOCK_FUTURE_FIXED_GUARD;
    pthread_mutex_unlock(&vbEngineClockStatsMutex);

    // Add guard time to current time
    VbUtilTimespecMsecAdd(&driver_curr_clock, guard_time, &(future_params->futureClock));

    // Add guard time to current Own time
    VbUtilTimespecMsecAdd(&own_curr_clock, guard_time, &(future_params->futureOwnClock));

    // Calculate seq number
    future_params->futureSeqNum = driver_curr_seq + (guard_time / MAC_CYCLE_DURATION);

    // Mark as found
    future_params->found = TRUE;

    // Break drivers loop
    ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current driver
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClockMonitorLoopCb(t_VBDriver *driver, void *args)
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
    ret = VbEngineClockRequest(driver);
  }

  if (ret == VB_ENGINE_ERROR_SKIP)
  {
    // Expected error code. Skip current driver
    ret = VB_ENGINE_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode ClockDeviationCheckLoopCb(t_VBDriver *driver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_clockDevCheck     *dev_param = (t_clockDevCheck *)args;

  if ((driver == NULL) || (dev_param == NULL))
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
    pthread_mutex_lock(&(driver->time.mutex));

    if (dev_param->referenceFound == FALSE)
    {
      // Take first friver as reference to compare to
      dev_param->referenceDev = driver->time.deviationAbs;
      dev_param->referenceFound = TRUE;
    }

    // Calculate deviation regarding first driver
    driver->time.deviationRel = driver->time.deviationAbs - dev_param->referenceDev;

    // Calculate min relative deviation
    if (driver->time.deviationRel < dev_param->stats.minDev)
    {
      dev_param->stats.minDev = driver->time.deviationRel;
    }

    // Calculate max relative deviation
    if (driver->time.deviationRel > dev_param->stats.maxDev)
    {
      dev_param->stats.maxDev = driver->time.deviationRel;
    }

    // Calculate max RTT
    if (driver->time.rttNs > dev_param->stats.maxRtt)
    {
      dev_param->stats.maxRtt = driver->time.rttNs;
    }

    pthread_mutex_unlock(&(driver->time.mutex));
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

t_VB_engineErrorCode VbEngineClockRequest( t_VBDriver *thisDriver )
{
  pthread_mutex_lock(&(thisDriver->time.mutex));

  // Store timestamp when sending clock request
  clock_gettime(CLOCK_MONOTONIC, &(thisDriver->time.lastClockReqTS));

  pthread_mutex_unlock(&(thisDriver->time.mutex));

  return VbEngineEAInterfaceSendFrame(VB_EA_OPCODE_CLOCK_REQ, NULL, 0, thisDriver);
}

/*******************************************************************/

t_VB_engineErrorCode    VbEngineClockRspProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode vb_err = VB_ENGINE_ERROR_NONE;

  if ((payload == NULL) || (thisDriver == NULL))
  {
    vb_err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (vb_err == VB_ENGINE_ERROR_NONE)
  {
    t_vbEAClockRsp *clock_rsp;
    struct timespec end_time;
    INT32S          err;
    INT64S          rx_delay_from_driver;

    pthread_mutex_lock(&(thisDriver->time.mutex));

    clock_gettime(CLOCK_REALTIME, &(thisDriver->time.ownClock));

    clock_gettime(CLOCK_MONOTONIC, &end_time);
    thisDriver->time.rttNs = VbUtilElapsetimeTimespecNs(&(thisDriver->time.lastClockReqTS), &end_time);

    clock_rsp = (t_vbEAClockRsp *)payload;
    thisDriver->time.lastClockRx.tv_sec = _ntohl(clock_rsp->tvSec);
    thisDriver->time.lastClockRx.tv_nsec = _ntohl(clock_rsp->tvNsec);
    thisDriver->time.lastSeqNumberRx = _ntohs(clock_rsp->seqNumber);
    thisDriver->time.lastSeqNumberValid = clock_rsp->validSeqNum;

    // Calculate adjusted clock:

    // It is assumed than delay when receiving a message from driver is RTT/2
    rx_delay_from_driver = NS_TO_US(thisDriver->time.rttNs) / 2;

    // Adjust
    err = VbUtilTimespecUsecAdd(&(thisDriver->time.lastClockRx), rx_delay_from_driver, &(thisDriver->time.adjClock));

    if (err != 0)
    {
      vb_err = VB_ENGINE_ERROR_CLOCK;
    }

    if (vb_err == VB_ENGINE_ERROR_NONE)
    {
      // Calculate driver clock deviation (in usec)
      thisDriver->time.deviationAbs = US_TO_MS(VbUtilDiffTimespecUs(&(thisDriver->time.adjClock), &(thisDriver->time.ownClock)));
    }

    pthread_mutex_unlock(&(thisDriver->time.mutex));
  }

  return vb_err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClockFutureTSGet(struct timespec *futureTs, INT32U clusterId, INT16U *futureSeqNum, struct timespec *futureTsOwnClock)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_clockFuture        future_params;

  bzero(&future_params, sizeof(future_params));

  if ((futureTs == NULL) && (futureSeqNum == NULL) && (futureTsOwnClock == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    future_params.found = FALSE;
    ret = VbEngineDatamodelClusterXDriversLoop(FutureClockDriverLoopCb, clusterId, &future_params);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (future_params.found == TRUE)
    {
      if (futureTs != NULL)
      {
        *futureTs = future_params.futureClock;
      }

      if (futureSeqNum != NULL)
      {
        *futureSeqNum = future_params.futureSeqNum;
      }

      if (futureTsOwnClock != NULL)
      {
        *futureTsOwnClock = future_params.futureOwnClock;
      }

      ret = VbEngineClusterTimeApplyUpdate(clusterId, future_params.futureSeqNum, future_params.futureOwnClock, future_params.futureClock);
    }
    else
    {
      ret = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClockAllDriversUpdate(void)
{
  t_VB_engineErrorCode ret;

  ret = VbEngineDatamodelDriversLoop(ClockMonitorLoopCb, NULL);

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClockAllDriversNTPDeviationCheck(void)
{
  t_VB_engineErrorCode ret;
  INT64S               max_rtt;
  INT64S               max_dev;
  INT64S               min_dev;
  INT64S               max_dev_span;

  // Check clock deviations
  ret = VbEngineClockDevCalc(&max_dev, &min_dev, &max_rtt);

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    pthread_mutex_lock(&vbEngineClockStatsMutex);
    vbEngineClockStats.maxDev = max_dev;
    vbEngineClockStats.minDev = min_dev;
    vbEngineClockStats.maxRtt = max_rtt;
    pthread_mutex_unlock(&vbEngineClockStatsMutex);

    max_dev_span = max_dev - min_dev;

    if (max_dev_span > CLOCK_MAX_DEV_SPAN)
    {
      VbLogPrintExt(VB_LOG_WARNING, VB_ENGINE_ALL_DRIVERS_STR, "NTP Check KO, Maximum clock deviation reached! : max_dev %lld; min_dev %lld; max_dev_span %lld",
                    max_dev, min_dev, max_dev_span);
    }
    else
    {
      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "NTP Check OK : max_dev %lld; min_dev %lld; max_dev_span %lld",
                    max_dev, min_dev, max_dev_span);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClockClusterUpdate(INT32U clusterId)
{
  t_VB_engineErrorCode ret;

  ret = VbEngineDatamodelClusterXDriversLoop(ClockMonitorLoopCb, clusterId, NULL);

  return ret;
}

/*******************************************************************/

void VbEngineClockMonitorInit(void)
{
  VbEngineClockMonitorStateSet(FALSE);
  pthread_mutex_lock(&vbEngineClockStatsMutex);
  bzero(&vbEngineClockStats, sizeof(vbEngineClockStats));
  pthread_mutex_unlock(&vbEngineClockStatsMutex);
}

/*******************************************************************/

void VbEngineClockMonitorStop(void)
{
  INT32S err;

  if (VbEngineClockMonitorStateGet() == TRUE)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopping clock timed task");

    err = TimerTaskDelete(vbEngineClockMonitorTimer, CLOCK_TASK_NAME);

    if (err == 0)
    {
      VbEngineClockMonitorStateSet(FALSE);
    }
    pthread_mutex_destroy(&vbEngineClockStatsMutex);
  }
}

/*******************************************************************/

BOOLEAN VbEngineClockMonitorRun(void)
{
  INT32S err;

  VbEngineClockMonitorStop();

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Starting clock timed task");

  pthread_mutex_init(&vbEngineClockStatsMutex, NULL);

  // Create periodic task
  err = TimerPeriodicTaskSet(CLOCK_TASK_NAME, CLOCK_UPDATE_INTERVAL, VbEngineClockMonitor, NULL, &vbEngineClockMonitorTimer);

  if (err == 0)
  {
    VbEngineClockMonitorStateSet(TRUE);
  }
  else
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Can't create clock monitor timed task!!");
    VbEngineClockMonitorStateSet(FALSE);
  }

  return VbEngineClockMonitorStateGet();
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineClockDevCalc(INT64S *maxDev, INT64S *minDev, INT64S *maxRtt)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_clockDevCheck      clock_dev_params;

  if ((maxDev == NULL) || (minDev == NULL) || (maxRtt == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    clock_dev_params.referenceFound = FALSE;
    clock_dev_params.stats.maxDev = 0;
    clock_dev_params.stats.maxRtt = 0;
    clock_dev_params.stats.minDev = 0;

    ret = VbEngineDatamodelDriversLoop(ClockDeviationCheckLoopCb, &clock_dev_params);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *maxDev = clock_dev_params.stats.maxDev;
    *minDev = clock_dev_params.stats.minDev;
    *maxRtt = clock_dev_params.stats.maxRtt;
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
