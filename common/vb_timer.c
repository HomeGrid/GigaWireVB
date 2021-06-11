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
 * @file vb_timer.c
 * @brief Common timers and clock functions implementation
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017/05/09
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

#include "types.h"
#include "vb_log.h"
#include "vb_timer.h"
#include "vb_util.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define TASK_MAX_NUM                (50)
#define TASK_NAME_LEN               (30)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  CHAR    name[TASK_NAME_LEN];
  INT32U  cnt;
  BOOLEAN used;
} t_taskListEntry;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_mutex_t       vbTimerListMutex;
static t_taskListEntry       vbTimerTaskList[TASK_MAX_NUM];

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VbTimerListTaskStart(const CHAR *name)
{
  BOOLEAN found = FALSE;
  INT32U  i;

  if (name != NULL)
  {
    pthread_mutex_lock(&vbTimerListMutex);

    // Search for given thread in list
    for (i = 0; i < TASK_MAX_NUM; i++)
    {
      if ((vbTimerTaskList[i].used == TRUE) &&
          (strncmp(vbTimerTaskList[i].name, name, TASK_NAME_LEN) == 0))
      {
        // Thread found in list, update counter
        vbTimerTaskList[i].cnt++;

        found = TRUE;
        break;
      }
    }

    if (found == FALSE)
    {
      // Search for a free entry
      for (i = 0; i < TASK_MAX_NUM; i++)
      {
        if (vbTimerTaskList[i].used == FALSE)
        {
          // Free entry found, copy thread info
          vbTimerTaskList[i].used = TRUE;
          vbTimerTaskList[i].cnt = 1;
          strncpy(vbTimerTaskList[i].name, name, TASK_NAME_LEN);
          vbTimerTaskList[i].name[TASK_NAME_LEN - 1] = '\0';

          break;
        }
      }
    }

    pthread_mutex_unlock(&vbTimerListMutex);
  }
}

/*******************************************************************/

static void VbTimerListTaskStop(const CHAR *name)
{
  INT32U  i;

  if (name != NULL)
  {
    pthread_mutex_lock(&vbTimerListMutex);

    // Search for given thread in list
    for (i = 0; i < TASK_MAX_NUM; i++)
    {
      if ((vbTimerTaskList[i].used == TRUE) &&
          (strncmp(vbTimerTaskList[i].name, name, TASK_NAME_LEN) == 0))
      {
        // Thread found in list, update counter
        if (vbTimerTaskList[i].cnt > 0)
        {
          vbTimerTaskList[i].cnt--;
        }

        break;
      }
    }

    pthread_mutex_unlock(&vbTimerListMutex);
  }
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

INT32S VbTimerInit(void)
{
  pthread_mutex_init(&vbTimerListMutex, NULL);
  memset(vbTimerTaskList, 0, sizeof(vbTimerTaskList));

  return 0;
}

/*******************************************************************/

INT32S TimerPeriodicTaskSet(const CHAR *name, INT32U periodMs, t_timerTaskCb handler, void *args, timer_t *timerId)
{
  struct sigevent         te;
  struct itimerspec       its;
  int                      err;
  INT32S                   ret = 0;
  INT32U                   msecs;

  if ((periodMs == 0) || (handler == NULL) || (timerId == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    // Configure timer
    te.sigev_notify = SIGEV_THREAD;
    te.sigev_notify_function = handler;
    te.sigev_notify_attributes = NULL;
    te.sigev_value.sival_ptr = args;

    err = timer_create(CLOCK_MONOTONIC, &te, timerId);

    if (err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%s) creating timer", strerror(errno));
      ret = -1;
    }
  }

  if (ret == 0)
  {
    its.it_interval.tv_sec = MS_TO_SEC(periodMs);

    msecs = periodMs - SEC_TO_MS(its.it_interval.tv_sec);
    its.it_interval.tv_nsec = MS_TO_NS(msecs);
    its.it_value = its.it_interval;
    err = timer_settime(*timerId, 0, &its, NULL);

    if (err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%s) setting timer", strerror(errno));

      // Release acquired resource
      timer_delete(*timerId);

      ret = -1;
    }
  }

  if ((ret == 0) && (name != NULL))
  {
    VbTimerListTaskStart(name);
  }

  return ret;
}

/*******************************************************************/

INT32S TimerOneShotTaskSet(const CHAR *name, INT32U timeoutMs, t_timerTaskCb handler, void *args, timer_t *timerId)
{
  struct sigevent         te;
  struct itimerspec       its;
  int                      err;
  INT32S                   ret = 0;
  INT32U                   msecs;

  if ((timeoutMs == 0) || (handler == NULL) || (timerId == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    // Configure timer
    te.sigev_notify = SIGEV_THREAD;
    te.sigev_notify_function = handler;
    te.sigev_notify_attributes = NULL;
    te.sigev_value.sival_ptr = args;

    err = timer_create(CLOCK_MONOTONIC, &te, timerId);

    if (err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%s) creating timer", strerror(errno));
      ret = -1;
    }
  }

  if (ret == 0)
  {
    // Configure timeout
    its.it_value.tv_sec = MS_TO_SEC(timeoutMs);
    msecs = timeoutMs - SEC_TO_MS(its.it_value.tv_sec);
    its.it_value.tv_nsec = MS_TO_NS(msecs);

    // Configure one shot setting interval to 0
    bzero(&(its.it_interval), sizeof(its.it_interval));

    err = timer_settime(*timerId, 0, &its, NULL);

    if (err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%s) setting timer", strerror(errno));
      ret = -1;
    }
  }

  if ((ret == 0) && (name != NULL))
  {
    VbTimerListTaskStart(name);
  }

  return ret;
}

/*******************************************************************/

INT32S TimerTaskDelete(timer_t timerId, const CHAR *name)
{
  INT32S  ret = 0;
  int     err;

  err = timer_delete(timerId);

  if (err != 0)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%s) releasing timer", strerror(errno));
    ret = -1;
  }

  if ((ret == 0) && (name != NULL))
  {
    VbTimerListTaskStop(name);
  }

  return ret;
}

/*******************************************************************/

void VbTimerListTaskDump(t_writeFun writeFun)
{
  INT32U  num_tasks = 0;
  INT32U  i;

  if (writeFun != NULL)
  {
    writeFun("\nTimed tasks:\n");
    writeFun("==========================================\n");
    writeFun("|            Name              |   Cnt   |\n");
    writeFun("==========================================\n");

    pthread_mutex_lock(&vbTimerListMutex);

    // Loop through thread list
    for (i = 0; i < TASK_MAX_NUM; i++)
    {
      if (vbTimerTaskList[i].used == TRUE)
      {
        writeFun("|%-30s| %7lu |\n", vbTimerTaskList[i].name, vbTimerTaskList[i].cnt);

        if (vbTimerTaskList[i].cnt > 0)
        {
          num_tasks++;
        }
      }
    }

    pthread_mutex_unlock(&vbTimerListMutex);

    writeFun("==========================================\n");
    writeFun("Number of timed tasks running : %u\n", num_tasks);
  }
}

/*******************************************************************/

/**
 * @}
**/


