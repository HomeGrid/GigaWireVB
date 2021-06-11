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
 * @file vb_counters.c
 * @brief Counters to test the VectorBoost
 *
 * @internal
 *
 * @author
 * @date 22/12/2014
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <string.h>
#include <pthread.h>

#include "vb_util.h"
#include "vb_counters.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_COUNTERS_MAX                 (100)
#define VB_COUNTERS_MAX_NAME_LENGTH      (40)

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

static pthread_mutex_t vbCountersMutex;
static INT32U          vbCounters[VB_COUNTERS_MAX];
static CHAR           *vbCountersNames[VB_COUNTERS_MAX];
static BOOL            vbCountersUsed[VB_COUNTERS_MAX];
static struct timespec vbCountersRunTime;
static void           (*vbCountersExtraReset)( void );

/*
 ************************************************************************
 ** Private function declaration
 ************************************************************************
 */

static t_vb_counter_error VbCountersRunTimeInit( void );

static t_vb_counter_error VbCountersReset( void );

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_vb_counter_error VbCountersRunTimeInit( void )
{
  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;

  clock_gettime(CLOCK_MONOTONIC, &vbCountersRunTime);

  return err;

}

/*******************************************************************/

static t_vb_counter_error VbCountersReset( void )
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;
  INT16U i;

  pthread_mutex_lock(&vbCountersMutex);

  for(i = 0 ; i < VB_COUNTERS_MAX ; i++ )
  {
    vbCounters[i] = 0;
  }

  pthread_mutex_unlock(&vbCountersMutex);

  if(vbCountersExtraReset != NULL)
  {
    vbCountersExtraReset();
  }
  return err;
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

t_vb_counter_error VbCountersInit( void (*CountersExtraReset)( void ) )
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;
  INT16U i;

  pthread_mutex_init(&vbCountersMutex, NULL);

  VbCountersRunTimeInit();

  for(i = 0 ; i < VB_COUNTERS_MAX ; i++ )
  {
    vbCounters[i] = 0;
    vbCountersUsed[i] = FALSE;
    vbCountersNames[i] = NULL;
  }

  vbCountersExtraReset = CountersExtraReset;

  return err;
}

/*******************************************************************/

t_vb_counter_error VbCounterInstall(INT16U conuterIndex, const CHAR *counterName)
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;

  pthread_mutex_lock(&vbCountersMutex);

  if(conuterIndex >= VB_COUNTERS_MAX)
  {
    err = VB_COUNTERS_ERROR_MAX_EXCEED;
  }
  else if(vbCountersUsed[conuterIndex])
  {
    err = VB_COUNTERS_ERROR_USED;
  }
  else
  {
    vbCountersUsed[conuterIndex] = TRUE;
    vbCounters[conuterIndex] = 0;
    vbCountersNames[conuterIndex] = (char *)counterName;
  }

  pthread_mutex_unlock(&vbCountersMutex);

  return err;
}

/*******************************************************************/

t_vb_counter_error VbCounterUninstall(INT16U conuterIndex)
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;

  pthread_mutex_lock(&vbCountersMutex);

  if(conuterIndex >= VB_COUNTERS_MAX)
  {
    err = VB_COUNTERS_ERROR_MAX_EXCEED;
  }
  else if(!vbCountersUsed[conuterIndex])
  {
    err = VB_COUNTERS_ERROR_NOT_USED;
  }
  else
  {
    vbCountersUsed[conuterIndex] = FALSE;
    vbCounters[conuterIndex] = 0;
    vbCountersNames[conuterIndex] = NULL;
  }

  pthread_mutex_unlock(&vbCountersMutex);

  return err;
}

/*******************************************************************/

t_vb_counter_error VbCounterValueSet(INT16U conuterIndex, INT32U value)
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;

  pthread_mutex_lock(&vbCountersMutex);

  if(conuterIndex >= VB_COUNTERS_MAX)
  {
    err = VB_COUNTERS_ERROR_MAX_EXCEED;
  }
  else if(!vbCountersUsed[conuterIndex])
  {
    err = VB_COUNTERS_ERROR_NOT_USED;
  }
  else
  {
    vbCounters[conuterIndex] = value;
  }

  pthread_mutex_unlock(&vbCountersMutex);

  return err;
}

/*******************************************************************/

t_vb_counter_error VbCounterMaxValueSet(INT16U conuterIndex, INT32U value)
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;

  pthread_mutex_lock(&vbCountersMutex);

  if(conuterIndex >= VB_COUNTERS_MAX)
  {
    err = VB_COUNTERS_ERROR_MAX_EXCEED;
  }
  else if(!vbCountersUsed[conuterIndex])
  {
    err = VB_COUNTERS_ERROR_NOT_USED;
  }
  else
  {
    if(value > vbCounters[conuterIndex])
    {
      vbCounters[conuterIndex] = value;
    }
  }

  pthread_mutex_unlock(&vbCountersMutex);

  return err;
}

/*******************************************************************/

t_vb_counter_error VbCounterIncrease(INT16U conuterIndex)
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;

  pthread_mutex_lock(&vbCountersMutex);

  if(conuterIndex >= VB_COUNTERS_MAX)
  {
    err = VB_COUNTERS_ERROR_MAX_EXCEED;
  }
  else if(!vbCountersUsed[conuterIndex])
  {
    err = VB_COUNTERS_ERROR_NOT_USED;
  }
  else
  {
    vbCounters[conuterIndex]++;
  }

  pthread_mutex_unlock(&vbCountersMutex);

  return err;
}

/*******************************************************************/

t_vb_counter_error VbCounterDecrease(INT16U conuterIndex)
{

  t_vb_counter_error err = VB_COUNTERS_ERROR_NONE;

  pthread_mutex_lock(&vbCountersMutex);

  if(conuterIndex >= VB_COUNTERS_MAX)
  {
    err = VB_COUNTERS_ERROR_MAX_EXCEED;
  }
  else if(!vbCountersUsed[conuterIndex])
  {
    err = VB_COUNTERS_ERROR_NOT_USED;
  }
  else
  {
    if(vbCounters[conuterIndex] > 0)
    {
      vbCounters[conuterIndex]--;
    }
  }

  pthread_mutex_unlock(&vbCountersMutex);

  return err;
}

/*******************************************************************/

BOOL VbCountersRunningTime(void *arg, void (*write_fun)(const char *fmt, ...), char **cmd)
{

  INT64S elapse_time_ms;
  INT32U miliseconds;
  INT32U seconds;
  INT32U minutes;
  INT32U hours;
  INT32U days;
  struct timespec actual_runtime;

  clock_gettime(CLOCK_MONOTONIC, &actual_runtime);

  pthread_mutex_lock(&vbCountersMutex);

  elapse_time_ms =VbUtilElapsetimeTimespecMs(vbCountersRunTime, actual_runtime);

  pthread_mutex_unlock(&vbCountersMutex);

  miliseconds = (elapse_time_ms %1000);
  seconds = ((elapse_time_ms / 1000) % 60);
  minutes = (elapse_time_ms / 60000) % 60;
  hours  = (elapse_time_ms / 3600000) % 24;
  days  = (elapse_time_ms / 86400000);

  write_fun("%04dd %02dh %02dm %02ds %03dms\n", days, hours, minutes, seconds, miliseconds);

  return TRUE;
}

/*******************************************************************/

BOOL VbCountersConsoleCmd(void *arg, void (*write_fun)(const char *fmt, ...), char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "i"))
    {
      // Dump counters
      ret = VbCountersConsoleSend(arg, write_fun, cmd);
    }
    else if (!strcmp(cmd[1], "r"))
    {
      // Reset counters
      ret = VbCountersConsoleReset(arg, write_fun, cmd);
    }
    else if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    write_fun("Usage:\n");
    write_fun("counters h : Shows this help\n");
    write_fun("counters i : Shows counters\n");
    write_fun("counters r : Reset counters\n");
  }

  return ret;
}

/*******************************************************************/

BOOL VbCountersConsoleSend(void *arg, void (*write_fun)(const char *fmt, ...), char **cmd)
{
  INT16U i;

  pthread_mutex_lock(&vbCountersMutex);

  for(i = 0 ; i < VB_COUNTERS_MAX ; i++ )
  {
    if(vbCountersUsed[i])
    {
      write_fun("%s = %d\n", vbCountersNames[i], vbCounters[i] );
    }
  }

  pthread_mutex_unlock(&vbCountersMutex);

  return TRUE;
}

/*******************************************************************/

BOOL VbCountersConsoleReset(void *arg, void (*write_fun)(const char *fmt, ...), char **cmd)
{

  VbCountersReset();

  return TRUE;
}

/*******************************************************************/

/**
 * @}
 **/
