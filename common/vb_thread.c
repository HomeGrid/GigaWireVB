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
 * @file vb_thread.c
 * @brief VB threads implementation
 *
 * @internal
 *
 * @author V.Grau
 * @date 03/04/2017
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "types.h"
#include "vb_thread.h"
#include "vb_log.h"
#include "vb_timer.h"
#include "vb_util.h"

#include "vb_priorities.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define THREAD_UNKNOWN_NAME           ("Unknown")
#define SIGNAL_HANDLER_THREAD_NAME    ("SignalHandler")
#define SIGNAL_ABORT                  (SIGINT)             // Used to cancel execution (Ctrl+C)
#define SIGNAL_TIMER                  (TIMER_SIGNAL_NUM)   // Used to wake up after a timer expiration
#define SIGNAL_WAKE_UP                (SIGUSR1)            // Used to wake up during a sleep
#define THREAD_MAX_NUM                (50)
#define THREAD_NAME_LEN               (30)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  CHAR    name[THREAD_NAME_LEN];
  INT32U  cnt;
  BOOLEAN used;
} t_threadListEntry;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static BOOL                    vbSignalProcThreadRunning = FALSE;
static pthread_t               vbSignalHandlerThread;
static t_threadListEntry       vbThreadList[THREAD_MAX_NUM];
static pthread_mutex_t         vbThreadListMutex;

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VbSignalHandlerStateSet(BOOL running)
{
  vbSignalProcThreadRunning = running;
}

/*******************************************************************/

static BOOL VbSignalHandlerStateGet(void)
{
  return vbSignalProcThreadRunning;
}

/*******************************************************************/

static void VbSignalDummyHandler(int signum)
{
  /*
   * This handler is only required to wake-up threads sleeping in "nanosleep" function.
   * From the nanosleep "man" documentation:
   * "suspends the execution of the calling thread until either at least the time
   * specified in *req has elapsed, or the delivery of a signal that triggers the
   * invocation of a handler in the calling thread..."
   */
}

/*******************************************************************/

static void *signals_processing_thread(void *p)  
{  
  // A thread will be created to run this function forever. It simply loops on
  // an infinite loop, blocking on sigwait() each time.
  //
  // It works like this: originally all threads have all "interesting" signals
  // (SIGINT and SIGUSR1 for now) blocked (this is done in the parent "main"
  // thread and all children automatically inherit the list of blocked signals):
  //
  //   Parent thread:
  //     - Block signals: pthread_sigmask()
  //     - Create threads:
  //       |
  //       |--------- Thread #1
  //       |--------- Thread #2
  //       ...
  //       |--------- Thread #N (*this* thread)
  //       ...
  //       |--------- Thread #M
  //
  //  When a signal is sent to the thread group, the kernel will deliver it to
  //  the first thread which does not have that particular signal blocked. In
  //  our case, because all signals are blocked by all threads, the signal will
  //  remain in a "pending" state and will be eventually delivered to the first
  //  thread which unblocks it (using "pthread_sigmask()")
  //
  //  It will *also* be delivered to the first thread which calls "sigwait()",
  //  which is exactly what we are doing here.
  //
  //  Thanks to this, *all* signals will always be processed in *this* thread.
  //
  //  Note that, when using threads and signal, this is the best solution as
  //  explined here:
  //
  //    http://maxim.int.ru/bookshelf/PthreadsProgram/htm/r_40.html
  //
  //  In particular, notice how using a signal handler is not possible because
  //  other posix API functions cannot be called from inside signal handlers
  //  (again, as explained in the previous link).
  //
  void (*f_callback)(INT32U signal_number) = p;
  sigset_t sigs_to_catch;  
  int caught;  

  VbLogPrint(VB_LOG_INFO, "Starting signal handler thread");

  sigemptyset(&sigs_to_catch);  
  sigaddset(&sigs_to_catch, SIGNAL_ABORT);

  while (VbSignalHandlerStateGet() == TRUE)
  {  
    sigwait(&sigs_to_catch, &caught);  
  
    if (VbSignalHandlerStateGet() == TRUE)
    {
      switch(caught)
      {
        case SIGNAL_ABORT:
        {
          VbLogPrint(VB_LOG_INFO, "Signal SIGINT received!!");
          f_callback(SIGNAL_ABORT);
          break;
        }
        default:
        {
          VbLogPrint(VB_LOG_INFO, "ERROR!!! It is impossible to reach this point!");
          break;
        }
      }
    }
  }

  printf("Signals handler thread finished!\n");

  return NULL;
}

/*******************************************************************/

/**
 * @brief Adds the given thread to a list of running threads (for debug purposes).
 * @param[in] name Thread name
 **/
static void VbThreadListThreadStart(const CHAR *name)
{
  BOOLEAN       found = FALSE;
  INT32U        i;
  const CHAR  *thr_name = THREAD_UNKNOWN_NAME;

  if (name != NULL)
  {
    thr_name = name;
  }

  pthread_mutex_lock(&vbThreadListMutex);

  // Search for given thread in list
  for (i = 0; i < THREAD_MAX_NUM; i++)
  {
    if ((vbThreadList[i].used == TRUE) &&
        (strncmp(vbThreadList[i].name, thr_name, THREAD_NAME_LEN) == 0))
    {
      // Thread found in list, update counter
      vbThreadList[i].cnt++;

      found = TRUE;
      break;
    }
  }

  if (found == FALSE)
  {
    // Search for a free entry
    for (i = 0; i < THREAD_MAX_NUM; i++)
    {
      if (vbThreadList[i].used == FALSE)
      {
        // Free entry found, copy thread info
        vbThreadList[i].used = TRUE;
        vbThreadList[i].cnt = 1;
        strncpy(vbThreadList[i].name, thr_name, THREAD_NAME_LEN);
        vbThreadList[i].name[THREAD_NAME_LEN - 1] = '\0';

        break;
      }
    }
  }

  pthread_mutex_unlock(&vbThreadListMutex);
}

/*******************************************************************/

/**
 * @brief Marks the given thread as not running (for debug purposes).
 * @param[in] name Thread name
 **/
static void VbThreadListThreadStop(const CHAR *name)
{
  INT32U          i;
  const CHAR    *thr_name = THREAD_UNKNOWN_NAME;

  if (name != NULL)
  {
    thr_name = name;
  }

  pthread_mutex_lock(&vbThreadListMutex);

  // Search for given thread in list
  for (i = 0; i < THREAD_MAX_NUM; i++)
  {
    if ((vbThreadList[i].used == TRUE) &&
        (strncmp(vbThreadList[i].name, thr_name, THREAD_NAME_LEN) == 0))
    {
      // Thread found in list, update counter
      if (vbThreadList[i].cnt > 0)
      {
        vbThreadList[i].cnt--;
      }

      break;
    }
  }

  pthread_mutex_unlock(&vbThreadListMutex);
}

/*******************************************************************/

/*
 * clock_nanosleep feature is not provided by ARMV7 and MIPS toolchains,
 * so, it is implemented here.
 */
#if (_CONFIG_TARGET_ == _ARMV7_) || (_CONFIG_TARGET_ == _MIPS_)
#include <assert.h>

#if HP_TIMING_AVAIL
# define CPUCLOCK_P(clock) \
  ((clock) == CLOCK_PROCESS_CPUTIME_ID                \
   || ((clock) & ((1 << CLOCK_IDFIELD_SIZE) - 1)) == CLOCK_THREAD_CPUTIME_ID)
#else
# define CPUCLOCK_P(clock) 0
#endif

#ifndef INVALID_CLOCK_P
# define INVALID_CLOCK_P(cl) \
  ((cl) < CLOCK_REALTIME || (cl) > CLOCK_THREAD_CPUTIME_ID)
#endif

/* This implementation assumes that these is only a `nanosleep' system
   call.  So we have to remap all other activities.  */
static int
clock_nanosleep (clockid_t clock_id, int flags, const struct timespec *req,
     struct timespec *rem)
{
  struct timespec now;

  if (__builtin_expect (req->tv_nsec, 0) < 0
      || __builtin_expect (req->tv_nsec, 0) >= 1000000000)
    return EINVAL;

  if (clock_id == CLOCK_THREAD_CPUTIME_ID)
    return EINVAL;    /* POSIX specifies EINVAL for this case.  */

#ifdef SYSDEP_NANOSLEEP
  SYSDEP_NANOSLEEP;
#endif

  if (CPUCLOCK_P (clock_id))
    return ENOTSUP;

  if (INVALID_CLOCK_P (clock_id))
    return EINVAL;

  /* If we got an absolute time, remap it.  */
  if (flags == TIMER_ABSTIME)
    {
      long int nsec;
      long int sec;

      /* Make sure we use safe data types.  */
      assert (sizeof (sec) >= sizeof (now.tv_sec));

      /* Get the current time for this clock.  */
      if (__builtin_expect (clock_gettime (clock_id, &now), 0) != 0)
  return errno;

      /* Compute the difference.  */
      nsec = req->tv_nsec - now.tv_nsec;
      sec = req->tv_sec - now.tv_sec - (nsec < 0);
      if (sec < 0)
  /* The time has already elapsed.  */
  return 0;

      now.tv_sec = sec;
      now.tv_nsec = nsec + (nsec < 0 ? 1000000000 : 0);

      /* From now on this is our time.  */
      req = &now;

      /* Make sure we are not modifying the struct pointed to by REM.  */
      rem = NULL;
    }
  else if (__builtin_expect (flags, 0) != 0)
    return EINVAL;
  else if (clock_id != CLOCK_REALTIME)
    /* Not supported.  */
    return ENOTSUP;

  return __builtin_expect (nanosleep (req, rem), 0) ? errno : 0;
}
#endif

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

BOOLEAN VbBlockSignals(void)
{
  int                 err;
  sigset_t            mask;
  struct sigaction    act;

  if (sigemptyset(&mask) == -1)
  {
    printf("Can't init signal [%s]", strerror(errno));
    return FALSE;
  }

  if (sigaddset(&mask, SIGNAL_ABORT) == -1)
  {
    printf("Can't add mask! [%s]", strerror(errno));
    return FALSE;
  }

  if (sigaddset(&mask, SIGNAL_TIMER) == -1)
  {
    printf("Can't add mask! [%s]", strerror(errno));
    return FALSE;
  }

  if (pthread_sigmask(SIG_BLOCK, &mask, NULL) < 0)
  {
    printf("Could not modify signals mask! [%s]", strerror(errno));
    return FALSE;
  }

  // Install a "dummy" handler for SIGNAL_WAKE_UP in order to wake-up sleeping threads
  bzero(&act, sizeof(act));
  act.sa_handler = VbSignalDummyHandler;
  err = sigaction(SIGNAL_WAKE_UP, &act, NULL);

  if (err < 0)
  {
    printf("Could not modify default signal action [%s]", strerror(errno));
    return FALSE;
  }

  return TRUE;
}

/*******************************************************************/

INT32S VbThreadSleep(INT32U sleepTimeMs)
{
  INT32U           msecs;
  struct timespec sleep_time;

  sleep_time.tv_sec = MS_TO_SEC(sleepTimeMs);

  msecs = sleepTimeMs - SEC_TO_MS(sleep_time.tv_sec);
  sleep_time.tv_nsec = MS_TO_NS(msecs);

  // Sleep
  nanosleep(&sleep_time, NULL);

  return 0;
}

/*******************************************************************/

INT32S VbThreadAbsTimeSleep(struct timespec *absTime)
{
  int                      err;
  INT32S                   ret = 0;

  if (absTime == NULL)
  {
    ret = -1;
  }

  if (ret == 0)
  {
    err = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, absTime, NULL);

    if (err != 0)
    {
      if (err == EINTR)
      {
        VbLogPrint(VB_LOG_INFO, "Thread was woken-up");
      }
      else
      {
        VbLogPrint(VB_LOG_ERROR, "Error (%d / %s) configuring sleep", err, strerror(errno));
        ret = -1;
      }
    }
  }

  return ret;
}

/*******************************************************************/

INT32S VbThreadWakeUp(pthread_t threadId)
{
  INT32S ret = 0;
  int   err;

  err = pthread_kill(threadId, SIGNAL_WAKE_UP);

  if (err != 0)
  {
    if (err == ESRCH)
    {
      // No such process. This is an expected error when thread is already finish when calling this function.
      ret = 0;
    }
    else
    {
      VbLogPrint(VB_LOG_WARNING, "Error %d : Can't send wake-up signal [%s]", err, strerror(err));
      ret = -1;
    }
  }

  return ret;
}

/*******************************************************************/

INT32S VbThreadCondSleepInit(pthread_mutex_t *mutex, pthread_cond_t  *cond)
{
  INT32S ret = 0;

  if((mutex == NULL) || (cond == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    pthread_condattr_t attr_con_var;

    pthread_mutex_init(mutex, NULL);
    pthread_condattr_init(&attr_con_var);
    pthread_cond_init(cond, &attr_con_var);
  }

  return ret;
}

/*******************************************************************/

INT32S VbThreadCondSleep(pthread_mutex_t *mutex, pthread_cond_t  *cond, INT32U sleepTimeMs)
{
  INT32S     ret = 0;
  int        os_err;

  if((mutex == NULL) || (cond == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    os_err = pthread_mutex_lock(mutex);

    if (os_err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d locking mutex", os_err);
      ret = -1;
    }
  }

  if (ret == 0)
  {
    if (sleepTimeMs > 0)
    {
      INT32S            util_err;
      struct timespec  ts;
      struct timespec  result_ts;

      clock_gettime(CLOCK_REALTIME, &ts);

      util_err = VbUtilTimespecMsecAdd(&ts, sleepTimeMs, &result_ts);

      if (util_err != 0)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d calculating timespec to wait", util_err);
        ret = -1;
      }

      if (ret == 0)
      {
        os_err = pthread_cond_timedwait(cond, mutex, &result_ts);
      }
    }
    else
    {
      os_err = pthread_cond_wait(cond, mutex);
    }

    if ((os_err != 0) && (os_err != ETIMEDOUT))
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d waiting for condition variable", os_err);
      ret = -1;
    }

    os_err = pthread_mutex_unlock(mutex);

    if (os_err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d unlocking mutex", os_err);
    }
  }

  return ret;
}

/*******************************************************************/

INT32S VbThreadCondWakeUp(pthread_mutex_t *mutex, pthread_cond_t  *cond)
{
  INT32S     ret = 0;
  int        os_err;

  if((mutex == NULL) || (cond == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    os_err = pthread_mutex_lock(mutex);

    if (os_err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d locking mutex", os_err);
      ret = -1;
    }
  }

  if (ret == 0)
  {
    os_err = pthread_cond_signal(cond);

    if (os_err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d signalling condition variable", os_err);
      ret = -1;
    }

    os_err = pthread_mutex_unlock(mutex);

    if (os_err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d unlocking mutex", os_err);
    }
  }

  return ret;
}

/*******************************************************************/

INT32S VbThreadInit(void)
{
  pthread_mutex_init(&vbThreadListMutex, NULL);
  memset(vbThreadList, 0, sizeof(vbThreadList));

  return 0;
}

/*******************************************************************/

BOOLEAN VbThreadHandleSignalsStart(void (*f)(INT32U signum))
{
  BOOLEAN running = FALSE;

  VbThreadHandleSignalsStop();

  VbLogPrint(VB_LOG_INFO, "Starting %s thread", SIGNAL_HANDLER_THREAD_NAME);

  VbSignalHandlerStateSet(TRUE);

  running = VbThreadCreate(SIGNAL_HANDLER_THREAD_NAME, signals_processing_thread, f, VB_SIGNALS_PROCESSING_PRIORITY, &vbSignalHandlerThread);

  if (running == FALSE)
  {
    VbLogPrint(VB_LOG_ERROR, "Can't create %s thread", SIGNAL_HANDLER_THREAD_NAME);
    VbSignalHandlerStateSet(FALSE);
  }

  return running;
}

/*******************************************************************/

void VbThreadHandleSignalsStop(void)
{
  if (VbSignalHandlerStateGet() == TRUE)
  {
    // Use printf instead of VbLogPrint as Log thread can be stopped at this point
    printf("Stopping %s thread...\n", SIGNAL_HANDLER_THREAD_NAME);

    VbSignalHandlerStateSet(FALSE);

    // Signal thread to finish
    pthread_kill(vbSignalHandlerThread, SIGNAL_ABORT);

    VbThreadJoin(vbSignalHandlerThread, SIGNAL_HANDLER_THREAD_NAME);

    printf("Stopped %s thread!\n", SIGNAL_HANDLER_THREAD_NAME);
  }
}

/*******************************************************************/

BOOLEAN VbThreadCreate(const CHAR *name, void *(*f)(void *), void *f_arg, int prio, pthread_t *thread_id)
{
  int err = 0;
  pthread_attr_t tattr;
  struct sched_param param;

  *thread_id = 0;

  // Clear attributes
  //
  err = pthread_attr_init(&tattr);
  if (err != 0)
  {
    VbLogPrint( VB_LOG_ERROR,"Error in pthread_attr_init() [0x%08p] (err %d)", f, err);
    return FALSE;
  }

  // All threads with a priority greater than "0" will be configured with a
  // "real-time" SCHED_FIFO scheduler policy.
  //
  if (prio != 0)
  {
    err = pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
    if (err != 0)
    {
      VbLogPrint( VB_LOG_ERROR,"Error in pthread_attr_setschedpolicy() [0x%08p] (err %d)", f, err);

      pthread_attr_destroy(&tattr);
      return FALSE;
    }

    prio = prio + sched_get_priority_min(SCHED_FIFO)-1;
    if (prio > sched_get_priority_max(SCHED_FIFO))
    {
      prio = sched_get_priority_max(SCHED_FIFO);
    }

    param.sched_priority = prio;
    VbLogPrint( VB_LOG_INFO,"PRIO: %d", prio);

    err = pthread_attr_setschedparam(&tattr, &param);
    if (err != 0)
    {
      VbLogPrint( VB_LOG_ERROR,"Error in pthread_attr_setschedparam() [0x%08p] (err %d)", f, err);

      pthread_attr_destroy(&tattr);
      return FALSE;
    }
  }

  // Adds thread name to list, for debug purposes
  VbThreadListThreadStart(name);

  // Here we go...
  //
  err = pthread_create(thread_id, &tattr, f, f_arg);
  if (err != 0)
  {
    VbLogPrint( VB_LOG_ERROR,"Error in pthread_create() [0x%08p] (err %d)", f, err);
    pthread_attr_destroy(&tattr);
    return FALSE;
  }

  pthread_attr_destroy(&tattr);

  return TRUE;
}

/*******************************************************************/

void VbThreadJoin(pthread_t threadId, const CHAR *name)
{
  pthread_join(threadId , NULL);

  if (name != NULL)
  {
    VbThreadListThreadStop(name);
  }
}

/*******************************************************************/

void VbThreadListThreadDump(t_writeFun writeFun)
{
  INT32U  num_thr = 0;
  INT32U  i;

  if (writeFun != NULL)
  {
    writeFun("\nThreads:\n");
    writeFun("==========================================\n");
    writeFun("|            Name              |   Cnt   |\n");
    writeFun("==========================================\n");

    pthread_mutex_lock(&vbThreadListMutex);

    // Loop through thread list
    for (i = 0; i < THREAD_MAX_NUM; i++)
    {
      if (vbThreadList[i].used == TRUE)
      {
        writeFun("|%-30s| %7lu |\n", vbThreadList[i].name, vbThreadList[i].cnt);

        if (vbThreadList[i].cnt > 0)
        {
          num_thr++;
        }
      }
    }

    pthread_mutex_unlock(&vbThreadListMutex);

    writeFun("==========================================\n");
    writeFun("Number of threads running : %u\n", num_thr);
  }
}

/*******************************************************************/

/**
 * @}
**/


