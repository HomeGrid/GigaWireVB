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
 * @file vb_thread.h
 * @brief VB threads interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 03/04/2017
 *
 **/

#ifndef VB_THREAD_H_
#define VB_THREAD_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_console.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */


/*
 ************************************************************************
 ** Public function definition (THREADS)
 ************************************************************************
 */

/**
 * @brief Block SIGINT (CTRL+c)
 *
 * If you want your program to handle CTRL+c gracefully (ie. instead of just
 * die, execute some exit code that takes care of freeing memory, closing files,
 * stopping queues, etc...) this is what you have to do:
 *
 *   1. In you program "main()" function, at the very beginning, call *THIS*
 *      function. It will block the CTRL+c signal, causing it to never be
 *      delivered to your program.
 *      You must do this *before* creating any other thread (ie. before calling
 *      VbThreadCreate()"). That way all future (child) threads will *also*
 *      inherit this "CTRL+c" blocking property (which is what we want).
 *
 *   2. Next, start with the typical initialization process that goes on in
 *      "main()". In particular, open all "queues" that you will be using to
 *      communicate among threads.
 *      Queue descriptors are inherited by child threads, that's why you need
 *      to create them before the next step.
 *
 *   3. Call "VbThreadHandleSignalsStart()". This will be create a new thread (with all
 *      the previously inherited queue descriptors available) that will simply
 *      process *blocked* signals (by means of a special system call) and
 *      execute a callback when this happens.
 *      This callback (provided as an argument to "VbThreadHandleSignalsStart()") will
 *      be able to post messages to all those queues previously opened in step
 *      (2) to inform all other threads that it is time to clean up and exit.
 *
 *
 * WARNING !!!
 *
 *   After calling this function, if you want to stop the execution of the
 *   program under GDB, you will need to manually send the "TERM" signal to the
 *   process, like this:
 *
 *     # kill -TERM <pid>
 *
 *   In other words, just pressing CTRL+C from GDB will *not* work (see here
 *   to understand why: https://sourceware.org/bugzilla/show_bug.cgi?id=9425)
 *
 *
 * @return TRUE if signals could be blocked
 *         FALSE otherwise
 *
 **/
BOOLEAN VbBlockSignals(void);

/**
 * @brief Create a special thread that will take care of signals.
 *
 * NOTE: You *must* call this function from "main()" only after certain other
 * tasks have been performed. Read the documentation for "VbBlockSignals()"
 * before using this other function.
 *
 * @param[in] f    Function that will be called when a signal is received. It
 *                 only receives one argument: the triggered signal number.
 *
 **/
BOOLEAN VbThreadHandleSignalsStart(void (*f)(INT32U signum));

/**
 * @brief Stops the signal thread handler.
 **/
void VbThreadHandleSignalsStop(void);

/**
 * @brief Helper function to create new threads
 *
 * @param[in] name   Thread name
 *
 * @param[in] f      Pointer to the thread entry function.
 *
 * @param[in] f_arg  Argument passed to f() when called at creation time.
 *
 * @param[in] prio   Thread priority. It works like this:
 *                   - If set to "0", the thread will be configured with the
 *                     default scheduler policy (ie. "SCHED_OTHER"). This means
 *                     it will share CPU with the rest of "normal" processes on
 *                     the system.
 *                   - If set to a value bigger than "0", the thread will be
 *                     configured with a SCHED_FIFO scheduler policy. This is a
 *                     "real-time" policy (meaning it takes precedence over all
 *                     other "normal" processes on the system) that causes the
 *                     thread to execute until it has nothing else to do or is
 *                     interrupted by another SCHED_FIFO thread with a higher
 *                     priority number.
 *
 * @param[out] thread_id  Thread ID which can be used to later identify the
 *                        just created thread.
 *
 * @return TRUE if the new thread could be created without problems.
 *         FALSE otherwise.
 **/
BOOLEAN VbThreadCreate(const CHAR *name, void *(*f)(void *), void *f_arg, int prio, pthread_t *thread_id);

/**
 * @brief Waits for given thread to finish.
 * @param[in] threadId Posix thread Id, as returned by @ref VbThreadCreate
 * @param[in] name Thread name, as used in @ref VbThreadCreate
 **/
void VbThreadJoin(pthread_t threadId, const CHAR *name);

/**
 * @brief Initializes internal variables
 * @return 0 if OK
 **/
INT32S VbThreadInit(void);

/**
 * @brief Suspends the execution of current thread until
 * either the the given time has elapsed or @ref VbThreadWakeUp
 * is called.
 * @param[in] sleepTimeMs Time to sleep in msecs
 * @return 0 if OK; -1 otherwise
 **/
INT32S VbThreadSleep(INT32U sleepTimeMs);

/**
 * @brief Configures a timed wait until the given absolute time is reached.
 * If the specified absolute time has already passed, then the timer expires immediately,
 * @param[in] absTime Absolute time to wait for.
 * @return 0 if OK: -1 otherwise
 **/
INT32S VbThreadAbsTimeSleep(struct timespec *absTime);

/**
 * @brief Wakes up a sleeping thread
 * @param[in] threadId Thread id to wake up
 * @return 0 if OK; -1 otherwise
 **/
INT32S VbThreadWakeUp(pthread_t threadId);

/**
 * @brief Initializes condition wait structure
 * @param[in] condSleep Condition wait structure
 * @return 0 if OK; -1 otherwise
 **/
INT32S VbThreadCondSleepInit(pthread_mutex_t *mutex, pthread_cond_t  *cond);

/**
 * @brief Performs a condition timed wait
 * @param[in] condSleep Condition wait structure
 * @param[in] sleepTimeMs Time to sleep before wake up
 * @return 0 if OK; -1 otherwise
 **/
INT32S VbThreadCondSleep(pthread_mutex_t *mutex, pthread_cond_t  *cond, INT32U sleepTimeMs);

/**
 * @brief Wakes up a thread waiting in a condition timed wait
 * @param[in] condSleep Condition wait structure
 * @return 0 if OK; -1 otherwise
 **/
INT32S VbThreadCondWakeUp(pthread_mutex_t *mutex, pthread_cond_t  *cond);

/**
 * @brief Dumps the running thread list
 * @param[in] writeFun Callback used to print
 **/
void VbThreadListThreadDump(t_writeFun writeFun);

#endif /* VB_THREAD_H_ */

/**
 * @}
**/


