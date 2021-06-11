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
 * @file vb_common.c
 * @brief Common functions implementation
 *
 * @internal
 *
 * @author V.Grau
 * @date 27/05/2015
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#if (_USE_MALLOC_MUTEX_ == 1)
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define WRAP(X) __wrap_##X
#define REAL(X) __real_##X

#define PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP \
  { { 0, 0, 0, PTHREAD_MUTEX_RECURSIVE_NP, 0, { 0 } } }

extern void    *REAL(calloc) (size_t nmemb, size_t size);
extern void    *REAL(malloc) (size_t size);
extern void     REAL(free) (void *ptr);
extern void    *REAL(realloc) (void *ptr, size_t size);
extern char    *REAL(strdup) (const char *s);
extern char    *REAL(strndup) (const char *s, size_t size);

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_mutex_t vbDriverMallocDebugMutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

void *WRAP(calloc) (size_t nmemb, size_t size)
{
  void           *ret;

  pthread_mutex_lock( &vbDriverMallocDebugMutex );
  ret = (void *)REAL(calloc) (nmemb, size);
  pthread_mutex_unlock( &vbDriverMallocDebugMutex );

  return ret;
}

/*******************************************************************/

void *WRAP(malloc) (size_t size)
{
  void           *ret;

  pthread_mutex_lock( &vbDriverMallocDebugMutex );
  ret = REAL(malloc) (size);
  pthread_mutex_unlock( &vbDriverMallocDebugMutex );

  return ret;
}

/*******************************************************************/

void WRAP(free) (void *ptr)
{
  pthread_mutex_lock( &vbDriverMallocDebugMutex );
  REAL(free) (ptr);
  pthread_mutex_unlock( &vbDriverMallocDebugMutex );
}

/*******************************************************************/

void *WRAP(realloc) (void *ptr, size_t size)
{
  void           *ret;

  pthread_mutex_lock( &vbDriverMallocDebugMutex );
  ret = (void *)REAL(realloc) (ptr, size);
  pthread_mutex_unlock( &vbDriverMallocDebugMutex );

  return ret;
}

/*******************************************************************/

char *WRAP(strdup) (const char *s)
{
  char           *ret;

  pthread_mutex_lock( &vbDriverMallocDebugMutex );
  ret = REAL(strdup) (s);
  pthread_mutex_unlock( &vbDriverMallocDebugMutex );

  return ret;
}

/*******************************************************************/

char *WRAP(strndup) (const char *s, size_t n)
{
  char           *ret;

  pthread_mutex_lock( &vbDriverMallocDebugMutex );
  ret = REAL(strndup) (s, n);
  pthread_mutex_unlock( &vbDriverMallocDebugMutex );

  return ret;
}

/*******************************************************************/

#endif

/**
 * @}
 **/


