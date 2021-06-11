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
 * @file vb_util.c
 * @brief Common util functions implementation
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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "types.h"
#include "vb_util.h"
#include "vb_log.h"
#include "vb_types.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define TIMESPEC_STR_MS_LEN             (4)                                           // Including terminating null byte
#define TIMESPEC_STR_HMS_LEN            (TIMESPEC_STR_LEN - TIMESPEC_STR_MS_LEN + 1)  // Including terminating null byte

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

void VbUtilCreateFolderAndParents(const char *dir_name)
{
  char *fn, *p;

  fn = strdup(dir_name);

  if(fn != NULL)
  {
    if (*fn == '/')
    {
      // This is an absolute path
      p = fn + 1;
    }
    else
    {
      p = fn;
    }

    do
    {
      while (*p && *p != '/')
      {
        p++;
      }

      if (!*p)
      {
        p = NULL;
      }
      else
      {
        *p = '\0';
      }

      if( access( fn, F_OK ) == -1 )
      {
        // Path doesn't already exist
        //
        mkdir(fn, 0777);
      }

      if (p)
      {
        *p++ = '/';
        while (*p && *p == '/')
        {
          p++;
        }
      }
    } while (p);

    free(fn);
  }
}

/*******************************************************************/

INT64S VbUtilElapsetimeTimespecMs( struct timespec inittime, struct timespec endtime )
{
  INT64S elapsed_ms;
  INT32S cmp;

  // Compare two timestamps: endtime shall be >= inittime
  cmp = VbUtilTimespecCmp(&endtime, &inittime);

  if (cmp == -1)
  {
    // inittime > endtime
    elapsed_ms = 0;
  }
  else
  {
    // endtime >= inittime
    elapsed_ms = ((endtime.tv_sec - inittime.tv_sec) * 1000) +
        ((endtime.tv_nsec - inittime.tv_nsec)/1000000);
  }

  return elapsed_ms;
}

/*******************************************************************/

INT64S VbUtilElapsetimeTimespecUs( struct timespec *inittime, struct timespec *endtime )
{
  INT64S elapsed_us;
  INT32S cmp;

  // Compare two timestamps: endtime shall be >= inittime
  cmp = VbUtilTimespecCmp(endtime, inittime);

  if (cmp == -1)
  {
    // inittime > endtime
    elapsed_us = 0;
  }
  else
  {
    // endtime >= inittime
    elapsed_us = ((endtime->tv_sec - inittime->tv_sec) * 1000000) +
        ((endtime->tv_nsec - inittime->tv_nsec)/1000);
  }

  return elapsed_us;
}

/*******************************************************************/

INT64S VbUtilElapsetimeTimespecNs( struct timespec *inittime, struct timespec *endtime )
{
  INT64S elapsed_ns;

  INT32S cmp;

  // Compare two timestamps: endtime shall be >= inittime
  cmp = VbUtilTimespecCmp(endtime, inittime);

  if (cmp == -1)
  {
    // inittime > endtime
    elapsed_ns = 0;
  }
  else
  {
    // endtime >= inittime
    elapsed_ns = ((endtime->tv_sec - inittime->tv_sec) * 1000000000) +
        (endtime->tv_nsec - inittime->tv_nsec);
  }

  return elapsed_ns;
}

/*******************************************************************/

INT64S VbUtilGetTimestampMicroS( struct timespec inittime)
{
  INT64S elapsed_ms;

  elapsed_ms = (inittime.tv_sec * 1000000) +(inittime.tv_nsec/1000);

  return elapsed_ms;
}

/*******************************************************************/

void VbUtilTimespecSubtract(const struct timespec *start, const struct timespec *stop,
                   struct timespec *result)
{
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }
}

/*******************************************************************/

INT32S VbUtilTimespecAdd(const struct timespec *t0, const struct timespec *t1, struct timespec *result)
{
  INT32S ret = 0;

  if ((t0 == NULL) || (t1 == NULL) || (result == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    result->tv_sec = t0->tv_sec + t1->tv_sec;
    result->tv_nsec = t0->tv_nsec + t1->tv_nsec;

    if (result->tv_nsec > NSEC_IN_SEC)
    {
      result->tv_nsec -= NSEC_IN_SEC;
      result->tv_sec++;
    }
  }

  return ret;
}

/*******************************************************************/

INT32S VbUtilMsecToTimespec(INT32U msec, struct timespec *result)
{
  INT32S ret = 0;

  if (result == NULL)
  {
    ret = -1;
  }

  if (ret == 0)
  {
    result->tv_sec = MS_TO_SEC(msec);
    result->tv_nsec = MS_TO_NS(msec % MSEC_IN_SEC);
  }

  return ret;
}

/*******************************************************************/

INT32S VbUtilUsecToTimespec(INT32U usec, struct timespec *result)
{
  INT32S ret = 0;

  if (result == NULL)
  {
    ret = -1;
  }

  if (ret == 0)
  {
    result->tv_sec = US_TO_SEC(usec);
    result->tv_nsec = US_TO_NS(usec % USEC_IN_SEC);
  }

  return ret;
}

/*******************************************************************/

INT32S VbUtilTimespecMsecAdd(const struct timespec *t0, INT32U msec, struct timespec *result)
{
  INT32S           ret = 0;
  struct timespec msec_to_ts;

  if ((t0 == NULL) || (result == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    // Translate msec to timespec
    ret = VbUtilMsecToTimespec(msec, &msec_to_ts);
  }

  if (ret == 0)
  {
    // Add msec to given timespec
    ret = VbUtilTimespecAdd(t0, &msec_to_ts, result);
  }

  return ret;
}

/*******************************************************************/

INT32S VbUtilTimespecUsecAdd(const struct timespec *t0, INT32U usec, struct timespec *result)
{
  INT32S           ret = 0;
  struct timespec usec_to_ts;

  if ((t0 == NULL) || (result == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    // Translate usec to timespec
    ret = VbUtilUsecToTimespec(usec, &usec_to_ts);
  }

  if (ret == 0)
  {
    // Add usec to given timespec
    ret = VbUtilTimespecAdd(t0, &usec_to_ts, result);
  }

  return ret;
}

/*******************************************************************/

INT32S VbUtilTimespecUsecSubtract(const struct timespec *t0, INT32U usec, struct timespec *result)
{
  INT32S           ret = 0;
  struct timespec usec_to_ts;

  if ((t0 == NULL) || (result == NULL))
  {
    ret = -1;
  }

  if (ret == 0)
  {
    // Translate usec to timespec
    ret = VbUtilUsecToTimespec(usec, &usec_to_ts);
  }

  if (ret == 0)
  {
    // Add usec to given timespec
    VbUtilTimespecSubtract(&usec_to_ts, t0, result);
  }

  return ret;
}

/*******************************************************************/

INT32S VbUtilTimespecCmp(const struct timespec *t0, const struct timespec *t1)
{
  INT32S ret;

  if (t0->tv_sec != t1->tv_sec)
  {
    if (t0->tv_sec > t1->tv_sec)
    {
      ret = 1;
    }
    else
    {
      ret = -1;
    }
  }
  else
  {
    if (t0->tv_nsec > t1->tv_nsec)
    {
      ret = 1;
    }
    else if (t0->tv_nsec < t1->tv_nsec)
    {
      ret = -1;
    }
    else
    {
      ret = 0;
    }
  }

  return ret;
}

/*******************************************************************/

INT64S VbUtilDiffTimespecUs( struct timespec *timeA, struct timespec *timeB )
{
  INT64S elapsed_us;
  INT32S cmp_val;

  cmp_val = VbUtilTimespecCmp(timeA, timeB);

  if (cmp_val == 1)
  {
    elapsed_us = VbUtilElapsetimeTimespecUs(timeB, timeA);
  }
  else if (cmp_val == -1)
  {
    elapsed_us = -VbUtilElapsetimeTimespecUs(timeA, timeB);
  }
  else
  {
    elapsed_us = 0;
  }

  return elapsed_us;
}

/*******************************************************************/

void VbUtilTimespecToString(CHAR *buffer, struct timespec timeStamp)
{
  if (buffer != NULL)
  {
    struct tm *local_time;

    local_time = localtime(&(timeStamp.tv_sec));

    if (local_time != NULL)
    {
      INT8U time_str_cnt = 0;

      /*
       * Getting timeStamp string:
       * time_str_cnt does not include the terminating null byte,
       * however TIMESPEC_STR_HMS_LEN shall include the terminating null byte.
       */
      time_str_cnt = strftime(buffer, TIMESPEC_STR_HMS_LEN, "%H:%M:%S.", local_time);

      if ((time_str_cnt > 0) && (time_str_cnt < TIMESPEC_STR_HMS_LEN))
      {
        // Limit msec to 999 to avoid a compilation warning of GCC-7
        INT32U msec = timeStamp.tv_nsec / 10000000;

        if (msec >= 1000)
        {
          msec = 999;
        }

        snprintf(&(buffer[time_str_cnt]), TIMESPEC_STR_MS_LEN, "%03d", msec);
      }
    }
  }
}

/*******************************************************************/

void VbUtilTimespecToFileName(CHAR *buffer, struct timespec timeStamp)
{
  if (buffer != NULL)
  {
    struct tm *local_time;

    local_time = localtime(&(timeStamp.tv_sec));

    if (local_time != NULL)
    {
      // Getting timeStamp string
      strftime(buffer, TIMESPEC_FILE_NAME_STR_LEN, "%Y_%m_%d_Time_%H_%M_%S", local_time);
    }
  }
}

/*******************************************************************/

void *memdup(const void* mem, size_t size)
{
   void* out = malloc(size);

   if(out != NULL)
   {
       memcpy(out, mem, size);
   }

   return out;
}

/*******************************************************************/

INT32S VbUtilStringToBuffer(CHAR **ptrToWrite, INT32U *remainingSize, const char *format, ...)
{
  INT32S  ret = 0;
  va_list args;
  int     write_res;

  if ((ptrToWrite == NULL) ||
      (*ptrToWrite == NULL) ||
      (format == NULL) ||
      (remainingSize == NULL))
  {
    // Bad args
    ret = -1;
  }

  if (ret == 0)
  {
    if (*remainingSize == 0)
    {
      // Empty buffer
      ret = -2;
    }
  }

  if (ret == 0)
  {
    va_start(args, format);
    write_res = vsnprintf(*ptrToWrite, *remainingSize, format, args);
    va_end(args);

    if (write_res >= *remainingSize)
    {
      // Output was truncated
      *remainingSize = 0;
      *ptrToWrite = NULL;
      ret = -2;
    }
    else
    {
      // Return 0 (no error) and update arguments
      *ptrToWrite += write_res;
      *remainingSize -= write_res;
      ret = 0;
    }
  }

  return ret;
}

/*******************************************************************/

CHAR *VbUtilTrimWhiteSpace(CHAR *str)
{
  CHAR *start = NULL;
  CHAR *end;

  if (str != NULL)
  {
    start = str;

    // Move start to first "no space" character
    while ((isspace((unsigned char)*start) != 0) && (*start != '\0'))
    {
      // Space found, move to next character
      start++;
    }

    // Found the first space character after the string
    end = start;

    while ((isspace((unsigned char)*end) == 0) && (*end != '\0'))
    {
      // No space found, move to next character
      end++;
    }

    // Write new null terminator
    *end = '\0';
  }

  return start;
}

/*******************************************************************/

void VbUtilQueueNameBuild(CHAR *prefix, CHAR *uniqueId, CHAR *queueName)
{
  if ((prefix != NULL) && (uniqueId != NULL) && (queueName != NULL))
  {
    snprintf(queueName, VB_QUEUE_NAME_LEN, "%s_%s", prefix, uniqueId);
    queueName[VB_QUEUE_NAME_LEN - 1] = '\0';
  }
}

/*******************************************************************/

/**
 * @}
**/


