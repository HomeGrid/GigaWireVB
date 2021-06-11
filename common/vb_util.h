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
 * @file vb_util.h
 * @brief Common functions interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 27/05/2015
 *
 **/

#ifndef VB_UTIL_H_
#define VB_UTIL_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <stdlib.h>

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

/// Calculates the absolute diff between two values
#define ABS_DIFF(X, Y)             (((X)>=(Y))?((X)-(Y)):((Y)-(X)))

#define DIFANY(t1,t0,max)          ( (t1>=t0)?(t1-t0):(max - (t0 - t1) + 1) )
#define DIF32(t1,t0)               ( (t1>=t0)?(t1-t0):(0xffffffff - (t0 - t1) + 1) )
#define DIF16(t1,t0)               ( (t1>=t0)?(t1-t0):(0xffff - (t0 - t1) + 1) )

#define MS_TO_10NS_UNITS(X)        ((X) * 100000)
#define MS_TO_US(X)                ((X) * 1000)
#define MS_TO_NS(X)                ((X) * 1000000)
#define US_TO_NS(X)                ((X) * 1000)
#define MS_TO_SEC(X)               ((X) / 1000)
#define US_TO_MS(X)                ((X) / 1000)
#define US_TO_SEC(X)               ((X) / 1000000)
#define NS_TO_US(X)                ((X) / 1000)
#define NS_TO_MS(X)                ((X) / 1000000)
#define SEC_TO_MS(X)               ((X) * 1000)
#define NSEC_IN_SEC                (1000000000)
#define USEC_IN_SEC                (1000000)
#define MSEC_IN_SEC                (1000)

#define MAX(A, B)                  ((A) >= (B) ? (A) : (B))
#define MIN(A, B)                  ((A) <= (B) ? (A) : (B))
#define CEIL(A, B)                 ( ( ((A) % (B)) == 0) ? ((A)/(B)):((A)/(B)+1) )

/// Calculates the %
#define IN_PERC(X, Y)              (((Y)>0)?(((X) * 100)/(Y)):0)

#define TIMESPEC_STR_LEN             (13)
#define TIMESPEC_FILE_NAME_STR_LEN   (26)

/// Endianness definitions
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      /* little endian processor definition short integer*/
#    define _htons(val) \
   ((INT16U)( (((INT16U)(val) >> 8) & 0x00FF) | \
   (((INT16U)(val) << 8) & 0xFF00) ))

#    define _htons_ghn(val) (INT16U) (val)

#    define _ntohs(val) \
   ((INT16U)( (((INT16U)(val) >> 8) & 0x00FF) | \
   (((INT16U)(val) << 8) & 0xFF00) ))

#   define _ntohs_ghn(val) (INT16U) (val)

     /* little endian processor definition 3 byte integer*/
#    define _htonm(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 4  ) & 0x000000FF ) ) | \
    ( ( ( (INT32U)(val) >> 12 ) & 0x00000F00 ) ) | \
    ( ( ( (INT32U)(val) << 12 ) & 0x0000F000 ) ) | \
    ( ( ( (INT32U)(val) << 4  ) & 0x00FF0000 ) ) )

#    define  _htonm_ghn(val) (INT32U)(val)

#    define _ntohm(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 4  ) & 0x000FF000 ) ) | \
    ( ( ( (INT32U)(val) >> 12 ) & 0x0000000F ) ) | \
    ( ( ( (INT32U)(val) << 12 ) & 0x00F00000 ) ) | \
    ( ( ( (INT32U)(val) << 4  ) & 0x00000FF0 ) ) )

#    define  _ntohm_ghn(val) (INT32U)(val)

      /* little endian processor definition long integer*/
#    define _htonl(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 24 ) & 0x000000FF ) ) | \
    ( ( ( (INT32U)(val) >> 8  ) & 0x0000FF00 ) ) | \
    ( ( ( (INT32U)(val) << 8  ) & 0x00FF0000 ) ) | \
    ( ( ( (INT32U)(val) << 24 ) & 0xFF000000 ) )\
   )
#    define _htonl_ghn(val) (INT32U) (val)

#    define _ntohl(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 24 ) & 0x000000FF ) ) | \
    ( ( ( (INT32U)(val) >> 8  ) & 0x0000FF00 ) ) | \
    ( ( ( (INT32U)(val) << 8  ) & 0x00FF0000 ) ) | \
    ( ( ( (INT32U)(val) << 24 ) & 0xFF000000 ) )\
   )
#    define _ntohl_ghn(val) (INT32U) (val)


#else
#  if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
      /* big endian processor definition */
#   define _htons_ghn(val) \
   ((INT16U)( (((INT16U)(val) >> 8) & 0x00FF) | \
   (((INT16U)(val) << 8) & 0xFF00) ))

#   define _htons(val) (INT16U) (val)

#   define _ntohs_ghn(val) \
   ((INT16U)( (((INT16U)(val) >> 8) & 0x00FF) | \
   (((INT16U)(val) << 8) & 0xFF00) ))

#   define _ntohs(val) (INT16U) (val)

    /* big endian processor definition 3 byte integer*/
#    define _htonm_ghn(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 4  ) & 0x000000FF ) ) | \
    ( ( ( (INT32U)(val) >> 12 ) & 0x00000F00 ) ) | \
    ( ( ( (INT32U)(val) << 12 ) & 0x0000F000 ) ) | \
    ( ( ( (INT32U)(val) << 4  ) & 0x00FF0000 ) ) )

#    define  _htonm(val) (INT32U)(val)

#    define _ntohm(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 4  ) & 0x000FF000 ) ) | \
    ( ( ( (INT32U)(val) >> 12 ) & 0x0000000F ) ) | \
    ( ( ( (INT32U)(val) << 12 ) & 0x00F00000 ) ) | \
    ( ( ( (INT32U)(val) << 4  ) & 0x00000FF0 ) ) )

#    define  _ntohm_ghn(val) (INT32U)(val)


  /* big endian processor definition long integer*/
#    define _htonl_ghn(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 24 ) & 0x000000FF ) ) | \
    ( ( ( (INT32U)(val) >> 8  ) & 0x0000FF00 ) ) | \
    ( ( ( (INT32U)(val) << 8  ) & 0x00FF0000 ) ) | \
    ( ( ( (INT32U)(val) << 24 ) & 0xFF000000 ) )\
   )
#    define _htonl(val) (INT32U) (val)

#    define _ntohl_ghn(val) \
   ( (INT32U)\
    ( ( ( (INT32U)(val) >> 24 ) & 0x000000FF ) ) | \
    ( ( ( (INT32U)(val) >> 8  ) & 0x0000FF00 ) ) | \
    ( ( ( (INT32U)(val) << 8  ) & 0x00FF0000 ) ) | \
    ( ( ( (INT32U)(val) << 24 ) & 0xFF000000 ) )\
   )

#    define _ntohl(val) (INT32U) (val)

# else
#   error "Endianness undefined"
# endif
#endif

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Creates a folder (mkdir()) and all needed parents
 *
 * @param[in] dir_name  Name of the folder you want to create. It can be either
 *                      a relative or absolute path. It it already exists,
 *                      nothing happens.
 *
 * Example:
 *
 *   VbUtilCreateFolderAndParents("/my_folder/my_subfolder")
 *
 **/
void VbUtilCreateFolderAndParents(const char *dir_name);

/**
 * @brief Calculates the elapsed time in msecs between two given timespecs
 * @param[in] inittime Init time
 * @param[in] endtime End time
 * @return Elapsed time in msecs. If inittime > endtime it returns 0 ms.
 **/
INT64S VbUtilElapsetimeTimespecMs( struct timespec inittime, struct timespec endtime );

/**
 * @brief Calculates the elapsed time in usecs between two given timespecs
 * @param[in] inittime Init time
 * @param[in] endtime End time
 * @return Elapsed time in usecs. If inittime > endtime it returns 0 ms.
 **/
INT64S VbUtilElapsetimeTimespecUs( struct timespec *inittime, struct timespec *endtime );

/**
 * @brief Calculates the elapsed time in nsecs between two given timespecs
 * @param[in] inittime Init time
 * @param[in] endtime End time
 * @return Elapsed time in nsecs. If inittime > endtime it returns 0 ms.
 **/
INT64S VbUtilElapsetimeTimespecNs( struct timespec *inittime, struct timespec *endtime );

/**
 * @brief Return time in usec of the passed timespec struct
 * @return Time in usec of given timespec struct
 **/
INT64S VbUtilGetTimestampMicroS( struct timespec inittime);

/**
 * @brief Subtract the ‘struct timespec’ values start and stop
 * @param[in] start Start time
 * @param[in] stop Stop time
 * @param[out] result Result time
 **/
void VbUtilTimespecSubtract(const struct timespec *start, const struct timespec *stop,
                   struct timespec *result);

/**
 * @brief Adds two timespecs structs
 * @param[in] t0 Pointer to timespec structure.
 * @param[in] t1 Pointer to timespec structure.
 * @param[out] result Pointer to result
 * @return 0: if OK; -1 if error
 **/
INT32S VbUtilTimespecAdd(const struct timespec *t0, const struct timespec *t1, struct timespec *result);

/**
 * @brief Builds a timespec structure with given msecs
 * @param[in] msec Milliseconds
 * @param[out] result Timespec structure conveying given msecs
 * @return 0: if OK; -1 if error
 **/
INT32S VbUtilMsecToTimespec(INT32U msec, struct timespec *result);

/**
 * @brief Builds a timespec structure with given usecs
 * @param[in] usec Microseconds
 * @param[out] result Timespec structure conveying given usecs
 * @return 0: if OK; -1 if error
 **/
INT32S VbUtilUsecToTimespec(INT32U usec, struct timespec *result);

/**
 * @brief Adds msec to a given timespec structure
 * @param[in] t0 Pointer to timespec structure
 * @param[in] msec Msec to add
 * @param[out] result Pointer to result
 * @return 0: if OK; -1 if error
 **/
INT32S VbUtilTimespecMsecAdd(const struct timespec *t0, INT32U msec, struct timespec *result);

/**
 * @brief Adds usec to a given timespec structure
 * @param[in] t0 Pointer to timespec structure
 * @param[in] usec Usec to add
 * @param[out] result Pointer to result
 * @return 0: if OK; -1 if error
 **/
INT32S VbUtilTimespecUsecAdd(const struct timespec *t0, INT32U usec, struct timespec *result);

/**
 * @brief Subtracts usec from a given timespec structure
 * @param[in] t0 Pointer to timespec structure
 * @param[in] usec Usec to subtract
 * @param[out] result Pointer to result
 * @return 0: if OK; -1 if error
 **/
INT32S VbUtilTimespecUsecSubtract(const struct timespec *t0, INT32U usec, struct timespec *result);

/**
 * @brief Compares two timespec structures.
 * @param[in] t0 Pointer to timespec structure.
 * @param[in] t1 Pointer to timespec structure.
 * @return 1: if t0 > t1; 0: if t0 == t1; -1: if t0 < t1
 **/
INT32S VbUtilTimespecCmp(const struct timespec *t0, const struct timespec *t1);

/**
 * @brief Returns the time difference between given timespecs in usecs
 * @param[in] timeA Timespec to compare
 * @param[in] timeB Timespec to compare
 * @return Time difference in usec
 **/
INT64S VbUtilDiffTimespecUs( struct timespec *timeA, struct timespec *timeB );

/**
 * @brief Duplicate a given region of memory
 * @param[in] mem Pointer to memory to duplicate
 * @param[in] size Size in bytes of memory to duplicate
 * @return Pointer to duplicated memory. It shall be freed by caller.
 **/
void* memdup(const void* mem, size_t size);

/**
 * @brief Formats the given time into a string as hh.mm.ss.xxx
 * @param[out] buffer Buffer to write string. Its length shall be @ref TIMESPEC_STR_LEN.
 * @param[in] timeStamp Time stamp
 **/
void VbUtilTimespecToString(CHAR *buffer, struct timespec timeStamp);

/**
 * @brief Formats the given time into a string useful to be used as file name:
 * (year_month_day_Time_hour_min_second)
 * @param[out] buffer Buffer to write string. Its length shall be @ref TIMESPEC_FILE_NAME_STR_LEN.
 * @param[in] timeStamp Time stamp
 **/
void VbUtilTimespecToFileName(CHAR *buffer, struct timespec timeStamp);

/**
 * @brief This function is similar to "snprintf" but it updates the pointer and the remaining size
 * according to written bytes in given buffer.
 * @param[in,out] ptrToWrite Pointer to a buffer to write the string to. A maximum of remainingSize bytes
 * can be written to buffer. If the maximum size is reached, the trailing '\0' byte will not be written to
 * buffer.
 * @param[in,out] remainingSize Remaining size in buffer.
 * @return 0 if the given string was written successfully to buffer and ptrToWrite and remainingSize has
 * been updated accordingly; -1 if bad arguments; -2 Not enough buffer to write to. Output was truncated
 * and args updated.
 **/
INT32S VbUtilStringToBuffer(CHAR **ptrToWrite, INT32U *remainingSize, const char *format, ...);

/**
 * @brief Removes trailing and leading whitespaces of given string.
 * @param[in] str String to trim
 * @return Trimmed string without spaces
 * @remarks Given string is modified, so this function does not work with constant strings
 **/
CHAR *VbUtilTrimWhiteSpace(CHAR *str);

/**
 * @brief Builds a queue name using given prefix and unique Id strings.
 * @param[in] prefix Prefix to use
 * @param[in] uniqueId Unique string to append at the end of queue name
 * @param[out] queueName Resulting queue name. This buffer shall have a size of @ref VB_QUEUE_NAME_LEN bytes
 **/
void VbUtilQueueNameBuild(CHAR *prefix, CHAR *uniqueId, CHAR *queueName);

#endif /* VB_UTIL_H_ */

/**
 * @}
 **/


