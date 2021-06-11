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
 * @addtogroup dek
 * @{
**/

/**
 * @file types.h
 * @brief Data types header
 *
 * @internal
 *
 * @author Paco Escuder
 * @date 29/03/2007
 *  
**/

// we use _INTERNAL_TYPES_H_ instead of _TYPES_H_ to avoid compilation problems with MINGW which uses a types.h
#if !defined(_INTERNAL_TYPES_H_)
#  define _INTERNAL_TYPES_H_

#  include <limits.h>
#  include <stdint.h>

/*
 *********************************************************************************************************
 *                                              DATA TYPES
 *                                         (Compiler Specific)
 *********************************************************************************************************
 */

typedef uint8_t INT8U;    ///< Unsigned 8-bit integer
typedef int8_t INT8S;      ///< Signed 8-bit integer
typedef uint16_t INT16U;  ///< Unsigned 16-bit integer
typedef int16_t INT16S;    ///< Signed 16-bit integer
typedef uint32_t INT32U;   ///< Unsigned 32-bit integer
typedef int32_t INT32S;     ///< Signed 32-bit integer
typedef uint64_t INT64U;  ///< Unsigned 64-bit integer
typedef int64_t INT64S;  ///< Signed 64-bit integer
typedef float FP32;             ///< Single precission floating point number
typedef double FP64;            ///< Double precission floating point number

#define INT2VOIDP(i) (void*)(uintptr_t)(i)
#define VOIDP2INT(i) (uintptr_t)(i)

#  ifndef _CHAR_DEFINED_
#    define _CHAR_DEFINED_
typedef char CHAR;              ///< Character type
#  endif

#  ifndef _BOOLEAN_DEFINED_
#    define _BOOLEAN_DEFINED_
typedef unsigned char BOOLEAN;  ///< Boolean data type. Accepts TRUE and FALSE values
#  endif

typedef unsigned int OS_STK;    ///< Each stack entry is 32-bit wide

/***** Internal types ******/

#  ifndef NULL
#    define	NULL        ((void *)0)
#  endif

#  ifndef NOTUSED
#    define NOTUSED 0
#  endif

#  ifndef  FALSE
#    define  FALSE                     0
#  endif

#  ifndef  TRUE
#    define  TRUE                      1
#  endif

#  ifndef MAX_BOOLEAN
#    define MAX_BOOLEAN              ((BOOLEAN)0x01U)
#  endif

#  ifndef MIN_BOOLEAN
#    define MIN_BOOLEAN              ((BOOLEAN)0x00U)
#  endif

#  ifndef MAX_INT8U
#    define MAX_INT8U                ((INT8U)0xFFU)
#  endif

#  ifndef MIN_INT8U
#    define MIN_INT8U                ((INT8U)0x00U)
#  endif

#  ifndef MAX_INT8S
#    define MAX_INT8S                ((INT8S)0x7F)
#  endif

#  ifndef MIN_INT8S
#    define MIN_INT8S                ((INT8S)0x80)
#  endif

#  ifndef MAX_INT16U
#    define MAX_INT16U               ((INT16U)0xFFFFU)
#  endif

#  ifndef MIN_INT16U
#    define MIN_INT16U               ((INT16U)0x0000U)
#  endif

#  ifndef MAX_INT16S
#    define MAX_INT16S               ((INT16S)0x7FFF)
#  endif

#  ifndef MIN_INT16S
#    define MIN_INT16S               ((INT16S)0x8000)
#  endif

#  ifndef MAX_INT32U
#    define MAX_INT32U               ((INT32U)0xFFFFFFFFUL)
#  endif

#  ifndef MIN_INT32U
#    define MIN_INT32U               ((INT32U)0x00000000UL)
#  endif

#  ifndef MAX_INT32S
#    define MAX_INT32S               ((INT32S)0x7FFFFFFFL)
#  endif

#  ifndef MIN_INT32S
#    define MIN_INT32S               ((INT32S)0x80000000L)
#  endif

#  ifndef MAX_INT64U
#    define MAX_INT64U               ((INT64U)0xFFFFFFFFFFFFFFFFULL)
#  endif

#  ifndef MIN_INT64U
#    define MIN_INT64U               ((INT64U)0x0000000000000000ULL)
#  endif

#  ifndef MAX_INT64S
#    define MAX_INT64S               ((INT64S)0x7FFFFFFFFFFFFFFFLL)
#  endif

#  ifndef MIN_INT64S
#    define MIN_INT64S               ((INT64S)0x8000000000000000LL)
#  endif

/**
 * Possible alignments: The following defines can be used with the next macros to
 * define aligned arrays. You can add more, just follow the intuitive way,
 * but do not use macros to align to values not power of 2.
 **/
#  define ALIGNED_4_BYTES      4
#  define ALIGNED_8_BYTES      8
#  define ALIGNED_16_BYTES    16
#  define ALIGNED_32_BYTES    32
#  define ALIGNED_64_BYTES    64
#  define ALIGNED_128_BYTES  128

/**
 * If you want to define an aligned array you have to use the following macro. Example:
 * Suppose you want to define this array
 * 
 * INT16U my_array[23];
 * 
 * and you want it to be aligned to 64 bytes in section .bss64. You should do this:
 * 
 * DEFINE_ALIGNED_ARRAY(INT16U, my_array, 23, ALIGNED_64_BYTES) SDRAM_BSS64; // <- keep an eye on this :)
 *
 * It may look strange, but doing it that way assures that the size of the array is big enough to 
 * be aligned properly and allow other variables in the same section to be aligned too.
 *
 * To add extra qualifiers, put them before the macro. Example: Align this to 64 bytes:
 *
 * static INT32S * static_array_of_pointers[7 * NUM_ELEMENTS];
 *
 * Becomes:
 * 
 * static DEFINE_ALIGNED_ARRAY(INT32S *, static_array_of_pointers, 7 * NUM_ELEMENTS, ALIGNED_64_BYTES) SDRAM_BSS64; // <- wow!
 *
 * Do not use this macro with arrays of 0 elements (don't use any macro at all, they're always aligned)
 * You can't use as TYPE any type that includes functions, pointers to functions, etc... typedef-ine it before.
 **/
#  define DEFINE_ALIGNED_ARRAY(TYPE, NAME, NUM_ELEMENTS, ALIGNMENT) \
  TYPE (NAME)[((NUM_ELEMENTS - 1) / SLCM(ALIGNMENT, sizeof(TYPE)) + 1) * SLCM(ALIGNMENT, sizeof(TYPE))]

/**
 * It is possible that the previous macro seems a bit confusing. In that case, it is also possible
 * to use the following one. However you _must_ assure that the type you declare the array and the 
 * type you put in the macro are the same.
 *
 * Using this macro, the previous examples would have been:
 * 
 * INT16U my_array[ALIGNED_ARRAY_ELEMENTS(23, INT16U, ALIGNED_64_BYTES)] SDRAM_BSS64; // <- the macro is between "[" and "]"
 *
 * static INT32S * static_array_of_pointers[ALIGNED_ARRAY_ELEMENTS(7 * NUM_ELEMENTS, INT32S *, ALIGNED_64_BYTES)] SDRAM_BSS64;
 *      // ^^--> remember to put the _same_ type, "INT32S *" in this case -->-->-->-->-->^^
 *
 * Also, for multidimensional arrays (like arr[5][13][7]) the following macro is the one that must be used. And it must be applied only to the
 * last dimension of the array. Example: Align this to 64 bytes:
 *
 * INT8S arr[5][13][7];
 *
 * Result:
 *  
 * INT8S arr[5][13][ALIGNED_ARRAY_ELEMENTS(7, INT8S, ALIGNED_64_BYTES)] SDRAM_BSS64; // <- the macro only in the rightmost dimension
 * // ^^-> put the type of individual elements ->^^ , do not put anything like INT8S ()[5][13]
 *
 * Do not use this macro with arrays of 0 elements (don't use any macro at all, they're always aligned)
 * You can't use as TYPE any type that includes functions, pointers to functions, etc... typedef-ine it before.
 **/
#  define ALIGNED_ARRAY_ELEMENTS(NUM_ELEMENTS, TYPE, ALIGNMENT) \
  (((NUM_ELEMENTS - 1) / SLCM(ALIGNMENT, sizeof(TYPE)) + 1) * SLCM(ALIGNMENT, sizeof(TYPE)))

/**
 * The following macro calculates the least common multiple of 2 numbers and divides it by second one.
 * The first number _must_ be a power of 2.
 * Both numbers _must_ be integers and greater than 0 (have to be signed integers)
 * This macro is used by the macros DEFINE_ALIGNED_ARRAY and ALIGNED_ARRAY_ELEMENTS.
 *
 * Special Least Common Multiple:
 **/
#  define SLCM(POWER_OF_2, OTHER_NUM) (((POWER_OF_2) / ((OTHER_NUM) & (~(OTHER_NUM) + 1))) ? ((POWER_OF_2) / ((OTHER_NUM) & (~(OTHER_NUM) + 1))) : 1)

/**
 * Now, suppose you want to define an aligned array in the stack, an array local to a function,
 * not an array placed in a particular section of the code, and you want it to be aligned to some boundary.
 * Then you have to use the following macro. Example: You have this array:
 *
 * INT32U my_local_array[55];
 *
 * and you want to align it to 64 bytes. Then you write:
 *
 * DEFINE_LOCAL_ALIGNED_ARRAY(INT32U, my_local_array, 55, ALIGNED_64_BYTES);
 *
 * Just that.
 *
 * Note that there are several limitations in this macro:
 *   - The macro defines an array called what you chose for the array prefixed with "__not_aligned_" and a pointer to TYPE
 *     that points to the first aligned address within the array.
 *   - The array defined is big enough so all the elements fit from the adddress of the pointer to the end of the array
 *   - The pointer defined is named NAME, and because it is a pointer, and not the real array, the operator sizeof
 *     will NOT return the value you expect. In particular, it can return 4 when you expect sizeof(TYPE)*NUM_ELEMENTS
 *   - This macro may not work with multidimensional arrays.
 *   - You can't use as TYPE any type that includes functions, pointers to functions, etc... typedef-ine it before.
 *   - Do not use this macro with arrays of 0 elements (don't use any macro at all, they're always aligned)
 **/
#  define DEFINE_LOCAL_ALIGNED_ARRAY(TYPE, NAME, NUM_ELEMENTS, ALIGNMENT)      \
  TYPE __not_aligned_ ## NAME[(NUM_ELEMENTS) + ((ALIGNMENT) + sizeof(TYPE) - 2) / sizeof(TYPE)]; \
  TYPE * NAME = (TYPE *)(((INT32U)__not_aligned_ ## NAME + (ALIGNMENT) - 1) & ~((ALIGNMENT) - 1))

/* 
 * DEPRECATED
 *
 * Typedefs
 *
 * - integers
 *
 *                 8-bit   16-bit  32-bit  64-bit
 *      unsigned   OCTET   WORD    DWORD   QWORD
 *      signed     CHAR    SHORT   LONG    LLONG
 *
 * - misc (floats, enums, ...)
 */

#  if (defined(OCTET) || defined(WORD) || defined(DWORD) || defined(QWORD) || \
     defined(CHAR) || defined(SHORT) || defined(LONG) || defined(LLONG))
#    error "Some integer types previously declared"
#  endif

/* 8-bit integers */
#  if (UCHAR_MAX == 0xFFU)
typedef unsigned char OCTET;

#    ifndef _CHAR_DEFINED_
#      define _CHAR_DEFINED_
typedef char CHAR;              /* Character type (define needed to avoid redef.)     */
#    endif
#  else
#    error "Unsupported OCTET/CHAR size"
#  endif

/* 16-bit integers */
#  if (UINT_MAX == 0xFFFFU)
typedef unsigned int WORD;
typedef int SHORT;
#  elif (USHRT_MAX == 0xFFFFU)
typedef unsigned short WORD;
typedef short SHORT;
#  else
#    error "Unsupported WORD/SHORT size"
#  endif

/* 32-bit integers */
#  if (ULONG_MAX == 0xFFFFFFFFUL)
typedef unsigned long DWORD;
typedef long LONG;
#  elif (UINT_MAX == 0xFFFFFFFFUL)
typedef unsigned int DWORD;
typedef int LONG;
#  else
#    error "Unsupported DWORD/LONG size"
#  endif

/* 64-bit integers */
#  if !defined(_CONFIG_LOADER_)
#    ifndef NCHECK_ULLONG
#      if (ULLONG_MAX == 0xFFFFFFFFFFFFFFFFULL)
typedef unsigned long long QWORD;
typedef long long LLONG;
#      else
/* don't complain if there is no long long */
#      endif
#    endif
#  endif
       // CONFIG_LOADER

/* convenience integer types */
typedef OCTET BYTE;

#  ifndef _WINDEF_H
#    ifndef BOOL_DECLARED
typedef OCTET BOOL;
#    endif
typedef SHORT INT;
#  endif
       //_WINDEF_H

/* floating point */
typedef float   FLOAT;
typedef double  DOUBLE;

/* special pointer types */
typedef DWORD   DRAM_P;
typedef DWORD   SRAM_P;
typedef DWORD   DM_P;
typedef DWORD   DP_P;
typedef DWORD   UCODE_P;
typedef DWORD   VP_P;

#  if !defined(TOOLS)
#    define OK           0
#    define FAIL        (-1)
#    define CONTINUE     1

#    ifdef REMOVE_DEBUG_LOGS
  /*@ignore@ */
#      define SYSLOG(x,...)
  /*@end@ */
#    else
#      define SYSLOG syslog
#    endif
#  endif
       //TOOLS

#  if defined(TOOLS)
#    define DEBUG(x) (x)
#    define SYSLOG(pri, fmt, args...) printf(fmt, ## args)
#    define ASSERT(x) if(!(x)) { printf("Assertion failed at line %d in %s\n", \
    __LINE__, __FILE__); exit(-1);}
/* common ioctl requests */
#    define FIOSEEK         (128 /* seek (LONG,LONG,LONG*)*/)
#    define FIOFLUSH        (130 /* flush (void)          */)
#  endif

/* error types not defined in libc */
#  if defined(TOOLS) 
#    define	ENOERR 0            /* no error */
#    define	ECRC 4              /* crc error */
#    define	EBADREQ 5           /* bad request */
#  endif

// TOOLS

#endif // _INTERNAL_TYPES_H_
