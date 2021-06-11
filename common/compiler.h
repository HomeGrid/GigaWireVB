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
 * @file compiler.h
 * @brief Compiler related definitions
 *
 * @internal
 *
 * @author V.Grau
 * @date 28/04/2015
 *  
**/

#if !defined(_COMPILER_H_)
#  define _COMPILER_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define PACKMEMBER           __attribute__((packed))
#define PACKSTRUCT           __attribute__((packed)) __attribute__((aligned(4)))
#define ALIGNED32            __attribute__((aligned (4)))
#define TYPE_PACKMEMBER(X)   __attribute__((packed)) X
#define TYPE_PACKSTRUCT(X)   __attribute__((packed)) __attribute__((aligned(4))) X
#define TYPE_ALIGNED32(X)    __attribute__((aligned(4))) X
#define TYPE_ALIGNED512(X)   __attribute__((aligned(64))) X
#define ATTRIBUTE(X,Y)       __attribute__((Y)) X
#define FUNC_PURE            __attribute__((pure))

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public inline function
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

#endif // _COMPILER_H_
