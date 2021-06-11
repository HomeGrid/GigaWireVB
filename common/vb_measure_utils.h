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
 * @file vb_measure_utils.h
 * @brief Measure common utilities interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016/12/01
 *
 **/

#ifndef VB_MEASURE_UTILS_H_
#define VB_MEASURE_UTILS_H_

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

#define VB_LAST_CARRIER_IDX (4095)

#define FREQ2GRIDCARRIERIDX(F, S)   (((F) == MAX_INT32U)?VB_LAST_CARRIER_IDX:(((((F)*VB_LAST_CARRIER_IDX)/200000) - 73)/(S)))
#define FREQ2ABSOLUTECARRIERIDX(F)  (((F) == MAX_INT32U)?VB_LAST_CARRIER_IDX:(((F)*VB_LAST_CARRIER_IDX)/200000))

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef enum {
  DMDEVICE    = 0,
  EPDEVICE    = 1
} t_DeviceType;

typedef enum {
  BNGNOISE_MEASURE  = 0,
  CFRMODULE_MEASURE  = 1,
  CFRMODULEPHASE_MEASURE  = 2
} t_MeasureType;

typedef enum{
   MEASURE_INT = 0,
   MEASURE_FLOAT
 }t_MeasureDataType;

 typedef enum{
   MEASURE_DB = 0,
   MEASURE_LINEAR
 }t_MeasureDataFormat;

typedef struct s_mesconfdata
 {
   INT16U  NumCycles;
   INT8U   StorageType;
   INT8U   SymbolsNumber;
   INT8U   TimeAveraging;
   INT8U   FrecuencyAveraging;
   INT32U  Offset;
   INT32U  Duration;
   INT8U   CFRMeasureType;
   t_MeasureDataType    MeasureDataType;
   t_MeasureDataFormat    MeasureDataFormat;
 }t_measconfdata;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Shows if given measure is valid
 * @param[in] planId Measure plan Id to check
 * @param[in] measure Pointer to measure to check
 * @return TRUE: if measure is valid and updated; FALSE: otherwise
 **/
BOOLEAN VbMeasureIsValid(INT8U planId, t_processMeasure *measure);

/**
 * @brief Dumps given measure
 * @param[in] mac Measurer MAC address
 * @param[in] nodeType Measurer node type
 * @param[in] driverId Driver id string
 * @param[in] measType Measure type
 * @param[in] measure Pointer to measure to dump (can be NULL when the measure is PSD)
 * @param[in] crossMeasure Pointer to cross measure data (can be NULL when measure is not a CFR one)
 * @param[in] psd Pointer to PSD measure (can be NULL when measure is not a PSD one)
 * @param[in] planId Current measure plan Id
 * @param[in] writeFun Pointer to write function
 **/
void VbMeasureDump(const INT8U *mac, t_nodeType nodeType, CHAR *driverId, t_measType measType, t_processMeasure *measure, t_crossMeasure *crossMeasure, t_PSD *psd, INT8U planId, t_writeFun writeFun);

#endif /* VB_MEASURE_UTILS_H_ */

/**
 * @}
 **/


