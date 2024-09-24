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
 * @file vb_SNR_calculation.c
 * @brief SNR calculation
 *
 * @internal
 *
 * @author
 * @date 19/02/2015
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <string.h>
#include <math.h>
#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_engine_drivers_list.h"
#include "vb_engine_cluster_list.h"
#include "vb_log.h"
#include "vb_util.h"
#include "vb_priorities.h"
#include "vb_thread.h"
#include "vb_engine_SNR_calculation.h"
#include "vb_engine_l2rPSD_calculation.h"
#include "vb_engine_metrics_reports.h"
#include "vb_engine_conf.h"
#include "vb_engine_process.h"
#include "vb_engine_measure.h"
#include "vb_engine_communication.h"
#include "vb_engine_cdta.h"

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_ENGINE_COMPUTATION_THREAD_NAME           "vb_engine_computation"

#define MAXVALUELINEARIZE025GRIDTABLE               (10)
#define MINVALUELINEARIZE025GRIDTABLE               (-200)
#define MAXINDEXLINEARIZE025GRIDTABLE               (840)
#define IS_LINEAR_MEASURE(MEASURE)                  (((MEASURE)->flags  & 0x01) == 1)

#define INDEX_LINEARIZE_TABLE(Value)                (Value > MAXVALUELINEARIZE025GRIDTABLE ? MAXINDEXLINEARIZE025GRIDTABLE :\
                                                     (Value < MINVALUELINEARIZE025GRIDTABLE ? 0 : \
                                                         (Value - MINVALUELINEARIZE025GRIDTABLE) * 4) )

#define MEASURE_NUM_SNR_CARRIER_VALID_LOW_BAND      (200)
#define MEASURE_NUM_SNR_CARRIER_VAL_GOOD_ENOUGH     (10)
#define MEASURE_NUM_CARRIERS_TXMODE_200MHZ          (3500)

#define VB_ENGINE_CHANNEL_CAPACITY_SNR_OFFSET       (5.5)
#define VB_ENGINE_CHANNEL_CAPACITY_MAX_BPC          (10.66) // 16/18*12
#define VB_ENGINE_CHANNEL_CAPACITY_MAX_BPC_LOW      (10) // 5/6*12

#define VB_ENGINE_CHANNEL_CAPACITY_SYMBOL_DURATION_SECONDS (0.00002176)
#define VB_ENGINE_1MBITS_PER_SEC                           (1000000)


/* This is a look up table to obtain the linear value from db.
 * The range of values to convert is [-200, 10] with 0.25 increment.
 * The formula used to obtain the values is 10^(X/10)
 */
const float LINEZLIZE_025GRID[] =
{
    1E-20,        1.05925E-20,  1.12202E-20,  1.1885E-20,   1.25893E-20,  1.33352E-20,  1.41254E-20,  1.49624E-20,  1.58489E-20,  1.6788E-20,
    1.77828E-20,  1.88365E-20,  1.99526E-20,  2.11349E-20,  2.23872E-20,  2.37137E-20,  2.51189E-20,  2.66073E-20,  2.81838E-20,  2.98538E-20,
    3.16228E-20,  3.34965E-20,  3.54813E-20,  3.75837E-20,  3.98107E-20,  4.21697E-20,  4.46684E-20,  4.73151E-20,  5.01187E-20,  5.30884E-20,
    5.62341E-20,  5.95662E-20,  6.30957E-20,  6.68344E-20,  7.07946E-20,  7.49894E-20,  7.94328E-20,  8.41395E-20,  8.91251E-20,  9.44061E-20,
    1E-19,        1.05925E-19,  1.12202E-19,  1.1885E-19,   1.25893E-19,  1.33352E-19,  1.41254E-19,  1.49624E-19,  1.58489E-19,  1.6788E-19,
    1.77828E-19,  1.88365E-19,  1.99526E-19,  2.11349E-19,  2.23872E-19,  2.37137E-19,  2.51189E-19,  2.66073E-19,  2.81838E-19,  2.98538E-19,
    3.16228E-19,  3.34965E-19,  3.54813E-19,  3.75837E-19,  3.98107E-19,  4.21697E-19,  4.46684E-19,  4.73151E-19,  5.01187E-19,  5.30884E-19,
    5.62341E-19,  5.95662E-19,  6.30957E-19,  6.68344E-19,  7.07946E-19,  7.49894E-19,  7.94328E-19,  8.41395E-19,  8.91251E-19,  9.44061E-19,
    1E-18,        1.05925E-18,  1.12202E-18,  1.1885E-18,   1.25893E-18,  1.33352E-18,  1.41254E-18,  1.49624E-18,  1.58489E-18,  1.6788E-18,
    1.77828E-18,  1.88365E-18,  1.99526E-18,  2.11349E-18,  2.23872E-18,  2.37137E-18,  2.51189E-18,  2.66073E-18,  2.81838E-18,  2.98538E-18,
    3.16228E-18,  3.34965E-18,  3.54813E-18,  3.75837E-18,  3.98107E-18,  4.21697E-18,  4.46684E-18,  4.73151E-18,  5.01187E-18,  5.30884E-18,
    5.62341E-18,  5.95662E-18,  6.30957E-18,  6.68344E-18,  7.07946E-18,  7.49894E-18,  7.94328E-18,  8.41395E-18,  8.91251E-18,  9.44061E-18,
    1E-17,        1.05925E-17,  1.12202E-17,  1.1885E-17,   1.25893E-17,  1.33352E-17,  1.41254E-17,  1.49624E-17,  1.58489E-17,  1.6788E-17,
    1.77828E-17,  1.88365E-17,  1.99526E-17,  2.11349E-17,  2.23872E-17,  2.37137E-17,  2.51189E-17,  2.66073E-17,  2.81838E-17,  2.98538E-17,
    3.16228E-17,  3.34965E-17,  3.54813E-17,  3.75837E-17,  3.98107E-17,  4.21697E-17,  4.46684E-17,  4.73151E-17,  5.01187E-17,  5.30884E-17,
    5.62341E-17,  5.95662E-17,  6.30957E-17,  6.68344E-17,  7.07946E-17,  7.49894E-17,  7.94328E-17,  8.41395E-17,  8.91251E-17,  9.44061E-17,
    1E-16,        1.05925E-16,  1.12202E-16,  1.1885E-16,   1.25893E-16,  1.33352E-16,  1.41254E-16,  1.49624E-16,  1.58489E-16,  1.6788E-16,
    1.77828E-16,  1.88365E-16,  1.99526E-16,  2.11349E-16,  2.23872E-16,  2.37137E-16,  2.51189E-16,  2.66073E-16,  2.81838E-16,  2.98538E-16,
    3.16228E-16,  3.34965E-16,  3.54813E-16,  3.75837E-16,  3.98107E-16,  4.21697E-16,  4.46684E-16,  4.73151E-16,  5.01187E-16,  5.30884E-16,
    5.62341E-16,  5.95662E-16,  6.30957E-16,  6.68344E-16,  7.07946E-16,  7.49894E-16,  7.94328E-16,  8.41395E-16,  8.91251E-16,  9.44061E-16,
    1E-15,        1.05925E-15,  1.12202E-15,  1.1885E-15,   1.25893E-15,  1.33352E-15,  1.41254E-15,  1.49624E-15,  1.58489E-15,  1.6788E-15,
    1.77828E-15,  1.88365E-15,  1.99526E-15,  2.11349E-15,  2.23872E-15,  2.37137E-15,  2.51189E-15,  2.66073E-15,  2.81838E-15,  2.98538E-15,
    3.16228E-15,  3.34965E-15,  3.54813E-15,  3.75837E-15,  3.98107E-15,  4.21697E-15,  4.46684E-15,  4.73151E-15,  5.01187E-15,  5.30884E-15,
    5.62341E-15,  5.95662E-15,  6.30957E-15,  6.68344E-15,  7.07946E-15,  7.49894E-15,  7.94328E-15,  8.41395E-15,  8.91251E-15,  9.44061E-15,
    1E-14,        1.05925E-14,  1.12202E-14,  1.1885E-14,   1.25893E-14,  1.33352E-14,  1.41254E-14,  1.49624E-14,  1.58489E-14,  1.6788E-14,
    1.77828E-14,  1.88365E-14,  1.99526E-14,  2.11349E-14,  2.23872E-14,  2.37137E-14,  2.51189E-14,  2.66073E-14,  2.81838E-14,  2.98538E-14,
    3.16228E-14,  3.34965E-14,  3.54813E-14,  3.75837E-14,  3.98107E-14,  4.21697E-14,  4.46684E-14,  4.73151E-14,  5.01187E-14,  5.30884E-14,
    5.62341E-14,  5.95662E-14,  6.30957E-14,  6.68344E-14,  7.07946E-14,  7.49894E-14,  7.94328E-14,  8.41395E-14,  8.91251E-14,  9.44061E-14,
    1E-13,        1.05925E-13,  1.12202E-13,  1.1885E-13,   1.25893E-13,  1.33352E-13,  1.41254E-13,  1.49624E-13,  1.58489E-13,  1.6788E-13,
    1.77828E-13,  1.88365E-13,  1.99526E-13,  2.11349E-13,  2.23872E-13,  2.37137E-13,  2.51189E-13,  2.66073E-13,  2.81838E-13,  2.98538E-13,
    3.16228E-13,  3.34965E-13,  3.54813E-13,  3.75837E-13,  3.98107E-13,  4.21697E-13,  4.46684E-13,  4.73151E-13,  5.01187E-13,  5.30884E-13,
    5.62341E-13,  5.95662E-13,  6.30957E-13,  6.68344E-13,  7.07946E-13,  7.49894E-13,  7.94328E-13,  8.41395E-13,  8.91251E-13,  9.44061E-13,
    1E-12,        1.05925E-12,  1.12202E-12,  1.1885E-12,   1.25893E-12,  1.33352E-12,  1.41254E-12,  1.49624E-12,  1.58489E-12,  1.6788E-12,
    1.77828E-12,  1.88365E-12,  1.99526E-12,  2.11349E-12,  2.23872E-12,  2.37137E-12,  2.51189E-12,  2.66073E-12,  2.81838E-12,  2.98538E-12,
    3.16228E-12,  3.34965E-12,  3.54813E-12,  3.75837E-12,  3.98107E-12,  4.21697E-12,  4.46684E-12,  4.73151E-12,  5.01187E-12,  5.30884E-12,
    5.62341E-12,  5.95662E-12,  6.30957E-12,  6.68344E-12,  7.07946E-12,  7.49894E-12,  7.94328E-12,  8.41395E-12,  8.91251E-12,  9.44061E-12,
    1E-11,        1.05925E-11,  1.12202E-11,  1.1885E-11,   1.25893E-11,  1.33352E-11,  1.41254E-11,  1.49624E-11,  1.58489E-11,  1.6788E-11,
    1.77828E-11,  1.88365E-11,  1.99526E-11,  2.11349E-11,  2.23872E-11,  2.37137E-11,  2.51189E-11,  2.66073E-11,  2.81838E-11,  2.98538E-11,
    3.16228E-11,  3.34965E-11,  3.54813E-11,  3.75837E-11,  3.98107E-11,  4.21697E-11,  4.46684E-11,  4.73151E-11,  5.01187E-11,  5.30884E-11,
    5.62341E-11,  5.95662E-11,  6.30957E-11,  6.68344E-11,  7.07946E-11,  7.49894E-11,  7.94328E-11,  8.41395E-11,  8.91251E-11,  9.44061E-11,
    1E-10,        1.05925E-10,  1.12202E-10,  1.1885E-10,   1.25893E-10,  1.33352E-10,  1.41254E-10,  1.49624E-10,  1.58489E-10,  1.6788E-10,
    1.77828E-10,  1.88365E-10,  1.99526E-10,  2.11349E-10,  2.23872E-10,  2.37137E-10,  2.51189E-10,  2.66073E-10,  2.81838E-10,  2.98538E-10,
    3.16228E-10,  3.34965E-10,  3.54813E-10,  3.75837E-10,  3.98107E-10,  4.21697E-10,  4.46684E-10,  4.73151E-10,  5.01187E-10,  5.30884E-10,
    5.62341E-10,  5.95662E-10,  6.30957E-10,  6.68344E-10,  7.07946E-10,  7.49894E-10,  7.94328E-10,  8.41395E-10,  8.91251E-10,  9.44061E-10,
    0.000000001,  1.05925E-09,  1.12202E-09,  1.1885E-09,   1.25893E-09,  1.33352E-09,  1.41254E-09,  1.49624E-09,  1.58489E-09,  1.6788E-09,
    1.77828E-09,  1.88365E-09,  1.99526E-09,  2.11349E-09,  2.23872E-09,  2.37137E-09,  2.51189E-09,  2.66073E-09,  2.81838E-09,  2.98538E-09,
    3.16228E-09,  3.34965E-09,  3.54813E-09,  3.75837E-09,  3.98107E-09,  4.21697E-09,  4.46684E-09,  4.73151E-09,  5.01187E-09,  5.30884E-09,
    5.62341E-09,  5.95662E-09,  6.30957E-09,  6.68344E-09,  7.07946E-09,  7.49894E-09,  7.94328E-09,  8.41395E-09,  8.91251E-09,  9.44061E-09,
    0.00000001,   1.05925E-08,  1.12202E-08,  1.1885E-08,   1.25893E-08,  1.33352E-08,  1.41254E-08,  1.49624E-08,  1.58489E-08,  1.6788E-08,
    1.77828E-08,  1.88365E-08,  1.99526E-08,  2.11349E-08,  2.23872E-08,  2.37137E-08,  2.51189E-08,  2.66073E-08,  2.81838E-08,  2.98538E-08,
    3.16228E-08,  3.34965E-08,  3.54813E-08,  3.75837E-08,  3.98107E-08,  4.21697E-08,  4.46684E-08,  4.73151E-08,  5.01187E-08,  5.30884E-08,
    5.62341E-08,  5.95662E-08,  6.30957E-08,  6.68344E-08,  7.07946E-08,  7.49894E-08,  7.94328E-08,  8.41395E-08,  8.91251E-08,  9.44061E-08,
    0.0000001,    1.05925E-07,  1.12202E-07,  1.1885E-07,   1.25893E-07,  1.33352E-07,  1.41254E-07,  1.49624E-07,  1.58489E-07,  1.6788E-07,
    1.77828E-07,  1.88365E-07,  1.99526E-07,  2.11349E-07,  2.23872E-07,  2.37137E-07,  2.51189E-07,  2.66073E-07,  2.81838E-07,  2.98538E-07,
    3.16228E-07,  3.34965E-07,  3.54813E-07,  3.75837E-07,  3.98107E-07,  4.21697E-07,  4.46684E-07,  4.73151E-07,  5.01187E-07,  5.30884E-07,
    5.62341E-07,  5.95662E-07,  6.30957E-07,  6.68344E-07,  7.07946E-07,  7.49894E-07,  7.94328E-07,  8.41395E-07,  8.91251E-07,  9.44061E-07,
    0.000001,     1.05925E-06,  1.12202E-06,  1.1885E-06,   1.25893E-06,  1.33352E-06,  1.41254E-06,  1.49624E-06,  1.58489E-06,  1.6788E-06,
    1.77828E-06,  1.88365E-06,  1.99526E-06,  2.11349E-06,  2.23872E-06,  2.37137E-06,  2.51189E-06,  2.66073E-06,  2.81838E-06,  2.98538E-06,
    3.16228E-06,  3.34965E-06,  3.54813E-06,  3.75837E-06,  3.98107E-06,  4.21697E-06,  4.46684E-06,  4.73151E-06,  5.01187E-06,  5.30884E-06,
    5.62341E-06,  5.95662E-06,  6.30957E-06,  6.68344E-06,  7.07946E-06,  7.49894E-06,  7.94328E-06,  8.41395E-06,  8.91251E-06,  9.44061E-06,
    0.00001,      1.05925E-05,  1.12202E-05,  1.1885E-05,   1.25893E-05,  1.33352E-05,  1.41254E-05,  1.49624E-05,  1.58489E-05,  1.6788E-05,
    1.77828E-05,  1.88365E-05,  1.99526E-05,  2.11349E-05,  2.23872E-05,  2.37137E-05,  2.51189E-05,  2.66073E-05,  2.81838E-05,  2.98538E-05,
    3.16228E-05,  3.34965E-05,  3.54813E-05,  3.75837E-05,  3.98107E-05,  4.21697E-05,  4.46684E-05,  4.73151E-05,  5.01187E-05,  5.30884E-05,
    5.62341E-05,  5.95662E-05,  6.30957E-05,  6.68344E-05,  7.07946E-05,  7.49894E-05,  7.94328E-05,  8.41395E-05,  8.91251E-05,  9.44061E-05,
    0.0001,       0.000105925,  0.000112202,  0.00011885,   0.000125893,  0.000133352,  0.000141254,  0.000149624,  0.000158489,  0.00016788,
    0.000177828,  0.000188365,  0.000199526,  0.000211349,  0.000223872,  0.000237137,  0.000251189,  0.000266073,  0.000281838,  0.000298538,
    0.000316228,  0.000334965,  0.000354813,  0.000375837,  0.000398107,  0.000421697,  0.000446684,  0.000473151,  0.000501187,  0.000530884,
    0.000562341,  0.000595662,  0.000630957,  0.000668344,  0.000707946,  0.000749894,  0.000794328,  0.000841395,  0.000891251,  0.000944061,
    0.001,        0.001059254,  0.001122018,  0.001188502,  0.001258925,  0.001333521,  0.001412538,  0.001496236,  0.001584893,  0.001678804,
    0.001778279,  0.001883649,  0.001995262,  0.002113489,  0.002238721,  0.002371374,  0.002511886,  0.002660725,  0.002818383,  0.002985383,
    0.003162278,  0.003349654,  0.003548134,  0.003758374,  0.003981072,  0.004216965,  0.004466836,  0.004731513,  0.005011872,  0.005308844,
    0.005623413,  0.005956621,  0.006309573,  0.006683439,  0.007079458,  0.007498942,  0.007943282,  0.008413951,  0.008912509,  0.009440609,
    0.01,         0.010592537,  0.011220185,  0.011885022,  0.012589254,  0.013335214,  0.014125375,  0.014962357,  0.015848932,  0.01678804,
    0.017782794,  0.018836491,  0.019952623,  0.02113489,   0.022387211,  0.023713737,  0.025118864,  0.026607251,  0.028183829,  0.029853826,
    0.031622777,  0.033496544,  0.035481339,  0.03758374,   0.039810717,  0.04216965,   0.044668359,  0.047315126,  0.050118723,  0.053088444,
    0.056234133,  0.059566214,  0.063095734,  0.066834392,  0.070794578,  0.074989421,  0.079432823,  0.084139514,  0.089125094,  0.094406088,
    0.1,          0.105925373,  0.112201845,  0.118850223,  0.125892541,  0.133352143,  0.141253754,  0.149623566,  0.158489319,  0.167880402,
    0.177827941,  0.188364909,  0.199526231,  0.211348904,  0.223872114,  0.237137371,  0.251188643,  0.266072506,  0.281838293,  0.298538262,
    0.316227766,  0.334965439,  0.354813389,  0.375837404,  0.398107171,  0.421696503,  0.446683592,  0.473151259,  0.501187234,  0.530884444,
    0.562341325,  0.595662144,  0.630957344,  0.668343918,  0.707945784,  0.749894209,  0.794328235,  0.841395142,  0.891250938,  0.944060876,
    1,            1.059253725,  1.122018454,  1.188502227,  1.258925412,  1.333521432,  1.412537545,  1.496235656,  1.584893192,  1.678804018,
    1.77827941,   1.883649089,  1.995262315,  2.11348904,   2.238721139,  2.371373706,  2.511886432,  2.66072506,   2.818382931,  2.985382619,
    3.16227766,   3.349654392,  3.548133892,  3.758374043,  3.981071706,  4.216965034,  4.466835922,  4.73151259,   5.011872336,  5.308844442,
    5.623413252,  5.956621435,  6.309573445,  6.683439176,  7.079457844,  7.498942093,  7.943282347,  8.413951416,  8.912509381,  9.440608763,
    10

};

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


/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

static t_VB_engineErrorCode VbChannelCapacityBandCalculate(INT8U spacing, INT16U firstCarrierIdx, INT16U lastCarrierIdx, INT8U *snr, float *bandCapacity);
static t_VB_engineErrorCode VbChannelCapacityCalculate(INT8U spacing, INT8U *snr, INT16U fisrtValidCarrier, INT16U numMeasures, float *capacity, INT8U maxNumBands);
static t_VB_engineErrorCode VbChannelCapacityAllBoostedFill(t_nodeType nodeType, t_nodeCdtaInfo *nodeCdtaInfo, float *capS1, float *capS2, INT8U maxNumBands);
static t_VB_engineErrorCode VbChannelCapacity1BoostedFill(t_nodeType nodeType, t_nodeCdtaInfo *nodeCdtaInfo, float *capS1, float *capS2, INT8U maxNumBands);
static t_VB_engineErrorCode VbChannelMaxCapacityGet( t_node *node, INT32U *maxCapacity, float *capS1, float *capS2, INT8U maxNumBands);
static t_VB_engineErrorCode VbEngineChannelCapacityCalculate(INT32U clusterId, BOOLEAN *contCalc);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_VB_engineErrorCode VbBoostingMaxNumBandsSet(t_VBDriver *driver, t_node *node, t_node *remoteNode)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  t_vbTxBandPlanMode tx_mode = VB_TX_MODE_200_MHZ;

  if ((driver == NULL) || (node == NULL) || (remoteNode == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    BOOLEAN mimo = remoteNode->measures.snrFullXtalk.mimoInd || node->measures.snrFullXtalk.mimoInd;
    INT16U  num_measures = node->measures.snrFullXtalk.spacing * node->measures.snrFullXtalk.numMeasures;
    INT16U  num_measures_remote = remoteNode->measures.snrFullXtalk.spacing * remoteNode->measures.snrFullXtalk.numMeasures;

    if ((VbEngineConfVbInUpstreamGet() == FALSE) && (node->type == VB_NODE_DOMAIN_MASTER))
    {
      // If Vb in upstream is disabled and node type is DM, discard own measures (DMs measures not performed).
      if ((mimo == TRUE) || (num_measures_remote < MEASURE_NUM_CARRIERS_TXMODE_200MHZ))
      {
        // 100 MHz mode, SISO 100, set 3 Bands only
        tx_mode = VB_TX_MODE_100_MHZ;
      }
    }
    else
    {
      if ((mimo == TRUE) || ((num_measures < MEASURE_NUM_CARRIERS_TXMODE_200MHZ) || (num_measures_remote < MEASURE_NUM_CARRIERS_TXMODE_200MHZ)))
      {
        // 100 MHz mode, SISO 100, set 3 Bands only
        tx_mode = VB_TX_MODE_100_MHZ;
      }
    }

    node->channelSettings.boostInfo.maxNumBands = VbEngineConfNumPSDBandAllocationGet(tx_mode);

    VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "%s : is_mimo %s; num_measures %u; num_measures_remote %u;  maxNumBands %u",
        node->MACStr, mimo?"YES":"NO", num_measures, num_measures_remote, node->channelSettings.boostInfo.maxNumBands);
  }

  return result;
}


static t_VB_engineErrorCode VbSnrDeviceLowBandGoodDetectionCalculate(const t_processMeasure *snrCalculated, t_nodeChannelSettings *channelSettings, INT16U lowBandLastIdx)
{

  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  INT16U actual_carrier;
  INT16U last_carrier;
  INT32U num_snr_carrier_good_enough = 0;
  INT32U num_snr_carrier_good_enough_thr = 0;

  if((snrCalculated == NULL) || (channelSettings == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    if((channelSettings->boostInfo.mode != VB_ENGINE_BOOST_MODE_FORCED_FULL) && (channelSettings->boostInfo.mode != VB_ENGINE_BOOST_MODE_FORCED_LOW))
    {
      channelSettings->boostInfo.mode = VB_ENGINE_BOOST_MODE_AUTO;
      last_carrier = lowBandLastIdx / snrCalculated->spacing;
      num_snr_carrier_good_enough_thr = MEASURE_NUM_SNR_CARRIER_VALID_LOW_BAND / snrCalculated->spacing;

      for(actual_carrier = 0 ; actual_carrier < last_carrier; actual_carrier++)
      {
        // Evaluate SNR in low band
        if (snrCalculated->measuresRx1[actual_carrier] > MEASURE_NUM_SNR_CARRIER_VAL_GOOD_ENOUGH)
        {
          if(++num_snr_carrier_good_enough >= num_snr_carrier_good_enough_thr)
          {
            // Seems to be good enough
            break;
          }
        }
      }

      if(num_snr_carrier_good_enough < num_snr_carrier_good_enough_thr)
      {
        // If would be dangerous (link flapping wise) to go to low band with such bad SNR
        // -> Add 1 more band
        channelSettings->boostInfo.forcedBandsBitmap = ((channelSettings->boostInfo.forcedBandsBitmap) | 2);
      }
    }
  }

  return result;
}

/*******************************************************************/

static INT8U SnrFloatToInt8U(float snrVal)
{
  float aux = 0;
  INT8U snr_fixed_point = 0;

  // Translate to 0.25dB units and round to next integer
  aux = snrVal * 4 + 0.5;

  // Check boundaries
  if (aux < 0)
  {
    aux = 0;
  }

  if (aux > MAX_INT8U)
  {
    aux = MAX_INT8U;
  }

  snr_fixed_point = (INT8U)aux;

  return snr_fixed_point;
}

/*******************************************************************/

static t_VB_engineErrorCode VbSnrSISOIndCalculate(t_VBDriver *driver, t_node *node,
                                                  INT8U *snrCalculated, const t_crossMeasureList *cfrMeasureList,
                                                  const t_processMeasure *bgnMeasure, INT32U lastXtalkCarrierIdx)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  INT32U i;
  INT32U last_cfr_xtalk_carrier;
  INT16U actual_carrier;
  INT16U aux_value_index_table;
  float temp_float;
  t_crossMeasure* cfr_cross_measure;
  float sum_cks_linearized_rx1 = 0;
  float ci_rx1_direct = 0;
  float ni_linearized_rx1;

  if ((driver == NULL) || (node == NULL) || (snrCalculated == NULL) ||
      (cfrMeasureList == NULL) || (bgnMeasure == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    for(actual_carrier = 0 ; actual_carrier < bgnMeasure->numMeasures; actual_carrier++)
    {
      sum_cks_linearized_rx1 = 0;
      ci_rx1_direct = 0;

      temp_float = (((float)(bgnMeasure->measuresRx1[actual_carrier]))/4)
                - bgnMeasure->rxg1Compensation;
      aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
      ni_linearized_rx1 = LINEZLIZE_025GRID[aux_value_index_table];

      for(i=0; i < cfrMeasureList->numCrossMeasures; i++)
      {
        cfr_cross_measure = (t_crossMeasure*)&cfrMeasureList->crossMeasureArray[i];
        if(cfr_cross_measure->ownCFR)
        {
          if(cfr_cross_measure->measure.measuresRx1 != NULL)
          {
            temp_float = (((float)(cfr_cross_measure->measure.measuresRx1[actual_carrier]))/4)
                                       - cfr_cross_measure->measure.rxg1Compensation;
            ci_rx1_direct = temp_float;
          }
          else
          {
            ci_rx1_direct = 0;

            VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - Own CFR missing",
                VbNodeTypeToStr(node->type), node->MACStr);

            result = VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES;
            break;
          }
        }
        else
        {
          if(cfr_cross_measure->measure.measuresRx1 != NULL)
          {
            // Get last carrier index according to profile
            last_cfr_xtalk_carrier = MIN(lastXtalkCarrierIdx, cfr_cross_measure->measure.carrierGridIdxCutProfile);
            if(actual_carrier < last_cfr_xtalk_carrier)
            {
              temp_float = (((float)(cfr_cross_measure->measure.measuresRx1[actual_carrier]))/4)
                                    - cfr_cross_measure->measure.rxg1Compensation;
              aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
              sum_cks_linearized_rx1  += LINEZLIZE_025GRID[aux_value_index_table];
            }
          }
        }
      }

      if(result == VB_ENGINE_ERROR_NONE)
      {
        temp_float = ci_rx1_direct - 10*log10(ni_linearized_rx1 + sum_cks_linearized_rx1);
        snrCalculated[actual_carrier] = SnrFloatToInt8U(temp_float);
      }

      if (result != VB_ENGINE_ERROR_NONE)
      {
        break;
      }
    }
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbSnrMIMOIndCalculate(t_VBDriver *driver, t_node *node,
                                                  INT8U *snrCalculatedS1, INT8U *snrCalculatedS2,
                                                  const t_crossMeasureList *cfrMeasureList,
                                                  const t_processMeasure *bgnMeasure, INT32U lastXtalkCarrierIdx)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  INT32U i;
  INT32U last_cfr_xtalk_carrier;
  INT16U actual_carrier;
  INT16U aux_value_index_table;
  BOOL crossed = FALSE;
  float temp_float;
  t_crossMeasure* cfr_cross_measure;
  float sum_cks_linearized_rx1 = 0;
  float sum_cks_linearized_rx2 = 0;
  float ci_rx1_direct = 0;
  float ci_rx1_crossed = 0;
  float ci_rx2_direct = 0;
  float ci_rx2_crossed = 0;
  float ni_linearized_rx1;
  float ni_linearized_rx2;

  if ((driver == NULL) || (node == NULL) || (snrCalculatedS1 == NULL) ||
      (snrCalculatedS2 == NULL) || (cfrMeasureList == NULL) || (bgnMeasure == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    for(actual_carrier = 0 ; actual_carrier < bgnMeasure->numMeasures ; actual_carrier+=2)
    {
      sum_cks_linearized_rx1 = 0;
      sum_cks_linearized_rx2 = 0;

      ci_rx1_direct = 0;
      ci_rx1_crossed = 0;
      ci_rx2_direct = 0;
      ci_rx2_crossed = 0;

      temp_float = (((float)(bgnMeasure->measuresRx1[actual_carrier]))/4)
                               - bgnMeasure->rxg1Compensation;
      aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
      ni_linearized_rx1 = LINEZLIZE_025GRID[aux_value_index_table];

      temp_float = (((float)(bgnMeasure->measuresRx2[actual_carrier]))/4)
                              - bgnMeasure->rxg2Compensation;
      aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
      ni_linearized_rx2 = LINEZLIZE_025GRID[aux_value_index_table];

      for(i=0; i < cfrMeasureList->numCrossMeasures; i++)
      {
        cfr_cross_measure = (t_crossMeasure*)&cfrMeasureList->crossMeasureArray[i];

        if(cfr_cross_measure->ownCFR)
        {
          // My Direct CFR
          if((cfr_cross_measure->measure.measuresRx1 != NULL) && (cfr_cross_measure->measure.measuresRx2 != NULL))
          {
            // h11
            temp_float = (((float)(cfr_cross_measure->measure.measuresRx1[actual_carrier]))/4)
                                       - cfr_cross_measure->measure.rxg1Compensation;
            ci_rx1_direct = temp_float;

            // h21
            temp_float = (((float)(cfr_cross_measure->measure.measuresRx2[actual_carrier+1]))/4)
                                        - cfr_cross_measure->measure.rxg2Compensation;
            ci_rx1_crossed = temp_float;

            crossed = (ci_rx1_direct >= ci_rx1_crossed)?FALSE:TRUE;

            //h22 (direct)
            temp_float = (((float)(cfr_cross_measure->measure.measuresRx2[actual_carrier]))/4)
                                         - cfr_cross_measure->measure.rxg2Compensation;
            ci_rx2_direct = temp_float;

            //h12 (crossed)
            temp_float = (((float)(cfr_cross_measure->measure.measuresRx1[actual_carrier+1]))/4)
                                      - cfr_cross_measure->measure.rxg1Compensation;
            ci_rx2_crossed = temp_float;
          }
          else
          {
            VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - Own CFR missing",
                VbNodeTypeToStr(node->type), node->MACStr);

            result = VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES;
            break;
          }
        }
        else
        {
          last_cfr_xtalk_carrier = MIN(lastXtalkCarrierIdx, cfr_cross_measure->measure.carrierGridIdxCutProfile);

          // Crosstalk CFR
          if( (cfr_cross_measure->measure.measuresRx1 != NULL) &&
              (actual_carrier < last_cfr_xtalk_carrier) )
          {
            if(cfr_cross_measure->measure.mimoMeas == TRUE)
            {
              // lin 10 ^ (h11/10)
              temp_float = ((float)(cfr_cross_measure->measure.measuresRx1[actual_carrier])/4)
                                             - cfr_cross_measure->measure.rxg1Compensation;

              aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
              sum_cks_linearized_rx1  += LINEZLIZE_025GRID[aux_value_index_table];

              // lin 10 ^ (h12/10)
              temp_float = ((float)(cfr_cross_measure->measure.measuresRx1[actual_carrier+1])/4)
                                             - cfr_cross_measure->measure.rxg1Compensation;

              aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
              sum_cks_linearized_rx1  += LINEZLIZE_025GRID[aux_value_index_table];
            }
            else
            {
              // actual_carrier already points to h11+h12
              temp_float = ((float)(cfr_cross_measure->measure.measuresRx1[actual_carrier])/4)
                                             - cfr_cross_measure->measure.rxg1Compensation;

              aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
              sum_cks_linearized_rx1  += LINEZLIZE_025GRID[aux_value_index_table];
            }
          }

          if( (cfr_cross_measure->measure.measuresRx2 != NULL) &&
              (actual_carrier < last_cfr_xtalk_carrier) )
          {
            if(cfr_cross_measure->measure.mimoMeas == TRUE)
            {
              // lin 10 ^ ((h22)/10)
              temp_float = ((float)(cfr_cross_measure->measure.measuresRx2[actual_carrier])/4)
                                             - cfr_cross_measure->measure.rxg2Compensation;

              aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
              sum_cks_linearized_rx2  += LINEZLIZE_025GRID[aux_value_index_table];

              // lin 10 ^ ((h21)/10)
              temp_float = ((float)(cfr_cross_measure->measure.measuresRx2[actual_carrier+1])/4)
                                            - cfr_cross_measure->measure.rxg2Compensation;

              aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
              sum_cks_linearized_rx2  += LINEZLIZE_025GRID[aux_value_index_table];
            }
            else
            {
              // actual_carrier already points to h22+h21
              temp_float = ((float)(cfr_cross_measure->measure.measuresRx2[actual_carrier])/4)
                                            - cfr_cross_measure->measure.rxg2Compensation;

              aux_value_index_table = INDEX_LINEARIZE_TABLE(temp_float);
              sum_cks_linearized_rx2  += LINEZLIZE_025GRID[aux_value_index_table];
            }
          }
        }

        if (result != VB_ENGINE_ERROR_NONE)
        {
          break;
        }
      }

      if( result == VB_ENGINE_ERROR_NONE)
      {
        if(crossed == FALSE)
        {
          // Stream 1 ====== Rx1
          // Stream 2 ====== Rx2

          // SNR Stream 1 = h11 - 10log(NoiseRx1 + SUM(lin h11 + lin h12))
          temp_float = ci_rx1_direct - 10*log10(ni_linearized_rx1 + sum_cks_linearized_rx1);
          snrCalculatedS1[actual_carrier>>1] = SnrFloatToInt8U(temp_float);

          // SNR Stream 2 = h22 - 10log(NoiseRx2 + SUM(lin h22 + lin h21))
          temp_float = ci_rx2_direct - 10*log10(ni_linearized_rx2 + sum_cks_linearized_rx2);
          snrCalculatedS2[actual_carrier>>1] = SnrFloatToInt8U(temp_float);
        }
        else
        {
          // Stream 1 ====== Rx2
          // Stream 2 ====== Rx1

          // SNR Stream 1 = h21 - 10log(NoiseRx2 + SUM(lin h22 + lin h21))
          temp_float = ci_rx1_crossed - 10*log10(ni_linearized_rx2 + sum_cks_linearized_rx2);
          snrCalculatedS1[actual_carrier>>1] = SnrFloatToInt8U(temp_float);

          // SNR Stream 2 = h12 - 10log(NoiseRx1 + SUM(lin h11 + lin h12))
          temp_float = ci_rx2_crossed - 10*log10(ni_linearized_rx1 + sum_cks_linearized_rx1);
          snrCalculatedS2[actual_carrier>>1] = SnrFloatToInt8U(temp_float);
        }
      }

    }
  }

  return result;
}

/*******************************************************************/

/**
 * @brief Calculate the SNR for a device
 * @param[in] driver Pointer to related driver
 * @param[in] node Pointer to related node
 * @param[in] BGNMeasure Background Noise measured
 * @param[in] CFRMeasureList CFR measured
 * @param[in] SNRCalculated SNR calculated or NULL if error
 * @param[in] xtalkCutOffFreqCarrier frequency above which Crosstalk shall be ignored in the SNR computation
 *
**/
static t_VB_engineErrorCode VbSnrDeviceCalculate(t_VBDriver *driver,
                                                  t_node *node,
                                                  const t_processMeasure *bgnMeasure,
                                                  const t_crossMeasureList *cfrMeasureList,
                                                  t_processMeasure *snrCalculated,
                                                  INT32U xtalkCutOffFreqCarrier)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  BOOL linear_measures;
  INT16U last_xtalk_carrier_idx;
  // SNR = Ci - 10*log10(10^(Ni/10) + SUM(10^(Ck/10))) where
  // Ci = line CFR
  // Ni = line BGN
  // Ck = Crosstalk CFR

  if ((snrCalculated == NULL) || (driver == NULL) || (node == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    if ((snrCalculated->measuresRx1 != NULL) ||
        (snrCalculated->measuresRx2 != NULL))
    {
       VbDatamodelNodeProcessMeasureDestroy(snrCalculated);
    }

    if(bgnMeasure == NULL)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - Missing measures of BGN to calculate SNR",
          VbNodeTypeToStr(node->type), node->MACStr);
      result = VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES;
    }
    else if (cfrMeasureList == NULL)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - Missing List of measures of CFR to calculate SNR",
          VbNodeTypeToStr(node->type), node->MACStr);
      result = VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES;
    }
    else if(bgnMeasure->measuresRx1 == NULL)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - Missing measure by carrier of BGN to calculate SNR",
          VbNodeTypeToStr(node->type), node->MACStr);
      result = VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES;
    }
    else if(bgnMeasure->numMeasures == 0)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - Number of measures 0 of BGN to calculate SNR",
          VbNodeTypeToStr(node->type), node->MACStr);
      result = VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES;
    }
    else if(cfrMeasureList->numCrossMeasures == 0)
    {
      VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - Number of measures 0 of CFR lists to calculate SNR",
          VbNodeTypeToStr(node->type), node->MACStr);
      result = VB_ENGINE_ERROR_SNRCALCERROR_NOMEASURES;
    }
    else
    {
      snrCalculated->errorCode     = VB_MEAS_ERRCODE_VALID;
      snrCalculated->firstCarrier  = bgnMeasure->firstCarrier;
      snrCalculated->flags         = bgnMeasure->flags;
      snrCalculated->planID        = bgnMeasure->planID;
      snrCalculated->spacing       = bgnMeasure->spacing;
      snrCalculated->numMeasures   = bgnMeasure->numMeasures;
      snrCalculated->mimoInd       = bgnMeasure->mimoInd;
      snrCalculated->mimoMeas      = bgnMeasure->mimoMeas;

      linear_measures = IS_LINEAR_MEASURE(bgnMeasure);

      if (linear_measures)
      {
        result = VB_ENGINE_ERROR_SNRCALCERROR_DATA_LINEAR;
      }
      else
      {
        if (snrCalculated->mimoInd == FALSE)
        {
          snrCalculated->measuresRx1 = (INT8U *)calloc(1, bgnMeasure->numMeasures);

          if (snrCalculated->measuresRx1 != NULL)
          {
            // Node measurer Mode is SISO
            // Look for Ni linearized,  Ci and sum of Cks linearizeds per carrier
            last_xtalk_carrier_idx = (xtalkCutOffFreqCarrier)/bgnMeasure->spacing;
            result = VbSnrSISOIndCalculate(driver, node, snrCalculated->measuresRx1, cfrMeasureList, bgnMeasure, last_xtalk_carrier_idx);
          }
          else
          {
            // Alloc Failed
            result = VB_ENGINE_ERROR_MALLOC;
          }
        }
        else
        {
          // Node measurer Mode is MIMO
          // SNR measure will be
          //    - Rx1 |Snr1 ci|Snr1 ci+2|Snr1 ci+4|....
          //    - Rx2 |Snr2 ci|Snr2 ci+2|Snr2 ci+4|....

          snrCalculated->measuresRx1 = (INT8U *)calloc(1, bgnMeasure->numMeasures>>1);

          if (snrCalculated->measuresRx1 == NULL)
          {
            // Alloc Failed
            result = VB_ENGINE_ERROR_MALLOC;
          }

          if (result == VB_ENGINE_ERROR_NONE)
          {
            snrCalculated->measuresRx2 = (INT8U *)calloc(1, bgnMeasure->numMeasures>>1);

            if (snrCalculated->measuresRx2 == NULL)
            {
              // Alloc Failed
              result = VB_ENGINE_ERROR_MALLOC;
            }
          }

          if (result == VB_ENGINE_ERROR_NONE)
          {
            // Look for Ni linearized,  Ci and sum of Cks linearizeds per carrier
            snrCalculated->numMeasures >>= 1;
            snrCalculated->spacing <<= 1;
            last_xtalk_carrier_idx = (xtalkCutOffFreqCarrier)/bgnMeasure->spacing;
            result = VbSnrMIMOIndCalculate(driver, node, snrCalculated->measuresRx1, snrCalculated->measuresRx2,
                cfrMeasureList, bgnMeasure, last_xtalk_carrier_idx);
          }
        }
      }
    }
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbSnrCalculateNodeLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT16U               first_carrier = 0;
  INT16U               low_band_idx_carrier = 0;
  BOOLEAN              bgn_meas_is_valid = FALSE;

  if ((domain == NULL) || (driver == NULL) || (node == NULL) || (args == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    BOOL cont = *((BOOL*)args);
    if(cont == FALSE)
    {
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineDatamodelDomainIsComplete(domain) == TRUE)
    {
      if (ret == VB_ENGINE_ERROR_NONE)
      {
        INT8U plan_id;
        // Check if at least BGN measure is valid

        ret = VbEngineMeasurePlanIdGet(driver->clusterId, &plan_id);
        if (ret == VB_ENGINE_ERROR_NONE)
        {
          bgn_meas_is_valid = VbMeasureIsValid(plan_id, &(node->measures.BGNMeasure));

          if (bgn_meas_is_valid == TRUE)
          {
            first_carrier = node->measures.BGNMeasure.firstCarrier;
          }
          else if ((node->linkedNode != NULL) &&
                   (VbMeasureIsValid(plan_id, &(node->linkedNode->measures.BGNMeasure)) == TRUE))
          {
            // Measure is not valid, get this value from remote node BGN measure
            first_carrier = node->linkedNode->measures.BGNMeasure.firstCarrier;
          }
          else
          {
            // First carrier valid not found
            ret = VB_ENGINE_ERROR_NOT_FOUND;
          }
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Set first valid carrier
        ret = VbEngineDataModelChannelCapacityFirstCarrierSet(node, first_carrier);
        if(ret == VB_ENGINE_ERROR_NONE)
        {
          ret = VbEngineLastRelativeLowBandCarrierIdxGet(first_carrier, &low_band_idx_carrier);
        }
      }

      if ((ret == VB_ENGINE_ERROR_NONE) && (bgn_meas_is_valid == TRUE))
      {
        // Calculate SNR for given node with full interference
#if VB_ENGINE_METRICS_ENABLED
          t_SNRMetricsData *data;
          INT8U *mac = NULL;
        mac = node->MAC;
        VbMetricsReportGenericEvent(VB_METRICS_EVENT_START_SNR_HIGH_CALC, mac, sizeof(INT8U) * ETH_ALEN);
        VbMetricsResetTimeMarkers();
#endif

        ret = VbSnrDeviceCalculate(driver,
                                   node,
                                   &(node->measures.BGNMeasure),
                                   &(node->measures.CFRMeasureList),
                                   &(node->measures.snrFullXtalk),
                                   node->measures.BGNMeasure.numMeasures * node->measures.BGNMeasure.spacing);

#if VB_ENGINE_METRICS_ENABLED
        if(VbMetricsGetStatus())
        {
          data = (t_SNRMetricsData *)malloc(sizeof(t_SNRMetricsData));
          if(data)
          {
            MACAddrClone(data->mac, mac);
        #if (_VALGRIND_ == 1)
            ANNOTATE_HAPPENS_BEFORE(data);
        #endif

            VbMetricsGetCurrentTimeMarkersValues(data->vbMetricsTimeMarkersList);
            VbMetricsReportEvent(VB_METRICS_EVENT_END_SNR_HIGH_CALC, data);
          }
        }
#endif
      }

      if ((ret == VB_ENGINE_ERROR_NONE) && (bgn_meas_is_valid == TRUE))
      {
        ret = VbSnrDeviceLowBandGoodDetectionCalculate(&(node->measures.snrFullXtalk), &(node->channelSettings),
                                                       low_band_idx_carrier);
      }

      if ((ret == VB_ENGINE_ERROR_NONE) && (bgn_meas_is_valid == TRUE))
      {
#if VB_ENGINE_METRICS_ENABLED
          t_SNRMetricsData *data;
          INT8U *mac = NULL;
        mac = node->MAC;
        VbMetricsReportGenericEvent(VB_METRICS_EVENT_START_SNR_LOW_CALC, mac, sizeof(INT8U) * ETH_ALEN);
        VbMetricsResetTimeMarkers();
#endif
        // Estimate SNR taking into account interference in low band and no interference from low band to the end of spectrum


        ret = VbSnrDeviceCalculate(driver,
                                   node,
                                   &(node->measures.BGNMeasure),
                                   &(node->measures.CFRMeasureList),
                                   &(node->measures.snrLowXtalk),
                                   low_band_idx_carrier);

#if VB_ENGINE_METRICS_ENABLED
        if(VbMetricsGetStatus())
        {
          data = (t_SNRMetricsData *)malloc(sizeof(t_SNRMetricsData));
          if(data)
          {
            MACAddrClone(data->mac, mac);
        #if (_VALGRIND_ == 1)
            ANNOTATE_HAPPENS_BEFORE(data);
        #endif

            VbMetricsGetCurrentTimeMarkersValues(data->vbMetricsTimeMarkersList);
            VbMetricsReportEvent(VB_METRICS_EVENT_END_SNR_LOW_CALC, data);
          }
        }
#endif

      }
    }
  }

  if ((ret != VB_ENGINE_ERROR_NONE) && (node != NULL))
  {
    VbLogPrintExt(VB_LOG_ERROR, driver->vbDriverID, "%s MAC %s - SNR calculation error %d",
        VbNodeTypeToStr(node->type), node->MACStr, ret);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode  VbSnrListDomainMacsCalculate(INT32U clusterId, BOOL *contCalc)
{
  t_VB_engineErrorCode ret;

  if(contCalc == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  // Loop through all domains and calculate SNR
  ret = VbEngineDatamodelClusterXAllNodesLoop(VbSnrCalculateNodeLoopCb, clusterId, contCalc);
  if (ret != VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "SNR Calculation Error %d", ret);
  }

  return ret;
}

/*******************************************************************/

static void *VbEngineSNRAndCapacityCompute(void *args)
{
  t_VB_engineErrorCode error = VB_ENGINE_ERROR_NONE;
  INT32U               clusterId;
  t_VBCluster         *cluster;

  if(args == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    cluster = ((t_VBCluster*)args);

    clusterId = cluster->clusterInfo.clusterId;

    // Loop through all domains of cluster Id and calculate SNR (low band & Full)
    error = VbSnrListDomainMacsCalculate(clusterId, &cluster->snrComputationThreadRunning);

    if(error == VB_ENGINE_ERROR_NONE)
    {
      // Then compute channel capacities
      error = VbEngineChannelCapacityCalculate(clusterId, &cluster->snrComputationThreadRunning);
    }

    if (cluster->snrComputationThreadRunning == TRUE)
    {
      if(error == VB_ENGINE_ERROR_NONE)
      {
        VbEngineProcessClusterXDriversEvSend(ENGINE_EV_COMPUTATION_OK, NULL, clusterId);
      }

      if (error != VB_ENGINE_ERROR_NONE)
      {
        // Send error event to restart from scratch
        VbEngineProcessClusterXDriversEvSend(ENGINE_EV_COMPUTATION_KO, NULL, clusterId);

        VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "SNR Capacity computation issue (%d)", error);
      }
    }
  }

  return NULL;
}

/*******************************************************************/

static t_VB_engineErrorCode VbChannelCapacityBandCalculate(INT8U spacing, INT16U firstCarrierIdx, INT16U lastCarrierIdx, INT8U *snr, float *bandCapacity)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  INT16U i;
  INT16U last_carrier_idx = 0;
  INT16U first_carrier_idx = 0;
  float temp_snr;
  float c_aprox[2];
  float c_aprox_to_acum;
  float c_aprox_acum  = 0;
  float max_bpc  = 0;

  if(snr == NULL)
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    last_carrier_idx = lastCarrierIdx/spacing;
    first_carrier_idx = firstCarrierIdx/spacing;
    max_bpc = ((firstCarrierIdx==0)? VB_ENGINE_CHANNEL_CAPACITY_MAX_BPC_LOW:VB_ENGINE_CHANNEL_CAPACITY_MAX_BPC);

    for(i = first_carrier_idx; i <  (last_carrier_idx); i++)
    {
      temp_snr = snr[i];
      temp_snr /= 4.0;
      c_aprox_to_acum = 0;

      c_aprox[0] = 0.334*(temp_snr-30.0-VB_ENGINE_CHANNEL_CAPACITY_SNR_OFFSET)+10;
      if( c_aprox[0] > max_bpc)
      {
        c_aprox[0] = max_bpc;
      }

      c_aprox[1] = 0.171*(temp_snr-5.3-VB_ENGINE_CHANNEL_CAPACITY_SNR_OFFSET)+2.134;
      if( c_aprox[1] > max_bpc)
      {
        c_aprox[1] = max_bpc;
      }

      if(c_aprox[0]>c_aprox_to_acum)
      {
        c_aprox_to_acum = c_aprox[0];
      }

      if(c_aprox[1]>c_aprox_to_acum)
      {
        c_aprox_to_acum = c_aprox[1];
      }

      c_aprox_acum += c_aprox_to_acum;
    }

    *bandCapacity = (c_aprox_acum * spacing) / VB_ENGINE_CHANNEL_CAPACITY_SYMBOL_DURATION_SECONDS;
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbChannelCapacityCalculate(INT8U spacing, INT8U *snr, INT16U firstValidCarrier, INT16U numMeasures, float *capacity, INT8U maxNumBands)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  INT16U i;
  INT16U first_carrier_idx = 0;
  INT16U last_carrier_idx = 0;
  t_psdBandAllocation* bands_allocation = VbEngineConfPSDBandAllocationGet();

  if( (snr == NULL) || (capacity == NULL) || (bands_allocation == NULL) )
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else if( (spacing == 0) || (firstValidCarrier > bands_allocation->lastCarrier[0]) )
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    INT16U last_snr_carrier;

    last_snr_carrier = spacing*numMeasures;
    for(i = 0; i <  maxNumBands; i++)
    {
      first_carrier_idx = (i==0)?0:(bands_allocation->lastCarrier[i-1]+1 - firstValidCarrier);
      last_carrier_idx = MIN((bands_allocation->lastCarrier[i] - firstValidCarrier), last_snr_carrier);
      VbChannelCapacityBandCalculate(spacing, first_carrier_idx, last_carrier_idx,
                                     snr, &capacity[i]);
    }
  }

  return result;
}

/*******************************************************************/

static t_VB_engineErrorCode VbChannelCapacityCalculateLoopCb(t_VBDriver *driver, t_domain *domain, t_node *node, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_node              *remote_node = NULL;
  INT16U               spacing = 0;
  INT8U                expected_plan_id;
  float                *cap_S1_per_bands = NULL;
  float                *cap_S2_per_bands = NULL;


  if ((driver == NULL) || (node == NULL) || (domain == NULL))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    BOOL cont = *((BOOL*)args);
    if(cont == FALSE)
    {
      ret = VB_ENGINE_ERROR_EXIT_LOOP_OK;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    if (VbEngineDatamodelDomainIsComplete(domain) == TRUE)
    {
      if (ret == VB_ENGINE_ERROR_NONE)
      {
        cap_S1_per_bands = calloc(VB_PSD_NUM_BANDS, sizeof(float));
        if(cap_S1_per_bands == NULL)
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }

        if(ret == VB_ENGINE_ERROR_NONE)
        {
          cap_S2_per_bands = calloc(VB_PSD_NUM_BANDS, sizeof(float));
          if(cap_S2_per_bands == NULL)
          {
            ret = VB_ENGINE_ERROR_MALLOC;
          }
        }

        if(ret == VB_ENGINE_ERROR_NONE)
        {
          // Get expected PlanId
          ret = VbEngineMeasurePlanIdGet(driver->clusterId, &expected_plan_id);
          if(ret == VB_ENGINE_ERROR_NONE)
          {
            // Get remote node
            remote_node = node->linkedNode;
            if (remote_node == NULL)
            {
              // Remote node not found. Expected error when EP is not present.
              VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Remote node of %s not found", node->MACStr);

              ret = VB_ENGINE_ERROR_SKIP;
            }
            else if (VbMeasureIsValid(expected_plan_id, &(remote_node->measures.snrFullXtalk)) == FALSE)
            {
              // SNR full xtalk not found. Expected error when VB in upstream is disabled (DMs measures not performed)
              VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "SNR with full xtalk not found for node MAC %s -> remote node MAC %s", node->MACStr, remote_node->MACStr);

              ret = VB_ENGINE_ERROR_SKIP;
            }
            else if (VbMeasureIsValid(expected_plan_id, &(remote_node->measures.snrLowXtalk)) == FALSE)
            {
              // SNR low xtalk not found. Expected error when VB in upstream is disabled (DMs measures not performed)
              VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "SNR with low xtalk not found for node MAC %s -> remote node MAC %s", node->MACStr, remote_node->MACStr);
              ret = VB_ENGINE_ERROR_SKIP;
            }
          }
        }

        if(ret == VB_ENGINE_ERROR_NONE)
        {
          // Calculate maximum number of bands to use
          ret = VbBoostingMaxNumBandsSet(driver, node, remote_node);
        }
  #if VB_ENGINE_METRICS_ENABLED
          VbMetricsReportDeviceEvent(VB_METRICS_EVENT_START_LOW_CHANN_CAP, node->MAC, (node->type == VB_NODE_DOMAIN_MASTER), driver, 0, 0);
  #endif

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // Clean bands capacities about to be filled again
          memset((INT8U *)node->cdtaInfo.bandCapacities, 0x00, sizeof(node->cdtaInfo.bandCapacities));

          // Compute channel capacity in low band rx1
          ret = VbChannelCapacityCalculate(remote_node->measures.snrFullXtalk.spacing,
                                           remote_node->measures.snrFullXtalk.measuresRx1,
                                           remote_node->channelSettings.firstValidCarrier,
                                           remote_node->measures.snrFullXtalk.numMeasures,
                                           cap_S1_per_bands,
                                           node->channelSettings.boostInfo.maxNumBands);
        }

        //*********************************************************************
        // Compute channel capacity of defined bands in all boosted situation
        //*********************************************************************
        if (ret == VB_ENGINE_ERROR_NONE)
        {
          if (remote_node->measures.snrFullXtalk.mimoInd == TRUE)
          {
            // EP is in MIMO mode
            // Compute channel capacity in low band rx2
            ret = VbChannelCapacityCalculate(remote_node->measures.snrFullXtalk.spacing,
                                             remote_node->measures.snrFullXtalk.measuresRx2,
                                             remote_node->channelSettings.firstValidCarrier,
                                             remote_node->measures.snrFullXtalk.numMeasures,
                                             cap_S2_per_bands,
                                             node->channelSettings.boostInfo.maxNumBands);
          }
          else
          {
            // EP is in SISO, rx2 = 0
            memset(cap_S2_per_bands, 0, sizeof(*cap_S2_per_bands)*VB_PSD_NUM_BANDS);
          }
        }
  #if VB_ENGINE_METRICS_ENABLED
          VbMetricsReportDeviceEvent(VB_METRICS_EVENT_END_LOW_CHANN_CAP, node->MAC, (node->type == VB_NODE_DOMAIN_MASTER), driver, 0, 0);
  #endif

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          ret = VbChannelCapacityAllBoostedFill(node->type,
                                                &node->cdtaInfo,
                                                cap_S1_per_bands,
                                                cap_S2_per_bands,
                                                node->channelSettings.boostInfo.maxNumBands);
          if(ret == VB_ENGINE_ERROR_NONE)
          {
            // Set low band capacity
            node->channelSettings.boostInfo.lowBandCapacity = ((INT32U)(cap_S1_per_bands[0] + cap_S2_per_bands[0]) / VB_ENGINE_1MBITS_PER_SEC);
  #if VB_ENGINE_METRICS_ENABLED
            VbMetricsReportDeviceEvent(VB_METRICS_EVENT_START_FULL_CHANN_CAP, node->MAC, (node->type == VB_NODE_DOMAIN_MASTER), driver, 0, 0);
  #endif
            VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Low band capacity of node with MAC %s -> %d",
                          node->MACStr,
                          node->channelSettings.boostInfo.lowBandCapacity);
          }
        }

        //*********************************************************************
        // Compute channel capacity of defined bands in 1 boosted situation
        //*********************************************************************
        if (ret == VB_ENGINE_ERROR_NONE)
        {
          spacing = (remote_node->measures.snrLowXtalk.spacing == 0) ? 1: remote_node->measures.snrLowXtalk.spacing;

          // Compute Max capacity
          ret = VbChannelCapacityCalculate(spacing,
                                           remote_node->measures.snrLowXtalk.measuresRx1,
                                           remote_node->channelSettings.firstValidCarrier,
                                           remote_node->measures.snrLowXtalk.numMeasures,
                                           cap_S1_per_bands,
                                           node->channelSettings.boostInfo.maxNumBands);
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          if (remote_node->measures.snrLowXtalk.mimoInd == TRUE)
          {
            // EP is in MIMO mode
            // Compute channel capacity in Rx2
            ret = VbChannelCapacityCalculate(spacing,
                                             remote_node->measures.snrLowXtalk.measuresRx2,
                                             remote_node->channelSettings.firstValidCarrier,
                                             remote_node->measures.snrLowXtalk.numMeasures,
                                             cap_S2_per_bands,
                                             node->channelSettings.boostInfo.maxNumBands);
          }
          else
          {
            // EP is in SISO, rx2 = 0
            memset(cap_S2_per_bands, 0, sizeof(*cap_S2_per_bands)*VB_PSD_NUM_BANDS);
          }

  #if VB_ENGINE_METRICS_ENABLED
          VbMetricsReportDeviceEvent(VB_METRICS_EVENT_END_FULL_CHANN_CAP, node->MAC, (node->type == VB_NODE_DOMAIN_MASTER), driver, 0, 0);
  #endif
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          ret = VbChannelCapacity1BoostedFill(node->type,
                                              &node->cdtaInfo,
                                              cap_S1_per_bands,
                                              cap_S2_per_bands,
                                              node->channelSettings.boostInfo.maxNumBands);
          if(ret == VB_ENGINE_ERROR_NONE)
          {
            ret = VbChannelMaxCapacityGet(node,
                                          &node->channelSettings.boostInfo.maxCapacity,
                                          cap_S1_per_bands, cap_S2_per_bands,
                                          node->channelSettings.boostInfo.maxNumBands);
          }

          if(ret == VB_ENGINE_ERROR_NONE)
          {
            node->channelSettings.interferenceDetectionCounter = 0;
            node->channelSettings.boostInfo.valid = TRUE;

            VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Max capacity of node %s -> %d", node->MACStr, node->channelSettings.boostInfo.maxCapacity);
          }
        }

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          node->channelSettings.boostInfo.lowBandCapacity = 0;
          node->channelSettings.boostInfo.maxCapacity = 0;
          node->channelSettings.interferenceDetectionCounter = 0;
          node->channelSettings.boostInfo.valid = FALSE;

          if (ret == VB_ENGINE_ERROR_SKIP)
          {
            // Expected error code. Skip current node
            ret = VB_ENGINE_ERROR_NONE;
          }

          if (driver != NULL)
          {
            VbLogPrintExt(VB_LOG_INFO, driver->vbDriverID, "Unable to calculate capacity for node %s", node->MACStr);
          }
        }

        if(cap_S1_per_bands != NULL)
        {
          free(cap_S1_per_bands);
        }

        if(cap_S2_per_bands != NULL)
        {
          free(cap_S2_per_bands);
        }
      }
    }
  }

  if ((ret != VB_ENGINE_ERROR_NONE) && (node != NULL) && (driver != NULL))
  {
    VbLogPrintExt(VB_LOG_DEBUG, driver->vbDriverID, "Error %d calculating channel capacity for node %s", ret, node->MACStr);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineChannelCapacityCalculate(INT32U clusterId, BOOL *contCalc)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if(contCalc == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineDatamodelClusterXAllNodesLoop(VbChannelCapacityCalculateLoopCb, clusterId, contCalc);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbChannelCapacityAllBoostedFill(t_nodeType nodeType, t_nodeCdtaInfo *nodeCdtaInfo, float *capS1, float *capS2, INT8U maxNumBands)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if( (nodeCdtaInfo == NULL) || (capS1 == NULL) || (capS2 == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U band_idx;
    INT32U rate_idx;
    INT32U capacity;

    for(band_idx = 0; band_idx < maxNumBands; band_idx++)
    {
      capacity = (INT32U)(capS1[band_idx] + capS2[band_idx])/VB_ENGINE_1MBITS_PER_SEC;

      for(rate_idx = VB_ENGINE_QOS_RATE_10_90; rate_idx < VB_ENGINE_QOS_RATE_LAST; rate_idx++)
      {
        if(nodeType == VB_NODE_DOMAIN_MASTER)
        {
          nodeCdtaInfo->bandCapacities[band_idx][rate_idx].capacityAllBoosted = (capacity*((rate_idx*10)-VB_ENGINE_STXOP_LOSS_EFF))/100;
        }
        else
        {
          nodeCdtaInfo->bandCapacities[band_idx][rate_idx].capacityAllBoosted = (capacity*((100 - (rate_idx*10))-VB_ENGINE_STXOP_LOSS_EFF))/100;
        }
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbChannelCapacity1BoostedFill(t_nodeType nodeType, t_nodeCdtaInfo *nodeCdtaInfo, float *capS1, float *capS2, INT8U maxNumBands)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if( (nodeCdtaInfo == NULL) || (capS1 == NULL) || (capS2 == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U band_idx;
    INT32U rate_idx;
    INT32U capacity;

    for(band_idx = 1; band_idx < maxNumBands; band_idx++)
    {
      capacity = (INT32U)(capS1[band_idx] + capS2[band_idx])/VB_ENGINE_1MBITS_PER_SEC;

      for(rate_idx = VB_ENGINE_QOS_RATE_10_90; rate_idx<VB_ENGINE_QOS_RATE_LAST; rate_idx++)
      {
        if(nodeType == VB_NODE_DOMAIN_MASTER)
        {
          nodeCdtaInfo->bandCapacities[band_idx][rate_idx].capacity1Boosted = (capacity*((rate_idx*10)-VB_ENGINE_STXOP_LOSS_EFF))/100;
        }
        else
        {
          nodeCdtaInfo->bandCapacities[band_idx][rate_idx].capacity1Boosted = (capacity*((100 - (rate_idx*10))-VB_ENGINE_STXOP_LOSS_EFF))/100;
        }
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbChannelMaxCapacityGet( t_node *node, INT32U *maxCapacity, float *capS1, float *capS2, INT8U maxNumBands)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U i;
  float capacity_sum = 0;

  if( (node == NULL) || (maxCapacity == NULL) || (capS1 == NULL) || (capS2 == NULL) )
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    for(i = 0; i < maxNumBands; i++)
    {
      capacity_sum += (capS1[i] + capS2[i]);
    }

    *maxCapacity = (INT32U)(capacity_sum)/VB_ENGINE_1MBITS_PER_SEC;
  }


  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbClusterSNRThreadStopCb(t_VBCluster *cluster, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if(cluster == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if (cluster->snrComputationThreadRunning == TRUE)
    {
      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopping %s thread...", VB_ENGINE_COMPUTATION_THREAD_NAME);

      cluster->snrComputationThreadRunning = FALSE;

      VbThreadJoin(cluster->snrComputationThread, VB_ENGINE_COMPUTATION_THREAD_NAME);

      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopped %s thread!", VB_ENGINE_COMPUTATION_THREAD_NAME);
    }
  }

  return ret;
}


/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

t_VB_engineErrorCode VbCalculateSNRAndCapacityRun(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  t_VBCluster          *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if (cluster->snrComputationThreadRunning == TRUE)
    {
      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Previous Measure plan still running");
      ret = VB_ENGINE_ERROR_ALREADY_STARTED;
    }
  }

  VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Run SNRCalc on cluster %d", clusterId);

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Starting %s thread", VB_ENGINE_COMPUTATION_THREAD_NAME);



    cluster->snrComputationThreadRunning = TRUE;
    if (FALSE == VbThreadCreate(VB_ENGINE_COMPUTATION_THREAD_NAME, VbEngineSNRAndCapacityCompute, cluster, VB_ENGINE_COMPUTATION_THREAD_PRIORITY, &cluster->snrComputationThread))
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Can't create %s thread", VB_ENGINE_COMPUTATION_THREAD_NAME);

      cluster->snrComputationThreadRunning = FALSE;
      ret = VB_ENGINE_ERROR_EA_THREAD_CREATE;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbCalculateSNRAndCapacityStop(INT32U clusterId)
{
  t_VB_engineErrorCode ret;
  t_VBCluster          *cluster;

  ret = VbEngineClusterByIdGet(clusterId, &cluster);
  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if (cluster->snrComputationThreadRunning == TRUE)
    {
      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopping %s thread...", VB_ENGINE_COMPUTATION_THREAD_NAME);

      cluster->snrComputationThreadRunning = FALSE;

      VbThreadJoin(cluster->snrComputationThread, VB_ENGINE_COMPUTATION_THREAD_NAME);

      VbLogPrintExt(VB_LOG_INFO, VB_ENGINE_ALL_DRIVERS_STR, "Stopped %s thread!", VB_ENGINE_COMPUTATION_THREAD_NAME);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbCalculateAllSNRAndCapacityStop(void)
{
  t_VB_engineErrorCode ret;

  ret = VbEngineDatamodelClustersLoop(VbClusterSNRThreadStopCb, NULL);

  return ret;
}


/*******************************************************************/

t_VB_engineErrorCode VbEngineSNRProbeForceRequest(t_VBDriver *driver, INT8U *mac)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_nodesMacList       driver_mac_list;
  INT8U                *macs_list;
  INT8U                *ptr;
  INT8U                num_macs;


  if (driver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(ret == VB_ENGINE_ERROR_NONE)
  {
    if(mac != NULL)
    {
      ret = VbEngineProcessMeasureSnrProbesRequest(1, mac, driver);
    }
    else
    {
      // Get MACs for this driver
      ret = VbEngineDatamodelNodesMacGet(driver, &driver_mac_list, TRUE);
      if(ret == VB_ENGINE_ERROR_NONE)
      {
        num_macs = driver_mac_list.dmsMacs.numNodes+driver_mac_list.epsMacs.numNodes;
        macs_list = calloc(1, (num_macs*ETH_ALEN));
        if(macs_list != NULL)
        {
          ptr = macs_list;
          memcpy(ptr, driver_mac_list.dmsMacs.ptr, driver_mac_list.dmsMacs.numNodes*ETH_ALEN);
          ptr += driver_mac_list.dmsMacs.numNodes*ETH_ALEN;
          memcpy(ptr, driver_mac_list.epsMacs.ptr, driver_mac_list.epsMacs.numNodes*ETH_ALEN);

          ret = VbEngineProcessMeasureSnrProbesRequest(num_macs, macs_list, driver);

          free(macs_list);
        }
        else
        {
          ret = VB_ENGINE_ERROR_MALLOC;
        }

        // Always relase memory
        VbEngineDatamodelMacListRelease(&driver_mac_list);
      }
      else
      {
        ret = VB_ENGINE_ERROR_DATA_MODEL;
      }
    }
  }

  return ret;
}




/*******************************************************************/

/**
 * @}
 **/
