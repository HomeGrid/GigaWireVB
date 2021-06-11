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
 * @file vb_LCMP_paramId.h
 * @brief Communications over LCMP implemented to vector boost library header
 *
 * @internal
 *
 * @author
 * @date 07/01/2015
 *
 **/

#ifndef VB_LCMP_PARAMID_H_
#define VB_LCMP_PARAMID_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_util.h"

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

#define PARAMETER_TYPE_SIZE            (1)
#define PARAMETER_TYPE_OFFSET          (0)
#define PARAMETER_VALUE_OFFSET         (PARAMETER_TYPE_OFFSET + PARAMETER_TYPE_SIZE)

#define LCMP_FIRST_OPCODE              (LCMP_READ_REQ)
#define LCMP_LAST_OPCODE               (LCMP_RSP)
#define LCMP_NUM_OPCODES               (LCMP_LAST_OPCODE - LCMP_FIRST_OPCODE + 1)
#define LCMP_INDEX_OPCODES(A)          ((A) - LCMP_FIRST_OPCODE)

#define LCMP_NOTIFY_FIRST_ID           (VB_DMREFCYCSTART)
#define LCMP_NOTIFY_LAST_ID            (VB_ALIGN_SYNCLOST)
#define LCMP_NOTIFY_NUM_IDS            (LCMP_NOTIFY_LAST_ID - LCMP_NOTIFY_FIRST_ID + 1)
#define LCMP_NOTIFY_ID_TO_INDEX(A)     ((A) - LCMP_NOTIFY_FIRST_ID)

#define LCMP_CONTROL_FIRST_ID          (VB_DMREFSET)
#define LCMP_CONTROL_LAST_ID           (VB_ENGINE_CONF)
#define LCMP_CONTROL_NUM_IDS           (LCMP_CONTROL_LAST_ID - LCMP_CONTROL_FIRST_ID + 1)
#define LCMP_CONTROL_ID_TO_INDEX(A)    ((A) - LCMP_CONTROL_FIRST_ID)

#define LCMP_PARAMETER_FIRST_ID        (VB_DOMAINMACS)
#define LCMP_PARAMETER_LAST_ID         (VB_CDTA)
#define LCMP_PARAMETER_NUM_IDS         (LCMP_PARAMETER_LAST_ID - LCMP_PARAMETER_FIRST_ID + 1)
#define LCMP_PARAMETER_ID_TO_INDEX(A)  ((A) - LCMP_PARAMETER_FIRST_ID)

#define LCMP_MAX_IDS_IN_GROUP          (MAX(MAX(LCMP_NOTIFY_NUM_IDS, LCMP_CONTROL_NUM_IDS), LCMP_PARAMETER_NUM_IDS))

typedef enum {
  VB_DMREFCYCSTART          = 0x11,
  VB_CYCSTART               = 0x12,
  VB_CYCQUERY               = 0x13,
  VB_CYCQUERYNOTIF          = 0x14,
  VB_MEASURE_BGN_IND        = 0x15,
  VB_MEASURE_CFR_AMP_IND    = 0x16,
  VB_MEASURE_CFR_PHASE_IND  = 0x17,
  VB_MEASURE_SNR_IND        = 0x18,
  VB_INGRESS_TRAFFIC_IND    = 0x19,
  VB_KEEP_ALIVE_IND         = 0x1A,
  VB_CYCQUERY_EXT           = 0x1B,
  VB_CYCQUERYNOTIF_EXT      = 0x1C,
  VB_ALIGN_SYNCLOST         = 0x1D,
} t_vb_HGF_NOTIF;

typedef enum {
  VB_DMREFSET               = 0x10,
  VB_MEASPLAN_CANCEL        = 0x11,
  VB_MEASURE_BGN_C          = 0x12,
  VB_MEASURE_CFR_AMP_C      = 0x13,
  VB_MEASURE_CFR_PHASE_C    = 0x14,
  VB_MEASURE_SNR_C          = 0x15,
  VB_INGRESS_TRAFFIC_MON    = 0x16,
  VB_CYCCHANGE              = 0x17,
  VB_ENGINE_CONF            = 0x18,
  VB_CLUSTER_STOP           = 0x19,
} t_vb_HGF_CONTROL;

typedef enum {
  VB_DOMAINMACS             = 0x10,
  VB_MEASPLAN               = 0x11,
  VB_THROUGHPUT             = 0x12,
  VB_SNR_ESTIMATED          = 0x13,
  VB_PSD                    = 0x14,
  VB_PSD_SHAPE              = 0x15,
  VB_ADDINFO_1              = 0x16,
  VB_CDTA                   = 0x17,
  VB_MACSEQNUM              = 0x18
} t_vb_HGF_PARAMETER;

typedef enum {
  HGF_PARAMETER               = 0x00,
  HGF_WRITE_PARAMETER_CONFIRM = 0x01,
  HGF_READ_PARAMETER          = 0x02,
  HGF_CONTROL                 = 0x03,
  HGF_CONTROL_CONFIRM         = 0x05,
  HGF_NOTIFY                  = 0x06,
  HGF_NOTIFY_CONFIRM          = 0x08,
  HGF_INFO                    = 0x09
} t_HGF_TLV;

typedef enum {
  LCMP_READ_REQ   = 0x160,
  LCMP_READ_CNF   = 0x161,
  LCMP_WRITE_REQ  = 0x162,
  LCMP_WRITE_CNF  = 0x163,
  LCMP_CTRL_REQ   = 0x164,
  LCMP_CTRL_CNF   = 0x165,
  LCMP_NOTIFY_IND = 0x166,
  LCMP_NOTIFY_RSP = 0x167,
  LCMP_IND        = 0x168,
  LCMP_RSP        = 0x169,
} t_LCMP_OPCODE;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

#endif /* VB_LCMP_PARAMID_H_ */

/**
 * @}
**/
