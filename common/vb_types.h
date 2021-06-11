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
 * @file vb_types.h
 * @brief VectorBoost common types
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016/11/30
 *
 **/

#ifndef VB_TYPES_H_
#define VB_TYPES_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <linux/if_ether.h>

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define MAC_CYCLE_DURATION          (40)  //ms
#define VB_FW_VERSION_LENGTH        (10)
#define VB_PARSE_MAX_PATH_LEN       (100)
#define VB_PSD_NUM_BANDS            (10)
#define VB_QUEUE_NAME_LEN           (50)
#define VB_ENGINE_ID_MAX_SIZE       (21) // 20 digits + null byte
#define VB_ALIGN_GHN_MAX_RELAYS     (4)
#define VB_ALIGN_GHN_MAX_TX_NODES   (VB_ALIGN_GHN_MAX_RELAYS + 1)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef enum
{
  VB_NODE_DOMAIN_MASTER = 0,
  VB_NODE_END_POINT,
  VB_NODE_TYPE_LAST,
} t_nodeType;

typedef enum
{
  VB_MEAS_ERRCODE_VALID = 0,
  VB_MEAS_ERRCODE_INVALID,
} t_measErrCode;

typedef enum
{
  VB_MEAS_TYPE_BGN = 0,
  VB_MEAS_TYPE_CFR,
  VB_MEAS_TYPE_SNRFULLXTALK,
  VB_MEAS_TYPE_SNRLOWXTALK,
  VB_MEAS_TYPE_SNRPROBE,
  VB_MEAS_TYPE_PSD,
  VB_MEAS_TYPE_LAST,
} t_measType;

typedef enum {
  VB_DEV_PRESENT = 0,
  VB_DEV_NEW,
  VB_DEV_LAST,
} t_vb_DevState;

typedef enum
{
  VB_ALIGN_MODE_COMMON_CLOCK = 0,
  VB_ALIGN_MODE_GHN,
  VB_ALIGN_MODE_PTP,
  VB_ALIGN_MODE_LAST,
} t_VBAlignmentMode;

typedef struct s_psdBandLevel
{
  INT16U stopCarrier;
  INT8U  attLevel;
} t_psdBandLevel;

typedef struct s_psdShape
{
  INT16U          numPSDBands;
  t_psdBandLevel  psdBandLevel[VB_PSD_NUM_BANDS];
  BOOL            sendUpdate;
} t_psdShape;

typedef struct s_PSDStep
{
  INT16U startCarrier;
  INT16S PSDLevel;
} t_PSDStep;

typedef struct s_MSB
{
  INT16U startCarrier;
  INT16U stopCarrier;
} t_MSB;

typedef struct s_PSDStepsList
{
  INT16U     NumPSDs;
  t_PSDStep *PSD;
} t_PSDStepsList;

typedef struct s_MSBsList
{
  INT16U NumMSBs;
  t_MSB* MSB;
} t_MSBsList;

typedef struct s_PSD
{
  t_PSDStepsList  PSDStepsList;
  t_MSBsList      MSBsList;
} t_PSD;

typedef struct s_processMeasure
{
  INT8U         planID;
  t_measErrCode errorCode;
  INT16U        firstCarrier;
  INT8U         spacing;
  INT8U         flags;
  INT16U        numMeasures;
  INT8U        *measures;
  INT8U        *measuresRx1;
  INT8U        *measuresRx2;
  INT8S         rxg1Compensation;
  INT8S         rxg2Compensation;
  BOOLEAN       mimoInd;
  BOOLEAN       mimoMeas;
  t_measType    type;
  INT32U        freqCutProfile;
  INT32U        carrierGridIdxCutProfile;
} t_processMeasure;

typedef struct s_crossMeasure
{
  INT8U             MAC[ETH_ALEN];
  BOOL              ownCFR;
  t_processMeasure  measure;
} t_crossMeasure;

typedef struct s_crossMeasureList
{
  INT16U          numCrossMeasures;
  t_crossMeasure *crossMeasureArray;
} t_crossMeasureList;

typedef struct s_nodeMasures
{
  t_processMeasure    BGNMeasure;
  t_crossMeasureList  CFRMeasureList;
  t_processMeasure    snrFullXtalk;
  t_processMeasure    SNRProbesMeasure;
  t_processMeasure    snrLowXtalk;
  t_PSD               Psd;
} t_nodeMeasures;

typedef struct s_additionalInfo1
{
  INT8U  fwVersion[VB_FW_VERSION_LENGTH];
  INT8U  qosRate;
  INT16U maxLengthTxop;
  INT16U extSeed;
} t_additionalInfo1;

typedef struct s_syncDetInfo
{
  BOOLEAN     allowed;
  INT8U       detDid;
  INT32U      hitCount;
  INT32S      reliability;
  INT32U      adcOutRms;
} t_syncDetInfo;

typedef struct s_alignInfo
{
  INT32U         macClock;
  INT16U         seqNum;
  t_syncDetInfo  syncDetsInfo[VB_ALIGN_GHN_MAX_TX_NODES];
} t_alignInfo;

typedef struct s_alignChange
{
  BOOLEAN updated;
  BOOLEAN clockEdge;
  INT16U  seqNumOffset;
} t_alignChange;

typedef struct s_measCollect
{
  BOOLEAN status;
} t_measCollect;

typedef CHAR t_vbQueueName[VB_QUEUE_NAME_LEN];

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Gets the node type name
 * @param[in] nodeType Node type
 * @return Node type name
 **/
static inline CHAR * VbNodeTypeToStr(t_nodeType nodeType)
{
  static CHAR *nodeTypeStr[VB_NODE_TYPE_LAST] = {"DM", "EP"};
  CHAR *ret = "--";

  if (nodeType < VB_NODE_TYPE_LAST)
  {
    ret = nodeTypeStr[nodeType];
  }

  return ret;
}

/**
 * @brief Gets the node type name
 * @param[in] nodeType Node type
 * @return Node type name
 **/
static inline CHAR * VbDevStateToStr(t_vb_DevState state)
{
  static CHAR *stateStr[VB_DEV_LAST] = {"PRESENT", "NEW"};
  CHAR *ret = "--";

  if (state < VB_DEV_LAST)
  {
    ret = stateStr[state];
  }

  return ret;
}

#endif /* VB_TYPES_H_ */

/**
 * @}
 **/


