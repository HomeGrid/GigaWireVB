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
 * @file vb_measure_utils.c
 * @brief Measure common utilities
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016/12/01
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"
#include "vb_types.h"
#include "vb_console.h"
#include "vb_mac_utils.h"

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

BOOLEAN VbMeasureIsValid(INT8U planId, t_processMeasure *measure)
{
  return ((measure != NULL) && (measure->errorCode == VB_MEAS_ERRCODE_VALID) && (measure->planID == planId));
}

/*******************************************************************/

void VbMeasureDump(const INT8U *mac, t_nodeType nodeType, CHAR *driverId, t_measType measType, t_processMeasure *measure, t_crossMeasure *crossMeasure, t_PSD *psd, INT8U planId, t_writeFun writeFun)
{
  static const CHAR *meas_type_names[VB_MEAS_TYPE_LAST] = {"BGN", "CFR", "SNRFULLXTALK", "SNRLOWXTALK", "SNRPROBE", "PSD"};
  const CHAR        *meas_type_str = NULL;
  CHAR               node_mac_str[MAC_STR_LEN];
  CHAR               mac_str[MAC_STR_LEN];
  BOOLEAN            valid = FALSE;

  if ((writeFun != NULL) && (mac != NULL) && (measType < VB_MEAS_TYPE_LAST))
  {
    // Get measure type
    meas_type_str = meas_type_names[measType];

    // Get MAC str
    MACAddrMem2str(node_mac_str, mac);

    if (nodeType == VB_NODE_DOMAIN_MASTER)
    {
      writeFun("| %s    ", node_mac_str);
    }
    else
    {
      writeFun("|    %s ", node_mac_str);
    }

    // Get node type name
    writeFun("| %4s ", VbNodeTypeToStr(nodeType));

    // Get node type name
    writeFun("| %20s ", driverId);

    // Show measure type
    writeFun("| %12s ", meas_type_str);

    // Show measured MAC (if any)
    if ((measure != NULL) && (measType == VB_MEAS_TYPE_CFR) && (crossMeasure != NULL))
    {
      MACAddrMem2str(mac_str, crossMeasure->MAC);
      writeFun("| %18s ", mac_str);
      writeFun("| %6s ", crossMeasure->ownCFR?"YES":"NO");
    }
    else
    {
      writeFun("| %18s ", "-");
      writeFun("| %6s ", "-");
    }

    // Expected planId
    writeFun("| %9u ", planId);

    // Calc measure status
    if (measType != VB_MEAS_TYPE_PSD)
    {
      valid = VbMeasureIsValid(planId, measure);
    }
    else
    {
      valid = ((psd != NULL) && (psd->MSBsList.MSB != NULL) && (psd->PSDStepsList.PSD != NULL));
    }

    writeFun("| %5s ", valid?"YES":"NO");
    if (measure != NULL)
    {
      writeFun("| %6u |", measure->planID);
    }
    else
    {
      writeFun("| %6s |", "-");
    }

    writeFun("\n");
  }
}

/*******************************************************************/

/**
 * @}
 **/


