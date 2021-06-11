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
 * @file vb_engine_l2rPSD_calculation.h
 * @brief PSD Left to right algorithm interface
 *
 * @internal
 *
 * @author
 * @date 22/12/2014
 *
 **/

#ifndef VB_L2RPSD_CALCULATION_H_
#define VB_L2RPSD_CALCULATION_H_

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


/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */
typedef struct s_psdl2rArgs
{
  INT32U qos;
  t_psdBandAllocation *psdBandsAllocation;
} t_psdl2rArgs;
/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Calculates the last low band carrier index (relative to first valid carrier, not the absolute one)
 * @param[in]  firstValidCarrier first valid carrier index
 * @param[out] lowBandRelativeIdx Index (relative to the first valid carrier) of the end of the low band
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineLastRelativeLowBandCarrierIdxGet(INT16U firstValidCarrier, INT16U* lowBandRelativeIdx);

/**
 * @brief Detects discrepancies between capacity calculated by Engine and capacity reported by the node.
 * @param[in] trafficReport Traffic report info
 * @param[in] nodeChannelSettings Node channel settings
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineSnrCalculatedCheck(t_trafficReport* trafficReport, t_nodeChannelSettings* nodeChannelSettings,
    t_VBDriver *thisDriver);

/**
 * @brief Updates low band capacity with value conveyed in last traffic report received from a node.
 * @param[in] thisDriver Pointer to driver
 * @param[in] nodeBoostInfo Pointer to node parameters to update
 * @param[in] lowBandCapacity Low band capacity value conveyed in traffic report message
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineLowBandCapacityUpdate(t_VBDriver *thisDriver, t_boostInfo *nodeBoostInfo, INT16U lowBandCapacity);

/**
 * @brief Allocate PSD bands to nodes based on calculated info
 * @param[in] thisDriver Pointer to driver
 * @param[in] domain Pointer to domain
 * @param[in] domain Pointer to node
 * @param[in] args bands information to be applied to this node
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineLeftToRightPSDShapeRun(t_VBDriver *driver, t_domain *domain, t_node *node, void *args);

#endif /* VB_L2RPSD_CALCULATION_H_ */
/**
 * @}
**/
