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
 * @file vb_engine_conf.h
 * @brief Engine configuration interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-03-22
 *
 **/

#ifndef VB_ENGINE_CONF_H_
#define VB_ENGINE_CONF_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_console.h"
#include "vb_log.h"
#include "vb_engine_datamodel.h"
#include "vb_engine_clock.h"
#include "vb_measure_utils.h"
#include "vb_engine_cdta.h"
#include "vb_engine_socket_alive.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define VB_ENGINE_VERSION_FILE           "vbEngineVersion"
#define VB_DRIVER_VERSION_FILE           "vbDriverVersion"
#define VB_ENGINE_STATE_FILE             "vbEngineStatus"
#define VB_DRIVER_STATE_FILE             "vbDriverStatus"
#define VB_ENGINE_MEASURES_FOLDER        "measures"

#define VB_ENGINE_HIGH_GRANULARITY_PSD_MNGT    (5)
#define VB_ENGINE_MEDIUM_GRANULARITY_PSD_MNGT  (3)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef enum
{
  VB_TX_MODE_100_MHZ = 0,
  VB_TX_MODE_200_MHZ,
  VB_TX_MODE_TYPE_LAST
} t_vbTxBandPlanMode;

typedef enum
{
  VB_BOOST_THR_TYPE_DEC_BOOST = 0,
  VB_BOOST_THR_TYPE_INC_BOOST,
  VB_BOOST_THR_TYPE_LAST
} t_vbBoostThrType;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Parses command line args and .ini configuration file
 * @param[in] argc Number of arguments
 * @param[in] argv Command line arguments
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfParse(int argc, char **argv);

/**
 * @brief Dumps Engine configuration
 * @param[in] writeFun Function to call to dump info
 **/
void VbEngineConfDump(t_writeFun writeFun);

/**
 * @brief Configures boost thresholds
 * @param[in] boostThr Array with new boost thresholds
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfBoostThrSet(INT32U *boostThr);

/**
 * @brief Gets the Connection mode
 * @return @ref TRUE - Server mode, FALSE - Client mode
 **/
BOOL VbEngineConfServerConnModeGet(void);

/**
 * @brief Gets boost threshold
 * @param[in] type Threshold type
 * @return Boost threshold value
 **/
INT32U VbEngineConfBoostThrGet(t_vbBoostThrType type);

/**
 * @brief Gets configured output path
 * @return Output path
 **/
CHAR *VbEngineConfOutputPathGet(void);

/**
 * @brief Gets configured engine Id
 * @return Engine Id
 **/
CHAR *VbEngineConfEngineIdGet(void);

/**
 * @brief Gets configured verbose level
 * @return Verbose level
 **/
t_vbLogLevel VbEngineConfVerboseLevelGet(void);

/**
 * @brief Gets configured console port
 * @return Console port
 **/
INT16U VbEngineConfConsolePortGet(void);

/**
 * @brief Shows if traffic metrics are enabled
 * @return Traffic metrics status
 **/
BOOL VbEngineConfTrafficMetricsEnabledGet(void);

/**
 * @brief Shows if metrics shall be saved to disk
 * @return TRUE: save metrics to disk; FALSE: otherwise
 **/
BOOL VbEngineConfSaveMetricsEnabledGet(void);

/**
 * @brief Gets the maximum metrics log size (in KB)
 * @return Maximum metrics log size (in KB)
 **/
INT32U VbEngineConfMaxMetricsLogSizeGet(void);

/**
 * @brief Returns the value of vdslCoexistence
 * @return TRUE or FALSE
 **/
BOOLEAN VbEngineConfVdslCoexGet(void);

/**
 * @brief Returns the value of SaveMeasures
 * @return TRUE or FALSE
 **/
BOOLEAN VbEngineConfSaveMeasuresGet(void);

/**
 * @brief Returns the value of VbInUpstream
 * @return TRUE or FALSE
 **/
BOOLEAN VbEngineConfVbInUpstreamGet(void);

/**
 * @brief Returns the value of CDTA
 * @return TRUE or FALSE
 **/
BOOLEAN VbEngineConfCDTAEnableGet(void);

/**
 * @brief Returns the value of CDTA
 * @return TRUE or FALSE
 **/
t_psdBandAllocation* VbEngineConfPSDBandAllocationGet(void);

/**
 * @brief Returns the value of CDTA
 * @param[in] txMode indicates if th transmission mode is in 100 MHz or 200 MHz
 * @return TRUE or FALSE
 **/
INT16U VbEngineConfNumPSDBandAllocationGet(t_vbTxBandPlanMode txMode);

/**
 * @brief Gets measure plan configuration
 * @param[in] measConfData Measure plan configuration
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfMeasurePlanGet(t_measconfdata **measConfData);

/**
 * @brief Gets socket alive configuration
 * @param[in] socket alive configuration
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfSocketAliveGet(t_socketAlive **socketAliveConf);

/**
 * @brief Gets cdta configuration
 * @param[in] cdta configuration
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfCDTADataGet(t_cdtaConf **cdtaConf);

/**
 * @brief Gets the SLA (Service Level Agreement) profile of mac passed as parameter
 * @param[in] mac of node
 * @param[out] sla SLA profile as set in the .ini file
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfSLAProfileGet(INT8U *mac, INT32U *sla);

/**
 * @brief Gets the profile of mac passed as parameter
 * @param[in] mac of node
 * @param[out] sla SLA profile as set in the .ini file
 * @param[out] slaWeight profile as set in the .ini file
 * @param[out] userWeight profile as set in the .ini file
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfProfileGet(INT8U *mac, INT32U *sla, INT32U* slaWeight, INT32U* userWeight);

/**
 * @brief Gets alignment mode
 * @param[out] alignment mode
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfAlignmentModeGet(t_VBAlignmentMode *alignmentMode);

/**
 * @brief Gets boosting algorithm run period
 * @return Boosting algorithm run period (in ms)
 **/
INT16U VbEngineConfBoostAlgPeriodGet(void);

/**
 * @brief Return if automatic seed feature is enable or not
 * @return Automatic seed status
 **/
BOOLEAN VbEngineConfSeedAutomaticEnableGet(void);

/**
 * @brief Gets minimum seed index to be used by automatic seed feature
 * @return Minimum seed index
 **/
INT16U VbEngineConfSeedMinIndexGet(void);

/**
 * @brief Gets maximum seed index to be used by automatic seed feature
 * @return Maximum seed index
 **/
INT16U VbEngineConfSeedMaxIndexGet(void);

/**
 * @brief Gets the number of MACs in the seed excluded list
 * @return Number of MAC that will not received an automatic seed
 **/
INT16U VbEngineConfSeedExcludedMacListSizeGet(void);

/**
 * @brief Gets the list of MACs excluded from automatic seed functionality
 * @return Pointer to the beginning of the MAC excluded list
 **/
INT8U *VbEngineConfSeedExcludedMacListGet(void);

/**
 * @brief Release resources reserved by this component
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfReleaseResources(void);

/**
 * @brief Gets minimum power to consider sync visibility
 * @return Minimum power (in HW format)
 **/
INT32U VbEngineConfAlignMinPowGet(void);

/**
 * @brief Gets minimum power to maintain sync visibility with
 * a node that was previously considered as visible
 * @return Minimum power (in HW format)
 **/
INT32U VbEngineConfAlignMinPowHystGet(void);

/**
 * @brief Shows if alignment metrics are enabled
 * @return TRUE: metrics are enabled; FALSE: otherwise
 **/
BOOLEAN VbEngineConfAlignMetricsEnabled(void);

/**
 * @brief Gets reliability threshold to consider sync visibility
 * @return Reliability threshold
 **/
INT32U VbEngineConfAlignRelThrGet(void);

/**
 * @brief Shows if reference priority list is enabled
 * @return TRUE: priority list is not empty; FALSE: otherwise
 **/
BOOLEAN VbEngineConfAlignPrioRefEnabled(void);

/**
 * @brief Number of elements in reference priority list
 * @return Number of elements
 **/
INT32U VbEngineConfAlignPrioRefListSizeGet(void);

/**
 * @brief Gets one item from the list of nodes with
 * high priority to be configured as sync reference.
 * @param[in] prioRefIdx   Index to get
 * @param[out] prioRefMac  MAC address with high priority to be reference node
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineConfAlignPrioRefItemGet(INT32U prioRefIdx, INT8U **prioRefMac);

/**
 * @brief Shows if socket alive mechanism is enabled
 * @return TRUE: enabled, FALSE: otherwise
 **/
BOOLEAN VbEngineConfSocketAliveEnableGet(void);

/**
 * @brief Return period of socket alive msg from engine to driver
 * @return period in ms
 **/
INT32U VbEngineConfSocketAlivePeriodGet(void);

/**
 * @brief Return number of consecutive message lost to close socket
 * @return number of message lost
 **/
INT32U VbEngineConfSocketAliveNLostThrGet(void);

/**
 * @brief Gets number of lines of persistent log feature
 * @return Number of lines
 **/
INT32U VbEngineConfPeristentLogNumLinesGet(void);

/**
 * @brief Gets verbose level of persistent log feature
 * @return Verbose level
 **/
t_vbLogLevel VbEngineConfPeristentLogVerboseLevelGet(void);

/**
 * @brief Shows if persistent log is circular
 * @return TRUE if log is circular; FALSE: otherwise
 **/
BOOLEAN VbEngineConfPersistentLogIsCircular(void);

/**
 * @brief Check if a given MAC is present in blacklist of a target node
 * @param[in] targetMac MAC address of the receiver node
 * @param[in] macToCheck MAC address of the transmitter node to check
 * @return TRUE: macToCheck is allowed for a given target node; FALSE: otherwise
 **/
BOOLEAN VbEngineConfAlignBlackListIsMacAllowed(INT8U *targetMac, INT8U *macToCheck);

/**
 * @brief Shows if blacklist is enabled
 * @return TRUE: blacklist is enabled; FALSE: otherwise
 **/
BOOLEAN VbEngineConfAlignBlackListIsEnabled(void);

#endif /* VB_ENGINE_CONF_H_ */

