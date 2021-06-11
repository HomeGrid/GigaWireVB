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
 * @file vb_driver_conf.h
 * @brief Vector boost driver configuration functionality
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-11
 *
 **/

#ifndef _VB_DRIVER_CONF_H_
#define _VB_DRIVER_CONF_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_DataModel.h"
#include "vb_log.h"

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

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Parses command line args and .ini configuration file
 * @param[in] argc Number of arguments
 * @param[in] argv Command line arguments
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbDriverConfParse(int argc, char **argv);

/**
 * @brief Whether the program runs in server mode
 * @return TRUE - Server mode, FALSE - Client mode
 **/
BOOL VbDriverConfServerModeGet(void);

/**
 * @brief Gets the remote IP (server IP) to use in client mode
 * @return in_addr server mode
 **/
CHAR *VbDriverConfRemoteIPGet(void);

/**
 * @brief Gets the family address type to use in client mode
 * @return Family address type (IPv4 or IPv6)
 **/
INT16U VbDriverConfFamilyGet(void);

/**
 * @brief Gets driver Id
 * @return Driver id to use
 **/
CHAR *VbDriverConfDriverIdGet(void);

/**
 * @brief Gets LCMP interface name to use
 * @return LCMP interface name to use
 **/
CHAR *VbDriverConfLcmpIfGet(void);

/**
 * @brief Gets external agent (VB Engine) interface name to use
 * @return External agent (VB Engine) interface name to use
 **/
CHAR *VbDriverConfEaIfGet(void);

/**
 * @brief Gets configured output path
 * @return Ouput path
 **/
CHAR *VbDriverConfOutputPathGet(void);

/**
 * @brief Gets configured verbose level
 * @return Verbose level
 **/
t_vbLogLevel VbDriverConfVerboseLevelGet(void);

/**
 * @brief Gets external agent (VB Engine) port to use
 * @return External agent (VB Engine) port to use
 **/
INT16U VbDriverConfEaPortGet(void);

/**
 * @brief Gets configured console port
 * @return Console port
 **/
INT16U VbDriverConfConsolePortGet(void);

/**
 * @brief Gets configured traffic reports period
 * @return Traffic reports period (in ms)
 **/
INT32U VbDriverConfTrafficReportPeriodGet(void);

/**
 * @brief Gets configured traffic report measure window
 * @return Traffic report measure window (in ms)
 **/
INT32U VbDriverConfTrafficReportMeasWinGet(void);

/**
 * @brief Gets configured traffic report threshold
 * @return Traffic report threshold (in Mbps)
 **/
INT32U VbDriverConfTrafficReportThresholdGet(void);

/**
 * @brief Gets configured thread creation interval for measure collect process
 * @return Time interval between thread creation
 **/
INT32U VbDriverConfMeasCollectThreadIntGet(void);

/**
 * @brief Gets default timeout for LCMP requests
 * @return Default timeout for LCMP requests (in ms)
 **/
INT32U VbDriverConfLcmpDefaultTimeoutGet(void);

/**
 * @brief Gets default N Attempt for LCMP requests
 * @return Default attempt for LCMP requests
 **/
INT32U VbDriverConfLcmpDefaultNAttemptGet(void);

/**
 * @brief Gets counfigured align method
 * @return Align method
 **/
BOOLEAN VbDriverConfAutoAlignGet(void);

/**
 * @brief Dumps Driver configuration
 * @param[in] writeFun Function to call to dump info
 **/
void VbDriverConfDump(t_writeFun writeFun);

/**
 * @brief Gets number of lines of persistent log feature
 * @return Number of lines
 **/
INT32U VbDriverConfPeristentLogNumLinesGet(void);

/**
 * @brief Gets verbose level of persistent log feature
 * @return Verbose level
 **/
t_vbLogLevel VbDriverConfPeristentLogVerboseLevelGet(void);

/**
 * @brief Shows if persistent log is circular
 * @return TRUE if log is circular; FALSE: otherwise
 **/
BOOLEAN VbDriverConfPersistentLogIsCircular(void);

#endif /* _VB_DRIVER_CONF_H_ */

/**
 * @}
 **/

