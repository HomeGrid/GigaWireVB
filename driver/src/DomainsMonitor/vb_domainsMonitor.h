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
 * @file vb_domainsMonitor.h
 * @brief Domains monitor interface
 *
 * @internal
 *
 * @author
 * @date 27/04/2015
 *
 **/

#ifndef VB_DOMAINS_MONITOR_H_
#define VB_DOMAINS_MONITOR_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public constantsdeviceMAC
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
 * @brief Stops the task which executes the domain monitor process
**/
void VbDomainsMonitorStop();

/**
 * @brief Executes the domains monitor process
**/
void VbDomainsMonitorRun(void);

/**
 * @brief This function initiates the domains monitor feature to default values
**/
void VbDomainsMonitorInit(void);

/**
 * @brief Gets Domains Monitor thread state
 * @return TRUE: Domains Monitor thread is running; FALSE: otherwise
 **/
BOOLEAN VbDomainsMonitorStateGet(void);

/**
 * @brief Shows if domains monitor has already performed its first discovery attempt.
 * @return TRUE: first attempt pending; FALSE: first attempt was done.
 **/
BOOLEAN VbDomainsMonitorFirstAttemptGet(void);

/**
 * @brief Forces a new network discovery
 **/
void VbDomainsMonitorSignal(void);

#endif /* VB_DOMAINS_MONITOR_H_ */

/**
 * @}
**/
