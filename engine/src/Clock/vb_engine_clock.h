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
 * @addtogroup vector_boost_engine
 * @{
 **/

/**
 * @file vb_engine_clock.h
 * @brief VB Drivers clock tracking
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-05-08
 *
 **/

#ifndef VB_ENGINE_CLOCK_H_
#define VB_ENGINE_CLOCK_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_engine_datamodel.h"

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

typedef struct s_timeChangeInfo
{
  struct timespec applyTs;
  INT16U          applySeqNum;
} t_timeChangeInfo;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Request system clock and sequence number (EAClock.req frame) to given driver
 * @param[in] thisDriver Driver to launch the request to
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClockRequest( t_VBDriver *thisDriver );

/**
 * @brief Process the EAClock.rsp received frame to extract system clock and sequence number from given driver
 * @param[in] payload Pointer to frame payload
 * @param[in] length Length in bytes of message
 * @param[in] thisDriver Driver related to received frame
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClockRspProcess(INT8U* payload, INT32U length, t_VBDriver *thisDriver);

/**
 * @brief Loops through all drivers and extracts a timestamp and sequence number located in the near future
 * @param[out] futureTs Timestamp located in the future for all drivers
 * @param[out] futureSeqNum Sequence number located in the future for all devices
 * @param[out] futureTsOwnClock It is the same timestamp value than "futureTs" but referred to engine own system clock
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClockFutureTSGet(struct timespec *futureTs, INT32U clusterId, INT16U *futureSeqNum, struct timespec *futureTsOwnClock);

/**
 * @brief Launch a EAClock.req frame to all drivers to update system clock and sequence number
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClockAllDriversUpdate(void);

/**
 * @brief Check NTP deviation of all drivers
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClockAllDriversNTPDeviationCheck(void);

/**
 * @brief Launch a EAClock.req frame to all drivers to update system clock and sequence number
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClockClusterUpdate(INT32U clusterId);

/**
 * @brief Initializes the clock monitor task
 **/
void VbEngineClockMonitorInit(void);

/**
 * @brief Stops the clock monitor task
 **/
void VbEngineClockMonitorStop(void);

/**
 * @brief Starts the clock monitor task
 **/
BOOLEAN VbEngineClockMonitorRun(void);

/**
 * @brief Calculates clock deviations between all drivers
 * @param[out] maxDev Maximum deviation found between drivers
 * @param[out] minDev Minimum deviation found between drivers
 * @param[out] maxRtt Maximum Round-trip time
 * @return @ref t_VB_engineErrorCode
 **/
t_VB_engineErrorCode VbEngineClockDevCalc(INT64S *maxDev, INT64S *minDev, INT64S *maxRtt);

#endif /* VB_ENGINE_CLOCK_H_ */

/**
 * @}
**/
