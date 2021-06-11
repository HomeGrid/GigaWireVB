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
 * @file vb_LCMP_com.h
 * @brief Communications over LCMP implemented to vector boost library header
 *
 * @internal
 *
 * @author
 * @date 07/01/2015
 *
 **/

#ifndef VB_LCMP_COM_H_
#define VB_LCMP_COM_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_DataModel.h"
#include "vb_util.h"
#include "vb_LCMP_paramId.h"

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

typedef enum {
  HGF_LCMP_ERROR_NONE = 0,
  HGF_LCMP_ERROR_SOCKET_NOT_OPENED = -1,
  HGF_LCMP_ERROR_ETH_IF = -2,
  HGF_LCMP_ERROR_SENDTO = -3,
  HGF_LCMP_ERROR_TIMEOUT = -4,
  HGF_LCMP_ERROR_DROP_PACKET = -5,
  HGF_LCMP_ERROR_RECVFROM = -6,
  HGF_LCMP_ERROR_BIND = -7,
  HGF_LCMP_ERROR_ROOM_BOOT = -8,
  HGF_LCMP_ERROR_DISCOVER = -9,
  HGF_LCMP_ERROR_NULL_MAC = -10,
  HGF_LCMP_ERROR_PTHREAD_CREATE = -11,
  HGF_LCMP_ERROR_PTHREAD_JOIN = -12,
  HGF_LCMP_ERROR_BAD_ARGS = -13,
  HGF_LCMP_ERROR_MALLOC = -14,
  HGF_LCMP_ERROR_MAX_IF = -15,
  HGF_LCMP_ERROR_ADDITIONAL_RESULT = -16,
  HGF_LCMP_ERROR_TOO_MANY_DEVICES = -17,
  HGF_LCMP_ERROR_PARAMETER_TYPE = -18,
  HGF_LCMP_ERROR_NO_DATA = -19,
  HGF_LCMP_ERROR_PARAM_ERROR = -20,
  HGF_LCMP_ERROR_CALLBACK_INSTALATION_ERROR = -21,
  HGF_LCMP_ERROR_CALLBACK_UNINSTALATION_ERROR = -22,
  HGF_LCMP_ERROR_NO_RESPONSE = -23,
  HGF_LCMP_ERROR_CNF_INCOMPLETE = -24,
  HGF_LCMP_ERROR_CNF_INVALID = -25,
  HGF_LCMP_ERROR_REQ_NOT_ALL_DEVICES = -26,
  HGF_LCMP_ERROR_REQ_UNKOWN_ERR = -27,
  HGF_LCMP_ERROR_NOTIFY_INVALID = -28,
  HGF_LCMP_ERROR_INVALID_VER = -29,
} t_HGF_LCMP_ErrorCode;

typedef enum {
  VB_LCMP_BROADCAST_RESPONSE_OK = 0,
  VB_LCMP_BROADCAST_RESPONSE_ERROR_NOONE_RESPONSE = -1,
  VB_LCMP_BROADCAST_RESPONSE_ERROR_NOT_ALL_RESPONSE = -2,
  VB_LCMP_BROADCAST_RESPONSE_BAD_ARGS = -3
} t_vb_LCMP_BroadcastResponseCheck;

typedef struct s_CallbackData{
  t_HTLVsLists *ReceivedValues;
  t_HGF_TLV expectedHGFOpcode;
  t_LCMP_OPCODE rxLcmpOpcode;
  INT8U paramID;
  pthread_mutex_t mutex;
  pthread_cond_t condition;
  BOOL frameReceived;
  BOOL sendack;
  BOOL markTimeStamp;
  INT8U srcMAC[ETH_ALEN];
}t_CallbackData;

typedef struct s_Callbacks
{
  BOOL               (*Callback)(const INT8U *, INT16U, t_CallbackData *);
  t_CallbackData       CallbackData;
  t_LCMP_OPCODE        filter;
  INT16U               transactionID;
  struct s_Callbacks *nextCallback;
  struct s_Callbacks *prevCallback;
} t_Callbacks;

typedef void (*t_lcmpNodesListRespCb) (BOOLEAN found, const INT8U *mac);

typedef struct s_LcmpComParams
{
  const t_ValuesArray    *reqValues;                ///< List of HTLVs values to send
  const INT8U            *dstMac;                   ///< If the sent is unicast it indicates the destination MAC. NULL otherwise
  t_Transmision            transmisionType;          ///< Shows the type of transmission
  t_HTLVsLists           **rspValuesList;            ///< List of HTLVs received in confirmation frames
  t_ValuesArray          **notifyValues;             ///< List of TLVs received in notify (only used in @ref VbLcmpControlWaitNotify)
  t_lcmpNodesListRespCb    respCb;                   ///< Callback to run when a expected response is found/not found for a given node
  INT32U                   timeoutMs;                ///< Request timeout (in ms)
  INT16U                   transactionId;            ///< Transaction Id to use. It enables multi-thread LCMP requests.
  INT8U                    paramIdReq;               ///< Parameter Id to request
  INT8U                    paramIdRsp;               ///< Expected parameter Id to receive (used in @ref VbLcmpNotifyAndWait and @ref VbLcmpControlWaitNotify)
  BOOLEAN                  ack;                      ///< ACK requested (used in @ref VbLcmpNotify)
  struct timespec          tsTx;                     ///< Time Stamp of transmitted packet
} t_lcmpComParams;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Initializes LCMP params structure with default values.
 * @param[in,out] lcmpParams Pointer to LCMP structure to initialize
 * @return @ref t_HGF_LCMP_ErrorCode
 **/
t_HGF_LCMP_ErrorCode VbLcmpParamsInit(t_lcmpComParams *lcmpParams);

/**
 * @brief Execute a read operation on LCMP
 * @param[in] lcmpParams Pointer to LCMP parameters
 * @return @ref t_HGF_LCMP_ErrorCode
**/
t_HGF_LCMP_ErrorCode VbLcmpRead(t_lcmpComParams *lcmpParams);

/**
 * @brief Execute a control operation on LCMP
 * @param[in] lcmpParams Pointer to LCMP parameters
 * @return @ref t_HGF_LCMP_ErrorCode
**/
t_HGF_LCMP_ErrorCode VbLcmpControl(t_lcmpComParams *lcmpParams);

/**
 * @brief Sends given notify and waits for a response
 * @param[in] lcmpParams Pointer to LCMP parameters
 * @return @ref t_HGF_LCMP_ErrorCode
**/
t_HGF_LCMP_ErrorCode VbLcmpNotifyAndWait(t_lcmpComParams *lcmpParams);

/**
 * @brief Sends a LCMP Notify
 * @param[in] lcmpParams Pointer to LCMP parameters
 * @return @ref t_HGF_LCMP_ErrorCode
**/
t_HGF_LCMP_ErrorCode VbLcmpNotify(t_lcmpComParams *lcmpParams);

/**
 * @brief Execute a write operation on LCMP
 * @param[in] lcmpParams Pointer to LCMP parameters
 * @return @ref t_HGF_LCMP_ErrorCode
**/
t_HGF_LCMP_ErrorCode VbLcmpWrite(t_lcmpComParams *lcmpParams);

/**
 * @brief Execute a complex operation. Sends a control frame, and wait its confirmation.
 * If this is received the function waits a notify frame.
 * @param[in] lcmpParams Pointer to LCMP parameters
 * @return @ref t_HGF_LCMP_ErrorCode
**/
t_HGF_LCMP_ErrorCode VbLcmpControlWaitNotify(t_lcmpComParams *lcmpParams);

/**
 * @brief This function initiates the VectorBoost LCMP component to default values
 * @param[in] ifeth Ethernet interface
**/
void VbLcmpInit(const char *ifeth);

/**
 * @brief Execute a complex operation. Sends a control frame, and wait its confirmation.
 * If this is received the function waits a notify frame.
 * @param[in] confirm_values_list List of TLVs received
 * @param[in] onlyDMs only check Domain Masters
 * @param[in] respCb callback to run when a expected response is found/not found for a given node
 * @return @ref t_vb_LCMP_BroadcastResponseCheck
**/
t_vb_LCMP_BroadcastResponseCheck VbLcmpNodesListResponseCheck( const t_HTLVsLists *confirm_values_list, BOOLEAN onlyDMs, t_lcmpNodesListRespCb respCb, struct timespec tsTx);

/**
 * @brief Wakes up a callback waiting in condition variable
 * @param[in] callbackData Pointer to callback data
 **/
void VbLcmpCallbackWakeUp(t_CallbackData *callbackData);

/**
 * @brief Gets LCMP Param Id from given payload
 * @param[in] lcmpValue Pointer to LCMP payload
 * @param[in] length Length of LCMP payload
 * @param[out] paramId LCMP Param Id
 * @return @ref t_HGF_LCMP_ErrorCode
 **/
t_HGF_LCMP_ErrorCode VbLcmpParamIdGet(const INT8U *lcmpValue, INT16U length, INT8U *paramId);

/**
 * @brief Installs a callback to receive LCMP messages conveying given parameter Id
 * @param[in] paramID Parameter Id to register
 * @param[out] callback_installed Callback info structure
 * @return @ref t_HGF_LCMP_ErrorCode
 **/
t_HGF_LCMP_ErrorCode VbLcmpWaitNotifyPermanentInstall(t_vb_HGF_NOTIF paramID,
   t_Callbacks **callback_installed);

/**
 * @brief Waits timeouNotify msecs and retrieves HTLVs received by given callback
 * @param[in] timeoutNotify Timeout to wait for response (in ms)
 * @param[out] receivedValues Received HTLVs
 * @param[out] callback_installed Callback info structure
 * @return @ref t_HGF_LCMP_ErrorCode
 **/
t_HGF_LCMP_ErrorCode VbLcmpWaitNotifyPermanent(INT16U timeoutNotify, t_HTLVsLists **receivedValues,
    t_Callbacks *callback_installed);

/**
 * @brief Uninstalls a callback
 * @param[in] callback_installed Callback info structure
 * @return @ref t_HGF_LCMP_ErrorCode
 **/
t_HGF_LCMP_ErrorCode VbLcmpWaitNotifyPermanentUninstall(t_Callbacks **callback_installed);

/**
 * @brief Display LCMP time statistics
 * @param[in] writeFun display function to be called
 **/
void vbLcmpTimeStatsDump(t_writeFun writeFun);

/**
 * @brief Reset LCMP time statistics
 **/
void vbLcmpTimeStatsReset(void);

/**
 * @brief Add temporary list (one Multicast response) into final list (all Multicast response)
 * @param[in] finalList final list where all repsonses are sotred
 * @param[in] tempList current response to be added to final list
 * @return @ref t_HGF_LCMP_ErrorCode.
 **/
t_HGF_LCMP_ErrorCode vbLcmpTempListToListAdd(t_HTLVsLists *finalList, t_HTLVsLists *tempList);

#endif /* VB_LCMP_COM_H_ */

/**
 * @}
**/
