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
 * @file vb_LCMP_com.c
 * @brief Communications over LCMP implemented to vector boost
 *
 * @internal
 *
 * @author
 * @date 07/01/2015
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "vb_driver_conf.h"
#include "vb_LCMP_com.h"
#include "vb_LCMP_dbg.h"
#include "vb_DataModel.h"
#include "vb_log.h"
#include "vb_LCMP_socket.h"
#include "vb_mac_utils.h"
#include "vb_counters.h"
#include "vb_thread.h"

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

/* see linux/if_ether.h */
#define ETH_HEADER_LEN          (ETH_HLEN)      /* Total octets in header.       */
#define ETH_MIN_FRAME_LEN       (ETH_ZLEN)      /* Min. octets in frame sans FCS */
#define ETH_USER_DATA_LEN       (ETH_DATA_LEN)  /* Max. octets in payload        */
#define ETH_MAX_FRAME_LEN       (ETH_FRAME_LEN) /* Max. octets in frame sans FCS */

#define ETH_P_NULL              (0x0)           /* No protocol above ETH */

#define BUF_SIZE                (ETH_MAX_FRAME_LEN)

#define HGFTL_SIZE              (sizeof(t_HGFTL))

#define VB_LCMP_TLV_MIN_SIZE    (3)

//ITU-T MAC address
static const INT8U MULTICAST_MAC_ADDRESS[] =
  { 0x01, 0x19, 0xA7, 0x52, 0x76, 0x96 };

static const INT8U BROADCAST_MAC_ADDRESS[] =
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct __attribute__ ((packed))
{
  INT8U Type;
  INT16U length;
} t_HGFTL;

typedef struct s_lcmpTransactionStatsData{
  BOOL   enable;
  INT64S mcastMaxRtt;
  INT64S mcastMinRtt;
  INT64S mcastAvgRtt;
  INT64U mcastNTransactions;
  INT64U mcastTooLateOrLost;
  INT64U mcastRetry;
  INT64S ucastMaxRtt;
  INT64S ucastMinRtt;
  INT64S ucastAvgRtt;
  INT64U ucastNTransactions;
  INT64U ucastRetry;
  INT64U ucastNTimeout;
}t_lcmpTransactionStatsData;

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */
static pthread_mutex_t     vbLcmpTransactionMutex = PTHREAD_MUTEX_INITIALIZER;
static INT16U              vbLcmpTransactionId = 0;


static t_lcmpTransactionStatsData vbLcmpTimeStats = {0};

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
*/

/**
 * @brief Sends a require LCMP frame
 * @param[in] lcmpOpcode LCMP opcode to use
 * @param[in] lcmpParams LCMP parameters
 * @return @ref t_HGF_LCMP_ErrorCode.
**/
static t_HGF_LCMP_ErrorCode VbLcmpReq(t_LCMP_OPCODE lcmpOpcode, t_lcmpComParams *lcmpParams);

/**
 * @brief This function is executed when a expected TLV is received
 * @param[in] LCMPvalue Data received
 * @param[in] length Length of data received
 * @param[out] CallbackData Data necessary for callback
 * @return Boolean indicating if some data received is the expected one.
**/
static BOOL VbLcmpCallback(const INT8U *lcmpValue, INT16U length, t_CallbackData *callbackData);

/**
 * @brief This function is executed when a expected TLV is received, and only one reception expected
 * @param[in] LCMPvalue Data received
 * @param[in] length Length of data received
 * @param[out] CallbackData Data necessary for callback
 * @return Boolean indicating if some data received is the expected one.
**/
static BOOL VbLcmpCallbackAndCondSignal(const INT8U *lcmpValue, INT16U length, t_CallbackData *callbackData);

/**
 * @brief non active wait to receive a signal to condition variable
 * @param[in] timeInMs Time out in miliseconds
 * @param[in] mutex Pointer to condition variable mutex
 * @param[in] condition Pointer to condition variable
 * @return TRUE if condition activated, FALSE if timeout
 **/
static BOOL VbLcmpwait(INT32U timeInMs, pthread_mutex_t *mutex, pthread_cond_t *condition);

/**
 * @brief Return the frame received state of callback
 * @param[in] callback
 * @return TRUE if frame received, FALSE if not frame received
 **/
static BOOL VbLcmpCallbackFrameReceived(t_Callbacks *callback);

/**
 * @brief Update Multicast round trip time stats
 * @param[in] tsTx time multicast packet was sent
 * @param[in] tsRx time one response was received
 * @return @ref t_HGF_LCMP_ErrorCode.
 **/
static t_HGF_LCMP_ErrorCode vbLcmpMcastStatsUpdate(struct timespec tsTx, struct timespec tsRx);

/**
 * @brief Update Unicast round trip time stats
 * @param[in] tsTx time unicast packet was sent
 * @param[in] tsRx time unicast response was received
 * @return @ref t_HGF_LCMP_ErrorCode.
 **/
static t_HGF_LCMP_ErrorCode vbLcmpUcastStatsUpdate(struct timespec tsTx, struct timespec tsRx);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_HGF_LCMP_ErrorCode vbLcmpMcastStatsUpdate(struct timespec tsTx, struct timespec tsRx)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;
  INT64S               rtt;

  vbLcmpTimeStats.mcastNTransactions++;

  rtt = VbUtilElapsetimeTimespecMs(tsTx, tsRx);
  if(rtt > vbLcmpTimeStats.mcastMaxRtt)
  {
    vbLcmpTimeStats.mcastMaxRtt = rtt;
  }

  if(rtt < vbLcmpTimeStats.mcastMinRtt)
  {
    vbLcmpTimeStats.mcastMinRtt = rtt;
  }

  // Lets weight current entry 1/4 compare to avg
  //xn+1 =  (xn + 1/4.xi)/(1+1/4) -> (xn + xi/4)*(4/5) -> (4xn + xi)/5
  vbLcmpTimeStats.mcastAvgRtt = ((vbLcmpTimeStats.mcastAvgRtt<<2) + rtt)/5;

  return ret;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode vbLcmpUcastStatsUpdate(struct timespec tsTx, struct timespec tsRx)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;
  INT64S               rtt;

  vbLcmpTimeStats.ucastNTransactions++;

  rtt = VbUtilElapsetimeTimespecMs(tsTx, tsRx);
  if(rtt > vbLcmpTimeStats.ucastMaxRtt)
  {
    vbLcmpTimeStats.ucastMaxRtt = rtt;
  }

  if(rtt < vbLcmpTimeStats.ucastMinRtt)
  {
    vbLcmpTimeStats.ucastMinRtt = rtt;
  }

  // Lets weight current entry 1/4 compare to avg
  //xn+1 =  (xn + 1/4.xi)/(1+1/4) -> (xn + xi/4)*(4/5) -> (4xn + xi)/5
  vbLcmpTimeStats.ucastAvgRtt = ((vbLcmpTimeStats.ucastAvgRtt<<2) + rtt)/5;

  return ret;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode VbLcmpDestMacGet(const INT8U *unicastMAC, t_Transmision transmisionType, const INT8U **dstMAC)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;
  const INT8U         *dst_mac = NULL;

  if (dstMAC == NULL)
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    switch(transmisionType)
    {
      case UNICAST:
      {
        dst_mac = unicastMAC;
        break;
      }
      case MULTICAST:
      case MULTICAST_DMS:
      {
        dst_mac = (INT8U *)MULTICAST_MAC_ADDRESS;
        break;
      }
      case BROADCAST:
      {
        dst_mac = (INT8U *)BROADCAST_MAC_ADDRESS;
        break;
      }
      default:
      {
        ret = HGF_LCMP_ERROR_BAD_ARGS;
        break;
      }
    }
  }

  if ((ret == HGF_LCMP_ERROR_NONE) && (dst_mac == NULL))
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    (*dstMAC) = dst_mac;
  }

  return ret;
}

/*******************************************************************/

static BOOL VbLcmpwait(INT32U timeInMs, pthread_mutex_t *mutex, pthread_cond_t *condition)
{

  struct timespec ts;
  INT32S n;
  int err_sem;
  BOOL returnValue = FALSE;


  if((mutex != NULL) && (condition != NULL))
  {
    err_sem = pthread_mutex_lock(mutex);
    if (err_sem != 0)
    {
      VbLogPrint( VB_LOG_ERROR, "pthread_mutex_lock failed!");
    }

    if (timeInMs > 0)
    {
      clock_gettime(CLOCK_REALTIME,&ts);

      ts.tv_sec += timeInMs / 1000;
      ts.tv_nsec += (1000 * 1000 * (timeInMs % 1000));
      ts.tv_sec += ts.tv_nsec / (1000 * 1000 * 1000);
      ts.tv_nsec %= (1000 * 1000 * 1000);

      n = pthread_cond_timedwait(condition, mutex, &ts);
    }
    else
    {
      n = pthread_cond_wait(condition, mutex);
    }

    if (err_sem == 0)
    {
      err_sem = pthread_mutex_unlock(mutex);
      if(err_sem != 0)
      {
        VbLogPrint( VB_LOG_ERROR, "phtread_mutex_unlock failed!");
      }
    }

    if (n == 0)
    {
      returnValue = TRUE;
    }
    else if (n == ETIMEDOUT)
    {
      vbLcmpTimeStats.ucastNTimeout++;
      returnValue = FALSE;
    }
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR,"VbLcmpwait mutex 0x%X; condition 0x%X",mutex, condition);
  }
  return returnValue;
}

/*******************************************************************/

static BOOL VbLcmpCallbackAndCondSignal(const INT8U *lcmpValue, INT16U length, t_CallbackData *callbackData)
{
  if((lcmpValue != NULL) && (callbackData != NULL))
  {
    if(VbLcmpCallback(lcmpValue, length, callbackData))
    {
      int err_sem;

      err_sem = pthread_mutex_lock(&(callbackData->mutex));
      if (err_sem != 0)
      {
        VbLogPrint(VB_LOG_ERROR,"pthread_mutex_lock failed!");
      }

      if(!(callbackData->frameReceived))
      {
        callbackData->frameReceived = TRUE;

        if ((callbackData->expectedHGFOpcode != HGF_WRITE_PARAMETER_CONFIRM) &&
            (callbackData->expectedHGFOpcode != HGF_CONTROL_CONFIRM) &&
            (callbackData->expectedHGFOpcode != HGF_NOTIFY_CONFIRM) &&
            (callbackData->ReceivedValues == NULL))
        {
          VbLogPrint(VB_LOG_ERROR, "Empty frame!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }

        if (0 != pthread_cond_signal(&(callbackData->condition)))
        {
          VbLogPrint(VB_LOG_ERROR,"pthread_cond_signal failed!");
        }
      }

      if(err_sem == 0)
      {
        err_sem = pthread_mutex_unlock(&(callbackData->mutex));
        if (err_sem != 0)
        {
          VbLogPrint(VB_LOG_ERROR,"pthread_mutex_unlock failed!");
        }
      }
    }
  }
  return TRUE;
}

/*******************************************************************/

static BOOL VbLcmpCallbackFrameReceived(t_Callbacks *callback)
{
  BOOL return_value;
  int err_sem;

  err_sem = pthread_mutex_lock(&(callback->CallbackData.mutex));
  if (err_sem != 0)
  {
    VbLogPrint(VB_LOG_ERROR, "pthread_mutex_lock failed!");
  }

  return_value = callback->CallbackData.frameReceived;

  if(err_sem == 0)
  {
    err_sem = pthread_mutex_unlock(&(callback->CallbackData.mutex));
    if (err_sem != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_mutex_unlock failed!");
    }
  }

  return return_value;
}

/*******************************************************************/

static BOOL VbLcmpCallback( const INT8U *lcmpValue, INT16U length, t_CallbackData *callbackData)
{

  INT16U htlv_length = 0;
  INT8U *htlv_data;
  INT16U htlv_length_confirm;
  INT32S remaining_bytes;
  t_ValuesArray *htlv_values_array = NULL;
  t_VB_comErrorCode comerror_code = VB_COM_ERROR_NONE;
  INT8U num_params_received = 0;
  BOOL values_received = FALSE;
  BOOLEAN msg_received = FALSE;

  remaining_bytes = length;
  if((lcmpValue != NULL) && (callbackData != NULL))
  {
    t_HGFTL *hgftl;
    while(remaining_bytes >= VB_LCMP_TLV_MIN_SIZE)
    {
      hgftl = (t_HGFTL *)lcmpValue;

      htlv_length = _ntohs_ghn(hgftl->length);

      if(hgftl->Type == callbackData->expectedHGFOpcode)
      {
        htlv_data = (INT8U *)(&lcmpValue[HGFTL_SIZE]);

        switch(callbackData->expectedHGFOpcode)
        {
          case HGF_WRITE_PARAMETER_CONFIRM:
          case HGF_CONTROL_CONFIRM:
          case HGF_NOTIFY_CONFIRM:
          {
            INT8U *param_id_received;

            param_id_received = &htlv_data[0];
            for(num_params_received = htlv_data[0] ; num_params_received > 0 ; num_params_received--)
            {
              param_id_received++;
              if(*param_id_received == callbackData->paramID)
              {
                htlv_length_confirm = 1;
                //htlv_data_confirm[0] = *param_id_received;
                values_received = TRUE;
                comerror_code = VbDatamodelValueToArrayAdd ( htlv_length_confirm, param_id_received,
                    &htlv_values_array );
              }
            }

            msg_received = TRUE;
            break;
          }
          default:
          {
            if(htlv_data[0] == callbackData->paramID)
            {
              if(htlv_length > 0)
              {
                values_received = TRUE;
                msg_received = TRUE;
                comerror_code = VbDatamodelValueToArrayAdd( htlv_length, htlv_data, &htlv_values_array);
              }
            }
            break;
          }
        }
      }

      remaining_bytes -= ( HGFTL_SIZE + htlv_length);
      lcmpValue += ( HGFTL_SIZE + htlv_length);
    }

    if (values_received)
    {
      comerror_code = VbDatamodelHtlvListAdd( htlv_values_array, callbackData->sendack,
          callbackData->markTimeStamp, callbackData->srcMAC, &callbackData->ReceivedValues);
    }

    if ((msg_received == TRUE) && (values_received == TRUE))
    {
      LcmpDbgMsgAdd(callbackData->srcMAC, FALSE, TRUE, callbackData->rxLcmpOpcode, callbackData->paramID);
    }
  }

  if (comerror_code != VB_COM_ERROR_NONE)
  {
    msg_received = FALSE;
  }

  if ((msg_received == TRUE) && (values_received == TRUE) && (callbackData->ReceivedValues == NULL))
  {
    VbLogPrint(VB_LOG_ERROR, "Empty frame!!!!!!!!!!!!");
  }

  return msg_received;
}

/*******************************************************************/

static t_HGF_LCMP_ErrorCode VbLcmpReq(t_LCMP_OPCODE lcmpOpcode, t_lcmpComParams *lcmpParams)
{
  INT8U                           *mmpl = NULL;
  t_HGF_LCMP_ErrorCode             result = HGF_LCMP_ERROR_NONE;
  t_VB_comErrorCode                vb_com_err = VB_COM_ERROR_NONE;
  t_vb_LCMP_BroadcastResponseCheck broadcast_response_check_result = VB_LCMP_BROADCAST_RESPONSE_OK;
  INT16U                           mmpl_length;
  const INT8U *                   dest_mac = NULL;
  INT16U                           lcmp_value_length = 0;
  INT8U                           *lcmp_value;
  INT16U                           offset_htlv;
  t_Callbacks                     *callback_installed = NULL;
  t_LCMP_OPCODE                    receive_opcode = 0;
  t_HGF_TLV                        receivehgf_opcode = 0;
  t_HGF_TLV                        sendhgf_opcode = 0;
  t_HTLVsLists                    *confirm_values = NULL;
  t_HTLVsLists                    *final_confirm_values = NULL;
  INT32U                           params_to_check_idx = 0;
  INT32U                           confirm_idx = 0;
  INT16U                           num_control;
  BOOLEAN                          param_confirmed = FALSE;
  BOOLEAN                          check_only_dms = FALSE;
  BOOLEAN                          store_ts_rx = FALSE;
  CHAR                             mac_str[MAC_STR_LEN];
  INT8U                            receive_param_id;

  if ((lcmpParams == NULL) ||
      (lcmpParams->reqValues == NULL))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    switch(lcmpOpcode)
    {
      case LCMP_CTRL_REQ:
      {
        receive_opcode    =  LCMP_CTRL_CNF;
        receivehgf_opcode = HGF_CONTROL_CONFIRM;
        receive_param_id  = lcmpParams->paramIdReq;
        sendhgf_opcode    = HGF_CONTROL;
        break;
      }
      case LCMP_WRITE_REQ:
      {
        receive_opcode    = LCMP_WRITE_CNF;
        receivehgf_opcode = HGF_WRITE_PARAMETER_CONFIRM;
        receive_param_id  = lcmpParams->paramIdReq;
        sendhgf_opcode    = HGF_PARAMETER;
        break;
      }
      case LCMP_READ_REQ:
      {
        receive_opcode    = LCMP_READ_CNF;
        receivehgf_opcode = HGF_PARAMETER;
        receive_param_id  = lcmpParams->paramIdReq;
        sendhgf_opcode    = HGF_READ_PARAMETER;
        break;
      }
      case LCMP_NOTIFY_IND:
      {
        receive_opcode    = LCMP_NOTIFY_IND;
        receivehgf_opcode = HGF_NOTIFY;
        receive_param_id  = lcmpParams->paramIdRsp;
        sendhgf_opcode    = HGF_NOTIFY;

        lcmpParams->transactionId = 0; // Force transaction id to 0 for IND packets
        break;
      }
      default:
      {
        result = HGF_LCMP_ERROR_BAD_ARGS;
        break;
      }
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    // Get proper destination MAC address
    result = VbLcmpDestMacGet(lcmpParams->dstMac, lcmpParams->transmisionType, &dest_mac);
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    // Store Tx/Rx time if needed by stats
    if(vbLcmpTimeStats.enable == TRUE)
    {
      clock_gettime(CLOCK_MONOTONIC, &lcmpParams->tsTx);
      store_ts_rx = TRUE;
    }

    // Get MAC string to be used in debug logs
    MACAddrMem2str(mac_str, dest_mac);

    LcmpDbgMsgAdd(dest_mac, TRUE, FALSE, lcmpOpcode, lcmpParams->paramIdReq);

    for (num_control = 0 ; num_control < lcmpParams->reqValues->NumValues ; num_control++ )
    {
      lcmp_value_length += HGFTL_SIZE + lcmpParams->reqValues->values[num_control].ValueLength;
    }

    if (lcmpOpcode == LCMP_NOTIFY_IND)
    {
      mmpl_length = lcmp_value_length + LCMP_IND_HEADER_SIZE;
    }
    else
    {
      mmpl_length = lcmp_value_length + LCMP_REQ_HEADER_SIZE;
    }

    mmpl = (INT8U *) calloc(1, mmpl_length); // Buffer for ethernet frame

    if (mmpl == NULL)
    {
      result = HGF_LCMP_ERROR_MALLOC;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    final_confirm_values = VbDatamodelHtlvListCreate();
    if(final_confirm_values == NULL)
    {
      result = HGF_LCMP_ERROR_MALLOC;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    if (lcmpParams->transmisionType == UNICAST)
    {
      callback_installed =
          LcmpCallBackInstall(
              VbLcmpCallbackAndCondSignal,
              receive_opcode,
              receivehgf_opcode,
              receive_param_id,
              lcmpParams->transactionId,
              store_ts_rx);
    }
    else
    {
      callback_installed =
          LcmpCallBackInstall(
              VbLcmpCallback,
              receive_opcode,
              receivehgf_opcode,
              receive_param_id,
              lcmpParams->transactionId,
              store_ts_rx);
    }

    if (callback_installed == NULL)
    {
      result = HGF_LCMP_ERROR_CALLBACK_INSTALATION_ERROR;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    if (lcmpOpcode == LCMP_NOTIFY_IND)
    {
      t_LCMP_ind_Header       *lcmp_header = NULL;
      lcmp_header                = (t_LCMP_ind_Header *)mmpl;
      lcmp_header->control       = LCMP_CONTROL_VB;
      lcmp_header->length        = _htons_ghn(lcmp_value_length);
      lcmp_header->transactionId = _htons_ghn(lcmpParams->transactionId);
      lcmp_header->notifAck      = 0;
    }
    else
    {
      t_LCMP_req_Header *lcmp_header;
      lcmp_header                = (t_LCMP_req_Header *)mmpl;
      lcmp_header->control       = LCMP_CONTROL_VB;
      lcmp_header->transactionId = _htons_ghn(lcmpParams->transactionId);
      lcmp_header->length        = _htons_ghn(lcmp_value_length);
      vb_com_err = LcmpMacGet(lcmp_header->AEMAC);
    }

    if (vb_com_err != VB_COM_ERROR_NONE)
    {
      result = HGF_LCMP_ERROR_ETH_IF;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    t_HGFTL *hgftl;

    if (lcmpOpcode == LCMP_NOTIFY_IND)
    {
      lcmp_value = &mmpl[LCMP_IND_VALUE_OFFSET];
    }
    else
    {
      lcmp_value = &mmpl[LCMP_REQ_VALUE_OFFSET];
    }
    offset_htlv = 0;

    for (num_control = 0 ; num_control < lcmpParams->reqValues->NumValues ; num_control++ )
    {
      lcmp_value += offset_htlv;
      hgftl = (t_HGFTL *)lcmp_value;
      hgftl->Type = sendhgf_opcode;
      hgftl->length = _htons_ghn(lcmpParams->reqValues->values[num_control].ValueLength);

      memcpy(&lcmp_value[offset_htlv + HGFTL_SIZE],
          lcmpParams->reqValues->values[num_control].Value,
          lcmpParams->reqValues->values[num_control].ValueLength);
      offset_htlv = HGFTL_SIZE + lcmpParams->reqValues->values[num_control].ValueLength;
    }

    // Send LCMP
    result = LcmpPacketSend(dest_mac, lcmpOpcode, mmpl_length, mmpl);

    if (result != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR,
                 "Error %d sending request; LCMP Opcode 0x%X; Param Id %u; Dest MAC %s",
                 result, lcmpOpcode, lcmpParams->paramIdReq, mac_str);
      VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_REQ_TX_ERROR);
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    BOOLEAN wait_resp;
    BOOLEAN no_resp;
    INT32U  n_retries = 0;
    INT32U  n_retries_max = VbDriverConfLcmpDefaultNAttemptGet();

    do
    {
      BOOLEAN frame_received;

      wait_resp = FALSE;

      if (lcmpParams->transmisionType == UNICAST)
      {
        if (!VbLcmpCallbackFrameReceived(callback_installed))
        {
          VbLcmpwait(lcmpParams->timeoutMs,
                     &(callback_installed->CallbackData.mutex),
                     &(callback_installed->CallbackData.condition));
        }
      }
      else
      {
        // Multicast
        VbThreadSleep(lcmpParams->timeoutMs);
      }

      confirm_values = LcmpCallBackReceiveGet(callback_installed, &frame_received);

      if(confirm_values != NULL)
      {
        result = vbLcmpTempListToListAdd(final_confirm_values, confirm_values);
        no_resp = FALSE;
      }
      else
      {
        no_resp = TRUE;
      }

      if (result == HGF_LCMP_ERROR_NONE)
      {
        // Analyse Rx
        if ((frame_received == TRUE) &&
            ((callback_installed->CallbackData.expectedHGFOpcode == HGF_CONTROL_CONFIRM) ||
             (callback_installed->CallbackData.expectedHGFOpcode == HGF_WRITE_PARAMETER_CONFIRM) ||
             (callback_installed->CallbackData.expectedHGFOpcode == HGF_NOTIFY_CONFIRM)) &&
             (no_resp == TRUE))
        {
          // Frame received but confirmation was not present
          VbLogPrint(VB_LOG_WARNING,
                     "LCMP Cnf invalid (ParamId %u not found); Op 0x%X; ParamId %u; MAC %s;",
                     callback_installed->CallbackData.expectedHGFOpcode,
                     lcmpOpcode,
                     lcmpParams->paramIdReq,
                     mac_str);
          VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_CNF_PARAMID_NOT_FOUND);

          if(lcmpParams->transmisionType == UNICAST)
          {
            vbLcmpTimeStats.ucastRetry++;
          }
          else
          {
            vbLcmpTimeStats.mcastRetry++;
          }

          wait_resp = (++n_retries < n_retries_max)?TRUE:FALSE;
          result = HGF_LCMP_ERROR_CNF_INVALID;
        }
        else if (no_resp == TRUE)
        {
          VbLogPrint(VB_LOG_WARNING,
                     "No response; LCMP Opcode 0x%X; Param Id %u; Dest MAC %s",
                     lcmpOpcode, lcmpParams->paramIdReq, mac_str);
          VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_REQ_NO_RESPONSE);

          if(wait_resp == TRUE)
          {
            if(lcmpParams->transmisionType == UNICAST)
            {
              vbLcmpTimeStats.ucastRetry++;
            }
            else
            {
              vbLcmpTimeStats.mcastRetry++;
            }
          }
          result = HGF_LCMP_ERROR_NO_RESPONSE;
        }
      }

      if (result == HGF_LCMP_ERROR_NONE)
      {
        if (((lcmpParams->transmisionType == UNICAST) && (final_confirm_values->NumLists != lcmpParams->reqValues->NumValues)) ||
            (final_confirm_values->head == NULL) ||
            (final_confirm_values->head->HTLVsArray == NULL) ||
            (final_confirm_values->head->HTLVsArray->NumValues == 0))
        {
          VbLogPrint(VB_LOG_ERROR,
                     "LCMP Cnf error; Op 0x%X; ParamId %u; MAC %s; Exp Cnf Val %u; Rx Cnf Values %u;",
                     lcmpOpcode,
                     lcmpParams->paramIdReq,
                     mac_str,
                     lcmpParams->reqValues->NumValues,
                     final_confirm_values->NumLists);
          VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_CNF_ERROR);

          wait_resp = (++n_retries < n_retries_max)?TRUE:FALSE;
          if(wait_resp == TRUE)
          {
            VbDatamodelHtlvsListValueDestroy(&final_confirm_values);
            final_confirm_values = VbDatamodelHtlvListCreate();
            vbLcmpTimeStats.ucastRetry++;
          }

          result = HGF_LCMP_ERROR_CNF_INCOMPLETE;
        }
      }

      // Check confirmation (only for CTRL.req and WRITE.req)
      if ((result == HGF_LCMP_ERROR_NONE) &&
          ((lcmpOpcode == LCMP_CTRL_REQ) || (lcmpOpcode == LCMP_WRITE_REQ)))
      {
        if (lcmpParams->transmisionType == UNICAST)
        {
          if (memcmp(final_confirm_values->head->srcMAC, dest_mac, ETH_ALEN) == 0)
          {
            t_ValuesArray *confirmation_array;

            vbLcmpUcastStatsUpdate(lcmpParams->tsTx, final_confirm_values->head->timeStamp);

            confirmation_array = final_confirm_values->head->HTLVsArray;
            for (params_to_check_idx = 0 ; params_to_check_idx < lcmpParams->reqValues->NumValues ; params_to_check_idx++ )
            {
              param_confirmed = FALSE;

              // Only one frame received, so check position #0 of "confirmation_array"
              for (confirm_idx = 0 ; confirm_idx < confirmation_array[0].NumValues ; confirm_idx++)
              {
                // ParamId to check is located in first byte of "Value" (payload) array
                if((lcmpParams->reqValues->values[params_to_check_idx].Value[0]) ==
                   (confirmation_array[0].values[confirm_idx].Value[0]))
                {
                  param_confirmed = TRUE;
                  break;
                }
              }

              if (param_confirmed == FALSE)
              {
                VbLogPrint(VB_LOG_WARNING,
                           "LCMP Cnf invalid (ParamId %u not found); Op 0x%X; ParamId %u; MAC %s; Exp Cnf Val %u; Rx Cnf Values %u;",
                           lcmpParams->reqValues->values[params_to_check_idx].Value[0],
                           lcmpOpcode,
                           lcmpParams->paramIdReq,
                           mac_str,
                           lcmpParams->reqValues->NumValues,
                           final_confirm_values->NumLists);
                VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_CNF_PARAMID_NOT_FOUND);

                wait_resp = (++n_retries < n_retries_max)?TRUE:FALSE;
                if(wait_resp == TRUE)
                {
                  VbDatamodelHtlvsListValueDestroy(&final_confirm_values);
                  final_confirm_values = VbDatamodelHtlvListCreate();
                  vbLcmpTimeStats.ucastRetry++;
                }

                result = HGF_LCMP_ERROR_CNF_INVALID;
                break;
              }
            }
          }
        }
        else
        {
          if (lcmpParams->transmisionType == MULTICAST)
          {
            // All nodes
            check_only_dms = FALSE;
          }
          else
          {
            // Only DMs
            check_only_dms = TRUE;
          }

          broadcast_response_check_result = VbLcmpNodesListResponseCheck(final_confirm_values, check_only_dms, lcmpParams->respCb, lcmpParams->tsTx);

          switch (broadcast_response_check_result)
          {
            case VB_LCMP_BROADCAST_RESPONSE_OK:
            {
              VbLogPrint(VB_LOG_INFO,
                         "LCMP Multicast Request OK; Op 0x%X; ParamId %u; MAC %s;",
                         lcmpOpcode,
                         lcmpParams->paramIdReq,
                         mac_str);

              break;
            }
            case VB_LCMP_BROADCAST_RESPONSE_ERROR_NOT_ALL_RESPONSE:
            case VB_LCMP_BROADCAST_RESPONSE_ERROR_NOONE_RESPONSE:
            {
              VbLogPrint(VB_LOG_WARNING,
                         "LCMP Multicast Request not received by all devices; Op 0x%X; ParamId 0x%X; MAC %s;",
                         lcmpOpcode,
                         lcmpParams->paramIdReq,
                         mac_str);
              VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_REQ_NOT_ALL_DEV);

              wait_resp = (++n_retries < n_retries_max)?TRUE:FALSE;
              if(wait_resp == TRUE)
              {
                vbLcmpTimeStats.mcastRetry++;
              }

              result = HGF_LCMP_ERROR_REQ_NOT_ALL_DEVICES;

              break;
            }
            default:
            {
              VbLogPrint(VB_LOG_ERROR,
                         "LCMP Multicast Request unknown error (%d); Op 0x%X; ParamId 0x%X; MAC %s;",
                         broadcast_response_check_result,
                         lcmpOpcode,
                         lcmpParams->paramIdReq,
                         mac_str);
              VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_REQ_UNKNOWN_ERROR);

              result = HGF_LCMP_ERROR_REQ_UNKOWN_ERR;

              break;
            }
          }
        }
      }
    }while(wait_resp == TRUE);
  }

  if( (lcmpParams != NULL) && (lcmpParams->rspValuesList != NULL) )
  {
    // Return confirmValues (regardless of error). Its allocated memory shall be released by caller.
    *(lcmpParams->rspValuesList) = final_confirm_values;
  }
  else
  {
    // Release memory
    VbDatamodelHtlvsListValueDestroy(&final_confirm_values);
  }

  if (callback_installed != NULL)
  {
    LcmpCallBackUninstall(callback_installed);
  }

  if(mmpl != NULL)
  {
    free(mmpl);
    mmpl = NULL;
  }

  return result;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpParamsInit(t_lcmpComParams *lcmpParams)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;

  if (lcmpParams == NULL)
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    bzero(lcmpParams, sizeof(*lcmpParams));
    lcmpParams->timeoutMs = VbDriverConfLcmpDefaultTimeoutGet();

    ret = pthread_mutex_lock(&(vbLcmpTransactionMutex));
    if (ret != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_mutex_lock failed! (err %d)", ret);
    }

    if (ret == 0)
    {
      lcmpParams->transactionId = vbLcmpTransactionId++; // Generate a new transaction Id (although it might be forced to 0 afterwards)
    }

    if (ret == 0)
    {
      ret = pthread_mutex_unlock(&(vbLcmpTransactionMutex));
      if (ret != 0)
      {
        VbLogPrint(VB_LOG_ERROR, "pthread_mutex_unlock failed! (err %d)", ret);
      }
    }
  }

  return ret;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpWrite(t_lcmpComParams *lcmpParams)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if ((lcmpParams == NULL) ||
      (lcmpParams->reqValues == NULL) ||
      ((lcmpParams->transmisionType == UNICAST) && (lcmpParams->dstMac == NULL)))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }
  else
  {
    result = VbLcmpReq(LCMP_WRITE_REQ, lcmpParams);
  }

  return result;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpControl(t_lcmpComParams *lcmpParams)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if ((lcmpParams == NULL) ||
      (lcmpParams->reqValues == NULL) ||
      ((lcmpParams->transmisionType == UNICAST) && (lcmpParams->dstMac == NULL)))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }
  else
  {
    result = VbLcmpReq(LCMP_CTRL_REQ, lcmpParams);
  }

  return result;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpRead(t_lcmpComParams *lcmpParams)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if ((lcmpParams == NULL) ||
      (lcmpParams->reqValues == NULL) ||
      (lcmpParams->rspValuesList == NULL) ||
      ((lcmpParams->transmisionType == UNICAST) && (lcmpParams->dstMac == NULL)))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }
  else
  {
    result = VbLcmpReq(LCMP_READ_REQ, lcmpParams);
  }

  return result;

}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpNotify(t_lcmpComParams *lcmpParams)
{
  INT8U                   *mmpl = NULL;
  t_HGF_LCMP_ErrorCode     result = HGF_LCMP_ERROR_NONE;
  INT16U                   mmpl_length = 0;
  const INT8U             *dest_mac = NULL;
  INT16U                   lcmp_value_length = 0;
  INT8U                   *lcmp_value = NULL;
  INT16U                   values_check = 0;
  CHAR                     mac_str[MAC_STR_LEN];

  if ((lcmpParams == NULL) ||
      (lcmpParams->reqValues == NULL))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    // Get proper destination MAC address
    result = VbLcmpDestMacGet(lcmpParams->dstMac, lcmpParams->transmisionType, &dest_mac);
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    // Store Tx/Rx time if needed by stats
    if(vbLcmpTimeStats.enable == TRUE)
    {
      clock_gettime(CLOCK_MONOTONIC, &lcmpParams->tsTx);
    }

    // Get MAC string to be used in debug logs
    MACAddrMem2str(mac_str, dest_mac);

    LcmpDbgMsgAdd(dest_mac, TRUE, FALSE, LCMP_NOTIFY_IND, lcmpParams->paramIdReq);

    for (values_check = 0 ; values_check < lcmpParams->reqValues->NumValues ; values_check++)
    {
      lcmp_value_length += HGFTL_SIZE + lcmpParams->reqValues->values[values_check].ValueLength;
    }

    mmpl_length = lcmp_value_length + LCMP_IND_HEADER_SIZE;
    mmpl = (INT8U *) calloc(1, mmpl_length); // Buffer for ethernet frame

    if (mmpl == NULL)
    {
      result = HGF_LCMP_ERROR_MALLOC;
    }
    else
    {
      t_LCMP_ind_Header       *lcmp_header = NULL;

      lcmp_header = (t_LCMP_ind_Header *)mmpl;
      lcmp_header->control = LCMP_CONTROL_VB;
      lcmp_header->length = _htons_ghn(lcmp_value_length);
      lcmp_header->transactionId = 0;// IND packet expects a 0 transaction id

      if (lcmpParams->ack)
      {
        lcmp_header->notifAck = 1;
      }
      else
      {
        lcmp_header->notifAck = 0;
      }

      lcmp_value = &mmpl[LCMP_IND_VALUE_OFFSET];

      t_HGFTL *hgftl;

      for (values_check = 0 ; values_check < lcmpParams->reqValues->NumValues ; values_check++)
      {
        hgftl = (t_HGFTL *)lcmp_value;
        hgftl->Type = HGF_NOTIFY;
        hgftl->length = _htons_ghn(lcmpParams->reqValues->values[values_check].ValueLength);
        memcpy(&lcmp_value[HGFTL_SIZE], lcmpParams->reqValues->values[values_check].Value, lcmpParams->reqValues->values[values_check].ValueLength);
        lcmp_value += HGFTL_SIZE + lcmpParams->reqValues->values[values_check].ValueLength;
      }

      result = LcmpPacketSend(dest_mac, LCMP_NOTIFY_IND, mmpl_length, mmpl);

      free(mmpl);
      mmpl = NULL;
    }
  }

  return result;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpWaitNotifyPermanentInstall(t_vb_HGF_NOTIF paramID,
   t_Callbacks **callback_installed)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if (callback_installed == NULL)
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  else if(*callback_installed != NULL)
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  else
  {
    (*callback_installed) =
        LcmpCallBackInstall(
            VbLcmpCallbackAndCondSignal,
            LCMP_NOTIFY_IND,
            HGF_NOTIFY,
            (INT8U)paramID,
            0,
            FALSE);

    if(*callback_installed == NULL)
    {

      result = HGF_LCMP_ERROR_CALLBACK_INSTALATION_ERROR;
    }
  }
  return result;
}


/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpWaitNotifyPermanentUninstall(t_Callbacks **callback_installed)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
  t_VB_comErrorCode err = VB_COM_ERROR_NONE;

  if (callback_installed == NULL)
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
#if(0)
  else if(*callback_installed != NULL)
  {
    VbLogPrint(VB_LOG_INFO, "");
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
#endif
  else
  {
    err = LcmpCallBackUninstall(*callback_installed);
  }

  if(result == HGF_LCMP_ERROR_NONE)
  {
    if (err == VB_COM_ERROR_NONE)
    {
      *callback_installed = NULL;
    }

    if(err != VB_COM_ERROR_NONE)
    {
      result = HGF_LCMP_ERROR_CALLBACK_UNINSTALATION_ERROR;
    }
  }

  return result;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpWaitNotifyPermanent(INT16U timeouNotify, t_HTLVsLists **receivedValues,
    t_Callbacks *callback_installed)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;
  BOOL frame_received;

  if (receivedValues == NULL)
  {
    result = HGF_LCMP_ERROR_PARAM_ERROR;
  }
  else if(callback_installed == NULL)
  {
    result = HGF_LCMP_ERROR_CALLBACK_INSTALATION_ERROR;
  }
  else
  {
    frame_received = VbLcmpCallbackFrameReceived(callback_installed);
    if(!frame_received)
    {
      VbLcmpwait(timeouNotify, &(callback_installed->CallbackData.mutex),
          &(callback_installed->CallbackData.condition));
      frame_received = VbLcmpCallbackFrameReceived(callback_installed);
    }

    if(!frame_received)
    {
      result = HGF_LCMP_ERROR_TIMEOUT;
    }
    else
    {
      (*receivedValues) = LcmpCallBackReceiveGet(callback_installed, NULL);
      if ((*receivedValues) == NULL)
      {
        result = HGF_LCMP_ERROR_NO_DATA;
      }
    }
  }
  return result;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpNotifyAndWait(t_lcmpComParams *lcmpParams)
{
  t_HGF_LCMP_ErrorCode result = HGF_LCMP_ERROR_NONE;

  if ((lcmpParams == NULL) ||
      (lcmpParams->reqValues == NULL) ||
      (lcmpParams->rspValuesList == NULL) ||
      ((lcmpParams->transmisionType == UNICAST) && (lcmpParams->dstMac == NULL)))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }
  else
  {
    result = VbLcmpReq(LCMP_NOTIFY_IND, lcmpParams);
  }

  return result;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpControlWaitNotify(t_lcmpComParams *lcmpParams)
{
  t_HGF_LCMP_ErrorCode   result = HGF_LCMP_ERROR_NONE;
  t_VB_comErrorCode      vb_com_err = VB_COM_ERROR_NONE;
  t_Callbacks           *callback_installed = NULL;
  t_HTLVsLists          *comfirm_values = NULL;
  BOOLEAN                frame_received = FALSE;
  CHAR                   mac_str[MAC_STR_LEN];

  if ((lcmpParams == NULL) ||
      (lcmpParams->dstMac == NULL) ||
      (lcmpParams->reqValues == NULL) ||
      (lcmpParams->notifyValues == NULL))
  {
    result = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    // Get MAC string to be used in debug logs
    MACAddrMem2str(mac_str, lcmpParams->dstMac);

    callback_installed =
        LcmpCallBackInstall(
            VbLcmpCallbackAndCondSignal,
            LCMP_NOTIFY_IND,
            HGF_NOTIFY,
            (INT8U)lcmpParams->paramIdRsp,
            lcmpParams->transactionId,
            FALSE);

    if (callback_installed == NULL)
    {
      VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_NOTIFY_OTHER_ERROR);
      result = HGF_LCMP_ERROR_CALLBACK_INSTALATION_ERROR;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    result = VbLcmpReq(LCMP_CTRL_REQ, lcmpParams);

    if (result == HGF_LCMP_ERROR_CNF_INVALID)
    {
      VbLogPrint(VB_LOG_WARNING, "LCMP Cnf not found : paramIdNotify 0x%X; paramIdControl 0x%X; MAC %s;",
          lcmpParams->paramIdRsp,
          lcmpParams->paramIdReq,
          mac_str);
    }
    else if (result != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Ctrl.req Failed (error %d); timeout %u; paramIdNotify 0x%X; paramIdControl 0x%X; MAC %s",
                result,
                lcmpParams->timeoutMs,
                lcmpParams->paramIdRsp,
                lcmpParams->paramIdReq,
                mac_str);
      VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_NOTIFY_REQ_ERROR);
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    if (!VbLcmpCallbackFrameReceived(callback_installed))
    {
      VbLcmpwait(lcmpParams->timeoutMs, &(callback_installed->CallbackData.mutex),
          &(callback_installed->CallbackData.condition));
    }

    frame_received =  VbLcmpCallbackFrameReceived(callback_installed);

    if (!frame_received)
    {
      VbLogPrint(VB_LOG_ERROR, "Notify not received; timeout %u; paramIdNotify 0x%X; paramIdControl 0x%X; MAC %s",
                lcmpParams->timeoutMs,
                lcmpParams->paramIdRsp,
                lcmpParams->paramIdReq,
                mac_str);
      VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_NOTIFY_TIMEOUT);

      result = HGF_LCMP_ERROR_TIMEOUT;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    comfirm_values = LcmpCallBackReceiveGet(callback_installed, NULL);

    if ((comfirm_values == NULL) ||
        (comfirm_values->head == NULL) ||
        (comfirm_values->head->HTLVsArray == NULL))
    {
      VbLogPrint(VB_LOG_ERROR, "No data in notify msg; timeout %u; paramIdNotify 0x%X; paramIdControl 0x%X; MAC %s",
                lcmpParams->timeoutMs,
                lcmpParams->paramIdRsp,
                lcmpParams->paramIdReq,
                mac_str);
      VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_NOTIFY_OTHER_ERROR);

      result = HGF_LCMP_ERROR_NO_DATA;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    vb_com_err = VbDatamodelHTLVsArrayCopy(comfirm_values->head->HTLVsArray, lcmpParams->notifyValues);

    if (vb_com_err != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d copying payload; timeout %u; paramIdNotify 0x%X; paramIdControl 0x%X; MAC %s",
          vb_com_err,
          lcmpParams->timeoutMs,
          lcmpParams->paramIdRsp,
          lcmpParams->paramIdReq,
          mac_str);
      VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_NOTIFY_OTHER_ERROR);

      result = HGF_LCMP_ERROR_NO_DATA;
    }
  }

  if (result == HGF_LCMP_ERROR_NONE)
  {
    // Check notified values

    if ((*(lcmpParams->notifyValues))->NumValues != lcmpParams->reqValues->NumValues)
    {
      // Incorrect number of notified values
      VbLogPrint(VB_LOG_ERROR, "Invalid notified values %u and expected %u; timeout %u; paramIdNotify 0x%X; paramIdControl 0x%X; MAC %s",
          (*(lcmpParams->notifyValues))->NumValues,
          lcmpParams->reqValues->NumValues,
          lcmpParams->timeoutMs,
          lcmpParams->paramIdRsp,
          lcmpParams->paramIdReq,
          mac_str);
      VbCounterIncrease(VB_DRIVER_COUNTER_LCMP_NOTIFY_OTHER_ERROR);

      result = HGF_LCMP_ERROR_NOTIFY_INVALID;
    }
  }

  // Always release memory
  VbDatamodelHtlvsListValueDestroy(&comfirm_values);
  LcmpCallBackUninstall(callback_installed);

  return result;
}

/******************************************************************/

void VbLcmpInit(const char *ifeth)
{
  vbLcmpTimeStats.enable = TRUE;
  vbLcmpTimeStats.mcastMinRtt = MAX_INT64S;
  vbLcmpTimeStats.mcastMaxRtt = MIN_INT64S;
  vbLcmpTimeStats.ucastMinRtt = MAX_INT64S;
  vbLcmpTimeStats.ucastMaxRtt = MIN_INT64S;

  LcmpInit(ifeth);
}

/******************************************************************/

void vbLcmpTimeStatsDump(t_writeFun writeFun)
{
  writeFun("Multicast Conf wait timeout        %lld\n", VbDriverConfLcmpDefaultTimeoutGet());
  writeFun("Multicast Conf n retry             %lld\n\n", VbDriverConfLcmpDefaultNAttemptGet());

  writeFun("Stats enabled                       %s\n",   (vbLcmpTimeStats.enable == TRUE)?"YES":"NO");
  writeFun("Multicast Stats\n");
  writeFun("  . num transactions      %lld\n", vbLcmpTimeStats.mcastNTransactions);
  writeFun("  . lost by timeout       %lld\n", vbLcmpTimeStats.mcastTooLateOrLost);
  writeFun("  . max Round Trip Time   %lld\n", vbLcmpTimeStats.mcastMaxRtt);
  writeFun("  . min Round Trip Time   %lld\n", vbLcmpTimeStats.mcastMinRtt);
  writeFun("  . Avg Round Trip Time   %lld\n", vbLcmpTimeStats.mcastAvgRtt);
  writeFun("  . Retries               %lld\n", vbLcmpTimeStats.mcastRetry);

  writeFun("Unicast Stats\n");
  writeFun("  . num transactions      %lld\n", vbLcmpTimeStats.ucastNTransactions);
  writeFun("  . max Round Trip Time   %lld\n", vbLcmpTimeStats.ucastMaxRtt);
  writeFun("  . min Round Trip Time   %lld\n", vbLcmpTimeStats.ucastMinRtt);
  writeFun("  . Avg Round Trip Time   %lld\n", vbLcmpTimeStats.ucastAvgRtt);
  writeFun("  . timeout               %lld\n", vbLcmpTimeStats.ucastNTimeout);
  writeFun("  . Retries               %lld\n", vbLcmpTimeStats.ucastRetry);

}

/******************************************************************/

void vbLcmpTimeStatsReset(void)
{
  BOOL enable;

  enable = vbLcmpTimeStats.enable;
  memset((INT8U*)&vbLcmpTimeStats, 0, sizeof(vbLcmpTimeStats));
  vbLcmpTimeStats.enable = enable;
}

/******************************************************************/

t_vb_LCMP_BroadcastResponseCheck VbLcmpNodesListResponseCheck( const t_HTLVsLists *confirm_values_list, BOOLEAN onlyDMs, t_lcmpNodesListRespCb respCb, struct timespec tsTx)
{
  t_vb_LCMP_BroadcastResponseCheck result = VB_LCMP_BROADCAST_RESPONSE_OK;
  INT16U                           num_nodes;
  INT8U                           *nodes_mac = NULL;
  BOOL                             mac_found = TRUE;
  INT16U                           num_nodes_check;
  t_HTLVValuesList                *htlv_values_list;
  INT16U                           num_htvl_lists_check;
  CHAR                             mac_str[MAC_STR_LEN];
  t_vbLogLevel                     log_level;

  if (confirm_values_list != NULL)
  {
    VbDatamodelGetActiveMacsArray(&num_nodes,&nodes_mac, onlyDMs);

    if ((num_nodes > 0) &&
        (confirm_values_list->NumLists < num_nodes))
    {
      log_level = VB_LOG_ERROR;
    }
    else
    {
      log_level = VB_LOG_INFO;
    }

    VbLogPrint(log_level, "Nodes on Data base %d", num_nodes);
    VbLogPrint(log_level, "Num responses received %d", confirm_values_list->NumLists);

    if ((confirm_values_list->NumLists >= num_nodes) || (respCb != NULL))
    {
      for (num_nodes_check = 0 ; num_nodes_check < num_nodes ; num_nodes_check++)
      {
        mac_found = FALSE;
        htlv_values_list = confirm_values_list->head;
        num_htvl_lists_check = 0;

        while ( (htlv_values_list != NULL)  && (!mac_found)
             && (num_htvl_lists_check < confirm_values_list->NumLists) )
        {
          num_htvl_lists_check++;

          if (memcmp(htlv_values_list->srcMAC, &nodes_mac[num_nodes_check*ETH_ALEN], ETH_ALEN) == 0)
          {
            mac_found = TRUE;
          }

          vbLcmpMcastStatsUpdate(tsTx, htlv_values_list->timeStamp);

          htlv_values_list = htlv_values_list->nextList;
        }

        if (!mac_found)
        {
          vbLcmpTimeStats.mcastTooLateOrLost++;
          result = VB_LCMP_BROADCAST_RESPONSE_ERROR_NOT_ALL_RESPONSE;

          if (respCb == NULL)
          {
            // As callback is not used, there is no reason to continue with the loop
            break;
          }
        }

        // Get MAC string to be used in debug log
        MACAddrMem2str(mac_str, &(nodes_mac[num_nodes_check*ETH_ALEN]));
        VbLogPrint(log_level, "Node %s response %s", mac_str, mac_found?"found":"not found");

        // Call callback
        if (respCb != NULL)
        {
          respCb(mac_found, &(nodes_mac[num_nodes_check * ETH_ALEN]));
        }
      }
    }
    else
    {
      vbLcmpTimeStats.mcastTooLateOrLost+=num_nodes;
      result = VB_LCMP_BROADCAST_RESPONSE_ERROR_NOT_ALL_RESPONSE;
    }
  }
  else
  {
    vbLcmpTimeStats.mcastTooLateOrLost++;
    result = VB_LCMP_BROADCAST_RESPONSE_ERROR_NOONE_RESPONSE;
  }

  if (nodes_mac != NULL)
  {
    // Always free allocated memory
    free(nodes_mac);
    nodes_mac = NULL;
  }

  return result;
}

/*******************************************************************/

void VbLcmpCallbackWakeUp(t_CallbackData *callbackData)
{
  if (callbackData != NULL)
  {
    int err;
    int err_signal;

    err = pthread_mutex_lock(&(callbackData->mutex));

    if (err != 0)
    {
      VbLogPrint(VB_LOG_ERROR, "pthread_mutex_lock failed! (err %d)", err);
    }

    if (err == 0)
    {
      err_signal = pthread_cond_signal(&(callbackData->condition));

      if (err_signal != 0)
      {
        VbLogPrint(VB_LOG_ERROR, "pthread_cond_signal failed! (err %d)", err);
      }
    }

    if (err == 0)
    {
      err = pthread_mutex_unlock(&(callbackData->mutex));
      if (err != 0)
      {
        VbLogPrint(VB_LOG_ERROR, "pthread_mutex_unlock failed! (err %d)", err);
      }
    }
  }
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode VbLcmpParamIdGet(const INT8U *lcmpValue, INT16U length, INT8U *paramId)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;

  if ((lcmpValue == NULL) || (paramId == NULL))
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    t_HGFTL *hgftl = (t_HGFTL *)lcmpValue;
    INT8U   *htlv_data;

    htlv_data = (INT8U *)(&lcmpValue[HGFTL_SIZE]);

    if ((hgftl->Type == HGF_WRITE_PARAMETER_CONFIRM) ||
        (hgftl->Type == HGF_CONTROL_CONFIRM) ||
        (hgftl->Type == HGF_NOTIFY_CONFIRM))
    {
      if (htlv_data[0] > 0)
      {
        // Get first param id found
        *paramId = htlv_data[1];
      }
      else
      {
        ret = HGF_LCMP_ERROR_PARAM_ERROR;
      }
    }
    else
    {
      *paramId = htlv_data[0];
    }
  }

  return ret;
}

/******************************************************************/

t_HGF_LCMP_ErrorCode vbLcmpTempListToListAdd(t_HTLVsLists *finalList, t_HTLVsLists *tempList)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;
  INT32U i;

  if((finalList == NULL) || (tempList == NULL))
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if(ret == HGF_LCMP_ERROR_NONE)
  {
    t_HTLVValuesList *elem_in_list = tempList->head;
    for(i=0;((i<tempList->NumLists) && (elem_in_list != NULL)) ;i++)
    {
      VbDatamodelHtlvListInsert(finalList, elem_in_list);
      elem_in_list = elem_in_list->nextList;
    }
  }

  if(tempList != NULL)
  {
    free(tempList);
  }

  return ret;
}

/******************************************************************/

/**
 * @}
 **/
