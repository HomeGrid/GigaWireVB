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
 * @file vb_traffic.c
 * @brief Traffic awareness feature
 *
 * @internal
 *
 * @author 
 * @date 27/04/2015
 *
 **/

/*
 ************************************************************************ 
 ** Included files
 ************************************************************************
 */
#include <pthread.h>

#include "types.h"

#include "vb_LCMP_com.h"
#include "vb_log.h"
#include "vb_EA_interface.h"
#include "vb_traffic.h"
#include "vb_thread.h"
#include "vb_priorities.h"
#include "vb_main.h"
#include "vb_timer.h"
#include "vb_driver_conf.h"

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

#define VB_TRAFFIC_THREAD_NAME                  ("TrafficAwareness")
#define VB_TRAFFIC_TIMER_NAME                   ("TrafficTimer")
#define VB_TRAFFIC_PARAM_TIMEOUT                (0)

#define VB_TRAFFIC_NUM_ATTEMPT_CONF             (10)

#define VB_TRAFFIC_REPORT_TO                    (vbTrafficReportPeriod * 5)
#define VB_TRAFFIC_REPORT_TO_ENGINE_PERIOD      (((vbTrafficReportMeasWin > 0)?vbTrafficReportMeasWin:vbTrafficReportPeriod) / 2)

#define VB_TRAFFIC_REPORT_ALL_NODES             (TRUE) // Always try to configure traffic reports in all nodes (DMs and EPs)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  INT16U threshold;
  INT16U measWin;
  INT16U period;
  INT16U timeout;
} t_vbTrafficConfParams;

/// Vector Boost VB_INGRESS_TRAFFIC_MON.req TLV value fields
struct PACKMEMBER _vectorboostTlvIngressTrafficMonReq
{
  INT32U          ctrlType:8;         ///< Control type: set to VB_INGRESS_TRAFFIC_MON
  INT32U          threshold:16;       ///< Used to trigger traffic reports depending on ingress traffic and channel capacity
  INT32U          measWin:16;         ///< Measure window (in ms)
  INT32U          period:16;          ///< Time interval to send periodic traffic reports (in ms). 0: periodic reports disabled
  INT32U          timeout:16;         ///< Timeout to wait for an action from VB engine (in ms). 0: disable wait
};
typedef struct _vectorboostTlvIngressTrafficMonReq TYPE_ALIGNED32(t_vectorboostTlvIngressTrafficMonReq);

/// Vector Boost VB_INGRESS_TRAFFIC: traffic report notify indication
struct PACKMEMBER _vectorboostTlvIngressTrafficInd
{
  INT32U          notifyType:8;                ///< Notify type: set to VB_INGRESS_TRAFFIC_IND
  INT32U          trafficPrio0:16;             ///< Ingress traffic estimation of priority 0 (in Mbps)
  INT32U          trafficPrio1:16;             ///< Ingress traffic estimation of priority 1 (in Mbps)
  INT32U          trafficPrio2:16;             ///< Ingress traffic estimation of priority 2 (in Mbps)
  INT32U          trafficPrio3:16;             ///< Ingress traffic estimation of priority 3 (in Mbps)
  INT32U          maxBuffPrio0:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          maxBuffPrio1:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          maxBuffPrio2:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          maxBuffPrio3:8;              ///< Maximum buffer usage for priority 0 (in %)
  INT32U          channelCapacity:16;          ///< Channel capacity (in Mbps)
  INT32U          effectiveChannelCapacity:16; ///< Effective channel capacity (taking into account overheads and tx time) (in Mbps)
  INT32U          desiredChannelCapacity:16;   ///< Desired channel capacity (in Mbps)
  INT32U          macEfficiency:8;             ///< MAC efficiency (in %)
};
typedef struct _vectorboostTlvIngressTrafficInd TYPE_ALIGNED32(t_vectorboostTlvIngressTrafficInd);


/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_t    vbTrafficThread           = 0;
static BOOLEAN      vbTrafficThreadRunning    = FALSE;
static INT32U       vbTrafficReportPeriod     = 0;
static INT32U       vbTrafficReportMeasWin    = 0;
static INT32U       vbTrafficReportThreshold  = 0;
static t_Callbacks *vbTafficCallbackInstalled = NULL;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VBTrafficStateSet(BOOLEAN running)
{
  vbTrafficThreadRunning = running;
}

/*******************************************************************/

static void VBTrafficRespCb(BOOLEAN found, const INT8U *mac)
{
  if (mac != NULL)
  {
    VbLogPrint(VB_LOG_INFO, "Node " MAC_PRINTF_FORMAT " -> trafficReport enabled = %s",
        MAC_PRINTF_DATA(mac), found?"YES":"NO");
  }
}

/*******************************************************************/

static t_VB_comErrorCode VBTrafficMonitorConf(t_Transmision transmisionType, const INT8U *dstMac, t_vbTrafficConfParams *params)
{
  t_VB_comErrorCode                    ret = VB_COM_ERROR_NONE;
  t_HGF_LCMP_ErrorCode                 lcmp_err = HGF_LCMP_ERROR_NONE;
  t_lcmpComParams                      lcmp_params;
  t_ValuesArray                       *control_values_array = NULL;
  t_vectorboostTlvIngressTrafficMonReq ctrl_req_value;

  if ((params == NULL) || ((transmisionType == UNICAST) && (dstMac == NULL)))
  {
    ret = VB_COM_ERROR_BAD_ARGS;

    VbLogPrint(VB_LOG_ERROR, "Bad arguments");
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Build the payload
    ctrl_req_value.ctrlType  = VB_INGRESS_TRAFFIC_MON;
    ctrl_req_value.measWin   = _htons_ghn(params->measWin);
    ctrl_req_value.period    = _htons_ghn(params->period);
    ctrl_req_value.threshold = _htons_ghn(params->threshold);
    ctrl_req_value.timeout   = _htons_ghn(params->timeout);

    ret = VbDatamodelValueToArrayAdd(sizeof(ctrl_req_value), (INT8U *)(&ctrl_req_value),
        &control_values_array);

    if (ret != VB_COM_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "VbDatamodelValueToArrayAdd; error %d",ret);
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      ret = VB_COM_ERROR_LCMP;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = transmisionType;
    lcmp_params.dstMac          = dstMac;
    lcmp_params.paramIdReq      = VB_INGRESS_TRAFFIC_MON;
    lcmp_params.reqValues       = control_values_array;
    lcmp_params.respCb          = VBTrafficRespCb;

    // Send control frame to configure traffic ingress monitoring
    lcmp_err = VbLcmpControl(&lcmp_params);

    if (lcmp_err == HGF_LCMP_ERROR_REQ_NOT_ALL_DEVICES)
    {
      VbLogPrint(VB_LOG_INFO, "Periodic traffic reports not configured in all devices (err %d)", lcmp_err);

      ret = VB_COM_ERROR_ANY_CONF_LOST;
    }
    else if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending IngressTrafficMonControl.req", lcmp_err);

      ret = VB_COM_ERROR_LCMP_CONTROL;
    }
  }

  // Always release memory
  VbDatamodelHTLVsArrayDestroy(&control_values_array);

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VBTrafficReportWait(t_Callbacks *callback_installed)
{
  t_VB_comErrorCode                  ret = VB_COM_ERROR_NONE;
  t_HGF_LCMP_ErrorCode               lcmp_err;
  t_ValuesArray                     *values_array = NULL;
  t_HTLVsLists                      *list_received_values = NULL;
  t_HTLVValuesList                  *received_values = NULL;
  t_vectorboostTlvIngressTrafficInd *vb_traffic_report = NULL;
  t_IngressTraffic                   traffic_report;
  t_bpsBandTrafficReport             bps_traffic_report;
  INT32U                             value_idx = 0;
  INT16U                             list_idx = 0;

  lcmp_err = VbLcmpWaitNotifyPermanent(VB_TRAFFIC_REPORT_TO, &list_received_values, callback_installed);

  if (VBTrafficStateGet() == FALSE)
  {
    // Traffic awareness thread is stopping
    ret = VB_COM_ERROR_NONE;
  }
  else
  {
    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      ret = VB_COM_ERROR_LCMP_WAITNOTIFY;
      VbLogPrint(VB_LOG_ERROR, "lcmp_err %d",lcmp_err);
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      if (list_received_values == NULL)
      {
        ret = VB_COM_ERROR_LCMP_WAITNOTIFY;
        VbLogPrint(VB_LOG_ERROR, "ReceivedValues 0x%X",list_received_values);
      }
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      list_idx = 0;
      received_values = list_received_values->head;

      while ((list_idx < list_received_values->NumLists) && (received_values != NULL))
      {
        list_idx++;
        values_array =  received_values->HTLVsArray;

        for (value_idx = 0; (value_idx < values_array->NumValues) && (ret == VB_COM_ERROR_NONE); value_idx++)
        {
          if (values_array->values[value_idx].ValueLength >= sizeof(t_vectorboostTlvIngressTrafficInd))
          {
            vb_traffic_report = (t_vectorboostTlvIngressTrafficInd *)values_array->values[value_idx].Value;

            if (vb_traffic_report->notifyType == VB_INGRESS_TRAFFIC_IND)

            {
              INT16U                            *vb_bps_traffic_report = NULL;
              INT32U i;

              vb_bps_traffic_report = (INT16U*)((INT8U*)vb_traffic_report + sizeof(t_vectorboostTlvIngressTrafficInd));

              traffic_report.channelCapacity = _ntohs_ghn(vb_traffic_report->channelCapacity);
              traffic_report.desiredCapacity = _ntohs_ghn(vb_traffic_report->desiredChannelCapacity);
              traffic_report.effectiveCapacity = _ntohs_ghn(vb_traffic_report->effectiveChannelCapacity);
              traffic_report.maxBuff[0] = vb_traffic_report->maxBuffPrio0;
              traffic_report.maxBuff[1] = vb_traffic_report->maxBuffPrio1;
              traffic_report.maxBuff[2] = vb_traffic_report->maxBuffPrio2;
              traffic_report.maxBuff[3] = vb_traffic_report->maxBuffPrio3;
              traffic_report.traffic[0] = _ntohs_ghn(vb_traffic_report->trafficPrio0);
              traffic_report.traffic[1] = _ntohs_ghn(vb_traffic_report->trafficPrio1);
              traffic_report.traffic[2] = _ntohs_ghn(vb_traffic_report->trafficPrio2);
              traffic_report.traffic[3] = _ntohs_ghn(vb_traffic_report->trafficPrio3);
              traffic_report.macEff = vb_traffic_report->macEfficiency;

              bps_traffic_report.nBands = _ntohs_ghn(*vb_bps_traffic_report);
              vb_bps_traffic_report++;
              for(i=0; (i < bps_traffic_report.nBands) && (i< VB_PSD_NUM_BANDS); i++)
              {
                bps_traffic_report.bpsBand[i] = _ntohs_ghn(*vb_bps_traffic_report);
                vb_bps_traffic_report++;
              }

              if(bps_traffic_report.nBands > VB_PSD_NUM_BANDS)
              {
                CHAR  mac_str[MAC_STR_LEN];

                MACAddrMem2str(mac_str, received_values->srcMAC);
                VbLogPrint(VB_LOG_ERROR, "Num Bands from MAC %s, nBands %u", mac_str, bps_traffic_report.nBands);
              }

              ret = VbDatamodelTrafficReportAdd(received_values->srcMAC, &traffic_report, &bps_traffic_report);

              if (ret != VB_COM_ERROR_NONE)
              {
                CHAR  mac_str[MAC_STR_LEN];

                MACAddrMem2str(mac_str, received_values->srcMAC);
                VbLogPrint(VB_LOG_ERROR, "com_err %d from MAC %s", ret, mac_str);

                if (ret != VB_COM_ERROR_NODEVICE)
                {
                  ret = VB_COM_ERROR_DATAMODEL;
                }
              }
            }
          }
        }
        received_values = received_values->nextList;
      }
    }
  }

  //Free memory
  VbDatamodelHtlvsListValueDestroy(&list_received_values);
  return ret;
}

/*******************************************************************/

/**
 * @brief Checks if all domains are sending traffic reports periodically
 * @return @ref t_VB_comErrorCode
 **/
static t_VB_comErrorCode VbTrafficTimeStampCheck(t_node *node, void *args)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  struct timespec   last_rx_ts;
  struct timespec   current_ts;
  INT64S            elapsed_time_ms;

  if (node == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  // Check timestamp only for nodes supporting periodic traffic reports
  if ((ret == VB_COM_ERROR_NONE) && (node->cap.trafficReports == TRUE))
  {
    // Get timestamp and current time
    last_rx_ts = node->ingressTraffic.lastRxTimeStamp;
    clock_gettime(CLOCK_MONOTONIC, &current_ts);

    // Check elapsed time
    elapsed_time_ms = VbUtilElapsetimeTimespecMs(last_rx_ts, current_ts);

    if (elapsed_time_ms >= VB_TRAFFIC_REPORT_TO)
    {
      // At least one node without expected traffic reports found!
      ret = VB_COM_ERROR_RECEIVE_TIMEOUT;

      VbLogPrint(VB_LOG_INFO, "Node %s timeout waiting for traffic reports (err %d)", node->MACStr, ret);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VBTrafficReportPeriodicCheck(void)
{
  t_VB_comErrorCode     ret;

  ret = VbDatamodelActiveNodeLoop(VbTrafficTimeStampCheck, TRUE, NULL);
  if (ret == VB_COM_ERROR_RECEIVE_TIMEOUT)
  {
    VbLogPrint(VB_LOG_INFO, "Periodic traffic report check fails -> reconfiguring traffic awareness (err %d)", ret);

    // At least one active node is not sending traffic reports, reconfigure traffic awareness
    ret = VBTrafficAllDomainsConf();
  }

  return ret;
}

/*******************************************************************/

static void VBTrafficTimer(sigval_t sigval)
{
  t_VB_comErrorCode  ret;
  t_vbEAMsg         *msg = NULL;

  // Check how many have sent traffic report
  ret = VbDatamodelGetAllTrafficReports(&msg);

  if(ret == VB_COM_ERROR_NONE)
  {
    // Send frame and free memory
    VbEAMsgTrafficAwarenessTrgSend(&msg);
  }
  else
  {
    // Always release memory
     VbEAMsgFree(&msg);
  }
}

/*******************************************************************/

static void *VBTrafficProcess(void *arg)
{
  t_VB_comErrorCode    com_err;
  t_HGF_LCMP_ErrorCode error_lcmp;
  BOOLEAN              run = TRUE;
  mqd_t                vb_main_queue;
  timer_t              timerId = 0;

  vb_main_queue = mq_open(VBQUEUENAME,O_WRONLY );

  // Configure traffic awareness in all domains
  com_err = VBTrafficAllDomainsConf();

  if (com_err != VB_COM_ERROR_NONE)
  {
    VbMainQEvSend(DRIVER_EV_TRAFFIC_KO, vb_main_queue, NULL);
  }
  else
  {
    error_lcmp = VbLcmpWaitNotifyPermanentInstall(VB_INGRESS_TRAFFIC_IND, &vbTafficCallbackInstalled);

    if (error_lcmp != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d installing callback to receive IngressTraffic.ind", error_lcmp);
      com_err = VB_COM_ERROR_LCMP;
    }

    if (com_err == VB_COM_ERROR_NONE)
    {
      INT32S timer_err;

      timer_err = TimerPeriodicTaskSet(VB_TRAFFIC_TIMER_NAME, VB_TRAFFIC_REPORT_TO_ENGINE_PERIOD, VBTrafficTimer, NULL, &timerId);

      if (timer_err != 0)
      {
        VbLogPrint(VB_LOG_ERROR, "Error %d creating a timer", timer_err);
        com_err = VB_COM_ERROR_THREAD;
      }
    }

    if (com_err == VB_COM_ERROR_NONE)
    {
      while ((VBTrafficStateGet() == TRUE) && (run == TRUE))
      {
        // Wait for traffic reports
        com_err = VBTrafficReportWait(vbTafficCallbackInstalled);

        if (VBTrafficStateGet() == TRUE)
        {
          if (com_err == VB_COM_ERROR_NONE)
          {
            // Check that all domains are sending periodic traffic reports
            com_err = VBTrafficReportPeriodicCheck();

            if (com_err != VB_COM_ERROR_NONE)
            {
              VbLogPrint(VB_LOG_ERROR, "Periodic traffic report check fails (err %d)", com_err);
            }
          }

          if (com_err != VB_COM_ERROR_NONE)
          {
            VbMainQEvSend(DRIVER_EV_TRAFFIC_KO, vb_main_queue, NULL);
            run = FALSE;
          }
        }
      }

      if (vbTafficCallbackInstalled != NULL)
      {
        error_lcmp = VbLcmpWaitNotifyPermanentUninstall(&vbTafficCallbackInstalled);

        if (error_lcmp != HGF_LCMP_ERROR_NONE)
        {
          VbLogPrint(VB_LOG_ERROR, "Uninstall permanent callback error: %d ", error_lcmp);
        }
      }

      TimerTaskDelete(timerId, VB_TRAFFIC_TIMER_NAME);
    }
    else
    {
      VbMainQEvSend(DRIVER_EV_TRAFFIC_KO, vb_main_queue, NULL);
    }
  }

  mq_close(vb_main_queue);

  vbTafficCallbackInstalled = NULL;

  return NULL;
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

void VbTrafficAwarenessRun(void)
{
  INT32U num_domains = VbDatamodelNmDomainsActiveGet(TRUE);

  VbTrafficAwarenessStop();

  // Only start traffic awareness when number of domains is > 0
  if (num_domains > 0)
  {
    VbLogPrint(VB_LOG_INFO, "Starting %s thread", VB_TRAFFIC_THREAD_NAME);

    VBTrafficStateSet(TRUE);
    if (FALSE == VbThreadCreate(VB_TRAFFIC_THREAD_NAME, VBTrafficProcess, NULL, VB_DRIVER_TRAFFIC_THREAD_PRIORITY, &vbTrafficThread))
    {
      VbLogPrint(VB_LOG_ERROR,"Can't create %s thread", VB_TRAFFIC_THREAD_NAME);
      VBTrafficStateSet(FALSE);
    }
  }
}

/*******************************************************************/

void VbTrafficAwarenessStop()
{
  if (VBTrafficStateGet() == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", VB_TRAFFIC_THREAD_NAME);

    VBTrafficStateSet(FALSE);

    if(vbTafficCallbackInstalled != NULL)
    {
      VbThreadCondWakeUp(&(vbTafficCallbackInstalled->CallbackData.mutex), &(vbTafficCallbackInstalled->CallbackData.condition));
    }

    VbThreadJoin(vbTrafficThread, VB_TRAFFIC_THREAD_NAME);

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", VB_TRAFFIC_THREAD_NAME);
  }
}

/*******************************************************************/

void VbTrafficInit(void)
{
  vbTrafficThread = 0;
  VBTrafficStateSet(FALSE);
  vbTrafficReportPeriod    = VbDriverConfTrafficReportPeriodGet();
  vbTrafficReportMeasWin   = VbDriverConfTrafficReportMeasWinGet();
  vbTrafficReportThreshold = VbDriverConfTrafficReportThresholdGet();
}

/*******************************************************************/

BOOLEAN VBTrafficStateGet(void)
{
  return vbTrafficThreadRunning;
}

/*******************************************************************/

t_VB_comErrorCode VBTrafficAllDomainsConf(void)
{
  t_VB_comErrorCode     ret = VB_COM_ERROR_NONE;
  t_vbTrafficConfParams traffic_params;
  INT32U                num_attempt = 0;
  t_Transmision         tx_type = MULTICAST_DMS;

  if (VbDatamodelNmDomainsAliveGet() > 0)
  {
    // Only try to configure traffic awareness if there is any domain
    VbLogPrint(VB_LOG_INFO, "Configuring traffic awareness");

    // Configure traffic monitor in modems
    traffic_params.measWin = vbTrafficReportMeasWin;
    traffic_params.threshold = vbTrafficReportThreshold;
    traffic_params.timeout = VB_TRAFFIC_PARAM_TIMEOUT;
    traffic_params.period = vbTrafficReportPeriod;

    if (VB_TRAFFIC_REPORT_ALL_NODES == TRUE)
    {
      tx_type = MULTICAST;
    }

    do
    {
      ret = VBTrafficMonitorConf(tx_type, NULL, &traffic_params);
    } while ((ret != VB_COM_ERROR_NONE) && (num_attempt++ < VB_TRAFFIC_NUM_ATTEMPT_CONF));

    if (ret != VB_COM_ERROR_NONE)
    {
      ret = VB_COM_ERROR_NOT_CONF;
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      // Update time stamp to check if nodes send traffic reports
      ret = VbDatamodelAllDomainsTrafficTimeStampUpdate();
    }
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
