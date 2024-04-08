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
 * @file vb_EA_interface.c
 * @brief External Agent interface
 *
 * @internal
 *
 * @author
 * @date 30/01/2015
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "types.h"

#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <mqueue.h>
#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_DataModel.h"
#include "vb_LCMP_com.h"
#include "vb_log.h"
#include "vb_console.h"
#include "vb_measurement.h"
#include "vb_EA_interface.h"
#include "vb_counters.h"
#include "vb_ea_communication.h"
#include "vb_main.h"
#include "vb_main_timer.h"
#include "vb_alignment.h"
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

#define SERVER_THREAD_NAME                          ("EAServer")
#define CONN_THREAD_NAME                            ("EAConnection")

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_vbEADesc      vbEAServerDesc;
static t_vbEADesc      vbEAConnDesc = { 0 };
static BOOL            vbEAServerMode;

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

static void VbEAVersionRspSend(t_vbEADesc *desc)
{
  t_VB_comErrorCode      err = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;
  INT32U                 payload_len;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if (desc == NULL)
    {
      err = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    payload_len = sizeof(t_vbEAVersionRsp);

    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_VERSION_RESP);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_vbEAVersionRsp payload;

    // Fill payload
    strncpy(payload.version, VERSION, VB_EA_VERSION_MAX_SIZE);
    // Force the null byte in last position
    payload.version[VB_EA_VERSION_MAX_SIZE - 1] = '\0';

    strncpy(payload.driverId, VbDriverConfDriverIdGet(), VB_EA_DRIVER_ID_MAX_SIZE);
    // Force the null byte in last position
    payload.driverId[VB_EA_DRIVER_ID_MAX_SIZE - 1] = '\0';

    // Multicast related information in order to adjust engine wait time with driver wait time
    payload.lcmpMcastTimeOut  = _htonl(VbDriverConfLcmpDefaultTimeoutGet());
    payload.lcmpMcastNAttempt = _htonl(VbDriverConfLcmpDefaultNAttemptGet());

    memcpy(msg->eaPayload.msg, &payload, sizeof(t_vbEAVersionRsp));

    // Send frame
    VbEAMsgSend(desc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);
}

/*******************************************************************/

static void VbEADomainRspSend(t_vbEADesc *desc)
{
  t_VB_comErrorCode  result = VB_COM_ERROR_NONE;
  t_vbEAError        ea_err;
  INT16U             num_domains_check = 0;
  INT16U             num_eps_check = 0;
  INT16U             num_domains;
  INT16U             memory_used = 0;
  t_DomainData      *domain_data = NULL;
  INT16U             num_eps;
  t_EpData          *ep_data = NULL;
  INT16U             payload_len;
  t_vbEAMsg         *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    result = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    if (desc == NULL)
    {
      result = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    payload_len = 2;

    result = VbDatamodelDomainsDataGet( &num_domains, &domain_data, TRUE, TRUE);

    if ((result == VB_COM_ERROR_NONE) && (num_domains > 0))
    {
      for(num_domains_check = 0 ; num_domains_check < num_domains ; num_domains_check++)
      {
        payload_len += VB_EA_DOMAINS_FRAME_DM_SIZE + (domain_data[num_domains_check].numEps * VB_EA_DOMAINS_FRAME_EP_SIZE);
      }

      // Allocate message
      ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_DOMAIN_RESP);

      if (ea_err != VB_EA_ERR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
        result = VB_COM_ERROR_MALLOC;
      }

      if (result == VB_COM_ERROR_NONE)
      {
        t_VB_comErrorCode  domain_err;
        t_vbEADomainRspDM *vb_gui_domain_rsp_payload_dm;
        t_vbEADomainRspEP *vb_gui_domain_rsp_payload_ep;
        INT8U             *buffer_domain;
        INT16U            *ptrn_nm_domains;

        ptrn_nm_domains = (INT16U *)(msg->eaPayload.msg);
        *ptrn_nm_domains = _htons(num_domains);
        memory_used += 2;

        buffer_domain = &((msg->eaPayload.msg)[VB_EA_DOMAINSFRAME_DOMAINS_OFFSET]);
        num_domains_check = 0;

        while ((num_domains_check < num_domains) && (memory_used <= msg->eaPayload.msgLen))
        {
          vb_gui_domain_rsp_payload_dm = (t_vbEADomainRspDM *)buffer_domain;

          domain_err = VbDatamodelEpsDataGet( domain_data[num_domains_check].mac, &num_eps, &ep_data );

          if (domain_err == VB_COM_ERROR_NONE)
          {
            memcpy(vb_gui_domain_rsp_payload_dm->DM_MAC, domain_data[num_domains_check].mac, ETH_ALEN);
            memcpy(vb_gui_domain_rsp_payload_dm->fwVersion, domain_data[num_domains_check].fwVersion, VB_FW_VERSION_LENGTH);
            vb_gui_domain_rsp_payload_dm->DM_ID= domain_data[num_domains_check].devID;
            vb_gui_domain_rsp_payload_dm->DM_Extseed =  _htons(domain_data[num_domains_check].extSeed);
            vb_gui_domain_rsp_payload_dm->qosRate =  _htons(domain_data[num_domains_check].qosRate);
            vb_gui_domain_rsp_payload_dm->maxLengthTxop =  _htons(domain_data[num_domains_check].maxLengthTxop);
            memory_used += VB_EA_DOMAINS_FRAME_DM_SIZE;

            vb_gui_domain_rsp_payload_dm->NumEps =  _htons(num_eps);

            buffer_domain += VB_EA_DOMAINS_FRAME_DM_SIZE;

            if ((num_eps > 0) && (ep_data != NULL))
            {
              vb_gui_domain_rsp_payload_ep = (t_vbEADomainRspEP *)buffer_domain;

              for (num_eps_check = 0 ; num_eps_check < num_eps; num_eps_check++ )
              {
                memory_used += VB_EA_DOMAINS_FRAME_EP_SIZE;
                memcpy(vb_gui_domain_rsp_payload_ep->EP_MAC, ep_data[num_eps_check].mac, ETH_ALEN);
                memcpy(vb_gui_domain_rsp_payload_ep->fwVersion, ep_data[num_eps_check].fwVersion, VB_FW_VERSION_LENGTH);
                vb_gui_domain_rsp_payload_ep->EP_ID = ep_data[num_eps_check].devID;
                buffer_domain += VB_EA_DOMAINS_FRAME_EP_SIZE;
              }
            }
          }

          num_domains_check++;

          if (ep_data != NULL)
          {
            free(ep_data);
            ep_data = NULL;
          }
        }
      }
    }
    else
    {
      // Allocate message
      ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_DOMAIN_RESP);

      if (ea_err != VB_EA_ERR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
        result = VB_COM_ERROR_MALLOC;
      }

      if (result == VB_COM_ERROR_NONE)
      {
        *((INT16U *)(msg->eaPayload.msg)) = 0;
      }
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    // Send frame
    VbEAMsgSend(desc, msg);
  }

  // Always release allocated memory
  VbEAMsgFree(&msg);

  if (domain_data != NULL)
  {
    free(domain_data);
    domain_data = NULL;
  }
}

/*******************************************************************/

static void VbEAClockRspSend(t_vbEADesc *desc)
{
  t_VB_comErrorCode err = VB_COM_ERROR_NONE;
  t_vbEAError       ea_err;
  t_vbEAMsg        *msg = NULL;
  INT16U            curr_seq_num;
  BOOLEAN           valid_seq_num;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if (desc == NULL)
    {
      err = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Get current sequence number
    err = VbAlignmentSeqNumGet(&curr_seq_num, &valid_seq_num);
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Allocate message structure and init header
    ea_err = VbEAMsgAlloc(&msg, VB_EA_CLOCK_RSP_SIZE, VB_EA_OPCODE_CLOCK_RSP);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending EAClock.rsp message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_vbEAClockRsp    *clock_rsp_frame;
    static struct timespec current_time;

    clock_rsp_frame = (t_vbEAClockRsp *)msg->eaPayload.msg;

    clock_gettime(CLOCK_REALTIME, &current_time);
    clock_rsp_frame->tvSec = _htonl((INT32U)current_time.tv_sec);
    clock_rsp_frame->tvNsec = _htonl((INT32U)current_time.tv_nsec);
    clock_rsp_frame->seqNumber = _htons(curr_seq_num);
    clock_rsp_frame->validSeqNum = valid_seq_num;

    VbEAMsgSend(desc, msg);
  }

  // Always release msg structure
  VbEAMsgFree(&msg);
}

/*******************************************************************/

static void VbEASocketAliveRspSend(t_vbEADesc *desc, INT8U *pld)
{
  t_VB_comErrorCode err = VB_COM_ERROR_NONE;
  t_vbEAError       ea_err;
  t_vbEAMsg        *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if ((desc == NULL) || (pld == NULL))
    {
      err = VB_COM_ERROR_BAD_ARGS;
    }
    else
    {
      t_vbEASocketAliveReq     *socket_alive_req = (t_vbEASocketAliveReq *)pld;
      BOOL enable;
      INT32U period;
      INT32U n_lost_msg_thr;

      desc->socketAliveCounter = 0;

      enable         = socket_alive_req->enable;
      period         = _ntohl(socket_alive_req->period);
      n_lost_msg_thr = _ntohl(socket_alive_req->nLostMsg);

      if(enable == TRUE)
      {
        if(VbMainSocketAliveTimerStatusGet() == FALSE)
        {
          VbMainSocketAliveTimerRun(desc, period, n_lost_msg_thr);
        }
      }
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Allocate message structure and init header
    ea_err = VbEAMsgAlloc(&msg, 0, VB_EA_OPCODE_SOCKET_ALIVE_RESP);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending EASocketAlive.rsp message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    VbEAMsgSend(desc, msg);
  }

  // Always release msg structure
  VbEAMsgFree(&msg);
}

/*******************************************************************/

static void VbEARxMsgCb(t_vbEADesc *desc, INT8U *rxMsg, INT32U rxMsgLen)
{
  t_VB_comErrorCode  err = VB_COM_ERROR_NONE;
  t_vbEAMsg         *msg = NULL;

  if ((rxMsg == NULL) || (desc == NULL) || (desc->queueId == -1))
  {
    err = VB_COM_ERROR_BAD_ARGS;

    VbLogPrint(VB_LOG_ERROR, "Parameters Error (Frame ptr: 0x%p; desc ptr: 0x%p)", rxMsg, desc);
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if (desc->connected == FALSE)
    {
      err = VB_COM_ERROR_SOCKET_NOT_OPENED;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_vbEAError            ea_err;

    ea_err = VbEAMsgParse(&msg, rxMsg);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      err = VB_COM_ERROR_EA;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
#if (_VALGRIND_ == 1)
    ANNOTATE_HAPPENS_BEFORE(msg);
#endif

    err = VbMainQEAFrameEvSend(msg, desc->queueId, NULL);
  }

  if (err != VB_COM_ERROR_NONE)
  {
    // Release message if error
    VbEAMsgFree(&msg);
  }
}

/*******************************************************************/

static void VbEADisconnectCb(t_vbEADesc *desc)
{
  if (desc != NULL)
  {
    // Send event
    VbMainQEvSend(DRIVER_EV_DISCONNECT, desc->queueId, NULL);
  }
}

/*******************************************************************/

static t_vbEAError VbEAConnectCb(t_vbEADesc *desc, struct sockaddr_in6 clientAddr, INT32S sockFd)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    if(vbEAServerMode == TRUE)
    {
      // Stop previous connection
      VbEAThreadStop(&vbEAConnDesc);

      if (vbEAConnDesc.connected == FALSE)
      {
        // Configure connection description
        VbEADescInit(&vbEAConnDesc);
        vbEAConnDesc.type           = VB_EA_TYPE_SERVER_CONN;

        strncpy(vbEAConnDesc.thrName, CONN_THREAD_NAME, VB_EA_THREAD_NAME_LEN);
        vbEAConnDesc.thrName[VB_EA_THREAD_NAME_LEN - 1] = '\0';

        vbEAConnDesc.queueName          = VBQUEUENAME;
        vbEAConnDesc.closeCb            = VbEADisconnectCb;
        vbEAConnDesc.disconnectCb       = NULL;
        vbEAConnDesc.connectCb          = NULL;
        vbEAConnDesc.processRxMsgCb     = VbEARxMsgCb;
        vbEAConnDesc.clientAddr         = clientAddr;
        vbEAConnDesc.sockFd             = sockFd;
        vbEAConnDesc.socketAliveCounter = 0;

        // Start new connection thread
        VbEAThreadStart(&vbEAConnDesc);
      }
    }

    // Send event
    VbMainQEvSend(DRIVER_EV_CONNECT, desc->queueId, NULL);
  }

  return ret;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

void VbEAStart( void )
{
  t_vbEADesc *desc;

  desc = (vbEAServerMode == TRUE) ? &vbEAServerDesc : &vbEAConnDesc;

  VbEAThreadStart(desc);
}

/*******************************************************************/

void VbEAStop( void )
{
  VbEAThreadStop(&vbEAConnDesc);
  if(vbEAServerMode == TRUE)
  {
    VbEAThreadStop(&vbEAServerDesc);
  }
}

/*******************************************************************/

void VbEAInit( const char *ifeth, INT16U eaiPort, CHAR *remoteIp, BOOL serverMode, INT16U family)
{
  vbEAServerMode = serverMode;

  if(serverMode == TRUE)
  {
    // Configure server description
    VbEADescInit(&vbEAServerDesc);
    vbEAServerDesc.type           = VB_EA_TYPE_SERVER;

    strncpy(vbEAServerDesc.thrName, SERVER_THREAD_NAME, VB_EA_THREAD_NAME_LEN);
    vbEAServerDesc.thrName[VB_EA_THREAD_NAME_LEN - 1] = '\0';

    vbEAServerDesc.queueName      = VBQUEUENAME;
    vbEAServerDesc.closeCb        = NULL;
    vbEAServerDesc.disconnectCb   = NULL;
    vbEAServerDesc.connectCb      = VbEAConnectCb;
    vbEAServerDesc.processRxMsgCb = NULL;
    strcpy((char *)(vbEAServerDesc.iface), ifeth);
    // in6addr_any allows connections from both IPv4 and IPv6 clients
    VbEAServerAddrSet(&vbEAServerDesc, in6addr_any, eaiPort);
  }
  else
  {
    // Configure client description
    VbEADescInit(&vbEAConnDesc);
    vbEAConnDesc.type           = VB_EA_TYPE_CLIENT;

    strncpy(vbEAConnDesc.thrName, CONN_THREAD_NAME, VB_EA_THREAD_NAME_LEN);
    vbEAConnDesc.thrName[VB_EA_THREAD_NAME_LEN - 1] = '\0';

    vbEAConnDesc.queueName      = VBQUEUENAME;
    vbEAConnDesc.closeCb        = VbEADisconnectCb;
    vbEAConnDesc.disconnectCb   = VbEADisconnectCb;
    vbEAConnDesc.connectCb      = VbEAConnectCb;
    vbEAConnDesc.processRxMsgCb = VbEARxMsgCb;
    vbEAConnDesc.args           = NULL;
    vbEAConnDesc.clientInfo     = NULL;

    VbEAClientAddrSet(&vbEAConnDesc, remoteIp, eaiPort,family);
  }
}

/*******************************************************************/

void VbEAFrameRxProcess(t_vbEAMsg *msg)
{
  t_VB_comErrorCode err = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    err = VB_COM_ERROR_BAD_ARGS;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    switch (msg->opcode)
    {
      case VB_EA_OPCODE_DOMAIN_REQ:
      {
        VbEADomainRspSend(&vbEAConnDesc);
        break;
      }
      case VB_EA_OPCODE_VERSION_REQ:
      {
        VbEAVersionRspSend(&vbEAConnDesc);
        break;
      }
      case VB_EA_OPCODE_VBDRIVER_STATE_REQ:
      {
        VbEAStateTrgSend(VbMainStateStrGet());
        break;
      }
      case VB_EA_OPCODE_CLOCK_REQ:
      {
        VbEAClockRspSend(&vbEAConnDesc);
        break;
      }

      case VB_EA_OPCODE_SOCKET_ALIVE_REQUEST:
      {
        VbEASocketAliveRspSend(&vbEAConnDesc, msg->eaPayload.msg);
        break;
      }

      default:
      {
        VbLogPrint(VB_LOG_ERROR, "Unexpected OPCODE 0x%02X", (INT32U)msg->opcode);
        break;
      }
    }
  }
}

/*******************************************************************/

t_VB_comErrorCode VbEATrafficAwarenessMultiFrameCreate(INT32U numReports, INT32U numBands, t_vbEAMsg **msg)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  INT32U                 payload_size;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if ((numReports == 0) || (msg == NULL))
    {
      ret = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    payload_size = numReports * (VB_EA_TRAFFIC_REPORT_RSP_SIZE + sizeof(INT16U)) + numBands*sizeof(INT16U) + VB_EA_TRAFFIC_REPORT_HDR_SIZE;

    // Allocate msg
    ea_err = VbEAMsgAlloc(msg, payload_size, VB_EA_OPCODE_TRAFFIC_AWARENESS_TRG);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      ret = VB_COM_ERROR_MALLOC;
    }
    else
    {
      t_vbEATrafficReportHdr *hdr = (t_vbEATrafficReportHdr *)((*msg)->eaPayload.msg);

      hdr->numReports = _htonl(numReports);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbEATrafficAwarenessFillMsg(const INT8U *macAddr, t_IngressTraffic *report, INT8U *msg)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if ((macAddr == NULL) || (report == NULL))
    {
      ret = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEATrafficReportRsp *traffic_trg;
    t_vbEATrafficBpsReport *bps_report;
    INT32U                  i;

    // Build the payload
    traffic_trg = (t_vbEATrafficReportRsp *)msg;
    MACAddrClone(traffic_trg->MAC, macAddr);
    traffic_trg->trafficPrio0 = _htons(report->traffic[0]);
    traffic_trg->trafficPrio1 = _htons(report->traffic[1]);
    traffic_trg->trafficPrio2 = _htons(report->traffic[2]);
    traffic_trg->trafficPrio3 = _htons(report->traffic[3]);
    traffic_trg->maxBuffPrio0 = report->maxBuff[0];
    traffic_trg->maxBuffPrio1 = report->maxBuff[1];
    traffic_trg->maxBuffPrio2 = report->maxBuff[2];
    traffic_trg->maxBuffPrio3 = report->maxBuff[3];
    traffic_trg->bpsCapacity = _htons(report->channelCapacity);
    traffic_trg->realCapacity = _htons(report->effectiveCapacity);
    traffic_trg->neededTheoricCapacity = _htons(report->desiredCapacity);
    traffic_trg->macEfficiency = report->macEff;

    bps_report = (t_vbEATrafficBpsReport *)((INT8U*)msg + sizeof(t_vbEATrafficReportRsp));
    bps_report->nBands = _htons(report->bpsBandsInfo.nBands);
    for(i = 0; i < report->bpsBandsInfo.nBands; i++)
    {
      bps_report->bpsBand[i] = _htons(report->bpsBandsInfo.bpsBand[i]);
    }
  }

  return ret;
}

/*******************************************************************/

void VbEAMsgTrafficAwarenessTrgSend(t_vbEAMsg **msg)
{
  t_vbEAError err;

  // Send frame
  err = VbEAMsgSend(&vbEAConnDesc, *msg);

  if(err != VB_EA_ERR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%d) sending Traffic report msg", err);
  }

  // Always release memory
  VbEAMsgFree(msg);
}

/*******************************************************************/

t_VB_comErrorCode VbEAPsdShapeCnfSend(INT8U writeStatus)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, VB_EA_PSD_SHAPING_CNF_SIZE, VB_EA_OPCODE_PSD_SHAPE_CFM);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEAPsdShapingCnf *psdShaping_pld;

    // Build the payload
    psdShaping_pld = (t_vbEAPsdShapingCnf *)msg->eaPayload.msg;
    psdShaping_pld->status = writeStatus;

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbEACdtaCnfSend(INT8U writeStatus)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, VB_EA_CDTA_CNF_SIZE, VB_EA_OPCODE_CDTA_CFM);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEACdtaCnf *cdta_pld;

    // Build the payload
    cdta_pld = (t_vbEACdtaCnf *)msg->eaPayload.msg;
    cdta_pld->status = writeStatus;

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return ret;
}


/*******************************************************************/

t_VB_comErrorCode VbEACfrRspSend(const INT8U *macMeasurer, const INT8U *macMeasured, INT8U planId, t_processMeasure *processMeasure)
{
  t_VB_comErrorCode      err = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMeasRspErrorCode rsp_err_code;
  t_vbEAMsg             *msg = NULL;
  INT32U                 payload_len;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if ((macMeasurer == NULL) || (macMeasured == NULL))
    {
      err = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_VB_comErrorCode      meas_err = VB_COM_ERROR_NONE;

    // Check measure result

    if (meas_err == VB_COM_ERROR_NODEVICE)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_MAC;
    }
    else if (meas_err == VB_COM_ERROR_NOCROSMEASUREDEVICE)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_CROSSMAC;
    }
    else if (processMeasure == NULL)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_MEASURE;
    }
    else if (processMeasure->planID != planId)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_PLANID;
    }
    else if (processMeasure->measures == NULL)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_MEASURE;
    }
    else
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_NONE;
    }

    // Calculate payload length
    payload_len = VB_EA_CFRFRAME_MEASURE_HEADER_SIZE;

    if (rsp_err_code == VB_EA_MEAS_RSP_ERR_NONE)
    {
      // Add measures size
      payload_len += processMeasure->numMeasures;
    }

    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_CFR_RESP);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_vbEACFRMeasure *cfr_measure_hdr;

    cfr_measure_hdr = (t_vbEACFRMeasure *)msg->eaPayload.msg;

    // Init header values
    bzero(cfr_measure_hdr, VB_EA_CFRFRAME_MEASURE_HEADER_SIZE);

    // Fill common parameters
    MACAddrClone(cfr_measure_hdr->MACMeasurer, macMeasurer);
    MACAddrClone(cfr_measure_hdr->MACMeasured, macMeasured);
    cfr_measure_hdr->ErrorCode = rsp_err_code;

    if (rsp_err_code == VB_EA_MEAS_RSP_ERR_NONE)
    {
      // Fill remaining measure parameters

      INT8U     *buffer_payload_measure;
      INT16U     i;

      cfr_measure_hdr->numCarriers      = _htons(processMeasure->numMeasures);
      cfr_measure_hdr->firstCarrier     = _htons(processMeasure->firstCarrier);
      cfr_measure_hdr->spacing          = processMeasure->spacing;
      cfr_measure_hdr->flags            = processMeasure->flags;
      cfr_measure_hdr->planId           = planId;
      cfr_measure_hdr->rxg1Compensation = processMeasure->rxg1Compensation;
      cfr_measure_hdr->rxg2Compensation = processMeasure->rxg2Compensation;
      cfr_measure_hdr->mimoInd          = processMeasure->mimoInd;
      cfr_measure_hdr->mimoMeas         = processMeasure->mimoMeas;

      buffer_payload_measure = msg->eaPayload.msg + VB_EA_CFRFRAME_MEASURE_HEADER_SIZE;

      for( i = 0 ; i < processMeasure->numMeasures ; i++ )
      {
        buffer_payload_measure[i] = processMeasure->measures[i];
      }
    }

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return VB_COM_ERROR_NONE;
}

/*******************************************************************/

t_VB_comErrorCode VbEABgnRspSend(const INT8U *mac, INT8U planId, t_processMeasure *processMeasure)
{
  t_VB_comErrorCode      err = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMeasRspErrorCode rsp_err_code;
  t_vbEAMsg             *msg = NULL;
  INT32U                 payload_len;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if (mac == NULL)
    {
      err = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if (processMeasure == NULL)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_MEASURE;
    }
    else if (processMeasure->planID != planId)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_PLANID;
    }
    else if (processMeasure->measures == NULL)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_MEASURE;
    }
    else if (processMeasure->errorCode != 0)
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_MEASURE;
    }
    else
    {
      rsp_err_code = VB_EA_MEAS_RSP_ERR_NONE;
    }

    // Calculate payload length
    payload_len = VB_EA_BGNFRAME_MEASURE_HEADER_SIZE;

    if (rsp_err_code == VB_EA_MEAS_RSP_ERR_NONE)
    {
      // Add measures size
      payload_len += processMeasure->numMeasures;
    }

    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_BGN_RESP);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_vbEABGNMeasure      *bgn_measure_hdr;

    bgn_measure_hdr = (t_vbEABGNMeasure *)msg->eaPayload.msg;

    // Init header values
    bzero(bgn_measure_hdr, VB_EA_BGNFRAME_MEASURE_HEADER_SIZE);

    // Fill common parameters
    MACAddrClone(bgn_measure_hdr->MAC, mac);
    bgn_measure_hdr->ErrorCode = rsp_err_code;

    if (rsp_err_code == VB_EA_MEAS_RSP_ERR_NONE)
    {
      // Fill remaining measure parameters

      INT8U     *buffer_payload_measure;
      INT16U     i;

      bgn_measure_hdr->numCarriers      = _htons(processMeasure->numMeasures);
      bgn_measure_hdr->firstCarrier     = _htons(processMeasure->firstCarrier);
      bgn_measure_hdr->spacing          = processMeasure->spacing;
      bgn_measure_hdr->flags            = processMeasure->flags;
      bgn_measure_hdr->rxg1Compensation = processMeasure->rxg1Compensation;
      bgn_measure_hdr->rxg2Compensation = processMeasure->rxg2Compensation;
      bgn_measure_hdr->mimoInd          = processMeasure->mimoInd;
      bgn_measure_hdr->mimoMeas         = processMeasure->mimoMeas;
      bgn_measure_hdr->planId           = planId;

      buffer_payload_measure = msg->eaPayload.msg + VB_EA_BGNFRAME_MEASURE_HEADER_SIZE;

      for( i = 0 ; i < processMeasure->numMeasures ; i++ )
      {
        buffer_payload_measure[i] = processMeasure->measures[i];
      }
    }

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return VB_COM_ERROR_NONE;
}

/*******************************************************************/

t_VB_comErrorCode VbEASnrProbesRspSend(const INT8U *mac, t_processMeasure *processMeasure)
{
  t_VB_comErrorCode      err = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMeasRspErrorCode rsp_err_code = VB_EA_MEAS_RSP_ERR_NONE;
  t_vbEAMsg             *msg = NULL;
  INT32U                 payload_len;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    if ((mac == NULL) || (processMeasure == NULL))
    {
      err = VB_COM_ERROR_BAD_ARGS;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Calculate payload length
    payload_len = VB_EA_SNRPROBE_MEASURE_HEADER_SIZE;

    // Add measures size
    payload_len += processMeasure->numMeasures;

    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_SNRPROBES_RESP);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_vbEASNRMeasure      *snrprobe_measure_hdr;

    snrprobe_measure_hdr = (t_vbEASNRMeasure *)msg->eaPayload.msg;

    // Init header values
    bzero(snrprobe_measure_hdr, VB_EA_SNRPROBE_MEASURE_HEADER_SIZE);

    // Fill common parameters
    MACAddrClone(snrprobe_measure_hdr->MAC, mac);
    snrprobe_measure_hdr->ErrorCode = VB_EA_MEAS_RSP_ERR_NONE;

    if (rsp_err_code == VB_EA_MEAS_RSP_ERR_NONE)
    {
      // Fill remaining measure parameters

      INT8U     *buffer_payload_measure;
      INT16U     i;

      snrprobe_measure_hdr->numCarriers      = _htons(processMeasure->numMeasures);
      snrprobe_measure_hdr->firstCarrier     = _htons(processMeasure->firstCarrier);
      snrprobe_measure_hdr->spacing          = processMeasure->spacing;
      snrprobe_measure_hdr->flags            = processMeasure->flags;
      snrprobe_measure_hdr->rxg1Compensation = processMeasure->rxg1Compensation;
      snrprobe_measure_hdr->rxg2Compensation = processMeasure->rxg2Compensation;
      snrprobe_measure_hdr->mimoInd          = processMeasure->mimoInd;
      snrprobe_measure_hdr->mimoMeas         = processMeasure->mimoMeas;

      buffer_payload_measure = msg->eaPayload.msg + VB_EA_SNRPROBE_MEASURE_HEADER_SIZE;

      for( i = 0 ; i < processMeasure->numMeasures ; i++ )
      {
        buffer_payload_measure[i] = processMeasure->measures[i];
      }
    }

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return err;
}

/*******************************************************************/

t_VB_comErrorCode VbEAMeasCollectionEndSend(INT8U planId)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;
  t_vbEAMeasureCollectEnd *measDone;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, VB_EA_MEAS_COLLECT_END_TRG_SIZE, VB_EA_OPCODE_MEAS_COLLECT_END);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    measDone = (t_vbEAMeasureCollectEnd *)msg->eaPayload.msg;
    measDone->PlanID = planId;

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);

    VbLogPrint(VB_LOG_INFO, "CFRs collection end sent (err %d)", ret);
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbEANetworkChangeReportTrgSend(void)
{

  t_VB_comErrorCode  result = VB_COM_ERROR_NONE;
  t_vbEAError        ea_err;
  INT8U              report_type;
  INT16U             num_DM_added;
  INT16U             num_EP_added;
  INT16U             num_DM_Rem;
  INT16U             num_EP_Rem;
  INT16U             payload_len;
  t_vbEAMsg         *msg = NULL;
  INT8U             *pld;

  if (VbEAEngineIsConnected() == FALSE)
  {
    result = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    result = VbDatamodelDomainsReportDataGet(&report_type, &num_DM_added, &num_EP_added, &num_DM_Rem, &num_EP_Rem, &pld);
    VbLogPrint(VB_LOG_INFO, "Domain Report type %d -(%d;%d;%d;%d)", report_type,
                                                                    num_DM_added, num_EP_added, num_DM_Rem, num_EP_Rem);

    if ( (result == VB_COM_ERROR_NONE) &&
         ( (((num_DM_added + num_EP_added + num_DM_Rem + num_EP_Rem)  > 0) && (report_type == VB_EA_DOMAIN_REPORT_DIFF)) ||
           (report_type == VB_EA_DOMAIN_REPORT_FULL)
         )
       )
    {
      payload_len = sizeof(t_vbEADomainDiffHdrRsp) +
                    sizeof(num_DM_added) + num_DM_added*sizeof(t_vbEADomainDiffRspDMAdded) +
                    sizeof(num_EP_added) + num_EP_added*sizeof(t_vbEADomainDiffRspEPAdded) +
                    sizeof(num_DM_Rem)   + num_DM_Rem*sizeof(t_vbEADomainDiffRspNodeRem)   +
                    sizeof(num_EP_Rem)   + num_EP_Rem*sizeof(t_vbEADomainDiffRspNodeRem);

      // Allocate message
      ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_NETWORK_CHANGE_REPORT);
      if (ea_err != VB_EA_ERR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
        result = VB_COM_ERROR_MALLOC;
      }

      if (result == VB_COM_ERROR_NONE)
      {
        INT8U                       *bufferResp;
        INT16U                      *ptrn_num;
        t_vbEADomainDiffRspDMAdded  *respDMAdded;
        t_vbEADomainDiffRspEPAdded  *respEPAdded;
        t_vbEADomainDiffRspNodeRem  *respNodeRem;
        INT32U                       i;

        // New DMs
        bufferResp = (INT8U *)(msg->eaPayload.msg);

        // Header content
        *bufferResp = report_type; //Diff
        bufferResp += sizeof(t_vbEADomainDiffHdrRsp);

        // Diffs content
        ptrn_num = (INT16U *)bufferResp;
        *ptrn_num = _htons(num_DM_added);
        bufferResp += 2;

        if(num_DM_added)
        {
          t_DomainData *domain_data;
          for(i=0; i<num_DM_added; i++)
          {
            INT16U maxLengthTxop;
            INT16U qosRate;

            domain_data = (t_DomainData *)pld;
            respDMAdded = (t_vbEADomainDiffRspDMAdded*)bufferResp;
            memcpy(respDMAdded->dmMAC, domain_data->mac, ETH_ALEN);
            VbDatamodelDMAddInfo1Read(domain_data->mac,
                                      respDMAdded->fwVersion,
                                      &qosRate,
                                      &maxLengthTxop);
            respDMAdded->qosRate = qosRate;
            respDMAdded->maxLengthTxop = maxLengthTxop;
            respDMAdded->dmDevId = domain_data->devID;
            respDMAdded->extSeed = domain_data->extSeed;
            respDMAdded->numEps = 0;

            bufferResp += sizeof(t_vbEADomainDiffRspDMAdded);
            pld        += sizeof(t_DomainData);
          }
        }

        // New EPs
        ptrn_num = (INT16U *)bufferResp;
        *ptrn_num = _htons(num_EP_added);
        bufferResp += 2;

        if(num_EP_added)
        {
          t_EpDiffData *ep_diff;

          for(i=0; i<num_EP_added; i++)
          {
            ep_diff = (t_EpDiffData *)pld;
            respEPAdded = (t_vbEADomainDiffRspEPAdded*)bufferResp;
            memcpy(respEPAdded->dmMAC, ep_diff->dmMac, ETH_ALEN);
            memcpy(respEPAdded->epMAC, ep_diff->epMac, ETH_ALEN);
            VbDatamodelEPAddInfo1Read(ep_diff->epMac,
                                      ep_diff->dmMac,
                                      respEPAdded->fwVersion);
            respEPAdded->epDevId = ep_diff->devId;

            bufferResp += sizeof(t_vbEADomainDiffRspEPAdded);
            pld        += sizeof(t_EpDiffData);
          }
        }

        // Rem DMs
        ptrn_num = (INT16U *)bufferResp;
        *ptrn_num = _htons(num_DM_Rem);
        bufferResp += 2;

        if(num_DM_Rem)
        {
          for(i=0; i<num_DM_Rem; i++)
          {
            respNodeRem = (t_vbEADomainDiffRspNodeRem*)bufferResp;
            memcpy(respNodeRem->mac, pld, ETH_ALEN);

            bufferResp += sizeof(t_vbEADomainDiffRspNodeRem);
            pld        += ETH_ALEN;
          }
        }

        // Rem EPs
        ptrn_num = (INT16U *)bufferResp;
        *ptrn_num = _htons(num_EP_Rem);
        bufferResp += 2;

        if(num_EP_Rem)
        {
          for(i=0; i<num_EP_Rem; i++)
          {
            respNodeRem = (t_vbEADomainDiffRspNodeRem*)bufferResp;
            memcpy(respNodeRem->mac, pld, ETH_ALEN);

            bufferResp += sizeof(t_vbEADomainDiffRspNodeRem);
            pld        += ETH_ALEN;
          }
        }

        VbDatamodelDomainsReportDataFree();
      }
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
  }

  // Always release allocated memory
  VbEAMsgFree(&msg);

  return result;
}


/*******************************************************************/

void VbEAStateTrgSend(const char *stateTEXT)
{
  t_VB_comErrorCode      err = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  INT32U                 payload_len;
  t_vbEAMsg             *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (stateTEXT == NULL)
  {
    err = VB_COM_ERROR_BAD_ARGS;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    payload_len = strlen(stateTEXT) + 1;

    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, payload_len, VB_EA_OPCODE_VBDRIVER_STATE_TRG);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Build payload
    strncpy((char *)msg->eaPayload.msg, stateTEXT, payload_len);

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);
}

/*******************************************************************/

BOOL VbEAEngineIsConnected(void)
{
  return (vbEAConnDesc.connected);
}

/*******************************************************************/

t_VB_comErrorCode VbEADriverFrameSend(t_vbEAMsg *msg)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (msg == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEAError ea_err;

    // Send frame
    ea_err = VbEAMsgSend(&vbEAConnDesc, msg);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending frame to engine", ea_err);
      ret = VB_COM_ERROR_EA;
    }
  }

  return ret;
}

/*******************************************************************/

t_vbEADesc *VbEAConnDescriptorGet(void)
{
  return &vbEAConnDesc;
}

/*******************************************************************/

t_VB_comErrorCode VbEAMeasurePlanErrorSend(INT8U planId, INT8U errorCode)
{
  t_VB_comErrorCode      err = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    err = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, VB_EA_MEASURE_END_TRG_SIZE, VB_EA_OPCODE_MEASURE_ERR_NOTIFY);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      err = VB_COM_ERROR_MALLOC;
    }
  }

  if (err == VB_COM_ERROR_NONE)
  {
    t_vbEAMeasureEndTrg *measureEndTrg;

    measureEndTrg = (t_vbEAMeasureEndTrg *)msg->eaPayload.msg;

    if (measureEndTrg != NULL)
    {
      measureEndTrg->planID = planId;
      measureEndTrg->errorCode = errorCode;

      VbEAMsgSend(&vbEAConnDesc, msg);
    }
    else
    {
      err = VB_COM_ERROR_MALLOC;
    }

    // Always release memory
    VbEAMsgFree(&msg);
  }

  return err;
}

/*******************************************************************/

t_VB_comErrorCode VbEAMeasurePlanRspSend(INT8U planId, INT8U errorCode)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEAMeasurePlanRsp *measurePlanCnf;
    ;
    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, VB_EA_MEASURE_PLAN_RSP_SIZE, VB_EA_OPCODE_MEASURE_PLAN_RESP);
    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      ret = VB_COM_ERROR_MALLOC;
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      measurePlanCnf = (t_vbEAMeasurePlanRsp *)msg->eaPayload.msg;
      measurePlanCnf->planID = planId;
      measurePlanCnf->errorCode = errorCode;

      ea_err = VbEAMsgSend(&vbEAConnDesc, msg);
      if (ea_err != VB_EA_ERR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error (%d) Sending EA msg", ea_err);
        ret = VB_COM_ERROR_EA;
      }
    }
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbEAMeasurePlanCancelCnfSend(INT8U planId, INT8U errorCode)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, VB_EA_MEASURE_PLAN_CANCEL_CNF_SIZE, VB_EA_OPCODE_MEASURE_PLAN_CANCEL_RSP);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEAMeasurePlanCancelCnf *measurePlanCancel_pld;

    // Build the payload
    measurePlanCancel_pld = (t_vbEAMeasurePlanCancelCnf *)msg->eaPayload.msg;
    measurePlanCancel_pld->planID = planId;
    measurePlanCancel_pld->status = errorCode;

    // Send frame
    VbEAMsgSend(&vbEAConnDesc, msg);
    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) Sending EA msg", ea_err);
      ret = VB_COM_ERROR_EA;
    }

  }

  // Always release memory
  VbEAMsgFree(&msg);

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbEAAlignSyncLostSend(const INT8U *reporterMac, INT8U syncDid)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;

  if (VbEAEngineIsConnected() == FALSE)
  {
    ret = VB_COM_ERROR_SOCKET_NOT_OPENED;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate msg
    ea_err = VbEAMsgAlloc(&msg, VB_EA_ALIGN_SYNC_LOST_TRG_SIZE, VB_EA_OPCODE_ALIGN_SYNC_LOST_TRG);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) allocating EA message", ea_err);
      ret = VB_COM_ERROR_MALLOC;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    t_vbEAAlignSyncLost *align_sync_lost_pld;

    // Build the payload
    align_sync_lost_pld = (t_vbEAAlignSyncLost *)msg->eaPayload.msg;

    MACAddrClone(align_sync_lost_pld->reporterMac, reporterMac);
    align_sync_lost_pld->syncDid = syncDid;

    // Send frame
    ea_err = VbEAMsgSend(&vbEAConnDesc, msg);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error (%d) sending EAAlignSyncLost.trg message", ea_err);
      ret = VB_COM_ERROR_EA;
    }
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return ret;
}

/*******************************************************************/

void VbEAConnDbgMsgDump(t_writeFun writeFun)
{
  pthread_mutex_lock(&(vbEAConnDesc.mutex));

  VbEADbgMsgDump(&(vbEAConnDesc.debugTable), writeFun);

  pthread_mutex_unlock(&(vbEAConnDesc.mutex));
}

/*******************************************************************/

void VbEAConnDbgMsgReset(void)
{
  pthread_mutex_lock(&(vbEAConnDesc.mutex));

  VbEADbgMsgReset(&(vbEAConnDesc.debugTable));

  pthread_mutex_unlock(&(vbEAConnDesc.mutex));
}

/*******************************************************************/

void VbEADestroy(void)
{
  VbEADescDestroy(&vbEAConnDesc);
}

/*******************************************************************/

/**
 * @}
 **/
