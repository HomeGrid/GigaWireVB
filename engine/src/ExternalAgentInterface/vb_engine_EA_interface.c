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

#include <string.h>
#include <stdio.h>

#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "vb_engine_datamodel.h"
#include "vb_engine_process.h"
#include "vb_log.h"
#include "vb_engine_EA_interface.h"
#include "vb_ea_communication.h"
#include "vb_engine_drivers_list.h"
#include "vb_counters.h"

/*
 ************************************************************************
 ** Public variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private constants and defines
 ************************************************************************
 */

#define ENGINE_EA_NEW_THREAD_STR                ("ea_driver_thread%u")
#define SERVER_THREAD_NAME                      ("EAServer")

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
static BOOL            vbEAServerMode;

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/**
 * @brief Function used to send the received frames to the engine process
 * param[in] desc Connection descriptor
 * param[in] frameRx Data buffer received
 * param[in] size Size of the data buffer
**/
static void VbEngineEAFrameRxCb(t_vbEADesc *desc, INT8U* frameRx, INT32U size);

/**
 * @brief Send a Connect event to engine process. Called from vb_EA_communication
 * @param[in] desc Connection descriptor
 * @param[in] driverAddr IP address of the connected driver
 * @param[in] sockFd Socket descriptor
 * @return @ref t_vbEAError
**/
static t_vbEAError VbEngineEAConnectCb(t_vbEADesc *desc, struct sockaddr_in6 driverAddr, INT32S sockFd);

/**
 * @brief Send a Disconnect event to engine process. Called from vb_EA_communication
 * @param[in] desc Connection descriptor
**/
static void VbEngineEADisconnectCb(t_vbEADesc *desc);

/**
 * @brief Used in server mode, to notify when the thread and all resources are ready to use.
 * Sends a connect event to the engine_process FSM
 * @param[in] desc Connection descriptor
 * @param[in] clientAddr Client IP address
 * @param[in] sockFd Socket file descriptor
 * @return @ref t_vbEAError
**/
static t_vbEAError VbEngineEADriverReadyCb(t_vbEADesc *desc, struct sockaddr_in6 clientAddr, INT32S sockFd);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VbEngineEAFrameRxCb(t_vbEADesc *desc, INT8U *frameRx, INT32U size)
{
  t_VB_engineErrorCode   error         = VB_ENGINE_ERROR_NONE;
  t_VBDriver            *this_driver;
  t_vbEAError            ea_err;
  t_vbEAMsg             *msg = NULL;

  if ((frameRx == NULL) || (desc == NULL) || ((desc != NULL) && ((desc->args == NULL) || (desc->queueId == -1))))
  {
    error = VB_ENGINE_ERROR_PARAMS;

    VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Parameters Error, Frame ptr: 0x%p desc ptr: 0x%p", frameRx, desc);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    if (desc->connected == FALSE)
    {
      error = VB_ENGINE_ERROR_NOT_READY;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    ea_err = VbEAMsgParse(&msg, frameRx);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error (%d) allocating EA message", ea_err);
      error = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    this_driver = (t_VBDriver *)desc->args;

#if (_VALGRIND_ == 1)
    ANNOTATE_HAPPENS_BEFORE(msg);
#endif

    error = VbEngineProcessEAFrameRx(msg, this_driver);
  }

  if (error != VB_ENGINE_ERROR_NONE)
  {
    // Release message if error
    VbEAMsgFree(&msg);
  }
}

/*******************************************************************/

static t_vbEAError VbEngineEAConnectCb(t_vbEADesc *desc, struct sockaddr_in6 driverAddr, INT32S sockFd)
{
  t_vbEAError          ret = VB_EA_ERR_NONE;
  t_VB_engineErrorCode engine_err = VB_ENGINE_ERROR_NONE;
  t_VBDriver          *this_driver = NULL;
  CHAR                 driverId[VB_EA_DRIVER_ID_MAX_SIZE];

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    if (vbEAServerMode == TRUE)
    {
      // Create driver struct
      engine_err = VbEngineDatamodelCreateDriver(NULL, &this_driver);

      if (engine_err == VB_ENGINE_ERROR_NONE)
      {
        // add Driver to the drivers list
        engine_err = VbEngineDrvListDriverAdd(this_driver);
      }

      if (engine_err == VB_ENGINE_ERROR_NONE)
      {
        // Set a default name based on the index inside the linked list. This index is unique
        sprintf(driverId, VB_ENGINE_DEFAULT_DRIVER_ID, (unsigned int)this_driver->l.index);

        VbEngineDatamodelDriverIdSet(driverId, this_driver, FALSE);

        // Configure connection description
#if (_VALGRIND_ == 1)
    ANNOTATE_HAPPENS_BEFORE(&this_driver->vbEAConnDesc);
#endif
        VbEADescInit(&this_driver->vbEAConnDesc);
        sprintf(this_driver->vbEAConnDesc.thrName, ENGINE_EA_NEW_THREAD_STR, (unsigned int)this_driver->l.index);
        this_driver->vbEAConnDesc.type           = VB_EA_TYPE_SERVER_CONN;
        this_driver->vbEAConnDesc.queueName      = VbEngineProcessQueueNameGet();
        this_driver->vbEAConnDesc.closeCb        = VbEngineEADisconnectCb;
        this_driver->vbEAConnDesc.disconnectCb   = NULL;
        this_driver->vbEAConnDesc.connectCb      = VbEngineEADriverReadyCb;
        this_driver->vbEAConnDesc.processRxMsgCb = VbEngineEAFrameRxCb;
        this_driver->vbEAConnDesc.threadStartCb  = NULL;
        this_driver->vbEAConnDesc.clientAddr     = driverAddr;
        this_driver->vbEAConnDesc.serverAddr     = desc->serverAddr;
        this_driver->vbEAConnDesc.sockFd         = sockFd;
        this_driver->vbEAConnDesc.args           = this_driver;
      }

      if (engine_err == VB_ENGINE_ERROR_NONE)
      {
        // Start new connection thread
        ret = VbEAThreadStart(&this_driver->vbEAConnDesc);
      }

      if ((engine_err != VB_ENGINE_ERROR_NONE) || (ret != VB_EA_ERR_NONE))
      {
        // Remove driver from drivers list
        VbEngineDrvListRemoveDriver(this_driver);

        // Release its related memory
        VbEngineDatamodelDriverDel(&this_driver);

        VbCounterIncrease(VB_ENGINE_COUNTER_ERR_CREATING_NEW_DRIVER);
      }
    }
    else
    {
      if (desc->args != NULL)
      {
        this_driver = (t_VBDriver *)desc->args;

        engine_err = VbEngineProcessEvSend(this_driver, ENGINE_EV_CONNECT, NULL);
      }
      else
      {
        engine_err = VB_ENGINE_ERROR_PARAMS;
      }
    }

    if (engine_err != VB_ENGINE_ERROR_NONE)
    {
      ret = VB_EA_ERR_OTHER;
    }
  }

  if (engine_err != VB_ENGINE_ERROR_NONE)
  {
    if (this_driver != NULL)
    {
      VbLogPrintExt(VB_LOG_ERROR, this_driver->vbDriverID, "Error establishing new connection (%d)", engine_err);
    }
    else
    {
      VbLogPrintExt(VB_LOG_ERROR, VB_ENGINE_ALL_DRIVERS_STR, "Error establishing new connection (%d)", engine_err);
    }
  }

  return ret;
}

/*******************************************************************/

static void VbEngineEADisconnectCb(t_vbEADesc *desc)
{
  t_VBDriver *this_driver;

  if((desc != NULL) && (desc->args != NULL))
  {
    this_driver = (t_VBDriver *)desc->args;

    VbEngineProcessEvSend(this_driver, ENGINE_EV_DISCONNECT, NULL);
  }
}

/*******************************************************************/

static t_vbEAError VbEngineEADriverReadyCb(t_vbEADesc *desc, struct sockaddr_in6 clientAddr, INT32S sockFd)
{
  t_vbEAError ret = VB_EA_ERR_NONE;
  t_VBDriver *this_driver;

  if ((desc == NULL) || (desc->args == NULL))
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    if(vbEAServerMode == TRUE)
    {
      this_driver = (t_VBDriver *)desc->args;

      VbEngineProcessEvSend(this_driver, ENGINE_EV_CONNECT, NULL);
    }
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

t_VB_engineErrorCode VbEngineEAProtocolDriverThreadStart(t_VBDriver *vbDriver)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEAError          ea_err;

  if (vbDriver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ea_err = VbEAThreadStart(&vbDriver->vbEAConnDesc);

    if (ea_err != VB_EA_ERR_NONE)
    {
      ret = VB_ENGINE_ERROR_EA_THREAD_CREATE;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEAProtocolDriverThreadStop(t_VBDriver *vbDriver, void *args)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEAError          ea_err;

  if (vbDriver == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ea_err = VbEAThreadStop(&vbDriver->vbEAConnDesc);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, vbDriver->vbDriverID, "Error %d stopping EA thread", ea_err);
      ret = VB_ENGINE_ERROR_EA_THREAD_STOP;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEAProtocolAllDriversThreadStop(void)
{
  t_VB_engineErrorCode ret;

  // Loop through all drivers and stop EA thread
  ret = VbEngineDatamodelDriversLoop(VbEngineEAProtocolDriverThreadStop, NULL);

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEAProtocolServerThreadStart(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEAError          ea_err;

  if (vbEAServerMode == FALSE)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ea_err = VbEAThreadStart(&vbEAServerDesc);

    if (ea_err != VB_EA_ERR_NONE)
    {
      ret = VB_ENGINE_ERROR_EA_THREAD_CREATE;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEAProtocolServerThreadStop(void)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  t_vbEAError          ea_err;

  if (vbEAServerMode == FALSE)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ea_err = VbEAThreadStop(&vbEAServerDesc);

    if (ea_err != VB_EA_ERR_NONE)
    {
      ret = VB_ENGINE_ERROR_EA_THREAD_STOP;
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEAInterfaceInit( const char *ifeth, struct in6_addr vbDriverIp,
    INT16U port, t_VBDriver *thisDriver, t_vbEAType type)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if(type == VB_EA_TYPE_SERVER)
  {
    vbEAServerMode = TRUE;
    // Configure server description
    VbEADescInit(&vbEAServerDesc);
    vbEAServerDesc.type           = VB_EA_TYPE_SERVER;

    strncpy(vbEAServerDesc.thrName, SERVER_THREAD_NAME, VB_EA_THREAD_NAME_LEN);
    vbEAServerDesc.thrName[VB_EA_THREAD_NAME_LEN - 1] = '\0';

    vbEAServerDesc.queueName      = VbEngineProcessQueueNameGet();
    vbEAServerDesc.closeCb        = NULL;
    vbEAServerDesc.disconnectCb   = NULL;
    vbEAServerDesc.threadStartCb  = NULL;
    vbEAServerDesc.connectCb      = VbEngineEAConnectCb;
    vbEAServerDesc.processRxMsgCb = NULL;
    strcpy((char *)(vbEAServerDesc.iface), ifeth);
    // in6addr_any allows connections from both IPv4 and IPv6 clients
    VbEAServerAddrSet(&vbEAServerDesc, in6addr_any, port);

  }
  else if(type == VB_EA_TYPE_CLIENT)
  {
    vbEAServerMode = FALSE;

    if (thisDriver == NULL)
    {
      ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      // Configure server description
      VbEADescInit(&thisDriver->vbEAConnDesc);
      thisDriver->vbEAConnDesc.type           = VB_EA_TYPE_CLIENT;

      strncpy(thisDriver->vbEAConnDesc.thrName, thisDriver->vbDriverID, VB_EA_THREAD_NAME_LEN);
      thisDriver->vbEAConnDesc.thrName[VB_EA_THREAD_NAME_LEN - 1] = '\0';

      thisDriver->vbEAConnDesc.queueName      = VbEngineProcessQueueNameGet();
      thisDriver->vbEAConnDesc.closeCb        = VbEngineEADisconnectCb;
      thisDriver->vbEAConnDesc.disconnectCb   = VbEngineEADisconnectCb;
      thisDriver->vbEAConnDesc.connectCb      = VbEngineEAConnectCb;
      thisDriver->vbEAConnDesc.processRxMsgCb = VbEngineEAFrameRxCb;
      thisDriver->vbEAConnDesc.threadStartCb  = NULL;
      thisDriver->vbEAConnDesc.args           = thisDriver;

      VbEAServerAddrSet(&thisDriver->vbEAConnDesc, vbDriverIp, port);
    }
  }

  return ret;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineEAInterfaceSendFrame(
    t_vbEAOpcode opcode, const INT8U *payload, INT16U payloadLength, t_VBDriver *thisDriver)
{
  t_VB_engineErrorCode result = VB_ENGINE_ERROR_NONE;
  t_vbEAError          ea_err;
  t_vbEAMsg           *msg = NULL;

  if (((payload == NULL) && (payloadLength != 0)) || (thisDriver == NULL))
  {
    result = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else if (thisDriver->vbEAConnDesc.connected == FALSE)
  {
    result = VB_ENGINE_ERROR_NOT_READY;
  }
  else
  {
    ea_err = VbEAMsgAlloc(&msg, payloadLength, opcode);

    if (ea_err != VB_EA_ERR_NONE)
    {
      VbLogPrintExt(VB_LOG_ERROR, thisDriver->vbDriverID, "Error (%d) allocating EA message", ea_err);
      result = VB_ENGINE_ERROR_MALLOC;
    }
  }

  if (result == VB_ENGINE_ERROR_NONE)
  {
    if ((payload != NULL) && (payloadLength > 0))
    {
      memcpy((char *)msg->eaPayload.msg, payload, payloadLength);
    }

    // Send frame
    VbEAMsgSend(&thisDriver->vbEAConnDesc, msg);
  }

  // Always release memory
  VbEAMsgFree(&msg);

  return result;
}

/*******************************************************************/

/**
 * @}
 **/
