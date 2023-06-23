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
 * @file vb_ea_communication.c
 * @brief External agent (VB driver <-> VB engine) communication functions
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016/12/05
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <mqueue.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#if (_VALGRIND_ == 1)
#include <valgrind/helgrind.h>
#endif

#include "types.h"
#include "vb_util.h"
#include "vb_log.h"
#include "vb_thread.h"

#include "vb_ea_communication.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_EA_TCP_NODELAY               (1) // TCP_NODELAY socket option to send messages as soon as possible, disabling Nagle's algorithm
#define VB_EA_THREAD_PRIORITY           (0)
#define VB_EA_INVALID_FD                (-1)
#define VB_EA_BUFFER_SIZE               (4 * 1024) // Shall be greater than VB_EA_HEADER_SIZE
#define VB_EA_CONNECTION_QUEUE_SIZE     (1)
#define VB_EA_TO_CONNECTIONS            (1000) //in msecs
#define VB_EA_DRIVER_PORT_MAX_SIZE      (6) // 5 digits + null byte

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

static const char *eaOpcodeString[VB_EA_OPCODE_LAST] =
  {
      "EAMeasurePlan.req",
      "EAMeasurePlan.rsp",
      "EADomain.req",
      "EADomain.rsp",
      "EAMeasureBgn.rsp",
      "EAMeasureCfr.rsp",
      "EAMeasureSnrProbes.req",
      "EAMeasureSnrProbes.rsp",
      "EAMeasurePsd.req",
      "EAMeasurePsd.rsp",
      "EAPsdShape.req",
      "EAPsdShape.rsp",
      "EATrafficAwareness.trg",
      "EANetworkChange.trg",
      "EAVersion.req",
      "EAVersion.rsp",
      "EAVbDriverState.req",
      "EAVbDriverState.rsp",
      "EAMeasureErr.trg",
      "EAVbDriverState.trg",
      "EAClock.req",
      "EAClock.rsp",
      "EACycQuery.req",
      "EACycQuery.rsp",
      "EACycChange.req",
      "EACycChange.rsp",
      "EAMeasCollect.req",
      "EAMeasCollect.rsp",
      "EAMeasCollectEnd.trg",
      "EAMeasPlanCancel.req",
      "EAMeasPlanCancel.rsp",
      "EACdta.req",
      "EACdta.rsp",
      "EARedirect.req",
      "EAAlignMode.req",
      "EAAlignMode.rsp",
      "EAAlignSyncLost.trg",
      "EAAlignClusterStop.req",
      "EAAlignClusterStop.cnf",
      "EAANetworkChangeDiff",
      "EAASocketAlive.req",
      "EASocketAlive.rsp"
  };

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static void VbEADbgMsgTableDump(t_vbEADbgEntry table[VB_EA_OPCODE_LAST], t_writeFun writeFun)
{
  INT32U       opcode = 0;
  CHAR         time_stamp_str[TIMESPEC_STR_LEN];

  if (table != NULL)
  {
    const CHAR  *msg_name;

    writeFun("====================================================================\n");
    writeFun("|  Opcode |          Msg Name            |   Cnt   |     T.Stamp   |\n");
    writeFun("====================================================================\n");

    for (opcode = 0; opcode < VB_EA_OPCODE_LAST; opcode++)
    {
      if (table[opcode].cnt > 0)
      {
        // Get msg name
        msg_name = VbEAOpcodeToStrGet(opcode);

        // Get timeStamp string
        VbUtilTimespecToString(time_stamp_str, table[opcode].timeStamp);

        writeFun("| %#7lX | %28s | %7u | %13s |\n", opcode, msg_name, table[opcode].cnt, time_stamp_str);
      }
    }

    writeFun("====================================================================\n");
  }
}

/*******************************************************************/

static t_vbEAError VbEADbgMsgAdd(t_vbEADesc *eaDesc, BOOLEAN txNrx, t_vbEAOpcode opcode)
{
  t_vbEAError          ret = VB_EA_ERR_NONE;

  if (opcode >= VB_EA_OPCODE_LAST)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    t_vbEADbgEntry  *entry;

    if (txNrx == TRUE)
    {
      entry = &(eaDesc->debugTable.txTable[opcode]);
    }
    else
    {
      entry = &(eaDesc->debugTable.rxTable[opcode]);
    }

    entry->cnt++;
    clock_gettime(CLOCK_REALTIME, &(entry->timeStamp));
  }

  return ret;
}

/*******************************************************************/

static void VbEASocketClose(t_vbEADesc *desc)
{
  if ((desc != NULL) && (desc->sockFd != VB_EA_INVALID_FD))
  {
    pthread_mutex_lock(&(desc->mutex));
    shutdown(desc->sockFd, SHUT_RDWR);
    close(desc->sockFd);
    desc->sockFd = VB_EA_INVALID_FD;
    pthread_mutex_unlock(&(desc->mutex));
  }
}

/*******************************************************************/

static t_vbEAError VbEaSocketOpen(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Prepare socket

    pthread_mutex_lock(&(desc->mutex));
    desc->sockFd = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
    pthread_mutex_unlock(&(desc->mutex));

    if (desc->sockFd == VB_EA_INVALID_FD)
    {
      ret = VB_EA_ERR_SOCKET;
      VbLogPrint(VB_LOG_ERROR, "Error opening socket [%s]", strerror(errno));
    }
  }

  return ret;
}

static t_vbEAError VbEaClientSocketOpen(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Prepare socket
    pthread_mutex_lock(&(desc->mutex));
    desc->sockFd = socket(desc->clientInfo->ai_family, desc->clientInfo->ai_socktype, desc->clientInfo->ai_protocol);
    pthread_mutex_unlock(&(desc->mutex));

    if (desc->sockFd == VB_EA_INVALID_FD)
    {
      ret = VB_EA_ERR_SOCKET;
      VbLogPrint(VB_LOG_ERROR, "Error opening socket [%s]", strerror(errno));
    }
  }

  return ret;
}

/*******************************************************************/

static t_vbEAError VbEaSocketConf(INT32S sockFd)
{
  t_vbEAError ret = VB_EA_ERR_NONE;
  INT32U      no_delay = VB_EA_TCP_NODELAY;
  int         sock_err;

  // Configuring socket with TCP_NODELAY option to send messages as soon as possible
  sock_err = setsockopt(sockFd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));

  if (sock_err == -1)
  {
    VbLogPrint(VB_LOG_ERROR, "Error (%s) while configuring socket", strerror(errno));
    ret = VB_EA_ERR_SOCKET;
  }

  return ret;
}

/*******************************************************************/

static t_vbEAError VbEAConnProcess(t_vbEADesc *desc)
{
  t_vbEAError  ret = VB_EA_ERR_NONE;
  t_vbEAOpcode rx_opcode = VB_EA_OPCODE_LAST;
  CHAR         str_addr[INET6_ADDRSTRLEN] = "unknown";

  if ((desc == NULL) || (desc->sockFd == VB_EA_INVALID_FD) || (desc->processRxMsgCb == NULL))
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    INT8U *buffer = NULL;

    desc->connected = TRUE;

    // Call callbacks, if installed
    if (desc->connectCb != NULL)
    {
      desc->connectCb(desc, desc->clientAddr, desc->sockFd);
    }

    while ((desc->running == TRUE) && (desc->connected == TRUE))
    {
      int     buffer_offset = 0;   // total bytes (from the current message) received
      ssize_t bytes_received = 0;  // bytes received in the last call to "recv()"
      int     payload_length = 0;  // length of the current message payload (initially unknown)

      buffer = desc->buffer;

      /*
       * Wait until a whole packet header is received. The payload
       * length is contained in this header and we can then resize
       * the reception buffer if needed.
       */
      do
      {
        bytes_received = recv(desc->sockFd, buffer + buffer_offset, VB_EA_HEADER_SIZE - buffer_offset, 0);

        if (bytes_received < 0)
        {
          // Socket error
          VbLogPrint(VB_LOG_ERROR, "Error reading from socket [%s]", strerror(errno));

          // Abort connection thread
          desc->connected = FALSE;
        }
        else if (bytes_received == 0)
        {
          // Socket was orderly closed
          VbLogPrint(VB_LOG_WARNING, "Socket was remotely closed");

          // Abort connection thread
          desc->connected = FALSE;
        }
        else
        {
          // Data was received
          buffer_offset += bytes_received;
        }

        if ((desc->connected == FALSE) || (desc->running == FALSE))
        {
          break;
        }
      } while (buffer_offset < VB_EA_HEADER_SIZE);

      // Parse the header
      if ((desc->running == TRUE) && (desc->connected == TRUE))
      {
        t_vbEAFrameHeader *frame_header;

        frame_header = (t_vbEAFrameHeader *)buffer;

        if (frame_header->VBCode != VB_EA_CODE_FLAG)
        {
          VbLogPrint(VB_LOG_ERROR, "Corrupted message received");

          // Abort connection thread
          desc->connected = FALSE;
        }
        else
        {
          payload_length = _ntohs(frame_header->length);
          rx_opcode = frame_header->opcode;
        }
      }

      if ((desc->running == TRUE) && (desc->connected == TRUE) && (payload_length > 0))
      {
        // Resize "buffer" if needed
        if (VB_EA_BUFFER_SIZE < (VB_EA_HEADER_SIZE + payload_length))
        {
          buffer = (INT8U *)malloc(VB_EA_HEADER_SIZE + payload_length);

          if (buffer == NULL)
          {
            // No available memory, abort connection
            desc->connected = FALSE;
          }
          else
          {
            memcpy(buffer, desc->buffer, VB_EA_BUFFER_SIZE);

            VbLogPrint(VB_LOG_WARNING, "BUFFER_STORAGE_SIZE < message length (%d). Increase its value.", VB_EA_HEADER_SIZE + payload_length);
          }
        }

        if ((desc->running == TRUE) && (desc->connected == TRUE) && (buffer != NULL))
        {
          // Finally, wait for the rest of the data to arrive
          do
          {
            bytes_received = recv(desc->sockFd, buffer + buffer_offset, VB_EA_HEADER_SIZE + payload_length - buffer_offset, 0);

            if (bytes_received < 0)
            {
              // Socket error
              VbLogPrint(VB_LOG_ERROR, "Error reading from socket [%s]", strerror(errno));

              // Abort connection thread
              desc->connected = FALSE;
            }
            else if (bytes_received == 0)
            {
              // Socket was orderly closed
              VbLogPrint(VB_LOG_ERROR, "Socket was remotely closed");

              // Abort connection thread
              desc->connected = FALSE;
            }
            else
            {
              // Data was received
              buffer_offset += bytes_received;
            }

            if ((desc->connected == FALSE) || (desc->running == FALSE))
            {
              break;
            }
          } while (buffer_offset < (VB_EA_HEADER_SIZE + payload_length));
        }
      }

      if ((desc->running == TRUE) && (desc->connected == TRUE))
      {
        // Process rx msg
        desc->processRxMsgCb(desc, buffer, payload_length);

        // Add frame to debug table
        pthread_mutex_lock(&(desc->mutex));
        ret = VbEADbgMsgAdd(desc, FALSE, rx_opcode);
        pthread_mutex_unlock(&(desc->mutex));
      }

      // Clean up
      if (buffer != desc->buffer)
      {
        free(buffer);
        buffer = NULL;
      }
    }

    if (desc->running == TRUE)
    {
      // Call disconnect CB
      if (desc->disconnectCb != NULL)
      {
        desc->disconnectCb(desc);
      }
    }

    desc->connected = FALSE;

    if (desc->type == VB_EA_TYPE_SERVER_CONN)
    {
      // Server connection
      inet_ntop(AF_INET6, &desc->clientAddr.sin6_addr, str_addr, sizeof(str_addr));
      VbLogPrint(VB_LOG_INFO, "Connection closed for %s", str_addr);
    }
    else
    {
      // Client connection
      inet_ntop(AF_INET6, &desc->serverAddr.sin6_addr, str_addr, sizeof(str_addr));
      VbLogPrint(VB_LOG_INFO, "Connection closed for %s", str_addr);
    }
  }

  return ret;
}

/*******************************************************************/

static t_vbEAError VbEAClientProcess(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;
  int        err;
  BOOLEAN    socket_ready;
  CHAR       str_addr[INET6_ADDRSTRLEN];

  if ((desc == NULL) || (desc->processRxMsgCb == NULL))
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    while (desc->running == TRUE)
    {
      // Open a new socket
      socket_ready = FALSE;
      ret = VbEaClientSocketOpen(desc);

      if (ret == VB_EA_ERR_NONE)
      {
        int client_addr_len = sizeof(desc->clientAddr);
        int sock_attr_err;

        VbLogPrint(VB_LOG_INFO, "Establishing a new connection...");

        // Try to connect to server
        err = connect(desc->sockFd, desc->clientInfo->ai_addr, desc->clientInfo->ai_addrlen);

        // Get socket attributes
        sock_attr_err = getsockname(desc->sockFd, (struct sockaddr *)&(desc->clientAddr), (socklen_t *)&client_addr_len);

        if (sock_attr_err != 0)
        {
          ret = VB_EA_ERR_SOCKET;
          VbLogPrint(VB_LOG_ERROR, "Local socket name unavailable. Error on connect [%s]", strerror(errno));
        }

        if (err != 0)
        {
          socket_ready = FALSE;
          inet_ntop(AF_INET6, &desc->clientAddr.sin6_addr, str_addr, sizeof(str_addr));
          VbLogPrint(VB_LOG_INFO, "Error on connect from port %s [%s]", str_addr, strerror(errno));
        }
        else
        {
          socket_ready = TRUE;
        }
      }

      if ((ret == VB_EA_ERR_NONE) && (socket_ready == TRUE))
      {
        inet_ntop(AF_INET6, &desc->clientAddr.sin6_addr, str_addr, sizeof(str_addr));
        VbLogPrint(VB_LOG_INFO, "Established a new connection with port %s...", str_addr);

        // Configuring socket with TCP_NODELAY option to send messages as soon as possible
        VbEaSocketConf(desc->sockFd);

        // Attend connection
        VbEAConnProcess(desc);
      }

      // Always close socket
      VbEASocketClose(desc);

      if (desc->running == TRUE)
      {
        // Sleep
        VbThreadSleep(VB_EA_TO_CONNECTIONS);
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_vbEAError VbEAServerProcess(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;
  int        sock_err;

  if ((desc == NULL) || (desc->connectCb == NULL))
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Open a new socket
    ret = VbEaSocketOpen(desc);
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Configure socket to reuse same address
    int reuseaddr = 1;


    // "SO_REUSEADDR" will let us reuse the same address (for the server) as in
    // a previous (crashed) execution of this same process.
    //
    // More info here:
    //
    //   http://stackoverflow.com/questions/14388706/socket-options-so-reuseaddr-and-so-reuseport-how-do-they-differ-do-they-mean-t
    //
    sock_err = setsockopt(desc->sockFd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr));

    if (sock_err == -1)
    {
      ret = VB_EA_ERR_SOCKET;
      VbLogPrint(VB_LOG_ERROR, "Error configuring socket (SO_REUSEADDR) [%s]", strerror(errno));
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Bind socket to a given device

    // "SO_BINDTODEVICE" tells the socket to bind to a particular network
    // interface (such as, for example, "eth0")
    sock_err = setsockopt(desc->sockFd, SOL_SOCKET, SO_BINDTODEVICE, desc->iface, strlen((const char*)desc->iface));

    if (sock_err == -1)
    {
      ret = VB_EA_ERR_SOCKET;
      VbLogPrint(VB_LOG_ERROR, "Error configuring socket (SO_BINDTODEVICE): iface %s [%s]", desc->iface, strerror(errno));
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Bind socket to a given port
    sock_err = bind(desc->sockFd, (struct sockaddr *)&(desc->serverAddr), sizeof(desc->serverAddr));

    if (sock_err == -1)
    {
      ret = VB_EA_ERR_SOCKET;
      VbLogPrint(VB_LOG_ERROR, "Error on binding [%s]", strerror(errno));
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Configure the socket to listen new connections
    sock_err = listen(desc->sockFd, VB_EA_CONNECTION_QUEUE_SIZE);

    if (sock_err == -1)
    {
      ret = VB_EA_ERR_SOCKET;
      VbLogPrint(VB_LOG_ERROR, "Error on listen [%s]", strerror(errno));
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Accept new connections

    while (desc->running == TRUE)
    {
      INT32S              client_conn = VB_EA_INVALID_FD;
      struct sockaddr_in6 cli_addr;
      socklen_t           cli_len = sizeof(cli_addr);

      VbLogPrint(VB_LOG_INFO, "Waiting for new connection...");

      bzero((char *)&cli_addr, sizeof(cli_addr));
      client_conn = accept(desc->sockFd, (struct sockaddr *)&cli_addr, &cli_len);

      // Check if thread is still running
      if (desc->running == TRUE)
      {
        if (client_conn == VB_EA_INVALID_FD)
        {
          VbLogPrint(VB_LOG_ERROR, "Error on accept [%s]", strerror(errno));
        }
        else
        {
          // New connection received from the client
          CHAR str_addr[INET6_ADDRSTRLEN];
          inet_ntop(AF_INET6, &cli_addr.sin6_addr, str_addr, sizeof(str_addr));
          VbLogPrint(VB_LOG_INFO, "New connection received from %s",str_addr);

          // Configuring socket with TCP_NODELAY option to send messages as soon as possible
          VbEaSocketConf(client_conn);

          // Process new client request
          ret = desc->connectCb(desc, cli_addr, client_conn);
        }
      }

      if ((desc->running == FALSE) || (ret != VB_EA_ERR_NONE))
      {
        // Communication thread is stopping or error detected, so release recently acquired socket
        shutdown(client_conn, SHUT_RDWR);
        close(client_conn);
        client_conn = VB_EA_INVALID_FD;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static void *VbEACommonThread(void *arg)
{
  t_vbEAError err = VB_EA_ERR_NONE;
  t_vbEADesc *desc = (t_vbEADesc *)arg;

  if (desc == NULL)
  {
    err = VB_EA_ERR_BAD_ARGS;
  }

  if (err == VB_EA_ERR_NONE)
  {
    // Open queue to post events
    desc->queueId = mq_open(desc->queueName, O_WRONLY);

    if (desc->queueId == VB_EA_INVALID_FD)
    {
      err = VB_EA_ERR_QUEUE;
      VbLogPrint(VB_LOG_ERROR, "Error opening queue [%s]", strerror(errno));
    }
  }

  if (err == VB_EA_ERR_NONE)
  {
    if (desc->threadStartCb != NULL)
    {
      desc->threadStartCb(desc);
    }
  }

  if (err == VB_EA_ERR_NONE)
  {
    if (desc->type == VB_EA_TYPE_SERVER)
    {
      // Run server process
      err = VbEAServerProcess(desc);
    }
    else if (desc->type == VB_EA_TYPE_SERVER_CONN)
    {
      // Run new server connection process
      err = VbEAConnProcess(desc);
    }
    else
    {
      // Run client process
      err = VbEAClientProcess(desc);
    }

    if (err != VB_EA_ERR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "EA thread %s : Error %d", desc->thrName, err);
    }
  }

  if (desc != NULL)
  {
    // Aborting thread, close connection
    VbEASocketClose(desc);

    if ((desc->running == TRUE) &&
        (desc->closeCb != NULL))
    {
      // Call closeCb
      desc->closeCb(desc);
    }
  }

  // Release resources
  if (desc != NULL)
  {
    if (desc->buffer != NULL)
    {
      free(desc->buffer);
      desc->buffer = NULL;
    }

    if (desc->queueId != VB_EA_INVALID_FD)
    {
      mq_close(desc->queueId);
    }
  }

  return NULL;
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_vbEAError VbEAThreadStart(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if ((desc == NULL) || (desc->type >= VB_EA_TYPE_LAST))
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Stop pending thread (if any)
    ret = VbEAThreadStop(desc);
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Init descriptor parameters
    pthread_mutex_init(&(desc->mutex), NULL);

    if (desc->type != VB_EA_TYPE_SERVER_CONN)
    {
      // Do not init connFd for a new connection
      desc->sockFd = VB_EA_INVALID_FD;
    }
    desc->queueId = VB_EA_INVALID_FD;
    desc->connected = FALSE;
    desc->buffer = (INT8U *)malloc(VB_EA_BUFFER_SIZE);

    if (desc->buffer == NULL)
    {
      ret = VB_EA_ERR_NO_MEMORY;
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    BOOLEAN running;

    VbLogPrint(VB_LOG_INFO, "Starting %s thread", desc->thrName);

    desc->running = TRUE;

    // Starting common EA thread
    running = VbThreadCreate(desc->thrName, VbEACommonThread, (void *)desc, VB_EA_THREAD_PRIORITY, &(desc->threadId));

    if (running == FALSE)
    {
      desc->running = FALSE;
      VbLogPrint(VB_LOG_ERROR, "Can't create %s thread", desc->thrName);
      ret = VB_EA_ERR_NOT_STARTED;
    }
  }

  return ret;
}

/*******************************************************************/

t_vbEAError VbEAThreadStop(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    if (desc->running == TRUE)
    {
      VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", desc->thrName);

      desc->running = FALSE;

      pthread_mutex_lock(&(desc->mutex));

      if (desc->sockFd != VB_EA_INVALID_FD)
      {
        shutdown(desc->sockFd, SHUT_RDWR);
      }

      pthread_mutex_unlock(&(desc->mutex));

      VbThreadJoin(desc->threadId, desc->thrName);

      pthread_mutex_destroy(&(desc->mutex));

      if(desc->buffer != NULL)
      {
        free(desc->buffer);
        desc->buffer = NULL;
      }

      VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", desc->thrName);
    }
  }

  return ret;
}

/*******************************************************************/

t_vbEAError VbEAMsgAlloc(t_vbEAMsg **msg, INT32U payloadLen, t_vbEAOpcode opCode)
{
  t_vbEAError       ret = VB_EA_ERR_NONE;
  INT32U            frame_len;

  if (msg == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    *msg = (t_vbEAMsg *)malloc(sizeof(t_vbEAMsg));
    if (*msg == NULL)
    {
      ret = VB_EA_ERR_NO_MEMORY;
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Calculating total frame length
    frame_len = payloadLen + VB_EA_HEADER_SIZE;

    // Allocate buffer for payload and header
    (*msg)->eaFullMsg.msg = (INT8U *)malloc(frame_len);

    if ((*msg)->eaFullMsg.msg == NULL)
    {
      // Release previously allocated memory
      free(*msg);
      *msg = NULL;

      ret = VB_EA_ERR_NO_MEMORY;
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    t_vbEAFrameHeader *header;

    // Init header
    header = (t_vbEAFrameHeader *)((*msg)->eaFullMsg.msg);
    header->VBCode = VB_EA_CODE_FLAG;
    header->length = _htons(payloadLen);
    header->opcode = opCode;

    // Init structure fields
    (*msg)->eaPayload.msg = (*msg)->eaFullMsg.msg + VB_EA_PAYLOAD_OFFSET;
    (*msg)->opcode = opCode;
    (*msg)->eaPayload.msgLen = payloadLen;
    (*msg)->eaFullMsg.msgLen = frame_len;
  }

  return ret;
}

/*******************************************************************/

t_vbEAError VbEAMsgFree(t_vbEAMsg **msg)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if ((msg == NULL) || ((*msg) == NULL))
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    if ((*msg)->eaFullMsg.msg != NULL)
    {
      free((*msg)->eaFullMsg.msg);
      (*msg)->eaFullMsg.msg = NULL;
    }

    free(*msg);
    *msg = NULL;
  }

  return ret;
}

/*******************************************************************/

t_vbEAError VbEAMsgParse(t_vbEAMsg **msg, INT8U *rxBuffer)
{
  t_vbEAError        ret = VB_EA_ERR_NONE;
  INT32U             frame_len;
  INT32U             payload_len;
  t_vbEAFrameHeader *header_rx;

  if (msg == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    *msg = (t_vbEAMsg *)malloc(sizeof(t_vbEAMsg));

    if (*msg == NULL)
    {
      ret = VB_EA_ERR_NO_MEMORY;
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    header_rx = (t_vbEAFrameHeader *)rxBuffer;

    // Calculating total frame and payload length
    payload_len = _ntohs(header_rx->length);
    frame_len = payload_len + VB_EA_HEADER_SIZE;

    // Allocate buffer for total frame length
    (*msg)->eaFullMsg.msg = (INT8U *)malloc(frame_len);

    if ((*msg)->eaFullMsg.msg == NULL)
    {
      ret = VB_EA_ERR_NO_MEMORY;
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    // Fill structure fields
    memcpy((*msg)->eaFullMsg.msg, rxBuffer, frame_len);
    (*msg)->eaPayload.msg = (*msg)->eaFullMsg.msg + VB_EA_PAYLOAD_OFFSET;
    (*msg)->opcode     = header_rx->opcode;
    (*msg)->eaPayload.msgLen = payload_len;
    (*msg)->eaFullMsg.msgLen   = frame_len;
  }

  return ret;
}

/*******************************************************************/

t_vbEAError VbEAMsgSend(t_vbEADesc *desc, t_vbEAMsg *msg)
{
  t_vbEAError       ret = VB_EA_ERR_NONE;
  INT16S            n = 0;

  if ((desc == NULL) || (desc->type == VB_EA_TYPE_SERVER))
  {
    // Frames shall be sent only over VB_EA_TYPE_CLIENT or VB_EA_TYPE_SERVER_CONN types
    ret = VB_EA_ERR_BAD_ARGS;

    VbLogPrint(VB_LOG_ERROR, "Socket not ready");
  }

  if (ret == VB_EA_ERR_NONE)
  {
    if ((desc->running == FALSE) || (desc->connected == FALSE))
    {
      ret = VB_EA_ERR_NOT_STARTED;

      VbLogPrint(VB_LOG_ERROR, "Socket not ready");
    }
  }

  if (ret == VB_EA_ERR_NONE)
  {
    pthread_mutex_lock(&(desc->mutex));

    if (desc->sockFd < 0)
    {
      ret = VB_EA_ERR_SOCKET;
    }
    else if ((msg == NULL) || (msg->eaFullMsg.msg == NULL) || (msg->eaFullMsg.msgLen == 0))
    {
      ret = VB_EA_ERR_BAD_ARGS;
    }
    else
    {
      n = send(desc->sockFd, msg->eaFullMsg.msg, msg->eaFullMsg.msgLen, MSG_NOSIGNAL);

#if (_VALGRIND_ == 1)
    ANNOTATE_HAPPENS_AFTER(desc);
#endif
      VbEADbgMsgAdd(desc, TRUE, msg->opcode);
    }

    pthread_mutex_unlock(&(desc->mutex));

    if (ret == VB_EA_ERR_SOCKET)
    {
      VbLogPrint(VB_LOG_ERROR, "Socket not ready");
    }
    else if (ret == VB_EA_ERR_BAD_ARGS)
    {
      VbLogPrint(VB_LOG_ERROR, "Error buffer empty");
    }
    else if (n < 0)
    {
      // Error sending frame, abort connection
      VbLogPrint(VB_LOG_ERROR, "Error writing to socket [%s]", strerror(errno));

      ret = VbEAThreadStop(desc);
    }
  }

  return ret;
}

/************************************************************************/

t_vbEAError VbEAServerAddrSet(t_vbEADesc *desc, struct in6_addr ipAddr, INT16U port)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    bzero((char *)&(desc->serverAddr), sizeof(desc->serverAddr));
    desc->serverAddr.sin6_family = AF_INET6;
    desc->serverAddr.sin6_addr = ipAddr;
    desc->serverAddr.sin6_port = htons(port);
  }

  return ret;
}

/************************************************************************/

t_vbEAError VbEAClientAddrSet(t_vbEADesc *desc, CHAR *ipAddr, INT16U port, INT16U family)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if ((desc == NULL) || (ipAddr == NULL))
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    struct addrinfo hints;
    char port_str[VB_EA_DRIVER_PORT_MAX_SIZE];

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = family;
    hints.ai_flags = AI_NUMERICHOST | AI_NUMERICSERV;
    hints.ai_socktype = SOCK_STREAM;
    sprintf(port_str, "%d", port);
    getaddrinfo(ipAddr, port_str, &hints, &desc->clientInfo);
  }

  return ret;
}

/************************************************************************/

t_vbEAError VbEADescInit(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }

  if (ret == VB_EA_ERR_NONE)
  {
    bzero(desc, sizeof(t_vbEADesc));
  }

  return ret;
}

/************************************************************************/

const CHAR *VbEAOpcodeToStrGet(t_vbEAOpcode opcode)
{
  const CHAR *opcode_str = "UNKNOWN";

  if (opcode < VB_EA_OPCODE_LAST)
  {
    opcode_str = eaOpcodeString[opcode];
  }

  return opcode_str;
}

/*******************************************************************/

void VbEADbgMsgReset(t_vbEADbgTable *table)
{
  bzero(table, sizeof(t_vbEADbgTable));
}

/*******************************************************************/

void VbEADbgMsgDump(t_vbEADbgTable *tables, t_writeFun writeFun)
{
  writeFun("TX EA Messages:\n");
  VbEADbgMsgTableDump(tables->txTable, writeFun);
  writeFun("\nRX EA Messages:\n");
  VbEADbgMsgTableDump(tables->rxTable, writeFun);
}

/*******************************************************************/

t_vbEAError VbEADescDestroy(t_vbEADesc *desc)
{
  t_vbEAError ret = VB_EA_ERR_NONE;

  if (desc == NULL)
  {
    ret = VB_EA_ERR_BAD_ARGS;
  }
  else
  {
    // Free information used to bind and listen on socket
    if (desc->clientInfo->ai_addr != NULL)
    {
      freeaddrinfo(desc->clientInfo);
    }
  }

  return ret;
}

/**
 * @}
 **/


