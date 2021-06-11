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
 * @file vb_main_console.c
 * @brief Console commands implementation
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016-10-04
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <string.h>

#include "types.h"

#include "vb_DataModel.h"
#include "vb_main.h"
#include "vb_main_console.h"
#include "vb_counters.h"
#include "vb_log.h"
#include "vb_LCMP_com.h"
#include "vb_LCMP_dbg.h"
#include "vb_traffic.h"
#include "vb_domainsMonitor.h"
#include "vb_EA_interface.h"
#include "vb_console.h"
#include "vb_driver_conf.h"
#include "vb_thread.h"
#include "vb_timer.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_CONSOLE_DIAG_TRANS_ID_OFFSET      (100)
#define VB_CONSOLE_DIAG_THREAD_NAME          ("Diagnostic")
#define VB_CONSOLE_DIAG_MAX_THREADS          (1000)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct s_consoleLoopArgs
{
  t_writeFun writeFun;
} t_consoleLoopArgs;

typedef struct
{
  INT32U     iters;
  INT32U     transactionId;
  INT32U     numDomainsOrig;
  pthread_t  threadId;
  BOOLEAN    running;
  t_writeFun writeFun;
} t_consoleDiagParams;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static BOOL VbKillConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  VbLogPrint(VB_LOG_INFO, "Exit requested from console");
  writeFun("Exiting...\n");

  VbMainKill();

  return TRUE;
}

/*******************************************************************/

static void *VbDiagnosticThread(void *args)
{
  t_VB_comErrorCode    err = VB_COM_ERROR_NONE;
  t_consoleDiagParams *params = (t_consoleDiagParams *)args;
  INT32U               num_domains = 0;
  BOOLEAN              net_change = FALSE;
  INT32U               i = 0;

  if (params != NULL)
  {
    for (i = 0; i < params->iters; i++)
    {
      err = VbDatamodelDomainDiscover(&net_change, &num_domains, params->transactionId, TRUE);

      if (err == VB_COM_ERROR_NONE)
      {
        if (num_domains != params->numDomainsOrig)
        {
          params->writeFun("[Thread %02lu][Iter %03lu] : ERROR! -> unexpected number of domains discovered (%u instead of %u)\n",
              (params->transactionId - VB_CONSOLE_DIAG_TRANS_ID_OFFSET), i, num_domains, params->numDomainsOrig);
        }
        else
        {
          params->writeFun("[Thread %02lu][Iter %03lu] : OK\n", (params->transactionId - VB_CONSOLE_DIAG_TRANS_ID_OFFSET), i);
        }
      }
      else
      {
        params->writeFun("[Thread %02lu][Iter %03lu] : ERROR! -> %d discovering networks\n",
            (params->transactionId - VB_CONSOLE_DIAG_TRANS_ID_OFFSET), i, err);
      }
    }
  }

  return NULL;
}

/*******************************************************************/

static BOOL VbDiagnosticConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOLEAN           ret = FALSE;
  BOOLEAN           show_help = FALSE;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "disc"))
    {
      t_VB_comErrorCode    err = VB_COM_ERROR_NONE;
      t_consoleDiagParams *params;
      INT32U               iters = 0;
      INT32U               threads = 0;
      INT32U               i = 0;
      INT32U               num_domains_first = 0;
      BOOLEAN              net_change = FALSE;

      // Domain discovery diagnostic

      iters = strtoul(cmd[2], NULL, 0);

      if (cmd[3] != NULL)
      {
        threads = strtoul(cmd[3], NULL, 0);

        if (threads > VB_CONSOLE_DIAG_MAX_THREADS)
        {
          threads = VB_CONSOLE_DIAG_MAX_THREADS;
        }
      }
      else
      {
        threads = 1;
      }

      // Enter IDLE mode
      writeFun("Entering IDLE mode...");
      VbMainIdleModeSet();
      writeFun("DONE\n");

      // Get initial number of domains
      err = VbDatamodelDomainDiscover(&net_change, &num_domains_first, 0, TRUE);

      if (err == VB_COM_ERROR_NONE)
      {
        // Allocate memory for thread parameters
        params = (t_consoleDiagParams *)calloc(1, threads * sizeof(*params));

        if (params != NULL)
        {
          writeFun("Number of domains : %u\n", num_domains_first);

          // Reset LCMP messages
          LcmpDbgMsgReset();

          // Create threads
          for (i = 0; i < threads; i++)
          {
            params[i].iters = iters;
            params[i].numDomainsOrig = num_domains_first;
            params[i].transactionId = i + VB_CONSOLE_DIAG_TRANS_ID_OFFSET;
            params[i].writeFun = writeFun;
            params[i].running =  VbThreadCreate(VB_CONSOLE_DIAG_THREAD_NAME, VbDiagnosticThread, (void *)&(params[i]), 0, &(params[i].threadId));

            if (params[i].running == FALSE)
            {
              writeFun("Error starting thread\n");
              break;
            }
          }

          // Wait for threads
          for (i = 0; i < threads; i++)
          {
            if (params[i].running == TRUE)
            {
              VbThreadJoin(params[i].threadId, VB_CONSOLE_DIAG_THREAD_NAME);
            }
          }

          // Dump LCMP messages
          LcmpDbgMsgDump(writeFun);

          // Release memory
          free(params);
          params = NULL;
        }
        else
        {
          writeFun("Error allocating memory for threads\n");
        }
      }

      // Exit IDLE mode
      writeFun("Returning to STAND-BY mode...\n");
      VbMainStart();

      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "idle"))
    {
      writeFun("Entering IDLE mode...");
      VbMainIdleModeSet();
      writeFun("DONE\n");

      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "start"))
    {
      writeFun("Returning to STANDBY mode...\n");
      VbMainStart();

      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("diag h                      : Shows this help\n");
    writeFun("diag disc <iters> [threads] : Runs domain discovery diagnostic (by default 1 thread is used)\n");
    writeFun("diag idle                   : Puts VB driver in IDLE state\n");
    writeFun("diag start                  : Returns VB driver to normal operation\n");
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDriverBasicReportLoopCb(t_Domains *domain, void *args)
{
  t_VB_comErrorCode  ret = VB_COM_ERROR_NONE;
  INT32U             traffic = 0;
  INT32U             type_traffic = 0;
  t_consoleLoopArgs *loop_args = (t_consoleLoopArgs *)args;

  if ((domain == NULL) || (loop_args == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    loop_args->writeFun("| %s ", domain->dm.MACStr);

    if (domain->eps.epsArray[0].used == TRUE)
    {
      loop_args->writeFun("| %s |", domain->eps.epsArray[0].MACStr);
    }
    else
    {
      loop_args->writeFun("|       LOST        |");
    }

    traffic = 0;
    for (type_traffic = 0 ; type_traffic < VECTORBOOST_INGRESS_TRAFFIC_PRIORITIES_NUMBER; type_traffic++)
    {
      traffic += domain->dm.ingressTraffic.traffic[type_traffic];
    }
    loop_args->writeFun("    %04d     |      %04d     |          %04d           |\n",
        (int)domain->dm.ingressTraffic.channelCapacity,
        (int)domain->dm.ingressTraffic.desiredCapacity,
        (int)traffic);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDriverCapReportLoopCb(t_node *node, void *args)
{
  t_VB_comErrorCode  ret = VB_COM_ERROR_NONE;
  t_consoleLoopArgs *loop_args = (t_consoleLoopArgs *)args;

  if ((node == NULL) || (loop_args == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (node->type == VB_NODE_DOMAIN_MASTER)
    {
      loop_args->writeFun("| %s    ", node->MACStr);
    }
    else
    {
      loop_args->writeFun("|    %s ", node->MACStr);
    }

    loop_args->writeFun("| %4s ", (node->type == VB_NODE_DOMAIN_MASTER)?"DM":"EP");
    loop_args->writeFun("| %10s ", node->addInfo1.fwVersion);
    loop_args->writeFun("| %14s |", node->cap.trafficReports?"YES":"NO");
    loop_args->writeFun("\n");
  }

  return ret;
}

/*******************************************************************/


/*******************************************************************/

static BOOLEAN VbDriverBasicReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs loop_args;
  time_t            t;
  struct tm        *tmu;
  struct timeval    tv;

  t = time(NULL);
  tmu = localtime(&t);
  gettimeofday(&tv, NULL);

  /*
Driver Report    [21:28:20 766ms]   Number of Lines: 4
Version:
State: STAND_BY   Measuring Traffic: YES  Monitoring net: YES  Engine connected: YES
Note: All values in Mbps.
=================================================================================================
|       DM          |       EP          | Current PHY | Requested PHY | L2 Requested Throughput |
=================================================================================================
| 00:13:9D:00:13:85 | 00:13:9D:00:10:85 |    720      |      730      |          529            |
| 00:13:9D:00:13:8F | 00:13:9D:00:10:8F |    210      |      143      |          104            |
| 00:13:9D:00:13:AE | 00:13:9D:00:10:AE |    208      |      142      |          104            |
| 00:13:9D:00:13:7C |       LOST        |    000      |      000      |          000            |
=================================================================================================
   */

  if (tmu != NULL)
  {
    writeFun("Drivers Report    [%02d:%02d:%02d %03dms]\t", tmu->tm_hour, tmu->tm_min, tmu->tm_sec, (int)(tv.tv_usec/ 1000));
  }

  loop_args.writeFun = writeFun;

  writeFun("Number of lines: %u\n", VbDatamodelNmDomainsAliveGet());
  writeFun("Driver version: %s\n", VERSION);

  writeFun("State: %s\tMeasuring Traffic: ",  VbMainStateStrGet());
  if (VBTrafficStateGet())
  {
    writeFun("YES\t");
  }
  else
  {
    writeFun("NO\t");
  }

  writeFun("Monitoring net: ");
  if (VbDomainsMonitorStateGet())
  {
    writeFun("YES\t");
  }
  else
  {
    writeFun("NO\t");
  }

  writeFun("Engine connected: ");
  if (VbEAEngineIsConnected())
  {
    writeFun("YES\n");
  }
  else
  {
    writeFun("NO\n");
  }

  writeFun("Note: All values in Mbps.\n");

  writeFun("=================================================================================================\n");
  writeFun("|       DM          |       EP          | Current PHY | Requested PHY | L2 Requested Throughput |\n");
  writeFun("=================================================================================================\n");

  // Loop through domains
  VbDatamodelDomainsLoop(VbDriverBasicReportLoopCb, &loop_args);

  writeFun("=================================================================================================\n");

  return TRUE;
}

/*******************************************************************/

static BOOLEAN VbDriverCapReport(void *arg, t_writeFun writeFun, char **cmd)
{
  t_consoleLoopArgs loop_args;

  /*
=============================================================
|         Node         | Type | FW Version | TrafficReports |
=============================================================
| 00:13:9D:00:13:7C    |   DM |       r527 |            YES |
|    00:13:9D:00:10:28 |   EP |       r527 |             NO |
| 00:13:9D:00:13:8F    |   DM |       r527 |            YES |
|    00:13:9D:00:10:2D |   EP |       r527 |             NO |
| 00:13:9D:00:13:A8    |   DM |       r527 |            YES |
|    00:13:9D:00:10:0F |   EP |       r527 |             NO |
| 00:13:9D:00:13:AE    |   DM |       r527 |            YES |
|    00:13:9D:00:10:2A |   EP |       r527 |             NO |
=============================================================
   */

  writeFun("=============================================================\n");
  writeFun("|         Node         | Type | FW Version | TrafficReports |\n");
  writeFun("=============================================================\n");

  loop_args.writeFun = writeFun;

  // Loop through domains
  VbDatamodelActiveNodeLoop(VbDriverCapReportLoopCb, TRUE, &loop_args);

  writeFun("=============================================================\n");

  return TRUE;
}


/*******************************************************************/

static BOOLEAN VbDriverReportConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] == NULL)
  {
    // Basic report
    ret = VbDriverBasicReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "cap"))
  {
    // Capabilities report
    ret = VbDriverCapReport(arg, writeFun, cmd);
  }
  else if (!strcmp(cmd[1], "thr"))
  {
    // Threads report
    VbThreadListThreadDump(writeFun);
    // Timer tasks report
    VbTimerListTaskDump(writeFun);
    ret = TRUE;
  }
  else if (!strcmp(cmd[1], "h"))
  {
    show_help = TRUE;
    ret = TRUE;
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("report      : Shows basic report\n");
    writeFun("report h    : Shows this help\n");
    writeFun("report cap  : Shows capabilities info\n");
    writeFun("report thr  : Shows threads info\n");
  }

  return ret;
}

/*******************************************************************/

static BOOL VbDriverVersConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  writeFun("%s\n",VERSION);
  return TRUE;
}

/*******************************************************************/

static BOOL VbDriverConfConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  VbDriverConfDump(writeFun);
  return TRUE;
}

/*******************************************************************/

BOOL VbEAConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "i"))
    {
      // Dump EA statistics
      VbEAConnDbgMsgDump(writeFun);
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "r"))
    {
      VbEAConnDbgMsgReset();
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "h"))
    {
      show_help = TRUE;
      ret = TRUE;
    }
    else
    {
      ret = FALSE;
    }
  }
  else
  {
    ret = FALSE;
  }

  if ((ret == FALSE) || (show_help == TRUE))
  {
    writeFun("Usage:\n");
    writeFun("ea h : Shows this help\n");
    writeFun("ea i : Shows Tx/Rx messages\n");
    writeFun("ea r : Reset Tx/Rx table\n");
  }

  return ret;
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

t_VB_comErrorCode VbMainConsoleInit(INT16U port)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (port != 0)
  {
    VbConsoleInit(port, VbDriverConfDriverIdGet());

    VbConsoleCommandRegister("kill",     VbKillConsoleCmd,           NULL);
    VbConsoleCommandRegister("diag",     VbDiagnosticConsoleCmd,     NULL);
    VbConsoleCommandRegister("counters", VbCountersConsoleCmd,       NULL);
    VbConsoleCommandRegister("report",   VbDriverReportConsoleCmd,   NULL);
    VbConsoleCommandRegister("vers",     VbDriverVersConsoleCmd,     NULL);
    VbConsoleCommandRegister("runtime",  VbCountersRunningTime,      NULL);
    VbConsoleCommandRegister("lcmp",     VbLcmpConsoleCmd,           NULL);
    VbConsoleCommandRegister("conf",     VbDriverConfConsoleCmd,     NULL);
    VbConsoleCommandRegister("ea",       VbEAConsoleCmd,             NULL);
    VbConsoleCommandRegister("log",      VbLogConsoleCmd,            NULL);
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
