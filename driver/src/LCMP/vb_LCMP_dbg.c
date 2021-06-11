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
 * @file vb_LCMP_dbg.c
 * @brief Debug LCMP communications
 *
 * @internal
 *
 * @author V.Grau
 * @date 2016/09/19
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <string.h>
#include "types.h"
#include "time.h"
#include <linux/if_ether.h>

#include "vb_mac_utils.h"
#include "vb_LCMP_com.h"
#include "vb_LCMP_dbg.h"
#include "vb_log.h"

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

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  INT32U          cnt;
  INT32U          cntProc;
  struct timespec timeStamp;
  INT8U           lastMac[ETH_ALEN];
} t_lcmpDbgEntry;

typedef struct
{
  t_lcmpDbgEntry  txTable[LCMP_NUM_OPCODES][LCMP_MAX_IDS_IN_GROUP];
  t_lcmpDbgEntry  rxTable[LCMP_NUM_OPCODES][LCMP_MAX_IDS_IN_GROUP];
  t_lcmpDbgEntry  noVbMsgs;
} t_lcmpDbgTable;

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

static t_lcmpDbgTable lcmpDbgTable;

static const CHAR *lcmpNotifyMsgName[] =
    {
        "DMRefCycStart.ind",
        "CycStart.ind",
        "CycQuery.ind",
        "CycQueryNotif.ind",
        "MeasBgnoise.ind",
        "MeasCfrAmp.ind",
        "MeasCfrPhase.ind",
        "MeasSnr.ind",
        "IngressTraffic.ind",
        "Alive.ind",
        "CycQueryExt.ind",
        "CycQueryNotifExt.ind",
        "AlignSyncLost.ind",
    };

static const CHAR *lcmpControlMsgName[] =
    {
        "DMRefSet",
        "MeasPlanCancel",
        "MeasBgnoise",
        "MeasCfrAmp",
        "MeasCfrPhase",
        "MeasSnr",
        "IngressTrafficMonControl",
        "CycChange",
        "EngineConf",
    };

static const CHAR *lcmpParamMsgName[] =
    {
        "DiscoverRead",
        "MeasPlan",
        "IngressTrafficRead",
        "SnrEstimated",
        "PsdRead",
        "PsdShapeWrite",
        "AddInfo1",
        "CdtaWrite",
    };

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

static void LcmpDbgMsgTableDump(t_lcmpDbgEntry table[LCMP_NUM_OPCODES][LCMP_MAX_IDS_IN_GROUP], t_writeFun writeFun)
{
  INT32U       lcmp_op = 0;
  INT32U       msg_id = 0;
  INT32U       lcmp_op_idx = 0;
  INT32U       msg_id_idx = 0;
  CHAR         time_stamp_str[TIMESPEC_STR_LEN];
  CHAR         mac_str[MAC_STR_LEN];

  if (table != NULL)
  {
    const CHAR  *msg_name;

    writeFun("==============================================================================================================\n");
    writeFun("| LCMP Op |   Id  |             Msg Name             |   Cnt   |   Lost   |        MAC       |     T.Stamp   |\n");
    writeFun("==============================================================================================================\n");

    for (lcmp_op = LCMP_FIRST_OPCODE; lcmp_op <= LCMP_LAST_OPCODE; lcmp_op++)
    {
      lcmp_op_idx = LCMP_INDEX_OPCODES(lcmp_op);

      if ((lcmp_op == LCMP_READ_REQ) ||
          (lcmp_op == LCMP_READ_CNF) ||
          (lcmp_op == LCMP_WRITE_REQ) ||
          (lcmp_op == LCMP_WRITE_CNF))
      {

        for (msg_id = LCMP_PARAMETER_FIRST_ID; msg_id <= LCMP_PARAMETER_LAST_ID; msg_id++)
        {
          msg_id_idx = LCMP_PARAMETER_ID_TO_INDEX(msg_id);

          if (table[lcmp_op_idx][msg_id_idx].cnt > 0)
          {
            // Get msg name
            msg_name = lcmpParamMsgName[msg_id_idx];

            // Get timeStamp string
            VbUtilTimespecToString(time_stamp_str, table[lcmp_op_idx][msg_id_idx].timeStamp);

            // Get MAC string
            MACAddrMem2str(mac_str, table[lcmp_op_idx][msg_id_idx].lastMac);

            writeFun("| %#7lX | %#5lX | %28s", lcmp_op, msg_id, msg_name);

            if ((lcmp_op == LCMP_READ_REQ) ||
                (lcmp_op == LCMP_WRITE_REQ))
            {
              writeFun(".req ");
            }
            else
            {
              writeFun(".cnf ");
            }

            writeFun("|%9lu|%10d|%18s| %13s |\n",
                table[lcmp_op_idx][msg_id_idx].cnt,
                table[lcmp_op_idx][msg_id_idx].cnt - table[lcmp_op_idx][msg_id_idx].cntProc,
                mac_str, time_stamp_str);
          }
        }
      }
      else if ((lcmp_op == LCMP_CTRL_REQ) ||
               (lcmp_op == LCMP_CTRL_CNF))
      {
        for (msg_id = LCMP_CONTROL_FIRST_ID; msg_id <= LCMP_CONTROL_LAST_ID; msg_id++)
        {
          msg_id_idx = LCMP_CONTROL_ID_TO_INDEX(msg_id);

          if (table[lcmp_op_idx][msg_id_idx].cnt > 0)
          {
            // Get msg name
            msg_name = lcmpControlMsgName[msg_id_idx];

            // Get timeStamp string
            VbUtilTimespecToString(time_stamp_str, table[lcmp_op_idx][msg_id_idx].timeStamp);

            // Get MAC string
            MACAddrMem2str(mac_str, table[lcmp_op_idx][msg_id_idx].lastMac);

            writeFun("| %#7lX | %#5lX | %28s", lcmp_op, msg_id, msg_name);

            if (lcmp_op == LCMP_CTRL_REQ)
            {
              writeFun(".req ");
            }
            else
            {
              writeFun(".cnf ");
            }

            writeFun("|%9lu|%10d|%18s| %13s |\n",
                table[lcmp_op_idx][msg_id_idx].cnt,
                table[lcmp_op_idx][msg_id_idx].cnt - table[lcmp_op_idx][msg_id_idx].cntProc,
                mac_str, time_stamp_str);
          }
        }
      }
      else if (lcmp_op == LCMP_NOTIFY_IND)
      {
        for (msg_id = LCMP_NOTIFY_FIRST_ID; msg_id <= LCMP_NOTIFY_LAST_ID; msg_id++)
        {
          msg_id_idx = LCMP_NOTIFY_ID_TO_INDEX(msg_id);

          if (table[lcmp_op_idx][msg_id_idx].cnt > 0)
          {
            // Get msg name
            msg_name = lcmpNotifyMsgName[msg_id_idx];

            // Get timeStamp string
            VbUtilTimespecToString(time_stamp_str, table[lcmp_op_idx][msg_id_idx].timeStamp);

            // Get MAC string
            MACAddrMem2str(mac_str, table[lcmp_op_idx][msg_id_idx].lastMac);

            writeFun("| %#7lX | %#5lX | %32s |%9lu|%10d|%18s| %13s |\n",
                lcmp_op,
                msg_id,
                msg_name,
                table[lcmp_op_idx][msg_id_idx].cnt,
                table[lcmp_op_idx][msg_id_idx].cnt - table[lcmp_op_idx][msg_id_idx].cntProc,
                mac_str,
                time_stamp_str);
          }
        }
      }
    }

    writeFun("==============================================================================================================\n");
  }
}

/*******************************************************************/

static void LcmpDbgNoVbMsgTableDump(t_writeFun writeFun)
{
  CHAR         time_stamp_str[TIMESPEC_STR_LEN];
  CHAR         mac_str[MAC_STR_LEN];

  writeFun("==============================================\n");
  writeFun("|   Cnt   |        MAC       |     T.Stamp   |\n");
  writeFun("==============================================\n");

  // Get timeStamp string
  VbUtilTimespecToString(time_stamp_str, lcmpDbgTable.noVbMsgs.timeStamp);

  // Get MAC string
  MACAddrMem2str(mac_str, lcmpDbgTable.noVbMsgs.lastMac);

  writeFun("|%9u|%18s| %13s |\n",
      lcmpDbgTable.noVbMsgs.cnt,
      mac_str,
      time_stamp_str);

  writeFun("==============================================\n");
}

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

void LcmpDbgMsgReset(void)
{
  memset(&lcmpDbgTable, 0, sizeof(lcmpDbgTable));
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode LcmpDbgMsgAdd(const INT8U *mac, BOOLEAN txNrx, BOOLEAN proc, t_LCMP_OPCODE lcmpOpcode, INT8U paramId)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;
  INT32U               lcmp_idx = LCMP_INDEX_OPCODES(lcmpOpcode);
  INT32U               param_idx = 0;
  CHAR                 mac_str[MAC_STR_LEN];

  if ((mac == NULL) || (lcmp_idx >= LCMP_NUM_OPCODES))
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    switch (lcmpOpcode)
    {
      case (LCMP_READ_REQ):
      case (LCMP_READ_CNF):
      case (LCMP_WRITE_REQ):
      case (LCMP_WRITE_CNF):
      {
        param_idx = LCMP_PARAMETER_ID_TO_INDEX(paramId);
        break;
      }
      case (LCMP_CTRL_REQ):
      case (LCMP_CTRL_CNF):
      {
        param_idx = LCMP_CONTROL_ID_TO_INDEX(paramId);
        break;
      }
      case (LCMP_NOTIFY_IND):
      case (LCMP_NOTIFY_RSP):
      {
        param_idx = LCMP_NOTIFY_ID_TO_INDEX(paramId);
        break;
      }
      default:
      {
        ret = HGF_LCMP_ERROR_BAD_ARGS;
        break;
      }
    }
  }

  if ((ret == HGF_LCMP_ERROR_NONE) && (param_idx >= LCMP_MAX_IDS_IN_GROUP))
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    t_lcmpDbgEntry  *entry;

    if (txNrx == TRUE)
    {
      entry = &(lcmpDbgTable.txTable[lcmp_idx][param_idx]);
      entry->cnt++;
      entry->cntProc++;
    }
    else
    {
      entry = &(lcmpDbgTable.rxTable[lcmp_idx][param_idx]);

      if (proc == TRUE)
      {
        entry->cntProc++;
      }
      else
      {
        entry->cnt++;
      }
    }

    MACAddrClone(entry->lastMac, mac);
    clock_gettime(CLOCK_REALTIME, &(entry->timeStamp));
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    // Get MAC string to be used in debug logs
    MACAddrMem2str(mac_str, mac);

    if (txNrx == TRUE)
    {
      VbLogPrint(VB_LOG_DEBUG, "Sending LCMP message to %s; lcmpOpcode 0x%X; paramID 0x%X", mac_str, lcmpOpcode, paramId);
    }
    else
    {
      VbLogPrint(VB_LOG_DEBUG, "Receiving LCMP message from %s; lcmpOpcode 0x%X; paramID 0x%X", mac_str, lcmpOpcode, paramId);
    }
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d; lcmpOpcode 0x%X; paramID 0x%X", ret, lcmpOpcode, paramId);
  }

  return ret;
}

/*******************************************************************/

t_HGF_LCMP_ErrorCode LcmpDbgNoVbMsgAdd(const INT8U *mac)
{
  t_HGF_LCMP_ErrorCode ret = HGF_LCMP_ERROR_NONE;

  if (mac == NULL)
  {
    ret = HGF_LCMP_ERROR_BAD_ARGS;
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    t_lcmpDbgEntry  *entry = &(lcmpDbgTable.noVbMsgs);

    entry->cnt++;
    entry->cntProc++;
    MACAddrClone(entry->lastMac, mac);
    clock_gettime(CLOCK_REALTIME, &(entry->timeStamp));
  }

  if (ret == HGF_LCMP_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_DEBUG, "Receiving No VB LCMP message from " MAC_PRINTF_FORMAT, MAC_PRINTF_DATA(mac));
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d counting No VB LCMP message", ret);
  }

  return ret;
}

/*******************************************************************/

void LcmpDbgMsgDump(t_writeFun writeFun)
{
  writeFun("TX LCMP Messages:\n");
  LcmpDbgMsgTableDump(lcmpDbgTable.txTable, writeFun);
  writeFun("\nRX LCMP Messages:\n");
  LcmpDbgMsgTableDump(lcmpDbgTable.rxTable, writeFun);
  writeFun("\nOther (no VB) LCMP Messages:\n");
  LcmpDbgNoVbMsgTableDump(writeFun);
}

/*******************************************************************/

BOOL VbLcmpConsoleCmd(void *arg, t_writeFun writeFun, char **cmd)
{
  BOOL ret = FALSE;
  BOOL show_help = FALSE;

  if (cmd[1] != NULL)
  {
    if (!strcmp(cmd[1], "i"))
    {
      // Dump LCMP statistics
      LcmpDbgMsgDump(writeFun);
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "r"))
    {
      // Reset counters
      LcmpDbgMsgReset();
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "t"))
    {
      // LCMP unicast and multicast time stats
      vbLcmpTimeStatsDump(writeFun);
      ret = TRUE;
    }
    else if (!strcmp(cmd[1], "tr"))
    {
      // LCMP unicast and multicast time stats
      vbLcmpTimeStatsReset();
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
    writeFun("lcmp h : Shows this help\n");
    writeFun("lcmp i : Shows LCMP Tx/Rx\n");
    writeFun("lcmp r : Reset LCMP Tx/Rx table\n");
    writeFun("lcmp t : Shows LCMP time related stats\n");
    writeFun("lcmp tr: Reset LCMP time related stats\n");
  }

  return ret;
}

/*******************************************************************/

/**
 * @}
 **/
