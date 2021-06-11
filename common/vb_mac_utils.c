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
 * @file vb_mac_utils.c
 * @brief MAC common utilities
 *
 * @internal
 *
 * @author V.Grau
 * @date 22/07/2015
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <linux/if_ether.h>
#include <ctype.h>

#include "types.h"
#include "vb_mac_utils.h"

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

void MACAddrClone(INT8U *dst, const INT8U *src)
{
  if ((dst != NULL) && (src != NULL))
  {
    memcpy(dst, src, ETH_ALEN);
  }
}

/*******************************************************************/

/**
 * @brief Converts a MAC address from a string to binary
 *
 * @param dst  Memory address
 * @param str  String
  **/
void MACAddrStr2mem(INT8U *dst, const char *str)
{
  INT32U          i;
  char            buff[3];

  memset(dst, 0, ETH_ALEN);
  buff[2] = 0;
  if ((str[0] == '0') && (tolower((int)str[1]) == 'x')) // format 0x0123456789ab
  {
    for (i = 0; i < ETH_ALEN; i++)
    {
      buff[0] = str[i * 2 + 2];
      buff[1] = str[i * 2 + 3];
      dst[i] = (char)strtoul(buff, NULL, 16);
    }
  }
  else if ((str[2] == ':') && (str[5] == ':') && (str[8] == ':') && (str[11] == ':') && (str[14] == ':')) // format 01:23:45:67:89:ab
  {
    for (i = 0; i < ETH_ALEN; i++)
    {
      buff[0] = str[i * 3];
      buff[1] = str[i * 3 + 1];
      dst[i] = (char)strtoul(buff, NULL, 16);
    }
  }
  else
  {
    memcpy(dst, str, ETH_ALEN);
  }
}

/*******************************************************************/

/**
 * @brief Converts a MAC address from binary to a string
 *
 * @param dst  Memory address
 * @param lmac
 *
 * @pre sizeof(dst)>MAC_SIZE_IN_BYTES*2+MAC_SIZE_IN_BYTES-1
 **/
void MACAddrMem2str(char *dst, const INT8U *lmac)
{
  sprintf(dst, "%02X:%02X:%02X:%02X:%02X:%02X", lmac[0], lmac[1], lmac[2], lmac[3], lmac[4],
           lmac[5]);
  dst[ETH_ALEN * 2 + (ETH_ALEN - 1)] = 0;
}

/*******************************************************************/

/**
 * @}
 **/

