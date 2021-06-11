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
 * @file vb_mac_utils.h
 * @brief MAC utilities interface
 *
 * @internal
 *
 * @author V.Grau
 * @date 22/07/2015
 *
 **/

#ifndef VB_MAC_UTILS_H_
#define VB_MAC_UTILS_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define MAC_STR_LEN (ETH_ALEN * 3)
#define MAC_ZERO    ((INT8U*)"\x00\x00\x00\x00\x00\x00")

// MAC_PRINTF_FORMAT and MAC_PRINTF_DATA can be used to simplify the
// display of MAC address when using printf like commands
// Example:
//        iprintf("  My MAC address is    " MAC_PRINTF_FORMAT "\n", MAC_PRINTF_DATA(my_mac));
// where my_mac is a variable of type t_mac
#  define MAC_PRINTF_FORMAT "%02x:%02x:%02x:%02x:%02x:%02x"
#  define MAC_PRINTF_DATA(mac) mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Quick MAC address comparison
 * This function provides a quick comparison of 2 MAC addresses (array of 6 INT8U)
 * without calling an external function. Inverse order, byte by byte comparison.
 * It must be used when CPU time is a strict requirement but firmware size can
 * be increased.
 *
 * @param mac1 First MAC address
 * @param mac2 Second MAC address
 *
 * @return TRUE if both MAC addresses are equal and FALSE otherwise.
 **/

static inline BOOLEAN MACAddrQuickCmp(const INT8U *mac1, const INT8U *mac2)
{
  BOOLEAN         ret_value = FALSE;

  if ((mac1[5] == mac2[5]) &&
      (mac1[4] == mac2[4]) &&
      (mac1[3] == mac2[3]) && (mac1[2] == mac2[2]) && (mac1[1] == mac2[1]) && (mac1[0] == mac2[0]))
  {
    ret_value = TRUE;
  }

  return ret_value;
}

/**
 * @brief Copies src MAC address to dst
 *
 * @param dst Destination
 * @param src Source
 **/
void MACAddrClone(INT8U *dst, const INT8U *src);

/**
 * @brief Converts a MAC address from a string to binary
 *
 * @param dst Memory address
 * @param str String
 **/
void            MACAddrStr2mem(INT8U *dst, const char *str);

/**
 * @brief Converts a MAC address from binary to a string
 *
 * @param dst String
 * @param lmac Memory address
 **/
void            MACAddrMem2str(char *dst, const INT8U *lmac);

#endif /* VB_MAC_UTILS_H_ */

/**
 * @}
 **/


