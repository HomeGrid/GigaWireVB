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

#ifndef VB_LCMP_SOCKET_H_
#define VB_LCMP_SOCKET_H_

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

#define LCMP_REQ_HEADER_SIZE    (sizeof(t_LCMP_req_Header))
#define LCMP_REQ_VALUE_OFFSET   LCMP_REQ_HEADER_SIZE
#define LCMP_IND_HEADER_SIZE    (sizeof(t_LCMP_ind_Header))
#define LCMP_IND_VALUE_OFFSET   LCMP_IND_HEADER_SIZE
#define LCMP_CNF_HEADER_SIZE    (sizeof(t_LCMP_cnf_Header))
#define LCMP_RSP_HEADER_SIZE    (sizeof(t_LCMP_rsp_Header))
#define LCMP_CONTROL_VB         (5)

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

typedef struct __attribute__ ((packed))
{
  INT8U control;
  INT8U AEMAC[ETH_ALEN];
  INT16U length;
  INT16U transactionId;
} t_LCMP_req_Header;

typedef struct __attribute__ ((packed))
{
  INT8U control;
  INT16U length;
  INT16U transactionId;
} t_LCMP_cnf_Header;

typedef struct __attribute__ ((packed))
{
  INT8U control;
  INT16U length;
  INT16U transactionId;
  INT8U notifAck;
} t_LCMP_ind_Header;

typedef struct __attribute__ ((packed))
{
  INT8U control;
  INT16U length;
  INT16U transactionId;
} t_LCMP_rsp_Header;

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */
/**
 * @brief Send a LCMP frame
 * @param[in] s Socket to use
 * @param[in] socket_address
 * @param[in] src_mac
 * @param[in] dst_mac
 * @param[in] LCMP_Opcodes Opcode to send
 * @param[in] MMPL_Length Length of MMPL to send
 * @param[in] MMPL MMPL to send
 * @return error code.
**/
t_HGF_LCMP_ErrorCode LcmpPacketSend(const INT8U *dstMac,t_LCMP_OPCODE lcmpOpcodes,INT16U mmplLength,
    const INT8U *mmpl);
/**
 * @brief Executes the rweception thread
 * @return error code
**/
t_HGF_LCMP_ErrorCode LcmpExecute( void );

/**
 * @brief Close socket
**/
void LcmpEnd( void );

/**
 * @brief Reads the local MAC
 * @param[out] MyMAC Local MAC
 * @return error code
**/
t_VB_comErrorCode LcmpMacGet( INT8U *myMac );

/**
 * @brief Install callback to receive LCMP frames
 * @param[in] Callback Function to execute to handle the frame
 * @param[in] OpcodeFilter Opcode to receive in this callback
 * @param[in] expectedHGFOpcode HGF Opcode to receive
 * @param[in] transactionId Transaction Id (if != 0 only Rx messages with this transaction Id will be processed)
 * @param[in] markTimeStamp Configuration to activate the time stamp
 * @return pointer to callback installed or NULL if error
**/
t_Callbacks *LcmpCallBackInstall(BOOL  (*callback)(const INT8U*,INT16U, t_CallbackData *),
                                    t_LCMP_OPCODE opcodeFilter, t_HGF_TLV expectedHgfOpcode,
                                    INT8U paramId, INT16U transactionId, BOOL markTimeStamp);

/**
 * @brief Get received values from allback and set pointer to NULL
 * @param[in] callback pointer to callback
 * @param[out] frameReceived TRUE: frame was received; FALSE: otherwise
 * @return pointer to lists of HTLVs
**/
t_HTLVsLists *LcmpCallBackReceiveGet(t_Callbacks *callback, BOOLEAN *frameReceived);

/**
 * @brief Uninstall a callback and free its reserved memory
 * @param[int/out] Callback
 * @return error code
**/
t_VB_comErrorCode LcmpCallBackUninstall(t_Callbacks *callback);

/**
 * @brief This function initiates the LCMP component to default values
 * @param[in] ifeth Ethernet interface
**/
void LcmpInit(const char *ifeth);

#endif /* VB_LCMP_SOCKET_H_ */

/**
 * @}
**/
