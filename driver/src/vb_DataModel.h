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
 * @file vb_DataModel.h
 * @brief Driver data model interface
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
 *
 **/

#ifndef VB_DATAMODEL_H_
#define VB_DATAMODEL_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_types.h"
#include "vb_mac_utils.h"
#include "vb_ea_communication.h"

/*
 ************************************************************************
 ** Public constants
 ************************************************************************
 */

#define VERSION                                        "3.0 r187"
#define VBQUEUENAME                                    "/VbDriverMainQ"
#define VECTORBOOST_INGRESS_TRAFFIC_PRIORITIES_NUMBER  (4)
#define VB_MAX_EPS                                     (1)
#define MAX_IP_ADDR_STR_SIZE                           (16) // "aaa.bbb.ccc.ddd\0"

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

/*
 * Driver FSM events:
 * Modifying this enum implies updating:
 * - t_vbDriverCountersIndex
 * - VB_COUNTERS_NAME
 * - driverEvString
 */
typedef enum
{
  DRIVER_EV_START = 0,
  DRIVER_EV_IDLE,
  DRIVER_EV_KILL,
  DRIVER_EV_CONNECT,
  DRIVER_EV_DISCONNECT,
  DRIVER_EV_NETCHANGE,
  DRIVER_EV_TRAFFIC_KO,
  DRIVER_EV_ALIGNCHECK_END,
  DRIVER_EV_ALIGNCHANGE_END,
  DRIVER_EV_PLAN_FAILED,
  DRIVER_EV_PLAN_CNF_OK,
  DRIVER_EV_MEAS_COLLECT_NODE_END,
  DRIVER_EV_MEAS_COLLECT_KO,
  DRIVER_EV_MEAS_COLLECT_ALL_END,
  DRIVER_EV_PSD_SHAPE_END_OK,
  DRIVER_EV_PSD_SHAPE_END_KO,
  DRIVER_EV_SYNC_LOST_KO,
  DRIVER_EV_RX_MEASPLAN_REQ,
  DRIVER_EV_RX_VERSION_REQ,
  DRIVER_EV_RX_STATE_REQ,
  DRIVER_EV_RX_DOMAINS_REQ,
  DRIVER_EV_RX_CYCQUERY_REQ,
  DRIVER_EV_RX_CLOCK_REQ,
  DRIVER_EV_RX_CYCCHANGE_REQ,
  DRIVER_EV_RX_MEASCOLLECT_REQ,
  DRIVER_EV_RX_MEASPLAN_CANCEL_REQ,
  DRIVER_EV_RX_PSD_SHAPE_REQ,
  DRIVER_EV_RX_SNRPROBES_REQ,
  DRIVER_EV_RX_CDTA_REQ,
  DRIVER_EV_RX_CDTA_END_OK,
  DRIVER_EV_RX_CDTA_END_KO,
  DRIVER_EV_RX_REDIRECT_REQ,
  DRIVER_EV_RX_ENGINE_CONF_REQ,
  DRIVER_EV_RX_CLUSTER_STOP_REQ,
  DRIVER_EV_RX_SOCKET_ALIVE_REQ,
  DRIVER_EV_LAST,
} t_driverEvent;

/*
 * Debug counters.
 * IMPORTANTE NOTE: always put in first positions event related counters,
 * in the same order as defined in t_driverEvent
 */
typedef enum
{
  VB_DRIVER_COUNTER_EV_START = 0,
  VB_DRIVER_COUNTER_EV_IDLE,
  VB_DRIVER_COUNTER_EV_KILL,
  VB_DRIVER_COUNTER_EV_CONNECT,
  VB_DRIVER_COUNTER_EV_DISCONNECT,
  VB_DRIVER_COUNTER_EV_NETCHANGE,
  VB_DRIVER_COUNTER_EV_TRAFFIC_KO,
  VB_DRIVER_COUNTER_EV_ALIGNCHECK_END,
  VB_DRIVER_COUNTER_EV_ALIGNCHANGE_END,
  VB_DRIVER_COUNTER_EV_PLAN_FAILED,
  VB_DRIVER_COUNTER_EV_PLAN_CNF_OK,
  VB_DRIVER_COUNTER_EV_MEAS_COLLECT_NODE_END,
  VB_DRIVER_COUNTER_EV_MEAS_COLLECT_KO,
  VB_DRIVER_COUNTER_EV_MEAS_COLLECT_ALL_END,
  VB_DRIVER_COUNTER_EV_PSD_SHAPE_END_OK,
  VB_DRIVER_COUNTER_EV_PSD_SHAPE_END_KO,
  VB_DRIVER_COUNTER_EV_SYNC_LOST_KO,
  VB_DRIVER_COUNTER_EV_RX_MEASPLAN_REQ,
  VB_DRIVER_COUNTER_EV_RX_VERSION_REQ,
  VB_DRIVER_COUNTER_EV_RX_STATE_REQ,
  VB_DRIVER_COUNTER_EV_RX_DOMAINS_REQ,
  VB_DRIVER_COUNTER_EV_RX_CYCQUERY_REQ,
  VB_DRIVER_COUNTER_EV_RX_CLOCK_REQ,
  VB_DRIVER_COUNTER_EV_RX_CYCCHANGE_REQ,
  VB_DRIVER_COUNTER_EV_RX_MEASCOLLECT_REQ,
  VB_DRIVER_COUNTER_EV_RX_MEASPLAN_CANCEL_REQ,
  VB_DRIVER_COUNTER_EV_RX_PSD_SHAPE_REQ,
  VB_DRIVER_COUNTER_EV_RX_SNRPROBES_REQ,
  VB_DRIVER_COUNTER_EV_RX_CDTA_REQ,
  VB_DRIVER_COUNTER_EV_CDTA_END_OK,
  VB_DRIVER_COUNTER_EV_CDTA_END_KO,
  VB_DRIVER_COUNTER_EV_RX_ENGINE_CONF_REQ,
  VB_DRIVER_COUNTER_NETWORK_DISCOVERY_ERROR,
  VB_DRIVER_COUNTER_MEASURE_BGN_SUCCESS,
  VB_DRIVER_COUNTER_MEASURE_BGN_RETRIES,
  VB_DRIVER_COUNTER_MEASURE_BGN_ERROR,
  VB_DRIVER_COUNTER_MEASURE_CFR_SUCCESS,
  VB_DRIVER_COUNTER_MEASURE_CFR_RETRIES,
  VB_DRIVER_COUNTER_MEASURE_CFR_ERROR,
  VB_DRIVER_COUNTER_MEASURE_SNR_SUCCESS,
  VB_DRIVER_COUNTER_MEASURE_SNR_ERROR,
  VB_DRIVER_COUNTER_LCMP_REQ_TX_ERROR,
  VB_DRIVER_COUNTER_LCMP_REQ_NO_RESPONSE,
  VB_DRIVER_COUNTER_LCMP_REQ_NOT_ALL_DEV,
  VB_DRIVER_COUNTER_LCMP_REQ_UNKNOWN_ERROR,
  VB_DRIVER_COUNTER_LCMP_CNF_PARAMID_NOT_FOUND,
  VB_DRIVER_COUNTER_LCMP_CNF_ERROR,
  VB_DRIVER_COUNTER_LCMP_NOTIFY_REQ_ERROR,
  VB_DRIVER_COUNTER_LCMP_NOTIFY_TIMEOUT,
  VB_DRIVER_COUNTER_LCMP_NOTIFY_OTHER_ERROR,
  VB_DRIVER_COUNTER_KEEP_ALIVE_TX,
  VB_DRIVER_COUNTERS_NUM
}t_vbDriverCountersIndex;

typedef enum
{
  UNICAST = 0,          ///< Expected to be received by an specific node
  MULTICAST,            ///< Expected to be received by all G.hn nodes
  MULTICAST_DMS,        ///< Expected to be received by all DMs
  BROADCAST             ///< Expected to be received by all G.hn nodes
}t_Transmision;

typedef enum {
  VB_COM_ERROR_NONE = 0,
  VB_COM_ERROR_MALLOC = -1,
  VB_COM_ERROR_SENDTO = -2,
  VB_COM_ERROR_RECEIVE_TIMEOUT = -3,
  VB_COM_ERROR_SOCKET_NOT_OPENED = -4,
  VB_COM_ERROR_ETH_IF = -5,
  VB_COM_ERROR_PROTOCOL = -6,
  VB_COM_ERROR_DMREF_UNEXPECTED = -7,
  VB_COM_ERROR_DMREF_SELECTION = -8,
  VB_COM_ERROR_DMREF_CONFIGURATION = -9,
  VB_COM_ERROR_ANY_CONF_LOST = -10,
  VB_COM_ERROR_ALL_CONF_LOST = -11,
  VB_COM_ERROR_LCMP_WRITE = -12,
  VB_COM_ERROR_LCMP_CONTROL = -13,
  VB_COM_ERROR_LCMP_WAITNOTIFY = -14,
  VB_COM_ERROR_RECEIVE_GUI = -15,
  VB_COM_ERROR_NODEVICE = -16,
  VB_COM_ERROR_NOCROSMEASUREDEVICE = -17,
  VB_COM_ERROR_SNRCALCERROR_NOMEASURES = -18,
  VB_COM_ERROR_ALIGN = -19,
  VB_COM_ERROR_CYCQUERY_NUMDEVICES = -20,
  VB_COM_ERROR_CYCQUERY_NON_ALIGNED = -21,
  VB_COM_ERROR_PARAM_ERROR = -22,
  VB_COM_ERROR_COLLECT_MEASURE = -23,
  VB_COM_ERROR_MEASURE_DATA_TYPE = -24,
  VB_COM_ERROR_SNRCALCERROR_DATA_LINEAR = -25,
  VB_COM_ERROR_BAD_ARGS = -26,
  VB_COM_ERROR_THREAD = -27,
  VB_COM_ERROR_INI_FILE = -28,
  VB_COM_ERROR_DATAMODEL = -29,
  VB_COM_ERROR_LCMP_READ = -30,
  VB_COM_ERROR_QUEUE = -31,
  VB_COM_ERROR_MEASURE_THREAD_ABORT = -32,
  VB_COM_ERROR_MEASURE_THREAD_CREATE = -33,
  VB_COM_ERROR_MEASURE_THREAD_MAX = -34,
  VB_COM_ERROR_EXIT_LOOP_OK = -35,
  VB_COM_ERROR_INVALID_TRANSITION = -36,
  VB_COM_ERROR_INVALID_STATE = -37,
  VB_COM_ERROR_OS = -38,
  VB_COM_ERROR_NOT_FOUND = -39,
  VB_COM_ERROR_EA = -40,
  VB_COM_ERROR_NOT_CONF = -41,
  VB_COM_ERROR_ALREADY_RUNNING = -42,
  VB_COM_ERROR_BAD_PLAN_ID = -43,
  VB_COM_ERROR_NOT_STARTED = -44,
  VB_COM_ERROR_LCMP = -45,
} t_VB_comErrorCode;

typedef enum
{
  DOMAINCHANGE_NO_DOMAIN = 0,
  DOMAINCHANGE_DM_ADDED  = 1,
  DOMAINCHANGE_EP_ADDED  = 2,
  DOMAINCHANGE_DM_REM    = 3,
  DOMAINCHANGE_EP_REM    = 4
}t_VbDriverDomainChangesBitmap;


typedef struct s_Values{
  INT16U ValueLength;
  INT8U *Value;
} t_Values;

typedef struct s_ValuesArray{
  INT16U NumValues;
  t_Values *values;
} t_ValuesArray;

typedef struct s_HTLVValuesList{
  t_ValuesArray *HTLVsArray;
  struct timespec timeStamp;
  INT8U srcMAC[ETH_ALEN];
  BOOL ack;
  struct s_HTLVValuesList *nextList;
} t_HTLVValuesList;

typedef struct s_HTLVsLists{
  INT16U NumLists;
  t_HTLVValuesList *head;
  t_HTLVValuesList *tail;
} t_HTLVsLists;

typedef struct s_nodeCapabilities
{
  BOOLEAN trafficReports;
} t_nodeCapabilities;

typedef struct
{
  INT16U   nBands;                       ///< number of bands where bps shall be reported
  INT16U   bpsBand[VB_PSD_NUM_BANDS];    ///< bps of each band
} t_bpsBandTrafficReport;

typedef struct s_IngressTraffic
{
  struct  timespec lastRxTimeStamp;
  INT16U  traffic[VECTORBOOST_INGRESS_TRAFFIC_PRIORITIES_NUMBER];
  INT8U   maxBuff[VECTORBOOST_INGRESS_TRAFFIC_PRIORITIES_NUMBER];
  INT16U  channelCapacity;
  INT16U  effectiveCapacity;
  INT16U  desiredCapacity;
  INT8U   macEff;
  INT32U  rxReports;
  BOOLEAN newReportReceived;
  t_bpsBandTrafficReport bpsBandsInfo;
} t_IngressTraffic;

typedef struct s_node
{
  INT8U                 MAC[ETH_ALEN];
  CHAR                  MACStr[MAC_STR_LEN];
  INT8U                 devID;
  t_nodeType            type;
  t_additionalInfo1     addInfo1;
  t_IngressTraffic      ingressTraffic;
  t_vb_DevState         state;
  BOOLEAN               used;
  t_nodeCapabilities    cap;
  struct s_node        *linkedNode;
} t_node;

typedef struct s_EpsLists
{
  INT16U  numEps;
  t_node  epsArray[VB_MAX_EPS];
} t_EpsList;

typedef struct s_DomainMACs
{
  t_node        dm;
  t_alignInfo   alignInfo;
  t_alignChange alignChange;
  t_EpsList     eps;
  BOOLEAN       discoverChangeFlag;
} t_Domains;

typedef struct s_DomainsList
{
  INT16U NumDomains;
  t_Domains *domainsArray;
} t_DomainsList;

typedef struct s_DomainData
{
  INT8U  mac[ETH_ALEN];
  INT8U  fwVersion[VB_FW_VERSION_LENGTH];
  INT16U qosRate;
  INT16U maxLengthTxop;
  INT8U  devID;
  INT16U extSeed;
  INT16U numEps;
  t_nodeCapabilities cap;
} t_DomainData;

typedef struct s_EpData
{
  INT8U mac[ETH_ALEN];
  INT8U fwVersion[VB_FW_VERSION_LENGTH];
  INT8U devID;
  t_vb_DevState DevState;
  t_nodeCapabilities cap;
} t_EpData;

typedef struct s_EpDiffData
{
  INT8U dmMac[ETH_ALEN];
  INT8U epMac[ETH_ALEN];
  INT8U devId;
} t_EpDiffData;

typedef struct s_VbDomainChangeDetails
{
  INT8U  reportType;
  INT32U numDMsAdded;
  INT32U numEPsAdded;
  INT32U numDMsRem;
  INT32U numEPsRem;
  INT32U changesBitmap;
  INT8U  *pInfo;
}t_DomainChangeDetails;


typedef t_VB_comErrorCode (*t_nodeLoopCb)(t_node *node, void *args);
typedef t_VB_comErrorCode (*t_domainLoopCb)(t_Domains *domain, void *args);

/*
 ************************************************************************
 ** Public function definition
 ************************************************************************
 */

/**
 * @brief Creates a new instance of HTLV and add in HTLVs list at tail
 * @param[in] ValuesList Pointer to values list
 * @param[in] sendAck Indication to send Ack to this parameter
 * @param[in] markTimeStamp Inditacion of save reception time stamp
 * @param[in] srcMac Source MAC
 * @param[in/out] HTLVList Pointer to HTLVs list
 * @return erro code.
**/
t_VB_comErrorCode VbDatamodelHtlvListAdd( const t_ValuesArray *valuesArray,
    BOOL sendAck,  BOOL markTimeStamp, const INT8U *srcMac, t_HTLVsLists  **htlvList );

/**
 * @brief Add a new structure to value array
 * @param[in] length
 * @param[in] value
 * @param[in] values_array
 * @return erro code.
**/
t_VB_comErrorCode VbDatamodelValueToArrayAdd( INT16U length, const INT8U* value, t_ValuesArray **values_array);

/**
 * @brief Destroy a HTLVsArray  and free the memory
 * @param[in/out] htlvsLists Pointer to HTLVs array
 * @return error code.
**/
t_VB_comErrorCode VbDatamodelHTLVsArrayDestroy( t_ValuesArray **HTLVsArray );

/**
 * @brief Copy a HTLVsArray
 * @param[in] srcArray
 * @param[out] destArray
 * @return error code.
**/
t_VB_comErrorCode VbDatamodelHTLVsArrayCopy( const t_ValuesArray *srcArray, t_ValuesArray **destArray );

/**
 * @brief Destroy a HTLVs list value and free the memory
 * @param[in/out] htlvsLists Pointer to HTLVs list
 * @return error code.
**/
t_VB_comErrorCode VbDatamodelHtlvsListValueDestroy( t_HTLVsLists **htlvsLists );

/**
 * @brief Destroy a Domain MACs list and free the memory
 * @return error code.
**/
void VbDatamodelAllDomainsListDestroy( void );

/**
 * @brief Returns the number of domains active for vectorboost
 * @param[in] grabMutex TRUE: Domain list mutex shall be grabbed; FALSE: otherwise
 * @return number of domains
**/
INT16U VbDatamodelNmDomainsActiveGet(BOOLEAN grabMutex);

/**
 * @brief Returns the number of discovered domains
 * @return Number of discovered domains
 **/
INT16U VbDatamodelNmDomainsAliveGet( void );

/**
 * @brief This function gets a list of macs of active DMs for vectorboost
 * @param[out] numDomains       Number of DMs active
 * @param[out] domainData       Array os structs to conain domains data
 * @param[in] fullInfo Retrieves extended domain info
 * @param[in] all Gets data for all domains, including domains without EP
 * @return error code.
**/
t_VB_comErrorCode VbDatamodelDomainsDataGet( INT16U *numDomains, t_DomainData **domainData, BOOLEAN fullInfo, BOOLEAN all);

/**
 * @brief This function gets the list of EPs MAC from a DM
 * @param[in] dmMac         Pointer to MAC of DM
 * @param[out] numEps       Number of actve EPs
 * @param[out] arrayEpsMac  Array of active EPs MAC
 * @return error code.
**/
t_VB_comErrorCode VbDatamodelEpsDataGet(const INT8U *dmMac, INT16U *numEps, t_EpData **epData);

/**
 * @brief This function gets the list of EPs MAC from a DM
 * @param[in] dmMac         Pointer to MAC of DM
 * @param[out] numEps       Number of actve EPs
 * @param[out] arrayEpsMac  Array of active EPs MAC
 * @return error code.
**/
t_VB_comErrorCode VbDatamodelDomainsReportDataGet(INT8U *reportType, INT16U *numDMAdded, INT16U *numEPAdded, INT16U *numDMRem, INT16U *numEPRem, INT8U **pld);

/**
 * @brief This function gets the list of EPs MAC from a DM
 * @param[in] dmMac         Pointer to MAC of DM
 * @param[out] numEps       Number of actve EPs
 * @param[out] arrayEpsMac  Array of active EPs MAC
 * @return error code.
**/
t_VB_comErrorCode VbDatamodelDomainsReportDataFree(void);

/**
 * @brief Executes a Vector Boost MAC discover
 * @param[out] netChange TRUE: if network has changed; FALSE: otherwise
 * @param[out] numDomains Number of domains discovered
 * @param[in] transactionId Transaction Id to be used in LCMP communication
 * @return @ref t_VB_comErrorCode
 */
t_VB_comErrorCode VbDatamodelDomainDiscover(BOOL *netChange, INT32U *numDomains, INT32U transactionId, BOOL firstTime);

/**
 * @brief Reads additional info from all nodes
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbDatamodelAddInfo1Get(void);

/**
 * @brief This function initiates the data model component to default values
**/
void VbDatamodelInit(void);

/**
 * @brief This functions return the number and MACS of network nodes
 * @param[out] numNodes
 * @param[out] arrayNodesMac
 * @param[in] onlyDMs Return only Domain Masters MAC address
 * @return Error code
 */
t_VB_comErrorCode VbDatamodelGetActiveMacsArray( INT16U *numNodes, INT8U** arrayNodesMac, BOOLEAN onlyDMs );

/**
 * @brief Adds a new traffic report to given node
 * @param[in] macAddr MAC address of the node
 * @param[in] report Traffic report to add
 * @param[in] bps_report bps Traffic report to add
 * @return @ref t_VB_comErrorCode
**/
t_VB_comErrorCode VbDatamodelTrafficReportAdd(const INT8U *macAddr, t_IngressTraffic *report, t_bpsBandTrafficReport *bps_report);

/**
 * @brief Check which domains received traffic reports and compose a single message with all of them
 * @param[in] msg pointer to the message created
 * @return @ref t_VB_comErrorCode
**/
t_VB_comErrorCode VbDatamodelGetAllTrafficReports(t_vbEAMsg **msg);

/**
 * @brief Updates alignment info for given domain
 * @param[in] macAddr MAC Address of the dm
 * @param[in] seqNum New sequence number
 * @param[in] macClock MAC clock
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbDatamodelDomainAlignInfoUpdate(const INT8U *macAddr, INT16U seqNum, INT32U macClock, t_syncDetInfo *syncAllDids);

/**
 * @brief Updates alignment corrections for given domain
 * @param[in] macAddr MAC Address of the Domain Master
 * @param[in] seqNumOffset Sequence number offset to apply to get aligned
 * @param[in] clockEdge Flag to change current clock edge
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbDatamodelDomainAlignChangeUpdate(const INT8U *macAddr, INT16U seqNumOffset, BOOLEAN clockEdge);

/**
 * @brief Updates traffic info timestamp for all domains
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbDatamodelAllDomainsTrafficTimeStampUpdate(void);

/**
 * @brief Shows if a node is active
 * @param[in] node Pointer to node
 * @return TRUE: if node is active; FALSE: otherwise
 **/
BOOLEAN VbDatamodelNodeIsActive(t_node *node);

/**
 * @brief Sets traffic report capability for given node
 * @param[in] mac Node to update
 * @param[in] enable TRUE: enable traffic report capability; FALSE: otherwise
 **/
t_VB_comErrorCode VbDatamodelNodeCapTrafficReportSet(const INT8U *mac, BOOLEAN enable);

/**
 * @brief Loops through all nodes (DMs and EPs) and executes given callback for each one
 * @param[in] loopCb Callback to execute
 * @param[in] lock TRUE: grab domains mutex before calling callback; FALSE: otherwise
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_comErrorCode
 * @remarks Domains mutex could be grabbed before calling callback
 **/
t_VB_comErrorCode VbDatamodelActiveNodeLoop(t_nodeLoopCb loopCb, BOOLEAN lock, void *args);

/**
 * @brief Loops through all domains and executes given callback for each one
 * @param[in] loopCb Callback to execute
 * @param[in] args Generic args pointer to pass to callback
 * @return @ref t_VB_comErrorCode
 * @remarks @ref vbDatamodelMutexDomains is grabbed before calling callback
 **/
t_VB_comErrorCode VbDatamodelDomainsLoop(t_domainLoopCb loopCb, void *args);

/**
 * @brief Gets the MAC address of DM with given domain index
 * @param[in] domainIdx Index inside the domains list
 * @param[in] mac MAC address of DM
 * @return @ref t_VB_comErrorCode
 **/
t_VB_comErrorCode VbDatamodelDMMACGet(INT32U domainIdx, INT8U *mac);

/**
 * @brief Translate event to counter
 * @param[in] event Event to translate
 * @return Counter related to given event
 **/
t_vbDriverCountersIndex VbDatamodelEvToCounter(t_driverEvent event);

/**
 * @brief Sets seed for a given node
 * @param[in] mac Node to update
 * @param[in] seed Seed index value
 * @param[in] did Device id value
 **/
t_VB_comErrorCode VbDatamodelNodeSeedIndexSet(const INT8U *mac, INT16U seedIndex, INT16U did);

/**
 * @brief Read AddInfo1 values for a given DM
 * @param[in] mac Node to update
 * @param[out] fwVersion fw version from Addinfo1
 * @param[out] qosRate   qos Rate from Addinfo1
 * @param[out] maxTxopLength max txop length from Addinfo1
 **/
t_VB_comErrorCode VbDatamodelDMAddInfo1Read(const INT8U *mac, INT8U *fwVersion, INT16U *qosRate, INT16U *maxTxopLength);

/**
 * @brief Read AddInfo1 values for a given EP
 * @param[in] epMac EP Node to be read
 * @param[in] dmMac DM Node the EP is registered to
 * @param[out] fwVersion fw version from Addinfo1
 **/
t_VB_comErrorCode VbDatamodelEPAddInfo1Read(const INT8U *epMac, const INT8U *dmMac, INT8U *fwVersion);

/**
 * @brief Insert a HTLV in HTLVs list at tail
 * @param[in/out] HTLVsLists Pointer to HTLVs list
 * @param[in/out] HTLVValuesList Pointer to HTLV
**/
void VbDatamodelHtlvListInsert( t_HTLVsLists *htlvsLists, t_HTLVValuesList *htlvValuesList );

/**
 * @brief Create a HTLVs list
 * @param[in] HTLVsLists Pointer to HTLVs list
 * @return pointer to HTLVs list created or NULL if error
**/
t_HTLVsLists *VbDatamodelHtlvListCreate( void );

#endif /* VB_DATAMODEL_H_ */

/**
 * @}
**/


