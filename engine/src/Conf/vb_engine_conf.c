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
 * @file vb_engine_conf.c
 * @brief Vector boost engine configuration functionality
 *
 * @internal
 *
 * @author V.Grau
 * @date 2017-03-22
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <arpa/inet.h>

#include "types.h"

#include "vb_types.h"
#include "vb_util.h"
#include "vb_engine_datamodel.h"
#include "vb_engine_drivers_list.h"
#include "vb_engine_conf.h"
#include "vb_measure_utils.h"
#include "vb_engine_EA_interface.h"
#include "vb_engine_measure.h"
#include "vb_engine_cdta.h"
#include "vb_engine_socket_alive.h"
#include "vb_engine_alignment.h"
#include "ezxml.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_ENGINE_CONF_DEFAULT_INI_FILE                  ("vb_engine.ini")
#define VB_ENGINE_CONF_DEFAULT_VERBOSE_LEVEL             (VB_LOG_ALWAYS)
#define VB_ENGINE_CONF_DEFAULT_CONSOLE_PORT              (60000)
#define VB_ENGINE_CONF_DEFAULT_TRAFFIC_METRICS_ENABLED   (TRUE)
#define VB_ENGINE_CONF_DEFAULT_SAVE_METRICS              (FALSE)
#define VB_ENGINE_CONF_DEFAULT_METRICS_MAX_LOG_KB_SPACE  (1024)
#define VB_ENGINE_CONF_DEFAULT_BOOSTTHR_DEC_BOOST        (70)
#define VB_ENGINE_CONF_DEFAULT_BOOSTTHR_INC_BOOST        (85)
#define VB_ENGINE_CONF_MAX_IFACE_LENGTH                  (32)
#define VB_ENGINE_CONF_DEFAULT_IN_UPSTREAM               (TRUE)
#define VB_ENGINE_CONF_DEFAULT_VDSL_COEX                 (FALSE)
#define VB_ENGINE_CONF_DEFAULT_SAVE_MEASURES             (FALSE)
#define VB_ENGINE_CONF_DEFAULT_SERVER_MODE               (FALSE)
#define VB_ENGINE_CONF_DEFAULT_ALIGN_MODE                (VB_ALIGN_MODE_COMMON_CLOCK)
#define VB_ENGINE_CONF_DEFAULT_BOOST_ALG_PERIOD          (150) // In ms

#define MAX_FILE_NAME_LENGTH                             (150)

#define VB_ENGINE_CONF_USER_PROFILE_MAX                  (48)
#define VB_ENGINE_CONF_SLA_DEFINITION_MAX                (10)

#define VB_ENGINE_CONF_SLA_DEFAULT_MBPS                  (1000)
#define VB_ENGINE_CONF_SLA_DEFAULT_SLA_WEIGHT            (10)
#define VB_ENGINE_CONF_SLA_DEFAULT_USER_WEIGHT           (1)

#define VB_ENGINE_CONF_SLA_NAME_MAX                      (16)

#define VB_ENGINE_CONF_PSD_BAND_0_END                    (577)  ///< 28  MHz (SISO & MIMO)
#define VB_ENGINE_CONF_PSD_BAND_1_END                    (1024) ///< 50  MHz (SISO & MIMO)
#define VB_ENGINE_CONF_PSD_BAND_2_END                    (1966) ///< 96  MHz (SISO & MIMO)
#define VB_ENGINE_CONF_PSD_BAND_3_END                    (2867) ///< 140 MHz (SISO)
#define VB_ENGINE_CONF_PSD_BAND_4_END                    (3858) ///< 200 MHz (SISO)

#define VB_ENGINE_CONF_DEFAULT_MEAS_NUM_CYCLES           (1)
#define VB_ENGINE_CONF_DEFAULT_MEAS_STORAGE_TYPE         (0)
#define VB_ENGINE_CONF_DEFAULT_MEAS_SYMBOLS_NUMBER       (8)
#define VB_ENGINE_CONF_DEFAULT_MEAS_TIME_AVG             (3)
#define VB_ENGINE_CONF_DEFAULT_MEAS_FREQ_AVG             (0)
#define VB_ENGINE_CONF_DEFAULT_MEAS_OFFSET               (2000000)
#define VB_ENGINE_CONF_DEFAULT_MEAS_DURATION             (50000)
#define VB_ENGINE_CONF_DEFAULT_MEAS_CFR_TYPE             (0)
#define VB_ENGINE_CONF_DEFAULT_MEAS_DATA_TYPE            (0)
#define VB_ENGINE_CONF_DEFAULT_MEAS_DATA_FORMAT          (0)

#define VB_ENGINE_CONF_DEFAULT_SEED_ENABLE               (TRUE)
#define VB_ENGINE_CONF_DEFAULT_SEED_MIN_INDEX            (0)
#define VB_ENGINE_CONF_DEFAULT_SEED_MAX_INDEX            (499)

#define VB_ENGINE_CONF_DEFAULT_ALIGN_RELTHR              (12) // % perc of reliability
#define VB_ENGINE_CONF_DEFAULT_ALIGN_MINPOW              (33) // -27dB In HW format
#define VB_ENGINE_CONF_DEFAULT_ALIGN_MINPOWHYST          (16) // -30dB In HW format
#define VB_ENGINE_CONF_DEFAULT_ALIGN_METRICS             (TRUE)

#define VB_ENGINE_CONF_DEFAULT_SOCKALIVE_ENABLE          (TRUE)
#define VB_ENGINE_CONF_DEFAULT_SOCKALIVE_PERIOD          (10000)
#define VB_ENGINE_CONF_DEFAULT_SOCKALIVE_NLOST_THR       (3)

#define VB_ENGINE_CONF_DEFAULT_PERSLOG_NUMLINES          (500)
#define VB_ENGINE_CONF_DEFAULT_PERSLOG_VERBOSE           (VB_LOG_ERROR)
#define VB_ENGINE_CONF_DEFAULT_PERSLOG_CIRCULAR          (TRUE)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct s_vbEngineProfile
{
  INT8U          mac[ETH_ALEN];   ///< Mac of user
  INT32U         sla;             ///< slaMbps
  INT32U         userWeight;      ///< user weight
  INT32U         slaWeight;       ///< sla weigth
} t_vbEngineUserProfile;

typedef struct s_vbEngineSLADefinition
{
  INT8U          name[VB_ENGINE_CONF_SLA_NAME_MAX];        ///< SLA Name
  INT32U         slaMbps;               ///< SLA in Mbps
  INT32U         weight;                ///< Weight
} t_vbEngineSLADefinition;

typedef struct s_vbEngineSLAs
{
  INT16U                          numSlas;
  t_vbEngineSLADefinition         definition[VB_ENGINE_CONF_SLA_DEFINITION_MAX];
} t_vbEngineSLAs;

typedef struct s_seedConf
{
  BOOLEAN enabled;
  INT16U  minSeedIndex;
  INT16U  maxSeedindex;
  INT16U  excludedMacListSize;
  INT8U   *excludedMacList;
}t_seedConf;

typedef struct s_macList
{
  INT32U  size;
  INT8U  *list;
} t_macList;

typedef struct s_alignParamsCustomEntry
{
  INT8U      targetNode[ETH_ALEN];
  t_macList  blackList;
} t_alignParamsCustomEntry;

typedef struct s_alignParamsCustom
{
  INT32U                      size;
  t_alignParamsCustomEntry   *entries;
} t_alignParamsCustom;

typedef struct s_alignParams
{
  t_VBAlignmentMode          alignMode;
  INT32U                     reliabilityThr;
  INT32U                     minPow;
  INT32U                     minPowHyst;
  BOOLEAN                    metrics;
  t_macList                  prioRefList;
  t_alignParamsCustom        nodeCustomList;
} t_alignParams;

typedef struct s_persistentLog
{
  INT32U          numLines;
  t_vbLogLevel    verboseLevel;
  BOOLEAN         circular;
} t_persistentLog;

typedef struct s_vbEngineConf
{
  CHAR                      engineId[VB_ENGINE_ID_MAX_SIZE];
  BOOL                      serverMode;
  INT16U                    serverPort;
  CHAR                      serverIface[VB_ENGINE_CONF_MAX_IFACE_LENGTH];
  CHAR                      outputPath[VB_PARSE_MAX_PATH_LEN];               ///< Debug info output path
  t_vbLogLevel              verboseLevel;                                    ///< Verbose level
  INT16U                    consolePort;                                     ///< VB Engine console port (0 to disable console)
  BOOL                      trafficMetricsEnabled;
  BOOL                      saveMetricsEnabled;
  INT32U                    maxMetricsLogSize;
  INT32U                    boostThresholds[VB_BOOST_THR_TYPE_LAST];         ///< Boost thresholds
  BOOLEAN                   vdslCoex;
  t_measconfdata            measPlanConf;
  t_cdtaConf                cdtaConf;
  t_seedConf                seedConf;
  BOOLEAN                   saveMeasures;
  BOOLEAN                   vbInUpstream;                                    ///< VB in upstream
  t_vbEngineUserProfile     userProfile[VB_ENGINE_CONF_USER_PROFILE_MAX];
  t_vbEngineSLAs            sla;
  t_psdBandAllocation       psdBandAllocation;
  INT16U                    boostAlgPeriod;
  t_alignParams             alignParams;
  t_persistentLog           persistentLog;
  t_socketAlive             socketAlive;
} t_vbEngineConf;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static t_vbEngineConf vbEngineConf;

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

static INT32U VbEngineNumEntriesGet(ezxml_t parentNode, const char *tag)
{
  INT32U               num_entries = 0;
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

  if (tag == NULL)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    ezxml_t ez_entry;

    // Get the number of custom entries
    for (ez_entry = ezxml_child(parentNode, tag); ez_entry != NULL; ez_entry = ez_entry->next)
    {
      num_entries++;
    }
  }

  return num_entries;
}

/*******************************************************************/

static t_VB_engineErrorCode UserProfileSLAInfoSet(INT8U *slaName, INT32U userProfileIdx)
{
  t_VB_engineErrorCode err_code = VB_ENGINE_ERROR_NOT_FOUND;
  INT32U i;

  for(i=0; i< vbEngineConf.sla.numSlas; i++)
  {
    if(strncmp((char *)slaName, (char *)vbEngineConf.sla.definition[i].name, VB_ENGINE_CONF_SLA_NAME_MAX) == 0)
    {
      vbEngineConf.userProfile[userProfileIdx].sla = vbEngineConf.sla.definition[i].slaMbps;
      vbEngineConf.userProfile[userProfileIdx].slaWeight = vbEngineConf.sla.definition[i].weight;
      err_code = VB_ENGINE_ERROR_NONE;
      break;
    }
  }

  return err_code;
}

/************************************************************************/

static t_VB_engineErrorCode UserProfileFileParse( ezxml_t userProfile, INT32U userProfileIdx)
{
  t_VB_engineErrorCode err_code = VB_ENGINE_ERROR_NONE;
  ezxml_t              ez_temp;

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(userProfile, "MAC");
    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      MACAddrStr2mem(vbEngineConf.userProfile[userProfileIdx].mac, ez_temp->txt);
      errno = 0;

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: UserProfile Mac value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading Mac parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(userProfile, "SLAName");
    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      err_code = UserProfileSLAInfoSet((INT8U*)ez_temp->txt, userProfileIdx);
      if ((errno != 0) || (err_code != VB_ENGINE_ERROR_NONE))
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid SLA Name\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading SLA Name parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(userProfile, "Weight");
    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      vbEngineConf.userProfile[userProfileIdx].userWeight = strtol(ez_temp->txt,NULL,10);
      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Weight value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading Weight parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  return err_code;
}

/************************************************************************/

static t_VB_engineErrorCode SLAsDefinitionFileParse( ezxml_t slaDefinition, INT32U slaIdx)
{
  t_VB_engineErrorCode err_code = VB_ENGINE_ERROR_NONE;
  ezxml_t              ez_temp;

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(slaDefinition, "Name");
    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      strncpy((char*)vbEngineConf.sla.definition[slaIdx].name, ez_temp->txt, VB_ENGINE_CONF_SLA_NAME_MAX);
      vbEngineConf.sla.definition[slaIdx].name[VB_ENGINE_CONF_SLA_NAME_MAX-1] = 0;
      errno = 0;

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: SLADefinition name\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading Name parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(slaDefinition, "SLAMbps");
    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      vbEngineConf.sla.definition[slaIdx].slaMbps = strtol(ez_temp->txt,NULL,10);
      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid SLAMbps value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading SLAMbps parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(slaDefinition, "Weight");
    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      vbEngineConf.sla.definition[slaIdx].weight = strtol(ez_temp->txt,NULL,10);
      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid weight value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading weight parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  return err_code;
}

/************************************************************************/

static t_VB_engineErrorCode DriverListParse(ezxml_t driversList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  ezxml_t              ez_driver;
  ezxml_t              ez_temp;
  INT16U               drivers_num;
  t_VBDriver          *driver;
  CHAR                 driver_id[VB_EA_DRIVER_ID_MAX_SIZE];
  struct in6_addr      ip_addr;
  INT16U               driver_port = 0;
  
  if (driversList == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    drivers_num = 0;

    for (ez_driver = ezxml_child(driversList, "Driver"); ez_driver != NULL; ez_driver = ez_driver->next)
    {
      ret = VbEngineDatamodelCreateDriver(NULL, &driver);

      if (ret != VB_ENGINE_ERROR_NONE)
      {
        printf("Engine Conf: Error %d creating new Driver\n", ret);
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        ret = VbEngineDrvListDriverAdd(driver);

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          printf("Engine Conf: Error %d Adding Driver to the driver list\n", ret);
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Set a default name based on the index inside the linked list. This index is unique
        sprintf(driver_id, VB_ENGINE_DEFAULT_DRIVER_ID, (unsigned int)driver->l.index);

        // Set driver name
        ret = VbEngineDatamodelDriverIdSet(driver_id, driver, TRUE);

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          printf("Engine Conf: Error %d setting Driver name\n", ret);
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        ez_temp = ezxml_child(ez_driver, "Port");

        if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
        {
          driver_port = strtol(ezxml_txt(ez_temp), NULL, 0);
        }
        else
        {
          printf("Engine Conf: Error reading Driver>Port parameter\n");
          ret = VB_ENGINE_ERROR_INI_FILE;
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        ez_temp = ezxml_child(ez_driver, "IP");

        if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
        {
          int                  ip_err;

          // Check if address is IPv6
          ip_err = inet_pton(AF_INET6, ezxml_trimtxt(ez_temp),  &(ip_addr));

          if (ip_err != 1)
          {
            struct in_addr ipV4_addr;

            // Check if address is IPv4
            ip_err = inet_pton(AF_INET, ezxml_trimtxt(ez_temp),  &(ipV4_addr.s_addr));

            if (ip_err != 1)
            {
              printf("Engine Conf: Error %d reading Driver>IP parameter\n", ip_err);
              ret = VB_ENGINE_ERROR_INI_FILE;
            }
            else
            {
              char ipV4_mapped[INET6_ADDRSTRLEN];

              // IPv6 has a special range designed to represent an IPv4 address
              // Format is "::FFFF:"+IPv4 address
              memset(ipV4_mapped, '\0', sizeof(ipV4_mapped));
              strcpy(ipV4_mapped,"::FFFF:");
              strcat(ipV4_mapped,ezxml_trimtxt(ez_temp));

              // Check if IPv4 mapped address is valid
              ip_err = inet_pton(AF_INET6, ipV4_mapped,  &(ip_addr));
              if (ip_err != 1)
              {
                printf("Engine Conf: Error %d reading Driver>IP parameter\n", ip_err);
                ret = VB_ENGINE_ERROR_INI_FILE;
              }
            }
          }
        }
        else
        {
          printf("Engine Conf: Error reading Driver>IP parameter\n");
          ret = VB_ENGINE_ERROR_INI_FILE;
        }
      }

      if (ret == VB_ENGINE_ERROR_NONE)
      {
        // Associate EAinterface in client mode to the driver
        ret = VbEngineEAInterfaceInit(NULL, ip_addr, driver_port, driver, VB_EA_TYPE_CLIENT);

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          printf("Engine Conf: Error %d configuring driver interface\n", ret);
        }
      }

      drivers_num++;
    } //end for

    if (drivers_num == 0)
    {
      printf("Engine Conf: Error no Driver in DriverList\n");
      ret = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  return ret;
}

/************************************************************************/

static t_VB_engineErrorCode VbEngineSeedConfigurationParse( ezxml_t seed_conf )
{
  t_VB_engineErrorCode err_code = VB_ENGINE_ERROR_NONE;
  ezxml_t              ez_temp;

  if (seed_conf == NULL)
  {
    err_code = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(seed_conf, "Enable");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.seedConf.enabled =  (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;;

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid Enabled value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading Enabled parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(seed_conf, "MinSeedIndex");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.seedConf.minSeedIndex = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid minSeed value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading minSeed parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(seed_conf, "MaxSeedIndex");

    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.seedConf.maxSeedindex = strtol(ezxml_txt(ez_temp),NULL,10);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid maxSeed value\n", errno, strerror(errno));
        err_code = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      printf("Engine Conf: Error reading maxSeed parameter\n");
      err_code = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (err_code == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(seed_conf, "ExcludedMacList");

    // This section is optional in the .ini file, so parse it only if present.
    if (ez_temp != NULL)
    {
      ezxml_t ez_excluded_mac;
      INT16U  numberOfExcludedMacs;

      // Get the number of excluded MACs
      numberOfExcludedMacs = VbEngineNumEntriesGet(ez_temp, "MAC");

      // Store the number of excluded MACs
      vbEngineConf.seedConf.excludedMacListSize = numberOfExcludedMacs;

      // Reserve memory to store the list of excluded MACs
      vbEngineConf.seedConf.excludedMacList = (INT8U *)malloc(numberOfExcludedMacs*ETH_ALEN*sizeof(INT8U));

      if (vbEngineConf.seedConf.excludedMacList == NULL)
      {
        printf("ERROR (no memory) parsing .ini file: Invalid AutomaticSeed/ExcludedMacList value\n");
        err_code = VB_ENGINE_ERROR_NO_MEMORY;
      }

      if (err_code == VB_ENGINE_ERROR_NONE)
      {
        // Store all MACs into a list
        numberOfExcludedMacs = 0;
        for (ez_excluded_mac = ezxml_child(ez_temp, "MAC"); ez_excluded_mac != NULL; ez_excluded_mac = ez_excluded_mac->next)
        {
          MACAddrStr2mem(vbEngineConf.seedConf.excludedMacList+(numberOfExcludedMacs*ETH_ALEN), ez_excluded_mac->txt);
          numberOfExcludedMacs++;
        }
      }
    }
  }

  return err_code;
}

/*******************************************************************/

static void VbEngineAlignCustomRelease(void)
{
  INT32U entry_idx;

  if (vbEngineConf.alignParams.nodeCustomList.entries != NULL)
  {
    for (entry_idx = 0; entry_idx < vbEngineConf.alignParams.nodeCustomList.size; entry_idx++)
    {
      if (vbEngineConf.alignParams.nodeCustomList.entries[entry_idx].blackList.list != NULL)
      {
        free(vbEngineConf.alignParams.nodeCustomList.entries[entry_idx].blackList.list);
        vbEngineConf.alignParams.nodeCustomList.entries[entry_idx].blackList.list = NULL;
        vbEngineConf.alignParams.nodeCustomList.entries[entry_idx].blackList.size = 0;
      }
    }

    free(vbEngineConf.alignParams.nodeCustomList.entries);
    vbEngineConf.alignParams.nodeCustomList.entries = NULL;
    vbEngineConf.alignParams.nodeCustomList.size = 0;
  }
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignBlackListItemGet(INT32U entryIdx, INT32U blackListIdx, INT8U **blackListMac)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((blackListMac == NULL) ||
      (vbEngineConf.alignParams.nodeCustomList.entries == NULL) ||
      (entryIdx >= vbEngineConf.alignParams.nodeCustomList.size) ||
      (vbEngineConf.alignParams.nodeCustomList.entries[entryIdx].blackList.list == NULL) ||
      (blackListIdx >= vbEngineConf.alignParams.nodeCustomList.entries[entryIdx].blackList.size))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *blackListMac = vbEngineConf.alignParams.nodeCustomList.entries[entryIdx].blackList.list + (blackListIdx * ETH_ALEN);
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignCustomEntryMACParse(ezxml_t customEntry, INT8U *targetMAC)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (targetMAC == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ezxml_t              ez_custom_entry_mac;

    // Get target entry MAC
    ez_custom_entry_mac = ezxml_child(customEntry, "MAC");

    if ((ez_custom_entry_mac != NULL) &&
        (ez_custom_entry_mac->txt != NULL))
    {
      MACAddrStr2mem(targetMAC, ezxml_trimtxt(ez_custom_entry_mac));
    }
    else
    {
      printf("ERROR parsing .ini file: Invalid AlignParams/CustomList/Entry/MAC value\n");
      ret = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignCustomEntryBlackListItemParse(ezxml_t blackListXml, t_macList *blackList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (blackList == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    INT32U    item_idx = 0;
    ezxml_t   ez_item;

    for (ez_item = ezxml_child(blackListXml, "MAC"); ez_item != NULL; ez_item = ez_item->next)
    {
      if ((item_idx < blackList->size) &&
          (ez_item->txt != NULL))
      {
        // Insert MAC in black list
        MACAddrStr2mem(blackList->list + (item_idx * ETH_ALEN), ezxml_trimtxt(ez_item));

        // Update index to write
        item_idx++;
      }
      else
      {
        printf("ERROR parsing .ini file: Invalid AlignParams/CustomList/Entry/BlackList value\n");
        ret = VB_ENGINE_ERROR_INI_FILE;
      }

      if (ret != VB_ENGINE_ERROR_NONE)
      {
        break;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignCustomEntryBlackListParse(ezxml_t customEntry, t_macList *blackList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (blackList == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ezxml_t ez_blacklist;

    // Get black list for given target
    ez_blacklist = ezxml_child(customEntry, "BlackList");

    if (ez_blacklist != NULL)
    {
      INT32U  black_list_items;

      // Get number of items in black list
      black_list_items = VbEngineNumEntriesGet(ez_blacklist, "MAC");

      // Store number of items
      blackList->size = black_list_items;

      if (black_list_items > 0)
      {
        // Allocate memory for black list entries
        blackList->list = (INT8U *)calloc(black_list_items, sizeof(INT8U) * ETH_ALEN);

        if (blackList->list == NULL)
        {
          printf("ERROR (no memory) parsing .ini file: Invalid AlignParams/CustomList value\n");
          ret = VB_ENGINE_ERROR_NO_MEMORY;
        }

        if (ret == VB_ENGINE_ERROR_NONE)
        {
          // Parse black list macs
          ret = VbEngineAlignCustomEntryBlackListItemParse(ez_blacklist, blackList);
        }
      }
      else
      {
        blackList->list = NULL;
      }
    }
    else
    {
      // No items in blacklist
      blackList->size = 0;
      blackList->list = NULL;
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignCustomEntryParse(ezxml_t customEntry, t_alignParamsCustomEntry *entry)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (entry == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Get target MAC address
    ret = VbEngineAlignCustomEntryMACParse(customEntry, entry->targetNode);
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    // Parse black list for given node
    ret = VbEngineAlignCustomEntryBlackListParse(customEntry, &(entry->blackList));
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignCustomListParse(ezxml_t customList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               custom_entries_num;

  // Get the number of custom entries
  custom_entries_num = VbEngineNumEntriesGet(customList, "Entry");

  // Store the number of custom entries
  vbEngineConf.alignParams.nodeCustomList.size = custom_entries_num;

  if (custom_entries_num > 0)
  {
    // Allocate memory for custom entries
    vbEngineConf.alignParams.nodeCustomList.entries = (t_alignParamsCustomEntry *)calloc(custom_entries_num, sizeof(t_alignParamsCustomEntry));

    if (vbEngineConf.alignParams.nodeCustomList.entries == NULL)
    {
      printf("ERROR (no memory) parsing .ini file: Invalid AlignParams/CustomList value\n");
      ret = VB_ENGINE_ERROR_NO_MEMORY;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      ezxml_t ez_custom_entry;
      INT32U  custom_idx = 0;

      for (ez_custom_entry = ezxml_child(customList, "Entry"); ez_custom_entry != NULL; ez_custom_entry = ez_custom_entry->next)
      {
        if (custom_idx < vbEngineConf.alignParams.nodeCustomList.size)
        {
          // Parse custom entry
          ret = VbEngineAlignCustomEntryParse(ez_custom_entry, &(vbEngineConf.alignParams.nodeCustomList.entries[custom_idx]));
          custom_idx++;
        }
        else
        {
          printf("ERROR parsing .ini file: Invalid AlignParams/CustomList/Entry value\n");
          ret = VB_ENGINE_ERROR_INI_FILE;
        }

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          break;
        }
      }
    }
  }
  else
  {
    // No custom entries
    vbEngineConf.alignParams.nodeCustomList.entries = NULL;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignPrioRefListParse(ezxml_t prioRefList)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  INT32U               prio_ref_num;

  // Get the number of entries
  prio_ref_num = VbEngineNumEntriesGet(prioRefList, "MAC");

  // Store the number of entries
  vbEngineConf.alignParams.prioRefList.size = prio_ref_num;

  if (prio_ref_num > 0)
  {
    // Allocate memory for entries
    vbEngineConf.alignParams.prioRefList.list = (INT8U *)calloc(prio_ref_num, sizeof(INT8U) * ETH_ALEN);

    if (vbEngineConf.alignParams.prioRefList.list == NULL)
    {
      printf("ERROR (no memory) parsing .ini file: Invalid AlignParams/PrioRefList value\n");
      ret = VB_ENGINE_ERROR_NO_MEMORY;
    }

    if (ret == VB_ENGINE_ERROR_NONE)
    {
      ezxml_t ez_mac_entry;
      INT32U  mac_idx = 0;

      for (ez_mac_entry = ezxml_child(prioRefList, "MAC"); ez_mac_entry != NULL; ez_mac_entry = ez_mac_entry->next)
      {
        if (mac_idx < vbEngineConf.alignParams.prioRefList.size)
        {
          // Parse MAC entry
          MACAddrStr2mem(vbEngineConf.alignParams.prioRefList.list + (mac_idx * ETH_ALEN), ezxml_trimtxt(ez_mac_entry));

          mac_idx++;
        }
        else
        {
          printf("ERROR parsing .ini file: Invalid AlignParams/PrioRefList/MAC value\n");
          ret = VB_ENGINE_ERROR_INI_FILE;
        }

        if (ret != VB_ENGINE_ERROR_NONE)
        {
          break;
        }
      }
    }
  }
  else
  {
    // No custom entries
    vbEngineConf.alignParams.prioRefList.list = NULL;
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEngineAlignParamsParse( ezxml_t alignConf )
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  ezxml_t              ez_temp;

  ez_temp = ezxml_child(alignConf, "RelThr");

  if ((ez_temp != NULL) && (ez_temp->txt != NULL))
  {
    errno = 0;
    vbEngineConf.alignParams.reliabilityThr = strtol(ez_temp->txt, NULL, 0);

    if (errno != 0)
    {
      printf("ERROR (%d:%s) parsing .ini file: Invalid AlignParams/RelThr value\n", errno, strerror(errno));
      ret = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(alignConf, "MinPow");

    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      vbEngineConf.alignParams.minPow = strtoul(ez_temp->txt, NULL, 0);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid AlignParams/MinPow value\n", errno, strerror(errno));
        ret = VB_ENGINE_ERROR_INI_FILE;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(alignConf, "MinPowHyst");

    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      vbEngineConf.alignParams.minPowHyst = strtoul(ez_temp->txt, NULL, 0);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid AlignParams/MinPowHyst value\n", errno, strerror(errno));
        ret = VB_ENGINE_ERROR_INI_FILE;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ezxml_t prio_ref;

    prio_ref = ezxml_child(alignConf, "PrioRefList");

    if (prio_ref != NULL)
    {
      ret = VbEngineAlignPrioRefListParse(prio_ref);
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(alignConf, "Metrics");

    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.alignParams.metrics = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid AlignParams/Metrics value\n", errno, strerror(errno));
        ret = VB_ENGINE_ERROR_INI_FILE;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(alignConf, "CustomList");

    // This section is optional in the .ini file, so parse it only if present.
    if (ez_temp != NULL)
    {
      ret = VbEngineAlignCustomListParse(ez_temp);
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_engineErrorCode VbEnginePersistentLogParse( ezxml_t persistentLogConf )
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  ezxml_t              ez_temp;

  ez_temp = ezxml_child(persistentLogConf, "NumLines");

  if ((ez_temp != NULL) && (ez_temp->txt != NULL))
  {
    errno = 0;
    vbEngineConf.persistentLog.numLines = strtoul(ez_temp->txt, NULL, 0);

    if (errno != 0)
    {
      printf("ERROR (%d:%s) parsing .ini file: Invalid PersistentLog/NumLines value\n", errno, strerror(errno));
      ret = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(persistentLogConf, "VerboseLevel");

    if ((ez_temp != NULL) && (ez_temp->txt != NULL))
    {
      errno = 0;
      vbEngineConf.persistentLog.verboseLevel = strtoul(ez_temp->txt, NULL, 0);

      if (errno != 0)
      {
        printf("ERROR (%d:%s) parsing .ini file: Invalid PersistentLog/VerboseLevel value\n", errno, strerror(errno));
        ret = VB_ENGINE_ERROR_INI_FILE;
      }
    }
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ez_temp = ezxml_child(persistentLogConf, "Circular");

    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbEngineConf.persistentLog.circular = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
  }

  return ret;
}

/************************************************************************/

static t_VB_engineErrorCode VbEngineConfFileRead(const char *path)
{
  t_VB_engineErrorCode error =  VB_ENGINE_ERROR_NONE;
  ezxml_t              engine = NULL;
  ezxml_t              drivers_list;
  ezxml_t              ez_temp;
  ezxml_t              measure_conf;
  ezxml_t              cdta_conf;
  ezxml_t              socket_alive_conf;
  ezxml_t              user_profiles;
  ezxml_t              align_params;
  struct in6_addr      ip_addr = { 0 };
  INT16U               num_user_profiles;

  // Init default values
  vbEngineConf.engineId[0] = '\0';
  vbEngineConf.serverMode = VB_ENGINE_CONF_DEFAULT_SERVER_MODE;
  vbEngineConf.serverPort = 0;
  vbEngineConf.serverIface[0] = '\0';
  vbEngineConf.outputPath[0] = '\0';
  vbEngineConf.verboseLevel  = VB_ENGINE_CONF_DEFAULT_VERBOSE_LEVEL;
  vbEngineConf.consolePort   = VB_ENGINE_CONF_DEFAULT_CONSOLE_PORT;
  vbEngineConf.trafficMetricsEnabled = VB_ENGINE_CONF_DEFAULT_TRAFFIC_METRICS_ENABLED;
  vbEngineConf.saveMetricsEnabled = VB_ENGINE_CONF_DEFAULT_SAVE_METRICS;
  vbEngineConf.maxMetricsLogSize = VB_ENGINE_CONF_DEFAULT_METRICS_MAX_LOG_KB_SPACE;
  vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_DEC_BOOST] = VB_ENGINE_CONF_DEFAULT_BOOSTTHR_DEC_BOOST;
  vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_INC_BOOST] = VB_ENGINE_CONF_DEFAULT_BOOSTTHR_INC_BOOST;
  vbEngineConf.vdslCoex = VB_ENGINE_CONF_DEFAULT_VDSL_COEX;
  vbEngineConf.saveMeasures = VB_ENGINE_CONF_DEFAULT_SAVE_MEASURES;
  vbEngineConf.vbInUpstream = VB_ENGINE_CONF_DEFAULT_IN_UPSTREAM;
  vbEngineConf.boostAlgPeriod = VB_ENGINE_CONF_DEFAULT_BOOST_ALG_PERIOD;

  vbEngineConf.psdBandAllocation.numBands200Mhz = VB_ENGINE_HIGH_GRANULARITY_PSD_MNGT;
  vbEngineConf.psdBandAllocation.numBands100Mhz = VB_ENGINE_MEDIUM_GRANULARITY_PSD_MNGT;
  vbEngineConf.psdBandAllocation.lastCarrier[0] = VB_ENGINE_CONF_PSD_BAND_0_END; ///< 28  MHz (SISO & MIMO)
  vbEngineConf.psdBandAllocation.lastCarrier[1] = VB_ENGINE_CONF_PSD_BAND_1_END; ///< 50  MHz (SISO & MIMO)
  vbEngineConf.psdBandAllocation.lastCarrier[2] = VB_ENGINE_CONF_PSD_BAND_2_END; ///< 96  MHz (SISO & MIMO)
  vbEngineConf.psdBandAllocation.lastCarrier[3] = VB_ENGINE_CONF_PSD_BAND_3_END; ///< 140 MHz (SISO)
  vbEngineConf.psdBandAllocation.lastCarrier[4] = VB_ENGINE_CONF_PSD_BAND_4_END; ///< 200 MHz (SISO)

  vbEngineConf.measPlanConf.NumCycles          = VB_ENGINE_CONF_DEFAULT_MEAS_NUM_CYCLES;
  vbEngineConf.measPlanConf.StorageType        = VB_ENGINE_CONF_DEFAULT_MEAS_STORAGE_TYPE;
  vbEngineConf.measPlanConf.SymbolsNumber      = VB_ENGINE_CONF_DEFAULT_MEAS_SYMBOLS_NUMBER;
  vbEngineConf.measPlanConf.TimeAveraging      = VB_ENGINE_CONF_DEFAULT_MEAS_TIME_AVG;
  vbEngineConf.measPlanConf.FrecuencyAveraging = VB_ENGINE_CONF_DEFAULT_MEAS_FREQ_AVG;
  vbEngineConf.measPlanConf.Offset             = VB_ENGINE_CONF_DEFAULT_MEAS_OFFSET;
  vbEngineConf.measPlanConf.Duration           = VB_ENGINE_CONF_DEFAULT_MEAS_DURATION;
  vbEngineConf.measPlanConf.CFRMeasureType     = VB_ENGINE_CONF_DEFAULT_MEAS_CFR_TYPE;
  vbEngineConf.measPlanConf.MeasureDataType    = VB_ENGINE_CONF_DEFAULT_MEAS_DATA_TYPE;
  vbEngineConf.measPlanConf.MeasureDataFormat  = VB_ENGINE_CONF_DEFAULT_MEAS_DATA_FORMAT;

  vbEngineConf.seedConf.enabled                = VB_ENGINE_CONF_DEFAULT_SEED_ENABLE;
  vbEngineConf.seedConf.minSeedIndex           = VB_ENGINE_CONF_DEFAULT_SEED_MIN_INDEX;
  vbEngineConf.seedConf.maxSeedindex           = VB_ENGINE_CONF_DEFAULT_SEED_MAX_INDEX;
  vbEngineConf.seedConf.excludedMacListSize    = 0;
  vbEngineConf.seedConf.excludedMacList        = NULL;

  vbEngineConf.alignParams.alignMode           = VB_ENGINE_CONF_DEFAULT_ALIGN_MODE;
  vbEngineConf.alignParams.minPow              = VB_ENGINE_CONF_DEFAULT_ALIGN_MINPOW;
  vbEngineConf.alignParams.minPowHyst          = VB_ENGINE_CONF_DEFAULT_ALIGN_MINPOWHYST;
  vbEngineConf.alignParams.reliabilityThr      = VB_ENGINE_CONF_DEFAULT_ALIGN_RELTHR;
  vbEngineConf.alignParams.prioRefList.size    = 0;
  vbEngineConf.alignParams.prioRefList.list    = NULL;
  vbEngineConf.alignParams.metrics             = VB_ENGINE_CONF_DEFAULT_ALIGN_METRICS;
  vbEngineConf.alignParams.nodeCustomList.size = 0;
  vbEngineConf.alignParams.nodeCustomList.entries = NULL;

  vbEngineConf.socketAlive.enable              = VB_ENGINE_CONF_DEFAULT_SOCKALIVE_ENABLE;
  vbEngineConf.socketAlive.period              = VB_ENGINE_CONF_DEFAULT_SOCKALIVE_PERIOD;
  vbEngineConf.socketAlive.nLostMsgThr         = VB_ENGINE_CONF_DEFAULT_SOCKALIVE_NLOST_THR;
  
  vbEngineConf.persistentLog.numLines          = VB_ENGINE_CONF_DEFAULT_PERSLOG_NUMLINES;
  vbEngineConf.persistentLog.verboseLevel      = VB_ENGINE_CONF_DEFAULT_PERSLOG_VERBOSE;
  vbEngineConf.persistentLog.circular          = VB_ENGINE_CONF_DEFAULT_PERSLOG_CIRCULAR;

  if (path == NULL)
  {
    error = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }
  else
  {
    engine = ezxml_parse_file(path);

    if(engine == NULL)
    {
      printf("Engine Conf: .ini file could not be opened\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read ConnectionMode
    ez_temp = ezxml_child(engine, "ServerConnMode");
    if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbEngineConf.serverMode = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
    else
    {
      printf("Engine Conf: Error reading ServerConnMode parameter\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    if (vbEngineConf.serverMode)
    {
      // Read ServerIface
      ez_temp = ezxml_child(engine, "ServerIface");
      if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
      {
        strncpy(vbEngineConf.serverIface, ezxml_trimtxt(ez_temp), VB_ENGINE_CONF_MAX_IFACE_LENGTH);
        vbEngineConf.serverIface[VB_ENGINE_CONF_MAX_IFACE_LENGTH - 1] = '\0';
      }
      else
      {
        printf("Engine Conf: Error reading ServerIface parameter\n");
        error = VB_ENGINE_ERROR_INI_FILE;
      }

      if (error == VB_ENGINE_ERROR_NONE)
      {
        // Read ServerPort
        ez_temp = ezxml_child(engine, "ServerPort");
        if((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
        {
          errno = 0;
          vbEngineConf.serverPort = strtol(ezxml_txt(ez_temp), NULL, 0);

          if(errno != 0)
          {
            printf("Engine Conf: Error reading ServerPort parameter\n");
            error = VB_ENGINE_ERROR_INI_FILE;
          }
        }
        else
        {
          error = VB_ENGINE_ERROR_INI_FILE;
        }
      }
    } //serverMode
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read vdslCoex
    ez_temp = ezxml_child(engine, "vdslCoex");
    if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbEngineConf.vdslCoex = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
    else
    {
      printf("Engine Conf: Error reading vdslCoex parameter\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read SaveMeasuresToDisk
    ez_temp = ezxml_child(engine, "SaveMeasuresToDisk");
    if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbEngineConf.saveMeasures = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
    else
    {
      // Use default value
    }

    // Read OutputPath
    ez_temp = ezxml_child(engine, "OutputPath");
    if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      strncpy(vbEngineConf.outputPath, ezxml_trimtxt(ez_temp), VB_PARSE_MAX_PATH_LEN);
      vbEngineConf.outputPath[VB_PARSE_MAX_PATH_LEN - 1] = '\0';
    }
    else
    {
      printf("Engine Conf: Error reading OutputPath parameter\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read VerboseLevel
    ez_temp = ezxml_child(engine, "VerboseLevel");
    if((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.verboseLevel = strtol(ezxml_txt(ez_temp), NULL, 0);
      if(errno != 0)
      {
        printf("Engine Conf: Error incorrect value in VerboseLevel parameter\n");
        error = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read ConsolePort
    ez_temp = ezxml_child(engine, "ConsolePort");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.consolePort = (INT16U)strtol(ezxml_txt(ez_temp), NULL, 0);

      if (errno != 0)
      {
        printf("Engine Conf: Error incorrect value in ConsolePort parameter\n");
        error = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read VbInUpstream
    ez_temp = ezxml_child(engine, "VbInUpstream");
    if ((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbEngineConf.vbInUpstream = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
    else
    {
      // Use default value
    }

    // Read EnableTrafficAndBoostMetrics
    ez_temp = ezxml_child(engine, "EnableTrafficAndBoostMetrics");
    if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbEngineConf.trafficMetricsEnabled = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
    else
    {
      // Use default value
    }

    //Read SaveMetricsToDisk
    ez_temp = ezxml_child(engine, "SaveMetricsToDisk");
    if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      vbEngineConf.saveMetricsEnabled = (strcmp(ezxml_trimtxt(ez_temp), "YES") == 0)? TRUE:FALSE;
    }
    else
    {
      // Use default value
    }

    // Read MaxLogFileSizeKB
    ez_temp = ezxml_child(engine, "MaxLogFileSizeKB");
    if((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.maxMetricsLogSize = strtol(ezxml_txt(ez_temp), NULL, 0);

      if(errno != 0)
      {
        printf("Engine Conf: Error incorrect value in MaxLogFileSizeKB parameter\n");
        error = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read AlignMode
    ez_temp = ezxml_child(engine, "AlignMode");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.alignParams.alignMode = (t_VBAlignmentMode)strtol(ezxml_txt(ez_temp), NULL, 0);

      if (errno != 0)
      {
        printf("Engine Conf: Error incorrect value in AlignMode parameter\n");
        error = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read Boosting algorithm period
    ez_temp = ezxml_child(engine, "BoostAlgPeriod");
    if ((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      errno = 0;
      vbEngineConf.boostAlgPeriod = (INT16U)strtol(ezxml_txt(ez_temp), NULL, 0);

      if (errno != 0)
      {
        printf("Engine Conf: Error incorrect value in BoostAlgPeriod parameter\n");
        error = VB_ENGINE_ERROR_INI_FILE;
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read EngineId
    ez_temp = ezxml_child(engine, "EngineId");
    if((ez_temp != NULL) && (ezxml_trimtxt(ez_temp) != NULL))
    {
      strncpy(vbEngineConf.engineId, ezxml_trimtxt(ez_temp), VB_ENGINE_ID_MAX_SIZE);
      vbEngineConf.engineId[VB_ENGINE_ID_MAX_SIZE - 1] = '\0';
    }
    else
    {
      printf("Engine Conf: Error reading EngineId parameter\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    // Read MaxLogFileSizeKB
    ez_temp = ezxml_child(engine, "BoostThr");
    if((ez_temp != NULL) && (ezxml_txt(ez_temp) != NULL))
    {
      CHAR *temp_str = ezxml_txt(ez_temp);
      CHAR *dec_boost_thr;
      CHAR *inc_boost_thr;

      dec_boost_thr = strsep(&temp_str, ",");

      if (dec_boost_thr == NULL)
      {
        error = VB_ENGINE_ERROR_INI_FILE;
      }

      if (error == VB_ENGINE_ERROR_NONE)
      {
        inc_boost_thr = temp_str;

        if (inc_boost_thr == NULL)
        {
          error = VB_ENGINE_ERROR_INI_FILE;
        }
      }

      if (error == VB_ENGINE_ERROR_NONE)
      {
        vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_DEC_BOOST] = strtoul(dec_boost_thr, NULL, 0);

        if (errno != 0)
        {
          error = VB_ENGINE_ERROR_INI_FILE;
        }
      }

      if (error == VB_ENGINE_ERROR_NONE)
      {
        vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_INC_BOOST] = strtoul(inc_boost_thr, NULL, 0);

        if (errno != 0)
        {
          error = VB_ENGINE_ERROR_INI_FILE;
        }
      }
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    cdta_conf = ezxml_child(engine, "CDTA");

    if(cdta_conf != NULL)
    {
      error = VbEngineCDTAConfigurationParse( cdta_conf );
    }
    else
    {
      printf("Engine Conf: Error reading CDTA Configuration section\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    ezxml_t              seed_conf;

    seed_conf = ezxml_child(engine, "AutomaticSeed");

    if(seed_conf != NULL)
    {
      error = VbEngineSeedConfigurationParse(seed_conf);
    }
    else
    {
      printf("Engine Conf: Error reading Automatic Seed Configuration section\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }


  if (error == VB_ENGINE_ERROR_NONE)
  {
    measure_conf = ezxml_child(engine, "MeasureConfiguration");

    if(measure_conf != NULL)
    {
      error = VbEngineMeasureConfigurationParse( measure_conf );
    }
    else
    {
      // Use default value
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    socket_alive_conf = ezxml_child(engine, "SocketAlive");

    if(socket_alive_conf != NULL)
    {
      error = VbEngineSocketAliveConfigurationParse( socket_alive_conf );
    }
    else
    {
      // Use default value
    }
  }

  //Parse drivers list only in client mode
  if ((error == VB_ENGINE_ERROR_NONE) && (vbEngineConf.serverMode == FALSE))
  {
    drivers_list = ezxml_child(engine, "DriversList");

    if (drivers_list != NULL)
    {
      error = DriverListParse(drivers_list);
    }
    else
    {
      printf("Engine Conf: Error DriversList not defined\n");
      error = VB_ENGINE_ERROR_INI_FILE;
    }
  }
  else if((error == VB_ENGINE_ERROR_NONE) && (vbEngineConf.serverMode == TRUE))
  {
    // Init EAInterface with the server thread
    error = VbEngineEAInterfaceInit(vbEngineConf.serverIface, ip_addr,
        vbEngineConf.serverPort, NULL, VB_EA_TYPE_SERVER);
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    ezxml_t              slas_definition;

    slas_definition = ezxml_child(engine, "SLAsDefinition");
    if(slas_definition != NULL)
    {
      for (slas_definition = ezxml_child(slas_definition, "SLAx"), vbEngineConf.sla.numSlas = 0;
          ((slas_definition != NULL) && (vbEngineConf.sla.numSlas < VB_ENGINE_CONF_SLA_DEFINITION_MAX)) ; slas_definition = slas_definition->next, vbEngineConf.sla.numSlas++)
      {
        error = SLAsDefinitionFileParse(slas_definition, vbEngineConf.sla.numSlas);
      }
    }
  }

  if(error == VB_ENGINE_ERROR_NONE)
  {
    user_profiles = ezxml_child(engine, "UserProfiles");
    if(user_profiles != NULL)
    {
      for (user_profiles = ezxml_child(user_profiles, "UserProfile"), num_user_profiles = 0;
           user_profiles != NULL ; user_profiles = user_profiles->next, num_user_profiles++)
      {
        error = UserProfileFileParse(user_profiles, num_user_profiles);
      }
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    align_params = ezxml_child(engine, "AlignParams");

    if (align_params != NULL)
    {
      error = VbEngineAlignParamsParse( align_params );
    }
  }

  if (error == VB_ENGINE_ERROR_NONE)
  {
    align_params = ezxml_child(engine, "PersistentLog");

    if (align_params != NULL)
    {
      error = VbEnginePersistentLogParse( align_params );
    }
  }

  if (engine != NULL)
  {
    ezxml_free(engine);
  }

  return error;
}

/************************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/************************************************************************/

t_VB_engineErrorCode VbEngineConfParse(int argc, char **argv)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;
  int                  opt;
  CHAR                 engine_ini_file[MAX_FILE_NAME_LENGTH] = VB_ENGINE_CONF_DEFAULT_INI_FILE;

  while ((opt = getopt(argc, argv, "cf:h")) != -1)
  {
    switch (opt)
    {
      case ('f'):
      {
        strncpy(engine_ini_file, optarg, MAX_FILE_NAME_LENGTH);
        engine_ini_file[MAX_FILE_NAME_LENGTH - 1] = '\0';
        break;
      }
      default:
      {
        ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
        break;
      }
    }
  }

  if (ret != VB_ENGINE_ERROR_NONE)
  {
    printf("Command line:\n\tvector_boost_engine [-f PATHFILEINI] [-h] \n");
    printf("Where:\n");
    printf("\t-f\tThis option allows the user to select ini file (length max %d).\n\t\tPATHFILEINI has to be the entire path name\n", MAX_FILE_NAME_LENGTH);
    printf("\t-h\tShow this help\n");
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    ret = VbEngineConfFileRead(engine_ini_file);

    if (ret != VB_ENGINE_ERROR_NONE)
    {
      printf("Configuration error (%d) parsing file %s!\n", ret, engine_ini_file);
    }
  }

  return ret;
}

/*******************************************************************/

void VbEngineConfDump(t_writeFun writeFun)
{
  INT32U i;

  writeFun("===================================================================================\n");
  writeFun("|                     Parameter                    |        Value                 |\n");
  writeFun("===================================================================================\n");
  writeFun("| %-48s | %28s |\n",               "Engine Id",              vbEngineConf.engineId);
  writeFun("| %-48s | %28s |\n",               "Engine connection mode", vbEngineConf.serverMode ? "SERVER" : "CLIENT");
  if(vbEngineConf.serverMode)
  {
    writeFun("| %-48s | %28s |\n",               "Server - interface", vbEngineConf.serverIface);
    writeFun("| %-48s | %28u |\n",               "Server - port",      vbEngineConf.serverPort);
  }
  writeFun("| %-48s | %28s |\n",               "Debug Output Path",    vbEngineConf.outputPath);
  writeFun("| %-48s | %28s |\n",               "Log Verbose Level",    VbVerboseLevelToStr(vbEngineConf.verboseLevel));
  writeFun("| %-48s | %28u |\n",               "Console Port",         vbEngineConf.consolePort);
  writeFun("| %-48s | %28s |\n",               "Save Meas to disk",    vbEngineConf.saveMeasures?"ENABLED":"DISABLED");
  writeFun("| %-48s | %28s |\n",               "Traffic metrics",      vbEngineConf.trafficMetricsEnabled?"ENABLED":"DISABLED");
  writeFun("| %-48s | %28s |\n",               "Save metrics to disk", vbEngineConf.saveMetricsEnabled?"ENABLED":"DISABLED");
  writeFun("| %-48s | %28u |\n",               "Max metrics log size", vbEngineConf.maxMetricsLogSize);
  writeFun("| %-48s | %28s |\n",               "VDSL Coex",            vbEngineConf.vdslCoex?"ENABLED":"DISABLED");
  writeFun("| %-48s | %28s |\n",               "VB in Upstream",       vbEngineConf.vbInUpstream?"ENABLED":"DISABLED");
  writeFun("| %-48s | %25u ms |\n",            "Boost - algorithm period",     vbEngineConf.boostAlgPeriod);
  writeFun("| %-48s |                     %3u /%3u |\n", "Boost - thresholds", vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_DEC_BOOST],
                                                                      vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_INC_BOOST]);

  writeFun("| %-48s | %28s |\n",               "CDTA - status",       vbEngineConf.cdtaConf.enabled?"ENABLED":"DISABLED");
  writeFun("| %-48s | %28u |\n",               "CDTA - max Down/Up Rate",            vbEngineConf.cdtaConf.maxDownUpRate);
  writeFun("| %-48s | %28u |\n",               "CDTA - min Down/Up",                 vbEngineConf.cdtaConf.minDownUpRate);
  writeFun("| %-48s | %28u |\n",               "PSD bands - Spect. Alloc. bands (200MHz)", vbEngineConf.psdBandAllocation.numBands200Mhz);
  writeFun("| %-48s | %28u |\n",               "PSD bands - Spect. Alloc. bands (100MHz)", vbEngineConf.psdBandAllocation.numBands100Mhz);

  for(i = 0; i < vbEngineConf.psdBandAllocation.numBands200Mhz; i++)
  {
    writeFun("| %-44s (%1u) | %28u |\n",               "PSD bands - Band", i, vbEngineConf.psdBandAllocation.lastCarrier[i]);
  }

  for(i = 0; i < vbEngineConf.sla.numSlas; i++)
  {
    writeFun("| %-48s |  %16s   %5u %2u |\n", "Profiles - SLA, Mbps, Weight", vbEngineConf.sla.definition[i].name,
                                                                  vbEngineConf.sla.definition[i].slaMbps,
                                                                  vbEngineConf.sla.definition[i].weight);
  }

  for(i = 0; i < VB_ENGINE_CONF_USER_PROFILE_MAX; i++)
  {
    if(vbEngineConf.userProfile[i].sla)
    {
      writeFun("| %-48s |   %02x:%02x:%02x:%02x:%02x:%02x / %6u |\n", "Profiles - MAC, SLA", vbEngineConf.userProfile[i].mac[0],
                                                                                         vbEngineConf.userProfile[i].mac[1],
                                                                                         vbEngineConf.userProfile[i].mac[2],
                                                                                         vbEngineConf.userProfile[i].mac[3],
                                                                                         vbEngineConf.userProfile[i].mac[4],
                                                                                         vbEngineConf.userProfile[i].mac[5],
                                                                                         vbEngineConf.userProfile[i].sla);
    }
  }

  writeFun("| %-48s | %28s |\n",               "Alignment - mode",                VbEngineAlignModeStringGet(vbEngineConf.alignParams.alignMode));
  writeFun("| %-48s | %28lu |\n",              "Alignment - reliability thr (%)", vbEngineConf.alignParams.reliabilityThr);
  writeFun("| %-48s | %18lu (%7.2f) |\n",      "Alignment - min pow",             vbEngineConf.alignParams.minPow, VbEngineAlignAdcOutToFPGet(vbEngineConf.alignParams.minPow));
  writeFun("| %-48s | %18lu (%7.2f) |\n",      "Alignment - min pow hyst",        vbEngineConf.alignParams.minPowHyst, VbEngineAlignAdcOutToFPGet(vbEngineConf.alignParams.minPowHyst));
  writeFun("| %-48s | %28s |\n",               "Alignment - metrics",             vbEngineConf.alignParams.metrics?"ENABLED":"DISABLED");

  if (VbEngineConfAlignPrioRefEnabled() == TRUE)
  {
    INT32U idx;

    for (idx = 0; idx < vbEngineConf.alignParams.prioRefList.size; idx++)
    {
      t_VB_engineErrorCode err;
      INT8U               *prio_ref_mac;

      err = VbEngineConfAlignPrioRefItemGet(idx, &prio_ref_mac);

      if (err == VB_ENGINE_ERROR_NONE)
      {
        writeFun("| %-43s (%2lu) |            " MAC_PRINTF_FORMAT " |\n",  "Alignment - Prio reference node",
            idx,
            MAC_PRINTF_DATA(prio_ref_mac));
      }
    }
  }

  writeFun("| %-48s | %28u |\n",               "Alignment - num custom entries",         vbEngineConf.alignParams.nodeCustomList.size);

  {
    INT32U entry_idx;

    for (entry_idx = 0; entry_idx < vbEngineConf.alignParams.nodeCustomList.size; entry_idx++)
    {
      INT32U black_list_idx;

      writeFun("| %-40s (%2lu)    |         " MAC_PRINTF_FORMAT "    |\n",  "Alignment - custom entry - target MAC",
          entry_idx,
          MAC_PRINTF_DATA(vbEngineConf.alignParams.nodeCustomList.entries[entry_idx].targetNode));

      writeFun("| %-48s | %28u |\n", "Alignment - custom entry - blacklist items",
          vbEngineConf.alignParams.nodeCustomList.entries[entry_idx].blackList.size);

      for (black_list_idx = 0; black_list_idx < vbEngineConf.alignParams.nodeCustomList.entries[entry_idx].blackList.size; black_list_idx++)
      {
        t_VB_engineErrorCode err;
        INT8U               *black_list_mac;

        err = VbEngineAlignBlackListItemGet(entry_idx, black_list_idx, &black_list_mac);

        if (err == VB_ENGINE_ERROR_NONE)
        {
          writeFun("| %-40s (%2lu,%2lu) |            " MAC_PRINTF_FORMAT " |\n",  "Alignment - custom entry - blacklist MAC",
              entry_idx,
              black_list_idx,
              MAC_PRINTF_DATA(black_list_mac));
        }
      }
    }
  }

  writeFun("| %-48s | %28s |\n",               "Automatic seeds - status",  vbEngineConf.seedConf.enabled?"ENABLED":"DISABLED");
  writeFun("| %-48s | %28u |\n",               "Automatic seeds - Min seed index",  vbEngineConf.seedConf.minSeedIndex);
  writeFun("| %-48s | %28u |\n",               "Automatic seeds - Max seed index",  vbEngineConf.seedConf.maxSeedindex);
  {
    INT32U idx;

    for (idx = 0; idx < vbEngineConf.seedConf.excludedMacListSize; idx++)
    {
      writeFun("| %-43s (%2lu) |            %02x:%02x:%02x:%02x:%02x:%02x |\n",  "Automatic seeds - Excluded MAC ", idx, vbEngineConf.seedConf.excludedMacList[0 + (idx*ETH_ALEN)],
                                                                                                              vbEngineConf.seedConf.excludedMacList[1 + (idx*ETH_ALEN)],
                                                                                                              vbEngineConf.seedConf.excludedMacList[2 + (idx*ETH_ALEN)],
                                                                                                              vbEngineConf.seedConf.excludedMacList[3 + (idx*ETH_ALEN)],
                                                                                                              vbEngineConf.seedConf.excludedMacList[4 + (idx*ETH_ALEN)],
                                                                                                              vbEngineConf.seedConf.excludedMacList[5 + (idx*ETH_ALEN)]);
    }
  }

  writeFun("| %-48s | %28u |\n",               "Persistent log - Number of lines",  vbEngineConf.persistentLog.numLines);
  writeFun("| %-48s | %28s |\n",               "Persistent log - Verbose level",    VbVerboseLevelToStr(vbEngineConf.persistentLog.verboseLevel));
  writeFun("| %-48s | %28s |\n",               "Persistent log - Circular",         vbEngineConf.persistentLog.circular?"ENABLED":"DISABLED");

  writeFun("| %-48s | %28s |\n",               "Socket Alive - status",  vbEngineConf.socketAlive.enable?"ENABLED":"DISABLED");
  writeFun("| %-48s | %28u |\n",               "Socket Alive - Period",  vbEngineConf.socketAlive.period);
  writeFun("| %-48s | %28u |\n",               "Socket Alive - Nlost",   vbEngineConf.socketAlive.nLostMsgThr);
  writeFun("===================================================================================\n");
}

/*******************************************************************/

BOOL VbEngineConfServerConnModeGet(void)
{
  return vbEngineConf.serverMode;
}

/*******************************************************************/

CHAR *VbEngineConfOutputPathGet(void)
{
  return vbEngineConf.outputPath;
}

/*******************************************************************/

CHAR *VbEngineConfEngineIdGet(void)
{
  return vbEngineConf.engineId;
}

/*******************************************************************/

t_vbLogLevel VbEngineConfVerboseLevelGet(void)
{
  return vbEngineConf.verboseLevel;
}

/*******************************************************************/

INT16U VbEngineConfConsolePortGet(void)
{
  return vbEngineConf.consolePort;
}

/*******************************************************************/

BOOL VbEngineConfTrafficMetricsEnabledGet(void)
{
  return vbEngineConf.trafficMetricsEnabled;
}

/*******************************************************************/

BOOL VbEngineConfSaveMetricsEnabledGet(void)
{
  return vbEngineConf.saveMetricsEnabled;
}

/*******************************************************************/

INT32U VbEngineConfMaxMetricsLogSizeGet(void)
{
  return vbEngineConf.maxMetricsLogSize;
}

/*******************************************************************/

BOOLEAN VbEngineConfVdslCoexGet(void)
{
  return vbEngineConf.vdslCoex;
}

/*******************************************************************/

BOOLEAN VbEngineConfSaveMeasuresGet(void)
{
  return vbEngineConf.saveMeasures;
}

/*******************************************************************/

BOOLEAN VbEngineConfVbInUpstreamGet(void)
{
  return vbEngineConf.vbInUpstream;
}

/*******************************************************************/

BOOLEAN VbEngineConfCDTAEnableGet(void)
{
  return vbEngineConf.cdtaConf.enabled;
}

/*******************************************************************/

t_psdBandAllocation* VbEngineConfPSDBandAllocationGet(void)
{
  return &vbEngineConf.psdBandAllocation;
}

/*******************************************************************/

INT16U VbEngineConfNumPSDBandAllocationGet(t_vbTxBandPlanMode txMode)
{
  return ((txMode == VB_TX_MODE_100_MHZ)?vbEngineConf.psdBandAllocation.numBands100Mhz:vbEngineConf.psdBandAllocation.numBands200Mhz);
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfBoostThrSet(INT32U *boostThr)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if (boostThr == NULL)
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_DEC_BOOST] = boostThr[VB_BOOST_THR_TYPE_DEC_BOOST];
    vbEngineConf.boostThresholds[VB_BOOST_THR_TYPE_INC_BOOST] = boostThr[VB_BOOST_THR_TYPE_INC_BOOST];
  }

  return ret;
}

/*******************************************************************/

INT32U VbEngineConfBoostThrGet(t_vbBoostThrType type)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  INT32U               thr_val = 0;

  if (type >= VB_BOOST_THR_TYPE_LAST)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    thr_val = vbEngineConf.boostThresholds[type];
  }

  return thr_val;
}


/*******************************************************************/

t_VB_engineErrorCode VbEngineConfMeasurePlanGet(t_measconfdata **measConfData)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

  if (measConfData == NULL)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    *measConfData = &vbEngineConf.measPlanConf;
  }

  return err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfSocketAliveGet(t_socketAlive **socketAliveConf)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

  if (socketAliveConf == NULL)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    *socketAliveConf = &vbEngineConf.socketAlive;
  }

  return err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfCDTADataGet(t_cdtaConf **cdtaConf)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

  if (cdtaConf == NULL)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    *cdtaConf = &vbEngineConf.cdtaConf;
  }

  return err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfAlignmentModeGet(t_VBAlignmentMode *alignmentMode)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;

  if (alignmentMode == NULL)
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (err == VB_ENGINE_ERROR_NONE)
  {
    *alignmentMode = vbEngineConf.alignParams.alignMode;
  }

  return err;
}

/*******************************************************************/

INT16U VbEngineConfBoostAlgPeriodGet(void)
{
  return vbEngineConf.boostAlgPeriod;
}

/*******************************************************************/

INT32U VbEngineConfAlignMinPowGet(void)
{
  return vbEngineConf.alignParams.minPow;
}

/*******************************************************************/

INT32U VbEngineConfAlignMinPowHystGet(void)
{
  return vbEngineConf.alignParams.minPowHyst;
}

/*******************************************************************/

BOOLEAN VbEngineConfAlignMetricsEnabled(void)
{
  return vbEngineConf.alignParams.metrics;
}

/*******************************************************************/

BOOLEAN VbEngineConfAlignPrioRefEnabled(void)
{
  BOOLEAN is_enabled = (vbEngineConf.alignParams.prioRefList.size > 0);

  return is_enabled;
}

/*******************************************************************/

INT32U VbEngineConfAlignPrioRefListSizeGet(void)
{
  INT32U size = vbEngineConf.alignParams.prioRefList.size;

  return size;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfAlignPrioRefItemGet(INT32U prioRefIdx, INT8U **prioRefMac)
{
  t_VB_engineErrorCode ret = VB_ENGINE_ERROR_NONE;

  if ((prioRefMac == NULL) ||
      (vbEngineConf.alignParams.prioRefList.list == NULL) ||
      (prioRefIdx >= vbEngineConf.alignParams.prioRefList.size))
  {
    ret = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if (ret == VB_ENGINE_ERROR_NONE)
  {
    *prioRefMac = vbEngineConf.alignParams.prioRefList.list + (prioRefIdx * ETH_ALEN);
  }

  return ret;
}

/*******************************************************************/

INT32U VbEngineConfAlignRelThrGet(void)
{
  return vbEngineConf.alignParams.reliabilityThr;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfProfileGet(INT8U *mac, INT32U *sla, INT32U* slaWeight, INT32U* userWeight)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  INT32U               i = 0;
  BOOLEAN              found = FALSE;

  if((mac == NULL) || (sla == NULL) || (slaWeight == NULL) || (userWeight == NULL))
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(err == VB_ENGINE_ERROR_NONE)
  {
    for(i=0; i<VB_ENGINE_CONF_USER_PROFILE_MAX; i++)
    {
      if(memcmp(vbEngineConf.userProfile[i].mac, mac, ETH_ALEN) == 0)
      {
        *sla = vbEngineConf.userProfile[i].sla;
        *slaWeight = vbEngineConf.userProfile[i].slaWeight;
        *userWeight = vbEngineConf.userProfile[i].userWeight;
        found = TRUE;
        break;
      }
    }

    if(found == FALSE)
    {
      *sla = VB_ENGINE_CONF_SLA_DEFAULT_MBPS;
      *slaWeight = VB_ENGINE_CONF_SLA_DEFAULT_SLA_WEIGHT;
      *userWeight = VB_ENGINE_CONF_SLA_DEFAULT_USER_WEIGHT;
      err = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  return err;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfSLAProfileGet(INT8U *mac, INT32U *sla)
{
  t_VB_engineErrorCode err = VB_ENGINE_ERROR_NONE;
  INT32U               i = 0;
  BOOLEAN              found = FALSE;

  if((mac == NULL) || (sla == NULL))
  {
    err = VB_ENGINE_ERROR_BAD_ARGUMENTS;
  }

  if(err == VB_ENGINE_ERROR_NONE)
  {
    for(i=0; i<VB_ENGINE_CONF_USER_PROFILE_MAX; i++)
    {
      if(memcmp(vbEngineConf.userProfile[i].mac, mac, ETH_ALEN) == 0)
      {
        *sla = vbEngineConf.userProfile[i].sla;
        found = TRUE;
        break;
      }
    }

    if(found == FALSE)
    {
      *sla = 1000;
      err = VB_ENGINE_ERROR_NOT_FOUND;
    }
  }

  return err;
}

/*******************************************************************/

BOOLEAN VbEngineConfSeedAutomaticEnableGet(void)
{
  return vbEngineConf.seedConf.enabled;
}

/*******************************************************************/

INT16U VbEngineConfSeedMinIndexGet(void)
{
  return vbEngineConf.seedConf.minSeedIndex;
}

/*******************************************************************/

INT16U VbEngineConfSeedMaxIndexGet(void)
{
  return vbEngineConf.seedConf.maxSeedindex;
}

/*******************************************************************/

INT32U VbEngineConfPeristentLogNumLinesGet(void)
{
  return vbEngineConf.persistentLog.numLines;
}

/*******************************************************************/

t_vbLogLevel VbEngineConfPeristentLogVerboseLevelGet(void)
{
  return vbEngineConf.persistentLog.verboseLevel;
}

/*******************************************************************/

BOOLEAN VbEngineConfPersistentLogIsCircular(void)
{
  return vbEngineConf.persistentLog.circular;
}

/*******************************************************************/

BOOLEAN VbEngineConfSocketAliveEnableGet(void)
{
  return vbEngineConf.socketAlive.enable;
}

/*******************************************************************/

INT32U VbEngineConfSocketAlivePeriodGet(void)
{
  return vbEngineConf.socketAlive.period;
}

/*******************************************************************/

INT32U VbEngineConfSocketAliveNLostThrGet(void)
{
  return vbEngineConf.socketAlive.nLostMsgThr;
}

/*******************************************************************/

INT16U VbEngineConfSeedExcludedMacListSizeGet(void)
{
  return vbEngineConf.seedConf.excludedMacListSize;
}

/*******************************************************************/

INT8U *VbEngineConfSeedExcludedMacListGet(void)
{
  return vbEngineConf.seedConf.excludedMacList;
}

/*******************************************************************/

t_VB_engineErrorCode VbEngineConfReleaseResources(void)
{
  if (vbEngineConf.seedConf.excludedMacList != NULL)
  {
    free(vbEngineConf.seedConf.excludedMacList);
    vbEngineConf.seedConf.excludedMacList = NULL;
  }

  VbEngineAlignCustomRelease();

  return VB_ENGINE_ERROR_NONE;
}

/*******************************************************************/

BOOLEAN VbEngineConfAlignBlackListIsMacAllowed(INT8U *targetMac, INT8U *macToCheck)
{
  INT32U  entry_idx;
  BOOLEAN is_allowed = TRUE;

  if ((VbEngineConfAlignBlackListIsEnabled() == TRUE) &&
      (vbEngineConf.alignParams.nodeCustomList.entries != NULL))
  {
    for (entry_idx = 0; entry_idx < vbEngineConf.alignParams.nodeCustomList.size; entry_idx++)
    {
      t_alignParamsCustomEntry *entry = &(vbEngineConf.alignParams.nodeCustomList.entries[entry_idx]);

      if (MACAddrQuickCmp(targetMac, entry->targetNode) == TRUE)
      {
        INT32U blacklist_idx;

        // Target node found in list. Now check if given MAC is in its blacklist
        if (entry->blackList.list != NULL)
        {
          for (blacklist_idx = 0; blacklist_idx < entry->blackList.size; blacklist_idx++)
          {
            t_VB_engineErrorCode err;
            INT8U               *blacklist_mac;

            // Get next MAC in black list
            err = VbEngineAlignBlackListItemGet(entry_idx, blacklist_idx, &blacklist_mac);

            if ((err == VB_ENGINE_ERROR_NONE) &&
                (MACAddrQuickCmp(blacklist_mac, macToCheck) == TRUE))
            {
              // MAC found in blacklist
              is_allowed = FALSE;

              break;
            }
          }
        }

        break;
      }
    }
  }

  return is_allowed;
}

/*******************************************************************/

BOOLEAN VbEngineConfAlignBlackListIsEnabled(void)
{
  BOOLEAN is_enabled;

  is_enabled = (vbEngineConf.alignParams.nodeCustomList.size > 0);

  return is_enabled;
}

/*******************************************************************/

/**
 * @}
 **/
