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
 * @file vb_DataModel.c
 * @brief Implements driver data model
 *
 * @internal
 *
 * @author
 * @date 23/12/2014
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
#include <pthread.h>

#include "vb_DataModel.h"
#include "vb_LCMP_com.h"
#include "vb_log.h"
#include "vb_counters.h"
#include "vb_EA_interface.h"

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

#define VB_DMDISCOVER_VALUE_SIZE                    (sizeof(t_VB_DMDiscover_Value))
#define VB_EPDISCOVER_VALUE_SIZE                    (sizeof(t_VB_EPDiscover_Value))
#define VB_TIME_STAMP_MAC_MSSIZE                    (sizeof(t_VB_TimeStampMacms_Value))
#define VB_ADDINFO1_VALUE_SIZE                      (sizeof(t_VB_AdditionalInfo1_Value))
#define VB_DOMAINS_LISTS                            (2)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct __attribute__ ((packed))
{
  INT8U MAC[ETH_ALEN];
  INT8U DevID;
  INT16U Extseed;
  INT8U NumEps;
} t_VB_DMDiscover_Value;

typedef struct __attribute__ ((packed))
{
  INT8U   MAC[ETH_ALEN];
  INT8U   fwVersion[10];
  INT32U  qosRate:16;
  INT32U  maxLengthTxop:16;
  INT32U  reserved1;
  INT32U  reserved2;
  INT32U  reserved3;
  INT32U  reserved4;
} t_VB_AdditionalInfo1_Value;

typedef struct __attribute__ ((packed))
{
  INT8U MAC[ETH_ALEN];
  INT8U DevID;
} t_VB_EPDiscover_Value;

/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_mutex_t       vbDatamodelMutexDomains = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t       vbDatamodelMutexDomainsDiscover = PTHREAD_MUTEX_INITIALIZER;
static t_DomainsList         vbDatamodelDomainsList[VB_DOMAINS_LISTS];
static t_DomainsList        *vbDatamodelDomains;
static t_DomainsList        *vbDatamodelDomainsNew;
static t_DomainChangeDetails vbDatamodelDomainsChanges = {0};

/*
 ************************************************************************
 ** Private function definition
 ************************************************************************
 */

/**
 * @brief This function looks for a DM's MAC in a domains list
 * @param[in] DMMAC Pointer to MAC for looking for
 * @param[in] DomainsList Pointer to Domains List where look for
 * @return pointer to DM found or NULL
**/
static t_Domains *VbDatamodelDmFind(const INT8U *dmMac);

/**
 * @brief Initiate the domain table
 */
static void VbDatamodelMacDomainsListInit( void );

static void VbDatamodelListEpsDestroy( t_EpsList *epsLists );

static t_VB_comErrorCode VbDatamodelEpAdd(t_Domains *domain, const INT8U *mac, INT8U devID);

/**
 * @brief Creates a new instance of Domain MACs list and add at tail
 * @param[in] mac
 * @param[in] devID
 * @param[in] extSeed
 * @return Pointer to new list element or NULL if malloc error.
**/
static t_VB_comErrorCode VbDatamodelDomainAdd(const INT8U *mac, INT8U devID, INT16U extSeed, t_Domains **newDomain);

static BOOL VbDatamodelLookforMacinMacsArray( INT8U *mac, INT8U* arrayMacs, INT16U numMacs );

static t_node *VbDatamodelActiveEpFind(const INT8U *epMac, t_EpsList *epsList);

static t_Domains *VbDatamodelActiveDmFind(const INT8U *dmMac);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelTrafficInfoInit(t_node *node)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (node == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    bzero(&(node->ingressTraffic), sizeof(node->ingressTraffic));
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelTrafficTimeStampInit(t_IngressTraffic *ingressTraffic)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (ingressTraffic == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    clock_gettime(CLOCK_MONOTONIC, &(ingressTraffic->lastRxTimeStamp));
  }

  return ret;
}

/*******************************************************************/

static BOOLEAN VbDatamodelLookforMacinMacsArray( INT8U *mac, INT8U* arrayMacs, INT16U numMacs )
{
  BOOLEAN   found = FALSE;
  INT16U    i = 0;

  if((mac != NULL) && (arrayMacs != NULL) && (numMacs > 0))
  {
    while((i < numMacs) && (!found))
    {
      if( memcmp(mac,&arrayMacs[i*ETH_ALEN],ETH_ALEN) == 0 )
      {
        found = TRUE;
      }
      i++;
    }
  }

  return found;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainsListDestroy(t_DomainsList *domainsList)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT16U            domain_idx;
  t_Domains        *domain;

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  if (domainsList == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (domainsList->domainsArray != NULL)
    {
      for (domain_idx = 0; domain_idx < domainsList->NumDomains; domain_idx++)
      {
        domain = (t_Domains *)(&(domainsList->domainsArray[domain_idx]));

        VbDatamodelListEpsDestroy(&(domain->eps));
        domain->dm.used = FALSE;
      }

      domainsList->NumDomains = 0;

      free(domainsList->domainsArray);
      domainsList->domainsArray = NULL;
    }
    else
    {
      // Ensure number of domains is 0
      domainsList->NumDomains = 0;
    }
  }

  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainsListAlloc(INT32U numDomains)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  vbDatamodelDomainsNew->NumDomains = numDomains;
  vbDatamodelDomainsNew->domainsArray = (t_Domains *)calloc(1, sizeof(t_Domains) * numDomains);

  if (vbDatamodelDomainsNew->domainsArray == NULL)
  {
    ret = VB_COM_ERROR_MALLOC;
  }

  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainAdd(const INT8U *mac, INT8U devID, INT16U extSeed, t_Domains **newDomain)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_Domains        *new_domain;
  INT32U            domain_idx;

  if ((mac == NULL) || (newDomain == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VB_COM_ERROR_NOT_FOUND;

    pthread_mutex_lock( &vbDatamodelMutexDomains );

    // Look for a free entry
    for (domain_idx = 0; domain_idx < vbDatamodelDomainsNew->NumDomains; domain_idx++)
    {
      new_domain = &vbDatamodelDomainsNew->domainsArray[domain_idx];

      if (new_domain->dm.used == FALSE)
      {
        // Free entry found, fill with new info
        new_domain->dm.used = TRUE;
        MACAddrClone(new_domain->dm.MAC, mac);
        MACAddrMem2str(new_domain->dm.MACStr, mac);
        new_domain->dm.devID = devID;
        new_domain->dm.type = VB_NODE_DOMAIN_MASTER;
        new_domain->dm.linkedNode = NULL;
        new_domain->dm.addInfo1.extSeed = extSeed;
        new_domain->dm.addInfo1.maxLengthTxop = 0;
        new_domain->dm.addInfo1.qosRate = 0;
        strcpy((char *)new_domain->dm.addInfo1.fwVersion, "r000");
        new_domain->dm.cap.trafficReports = TRUE;

        // Init traffic info
        VbDatamodelTrafficInfoInit(&(new_domain->dm));

        // Update output
        *newDomain = new_domain;

        ret = VB_COM_ERROR_NONE;
        break;
      }
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );

  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelEpAdd(t_Domains *domain, const INT8U *mac, INT8U devID)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_node           *ep = NULL;
  INT32U            ep_idx;

  if ((mac == NULL) || (domain == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    ret = VB_COM_ERROR_NOT_FOUND;

    pthread_mutex_lock( &vbDatamodelMutexDomains );

    // Look for a free entry
    for (ep_idx = 0; ep_idx < VB_MAX_EPS; ep_idx++)
    {
      ep = &(domain->eps.epsArray[ep_idx]);

      if (ep->used == FALSE)
      {
        ep->used = TRUE;
        MACAddrClone(ep->MAC, mac);
        MACAddrMem2str(ep->MACStr, mac);
        ep->devID = devID;
        ep->type = VB_NODE_END_POINT;
        ep->linkedNode = &(domain->dm);
        domain->dm.linkedNode = ep;
        strcpy((char *)ep->addInfo1.fwVersion, "r000");
        ep->cap.trafficReports = TRUE;

        // Init traffic info
        VbDatamodelTrafficInfoInit(ep);

        // Update number of EPs
        domain->eps.numEps++;

        ret = VB_COM_ERROR_NONE;
        break;
      }
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );

  }

  return ret;
}

/*******************************************************************/

static void VbDatamodelListEpsDestroy( t_EpsList *epsLists )
{
  t_VB_comErrorCode err = VB_COM_ERROR_NONE;
  t_node           *this_ep;
  INT32U            num_ep;

  if (epsLists == NULL)
  {
    err = VB_COM_ERROR_BAD_ARGS;
  }

  if (err == VB_COM_ERROR_NONE)
  {
    for (num_ep = 0 ; num_ep < epsLists->numEps ; num_ep++)
    {
      this_ep = &(epsLists->epsArray[num_ep]);
      this_ep->used = FALSE;
    }

    epsLists->numEps = 0;
  }
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelNodeCapabilitiesUpdate(t_node *node)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;

  if (node == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // To be completed with capabilities, if needed
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelAllNodesCapabilitiesUpdate(void)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx = 0;
  INT32U            ep_idx = 0;
  t_EpsList        *eps_list = NULL;

  if (vbDatamodelDomains != NULL)
  {
    // Loop through all nodes
    for (domain_idx = 0; (domain_idx < vbDatamodelDomains->NumDomains) && (ret == VB_COM_ERROR_NONE); domain_idx++)
    {
      // Get EPs list
      eps_list = &(vbDatamodelDomains->domainsArray[domain_idx].eps);

      for (ep_idx = 0; (ep_idx < eps_list->numEps) && (ret == VB_COM_ERROR_NONE); ep_idx++)
      {
        ret = VbDatamodelNodeCapabilitiesUpdate(&(eps_list->epsArray[ep_idx]));
      }
    }
  }

  if (ret != VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error updating capabilities: %d", ret);
  }

  return ret;
}

/*******************************************************************/

static t_Domains *VbDatamodelDmFind(const INT8U *dmMac)
{
  t_Domains *return_dm = NULL;
  INT8U *dm_mac;
  INT16U num_domains_check;

  if ((vbDatamodelDomains != NULL) && (dmMac != NULL))
  {
    for (num_domains_check = 0 ; num_domains_check < vbDatamodelDomains->NumDomains ; num_domains_check++)
    {
      dm_mac = vbDatamodelDomains->domainsArray[num_domains_check].dm.MAC;

      if (memcmp(dm_mac,dmMac,ETH_ALEN) == 0)
      {
        return_dm = &vbDatamodelDomains->domainsArray[num_domains_check];
        break;
      }
    }
  }

  return return_dm;
}

/*******************************************************************/

static t_node *VbDatamodelEpFind(const INT8U *epMac, t_EpsList *epsList)
{
  t_node *return_ep = NULL;
  INT16U num_eps_check;

  if((epsList != NULL) && (epMac != NULL))
  {
    for(num_eps_check = 0 ; num_eps_check < epsList->numEps; num_eps_check++ )
    {
      if(memcmp(epsList->epsArray[num_eps_check].MAC,epMac,ETH_ALEN) == 0)
      {
        return_ep = &(epsList->epsArray[num_eps_check]);
        break;
      }
    }
  }
  return return_ep;
}

/*******************************************************************/

static t_Domains *VbDatamodelActiveDmFind(const INT8U *dmMac)
{
  t_Domains *return_dm = NULL;
  INT8U *dm_mac;
  INT16U num_domains_check;

  if((vbDatamodelDomains != NULL) && (dmMac != NULL))
  {
    for(num_domains_check = 0 ; num_domains_check < vbDatamodelDomains->NumDomains ; num_domains_check++)
    {
      dm_mac = vbDatamodelDomains->domainsArray[num_domains_check].dm.MAC;
      if ((memcmp(dm_mac,dmMac,ETH_ALEN) == 0) &&
          (VbDatamodelNodeIsActive(&(vbDatamodelDomains->domainsArray[num_domains_check].dm)) == TRUE))
      {
        return_dm = &vbDatamodelDomains->domainsArray[num_domains_check];
        break;
      }
    }
  }

  return return_dm;
}

/*******************************************************************/

static t_node *VbDatamodelActiveEpFind(const INT8U *epMac, t_EpsList *epsList)
{
  t_node *return_ep = NULL;
  INT16U num_eps_check;

  if((epsList != NULL) && (epMac != NULL))
  {
    for(num_eps_check = 0 ; num_eps_check < epsList->numEps; num_eps_check++ )
    {
      if ((memcmp(epsList->epsArray[num_eps_check].MAC,epMac,ETH_ALEN) == 0) &&
          (VbDatamodelNodeIsActive(&(epsList->epsArray[num_eps_check])) == TRUE))
      {
        return_ep = &(epsList->epsArray[num_eps_check]);
        break;
      }
    }
  }
  return return_ep;
}

/*******************************************************************/

static t_node *VbDatamodelActiveNodeFind(const INT8U *mac)
{
  t_node    *node = NULL;
  INT16U     domain_idx = 0;
  t_Domains *domain = NULL;
  t_EpsList *eps_list = NULL;

  if ((vbDatamodelDomains != NULL) && (mac != NULL))
  {
    // Search MAC in DMs list
    domain = VbDatamodelActiveDmFind(mac);

    if (domain == NULL)
    {
      // DM not found, check EPs

      for (domain_idx = 0; domain_idx < vbDatamodelDomains->NumDomains; domain_idx++)
      {
        domain = &(vbDatamodelDomains->domainsArray[domain_idx]);

        if (VbDatamodelNodeIsActive(&(domain->dm)) == TRUE)
        {
          eps_list = &(domain->eps);

          node = VbDatamodelActiveEpFind(mac, eps_list);

          if (node != NULL)
          {
            // Node found, it is an EP
            break;
          }
        }
      }
    }
    else
    {
      // Node found, it is a DM
      node = &(domain->dm);
    }
  }

  return node;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelTLVNumDomainsGet(const t_HTLVsLists *htlvsLists, INT32U *numDomains)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_HTLVValuesList *list_element = NULL;
  INT32U            list_idx;
  t_ValuesArray    *list_values = NULL;
  INT16U            value_idx;
  t_Values         *value;
  INT32U            num_domains = 0;

  if ((htlvsLists == NULL) || (numDomains == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    list_element = htlvsLists->head;
    list_idx = 0;

    while ((list_element != NULL) && (list_idx < htlvsLists->NumLists))
    {
      list_idx++;
      list_values = list_element->HTLVsArray;

      if (list_values->NumValues > 0)
      {
        for (value_idx = 0; value_idx < list_values->NumValues; value_idx++ )
        {
          value = &(list_values->values[value_idx]);

          if (value->Value[0] == VB_DOMAINMACS)
          {
            // Increase number of domains discovered
            num_domains++;
          }
        }
      }
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *numDomains = num_domains;
  }

  return ret;
}

/*******************************************************************/
#if 0
static t_VB_comErrorCode VbDatamodelDomainListNetChangeCheck(BOOLEAN *networkChange)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  BOOLEAN           network_change = FALSE;
  INT32U            domain_idx;
  t_Domains        *domain;
  t_Domains        *domain_old;
  t_node           *ep;
  t_node           *ep_old;

  if (networkChange == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (vbDatamodelDomainsNew == NULL)
  {
    ret = VB_COM_ERROR_DATAMODEL;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Check if number of discovered domains remains the same than before
    if (vbDatamodelDomainsNew->NumDomains != vbDatamodelDomains->NumDomains)
    {
      // Different number of domains discovered, network has changed
      network_change = TRUE;
    }
  }

  if ((ret == VB_COM_ERROR_NONE) && (network_change == FALSE))
  {
    // Loop through recently discovered domains
    for (domain_idx = 0; domain_idx < vbDatamodelDomainsNew->NumDomains; domain_idx++)
    {
      domain = &(vbDatamodelDomainsNew->domainsArray[domain_idx]);

      // Search this domain in old list
      domain_old = VbDatamodelDmFind(domain->dm.MAC);

      if (domain_old != NULL)
      {
        // Domain found

        // Check extended seed and device Id
        if ((domain->dm.addInfo1.extSeed != domain_old->dm.addInfo1.extSeed) ||
            (domain->dm.devID != domain_old->dm.devID))
        {
          network_change = TRUE;
        }

        if (network_change == FALSE)
        {
          // Check EPs
          if (domain->eps.numEps != domain_old->eps.numEps)
          {
            // Different number of EPs, network has changed
            network_change = TRUE;
          }
          else if (domain->eps.numEps > 0)
          {
            // Search for the same EP
            ep = &(domain->eps.epsArray[0]);
            ep_old = VbDatamodelEpFind(ep->MAC, &(domain_old->eps));

            if (ep_old == NULL)
            {
              // Node not found, network has changed
              network_change = TRUE;
            }
            else
            {
              // Check deviceId
              if (ep->devID != ep_old->devID)
              {
                network_change = TRUE;
              }
            }
          }
        }
      }
      else
      {
        // Node not found, network has changed
        network_change = TRUE;
      }

      if (network_change == TRUE)
      {
        // Exit loop
        break;
      }
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *networkChange = network_change;
  }

  return ret;
}
#endif
/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainDMsNewListBuild(INT8U *pld, INT32U *numNewDMs, BOOL fullReport)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx;
  t_Domains        *domain;
  t_Domains        *domain_old;

  if ( (vbDatamodelDomainsNew == NULL) || (numNewDMs == NULL) )
  {
    ret = VB_COM_ERROR_DATAMODEL;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *numNewDMs = 0;

    // Loop through recently discovered domains
    for (domain_idx = 0; domain_idx < vbDatamodelDomainsNew->NumDomains; domain_idx++)
    {
      domain = &(vbDatamodelDomainsNew->domainsArray[domain_idx]);

      // Search this domain in old list
      domain_old = (fullReport == FALSE)?VbDatamodelDmFind(domain->dm.MAC):NULL;

      if (domain_old == NULL)
      {
        t_DomainData *dm_info;

        dm_info = (t_DomainData *)pld;
        // Node not found, New DM
        memcpy(dm_info->mac, domain->dm.MAC, ETH_ALEN);
        dm_info->devID      = domain->dm.devID;
        dm_info->extSeed    = domain->dm.addInfo1.extSeed;
        dm_info->numEps     = domain->eps.numEps;

        VbLogPrint(VB_LOG_INFO, "NEW DM -> %s", domain->dm.MACStr);

        pld += sizeof(t_DomainData);
        (*numNewDMs)++;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainEPsNewListBuild(INT8U *pld, INT32U *numNewEPs, BOOL fullReport)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx;
  t_Domains        *domain;
  t_Domains        *domain_old;
  t_node           *ep;
  BOOLEAN           copy = FALSE;

  if ( (vbDatamodelDomainsNew == NULL) || (numNewEPs == NULL) )
  {
    ret = VB_COM_ERROR_DATAMODEL;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *numNewEPs = 0;

    // Loop through recently discovered domains
    for (domain_idx = 0; domain_idx < vbDatamodelDomainsNew->NumDomains; domain_idx++)
    {
      domain = &(vbDatamodelDomainsNew->domainsArray[domain_idx]);
      if(domain->eps.numEps > 0)
      {
        if(fullReport == FALSE)
        {
          if(domain->discoverChangeFlag)
          {
            copy = TRUE;
          }
          else
          {
            // Search this domain in old list
            domain_old = VbDatamodelDmFind(domain->dm.MAC);
            if(domain_old != NULL)
            {
              // DM MAC found in both (New and Old, check EP now)
              ep = VbDatamodelEpFind(domain->eps.epsArray[0].MAC, &domain_old->eps);
              if (ep == NULL)
              {
                copy = TRUE;
              }
            }
            else
            {
              copy = TRUE;
            }
          }
        }
        else
        {
          copy = TRUE;
        }

        if(copy == TRUE)
        {
          t_EpDiffData *ep_info;

          ep_info = (t_EpDiffData *)pld;

          // Node not found, New DM
          memcpy(ep_info->dmMac, domain->dm.MAC, ETH_ALEN);
          memcpy(ep_info->epMac, domain->eps.epsArray[0].MAC, ETH_ALEN);
          ep_info->devId      = domain->eps.epsArray[0].devID;

          VbLogPrint(VB_LOG_INFO, "NEW EP -> [%02x %02x %02x %02x %02x %02x]",
                     domain->eps.epsArray[0].MAC[0],
                     domain->eps.epsArray[0].MAC[1],
                     domain->eps.epsArray[0].MAC[2],
                     domain->eps.epsArray[0].MAC[3],
                     domain->eps.epsArray[0].MAC[4],
                     domain->eps.epsArray[0].MAC[5]);
          VbLogPrint(VB_LOG_INFO, "NEW EP from DM -> [%02x %02x %02x %02x %02x %02x]",
                     domain->dm.MAC[0],
                     domain->dm.MAC[1],
                     domain->dm.MAC[2],
                     domain->dm.MAC[3],
                     domain->dm.MAC[4],
                     domain->dm.MAC[5]);

          pld += sizeof(t_EpDiffData);
          (*numNewEPs)++;
        }

        copy = FALSE;
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainDMsRemListBuild(INT8U *pld, INT32U *numRemDMs)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx;
  INT32U            domain_new_idx;
  t_Domains        *domain;
  t_Domains        *domain_old;
  BOOL              found = FALSE;

  if ( (vbDatamodelDomainsNew == NULL) || (numRemDMs == NULL) || (pld == NULL) )
  {
    ret = VB_COM_ERROR_DATAMODEL;
  }

  if ( ret == VB_COM_ERROR_NONE )
  {
    *numRemDMs = 0;

    // Loop through old domains
    for (domain_idx = 0; domain_idx < vbDatamodelDomains->NumDomains; domain_idx++)
    {
      domain_old = &(vbDatamodelDomains->domainsArray[domain_idx]);
      found = FALSE;

      // Loop through recently discovered domains to check if old MAC have disappeared
      for (domain_new_idx = 0; domain_new_idx < vbDatamodelDomainsNew->NumDomains; domain_new_idx++)
      {
        domain = &(vbDatamodelDomainsNew->domainsArray[domain_new_idx]);
        if(memcmp(domain->dm.MAC, domain_old->dm.MAC, ETH_ALEN) == 0)
        {
          found = TRUE;
          break;
        }
      }

      if(found == FALSE)
      {
        // Node not found, Rem DM
        memcpy(pld, domain_old->dm.MAC, ETH_ALEN);

        VbLogPrint(VB_LOG_INFO, "DEL DM -> %s", domain_old->dm.MACStr);

        pld += ETH_ALEN;
        (*numRemDMs)++;
      }
    }
  }

  return ret;
}


/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainEPsRemListBuild(INT8U *pld, INT32U *numRemEPs)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx;
  INT32U            domain_new_idx;
  t_Domains        *domain;
  t_Domains        *domain_old;
  BOOL              found = FALSE;

  if ( (vbDatamodelDomainsNew == NULL) || (numRemEPs == NULL) )
  {
    ret = VB_COM_ERROR_DATAMODEL;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *numRemEPs = 0;

    // Loop through recently old domains
    for (domain_idx = 0; domain_idx < vbDatamodelDomains->NumDomains; domain_idx++)
    {
      domain_old = &(vbDatamodelDomains->domainsArray[domain_idx]);
      found = FALSE;

      if(domain_old->eps.numEps > 0)
      {
        // Loop through recently discovered domains to check if old MAC have disappeared
        for (domain_new_idx = 0; domain_new_idx < vbDatamodelDomainsNew->NumDomains; domain_new_idx++)
        {
          domain = &(vbDatamodelDomainsNew->domainsArray[domain_new_idx]);
          if(memcmp(domain->eps.epsArray[0].MAC, domain_old->eps.epsArray[0].MAC, ETH_ALEN) == 0)
          {
            found = TRUE;
            break;
          }
        }

        if(found == FALSE)
        {
          // Node not found, Rem DM
          memcpy(pld, domain_old->eps.epsArray[0].MAC, ETH_ALEN);

          VbLogPrint(VB_LOG_INFO, "DEL EP -> %s", domain_old->eps.epsArray[0].MACStr);

          pld += ETH_ALEN;
          (*numRemEPs)++;
        }
      }
    }
  }

  return ret;
}


/*******************************************************************/

static t_VB_comErrorCode VbDatamodelDomainListDataRecover(void)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx;
  t_Domains        *domain;
  t_Domains        *domain_old;
  t_node           *ep;
  t_node           *ep_old;
  INT16U            seed;
  INT8U             device_id;

  if (vbDatamodelDomainsNew == NULL)
  {
    ret = VB_COM_ERROR_DATAMODEL;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Loop through recently discovered domains
    for (domain_idx = 0; domain_idx < vbDatamodelDomainsNew->NumDomains; domain_idx++)
    {
      domain = &(vbDatamodelDomainsNew->domainsArray[domain_idx]);

      // Search this domain in old list
      domain_old = VbDatamodelDmFind(domain->dm.MAC);

      if (domain_old != NULL)
      {
        // Domain found

        // Backup seed and device Id (they could be changed)
        seed = domain->dm.addInfo1.extSeed;
        device_id = domain->dm.devID;

        // Copy old info to new container
        memcpy(&(domain->dm), &(domain_old->dm), sizeof(domain->dm));

        // Restore seed and device Id
        domain->dm.addInfo1.extSeed = seed;
        domain->dm.devID = device_id;

        /*
         * Set old container to 0s to avoid releasing dynamically allocated memory when old list is released.
         * Pointers to dynamic memory (like measures) are now located in new container.
         */
        bzero(&(domain_old->dm), sizeof(domain_old->dm));

        domain->dm.state = VB_DEV_PRESENT;
        VbLogPrint(VB_LOG_INFO, "DM detected -> %s : Already present", domain->dm.MACStr);

        // Check EPs
        if (domain->eps.numEps > 0)
        {
          // Search for the same EP
          ep = &(domain->eps.epsArray[0]);
          ep_old = VbDatamodelEpFind(ep->MAC, &(domain_old->eps));

          if (ep_old != NULL)
          {
            // Copy info from old EP

            // Backup seed and device Id (they could be changed)
            seed = ep->addInfo1.extSeed;
            device_id = ep->devID;

            // Copy old info to new container
            memcpy(ep, ep_old, sizeof(*ep));

            // Restore seed and device Id
            ep->addInfo1.extSeed = seed;
            ep->devID = device_id;

            /*
             * Set old container to 0s to avoid releasing dynamically allocated memory when old list is released.
             * Pointers to dynamic memory (like measures) are now located in new container.
             */
            bzero(ep_old, sizeof(*ep_old));

            ep->state = VB_DEV_PRESENT;
            VbLogPrint(VB_LOG_INFO, "EP detected -> %s : Already present", ep->MACStr);
          }
          else
          {
            // New EP detected
            ep->state = VB_DEV_NEW;
            VbLogPrint(VB_LOG_INFO, "EP detected -> %s : NEW", ep->MACStr);
          }
        }

        // Update linkedNode pointers
        domain->dm.linkedNode = &(domain->eps.epsArray[0]);
        domain->eps.epsArray[0].linkedNode = &(domain->dm);
      }
      else
      {
        // New domain
        domain->dm.state = VB_DEV_NEW;

        VbLogPrint(VB_LOG_INFO, "DM detected -> %s : NEW", domain->dm.MACStr);

        if (domain->eps.numEps > 0)
        {
          domain->eps.epsArray[0].state = VB_DEV_NEW;
          VbLogPrint(VB_LOG_INFO, "EP detected -> %s : NEW", domain->eps.epsArray[0].MACStr);
        }
      }
    }
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelTLVDomainProcess(const t_HTLVsLists *htlvsLists, BOOL *netChange, BOOL firstTime)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  t_HTLVValuesList      *list_element = NULL;
  t_VB_DMDiscover_Value *dm_discover_value = NULL;
  t_VB_EPDiscover_Value *ep_discover_value = NULL;
  t_Values              *value;
  INT32U                 list_idx;
  INT32U                 offset;
  INT32U                 value_idx;
  t_ValuesArray         *list_values = NULL;
  INT32U                 num_domains;
  t_Domains             *new_domain = NULL;
  BOOLEAN                network_change_detected = FALSE;
  BOOLEAN                network_change_rx = FALSE;
  INT8U                 *pld = NULL;

  if ((htlvsLists == NULL) || (netChange == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Get number of discovered domains
    ret = VbDatamodelTLVNumDomainsGet(htlvsLists, &num_domains);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Release previous domains list
    ret = VbDatamodelDomainsListDestroy(vbDatamodelDomainsNew);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate a new list
    ret = VbDatamodelDomainsListAlloc(num_domains);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Build the new domains list
    list_element = htlvsLists->head;
    list_idx = 0;

    while ((list_element != NULL) && (list_idx < htlvsLists->NumLists))
    {
      list_idx++;
      list_values = list_element->HTLVsArray;

      for (value_idx = 0; value_idx < list_values->NumValues; value_idx++ )
      {
        value = &(list_values->values[value_idx]);

        if ((value->Value[0] == VB_DOMAINMACS) && (value->ValueLength >= VB_DMDISCOVER_VALUE_SIZE))
        {
          dm_discover_value = (t_VB_DMDiscover_Value *)&(value->Value[1]);

          // Add domain to domains list
          ret = VbDatamodelDomainAdd(dm_discover_value->MAC, dm_discover_value->DevID, _ntohs_ghn(dm_discover_value->Extseed), &new_domain);

          // Add ep to domain
          if ((ret == VB_COM_ERROR_NONE) && (dm_discover_value->NumEps > 0))
          {
            offset = 1 + VB_DMDISCOVER_VALUE_SIZE;
            ep_discover_value =  (t_VB_EPDiscover_Value *)(&(value->Value[offset]));

            ret = VbDatamodelEpAdd(new_domain, ep_discover_value->MAC, ep_discover_value->DevID);
          }

          if (ret == VB_COM_ERROR_NONE)
          {
            // Check if network change flag is present and it was not read previously
            if (network_change_rx == FALSE)
            {
              offset = (1 + VB_DMDISCOVER_VALUE_SIZE + (dm_discover_value->NumEps * VB_EPDISCOVER_VALUE_SIZE));

              // Check if this frame contains the network change flag
              if (value->ValueLength == (offset + 1))
              {
                network_change_rx = (BOOLEAN)(value->Value[offset]);
                new_domain->discoverChangeFlag = (BOOLEAN)(value->Value[offset]);
                if (network_change_rx == TRUE)
                {
                  VbLogPrint(VB_LOG_INFO, "Network change advised by MAC %s", new_domain->dm.MACStr);
                }
              }
              else
              {
                new_domain->discoverChangeFlag = FALSE;
              }
            }
          }
        }

        if (ret != VB_COM_ERROR_NONE)
        {
          VbLogPrint(VB_LOG_ERROR, "Error %d discovering domains", ret);
          break;
        }
      }

      list_element = list_element->nextList;
    }
  }

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  if (ret == VB_COM_ERROR_NONE)
  {
    INT8U  *pld_ptr;
    INT32U  num_info_nodes;

    // Get max domains between old and new domain
    num_info_nodes = MAX(vbDatamodelDomains->NumDomains, num_domains);
    vbDatamodelDomainsChanges.reportType = (firstTime == TRUE)? VB_EA_DOMAIN_REPORT_FULL:VB_EA_DOMAIN_REPORT_DIFF;

    // Over allocate memory, will be free later
    pld = calloc(1, num_info_nodes*(sizeof(t_DomainData)+ sizeof(t_EpDiffData) + 2*ETH_ALEN));
    if(pld != NULL)
    {
      pld_ptr = pld;
      ret = VbDatamodelDomainDMsNewListBuild(pld_ptr, &vbDatamodelDomainsChanges.numDMsAdded, firstTime);
      if (ret == VB_COM_ERROR_NONE)
      {
        pld_ptr += vbDatamodelDomainsChanges.numDMsAdded*sizeof(t_DomainData);
        ret = VbDatamodelDomainEPsNewListBuild(pld_ptr, &vbDatamodelDomainsChanges.numEPsAdded, firstTime);
      }

      if (ret == VB_COM_ERROR_NONE)
      {
        if(firstTime == FALSE)
        {
          pld_ptr += vbDatamodelDomainsChanges.numEPsAdded*sizeof(t_EpDiffData);
          ret = VbDatamodelDomainDMsRemListBuild(pld_ptr, &vbDatamodelDomainsChanges.numDMsRem);
        }
        else
        {
          vbDatamodelDomainsChanges.numDMsRem = 0;
        }
      }

      if (ret == VB_COM_ERROR_NONE)
      {
        if(firstTime == FALSE)
        {
          pld_ptr += vbDatamodelDomainsChanges.numDMsRem*ETH_ALEN;
          ret = VbDatamodelDomainEPsRemListBuild(pld_ptr, &vbDatamodelDomainsChanges.numEPsRem);
        }
        else
        {
          vbDatamodelDomainsChanges.numEPsRem = 0;
        }
      }

      if( vbDatamodelDomainsChanges.numDMsAdded ||
          vbDatamodelDomainsChanges.numEPsAdded ||
          vbDatamodelDomainsChanges.numDMsRem   ||
          vbDatamodelDomainsChanges.numEPsRem )
      {
        if(vbDatamodelDomainsChanges.pInfo != NULL)
        {
          free(vbDatamodelDomainsChanges.pInfo);
        }
        vbDatamodelDomainsChanges.pInfo = pld;
        vbDatamodelDomainsChanges.changesBitmap  = (vbDatamodelDomainsChanges.numDMsAdded? (1<<DOMAINCHANGE_DM_ADDED):0 );
        vbDatamodelDomainsChanges.changesBitmap |= (vbDatamodelDomainsChanges.numEPsAdded? (1<<DOMAINCHANGE_EP_ADDED):0 );
        vbDatamodelDomainsChanges.changesBitmap |= (vbDatamodelDomainsChanges.numDMsRem  ? (1<<DOMAINCHANGE_DM_REM)  :0 );
        vbDatamodelDomainsChanges.changesBitmap |= (vbDatamodelDomainsChanges.numEPsRem  ? (1<<DOMAINCHANGE_EP_REM)  :0 );
        network_change_detected = TRUE;

        VbLogPrint(VB_LOG_INFO, "Network change first time %d, DM added %d, EP Added %d, DM Rem %d, EP Rem %d",
                                                     firstTime,
                                                     vbDatamodelDomainsChanges.numDMsAdded,
                                                     vbDatamodelDomainsChanges.numEPsAdded,
                                                     vbDatamodelDomainsChanges.numDMsRem,
                                                     vbDatamodelDomainsChanges.numEPsRem);

      }
    }

    network_change_rx = FALSE;
    if ((network_change_rx || network_change_detected))
    {
      // Network has changed, so recover data from old list and swap pointers to use new list
      ret = VbDatamodelDomainListDataRecover();

      if (ret == VB_COM_ERROR_NONE)
      {
        t_DomainsList *aux_ptr;

        // Swap pointers
        aux_ptr = vbDatamodelDomains;
        vbDatamodelDomains = vbDatamodelDomainsNew;
        vbDatamodelDomainsNew = aux_ptr;
      }
    }
  }

  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  if (ret == VB_COM_ERROR_NONE)
  {
    // Release previous domains list
    ret = VbDatamodelDomainsListDestroy(vbDatamodelDomainsNew);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *netChange = (network_change_rx || network_change_detected);
    if(*netChange == FALSE)
    {
      if(pld != NULL)
      {
        free(pld);
      }
    }
  }
  else
  {
    if(pld != NULL)
    {
      free(pld);
    }
  }

  if ((network_change_rx || network_change_detected))
  {
    VbLogPrint(VB_LOG_INFO, "Network has changed! Domains list updated (err %d)", ret);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelNoTLVDomainTimeOutProcess(BOOL *netChange)
{
  t_VB_comErrorCode      ret = VB_COM_ERROR_NONE;
  BOOLEAN                network_change_detected = FALSE;
  INT8U                 *pld = NULL;

  if (netChange == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Release previous domains list
    ret = VbDatamodelDomainsListDestroy(vbDatamodelDomainsNew);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Allocate a new list
    vbDatamodelDomainsNew->NumDomains = 0;
    vbDatamodelDomainsNew->domainsArray = NULL;
  }

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  if (ret == VB_COM_ERROR_NONE)
  {
    INT8U  *pld_ptr;
    INT32U  num_info_nodes;

    // Get num of old domains
    num_info_nodes = vbDatamodelDomains->NumDomains;

    // Over allocate memory, will be free later
    pld = calloc(1, num_info_nodes*(sizeof(t_DomainData)));
    VbLogPrint(VB_LOG_INFO, "TO Calloc  %d %x)", num_info_nodes*(sizeof(t_DomainData)), pld);

    pld_ptr = pld;
    if(pld != NULL)
    {
      ret = VbDatamodelDomainDMsRemListBuild(pld_ptr, &vbDatamodelDomainsChanges.numDMsRem);
      if (ret == VB_COM_ERROR_NONE)
      {
        if(vbDatamodelDomainsChanges.numDMsRem > 0)
        {
          if(vbDatamodelDomainsChanges.pInfo != NULL)
          {
            free(vbDatamodelDomainsChanges.pInfo);
          }
          vbDatamodelDomainsChanges.pInfo = pld;
          vbDatamodelDomainsChanges.changesBitmap |= (vbDatamodelDomainsChanges.numDMsRem  ? (1<<DOMAINCHANGE_DM_REM)  :0 );
          network_change_detected = TRUE;
        }
      }
    }

    if ((network_change_detected))
    {
      // Network has changed, so recover data from old list and swap pointers to use new list
      if (ret == VB_COM_ERROR_NONE)
      {
        t_DomainsList *aux_ptr;

        // Swap pointers
        aux_ptr = vbDatamodelDomains;
        vbDatamodelDomains = vbDatamodelDomainsNew;
        vbDatamodelDomainsNew = aux_ptr;
      }
    }
  }

  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  if (ret == VB_COM_ERROR_NONE)
  {
    // Release previous domains list
    ret = VbDatamodelDomainsListDestroy(vbDatamodelDomainsNew);
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    *netChange = network_change_detected;
    if(*netChange == FALSE)
    {
      if(pld != NULL)
      {
        free(pld);
      }
    }
  }
  else
  {
    if(pld != NULL)
    {
      free(pld);
    }
  }

  if (network_change_detected)
  {
    VbLogPrint(VB_LOG_INFO, "Network has changed (TO)! Domains list updated (err %d)", ret);
  }

  return ret;
}

/*******************************************************************/

static t_VB_comErrorCode VbDatamodelTLVAddInfo1Process(const t_HTLVsLists *htlvsLists)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  t_HTLVValuesList *read_htlv_values_list = NULL;
  t_Domains *domain = NULL;
  t_node *ep;
  t_VB_AdditionalInfo1_Value *additional_info1_value = NULL;
  t_Values *value;
  INT16U num_htvl_lists_check;
  INT16U num_domains;
  INT16U values_check;
  t_ValuesArray *read_values = NULL;

  read_htlv_values_list = htlvsLists->head;
  num_htvl_lists_check = 0;

  while( (read_htlv_values_list != NULL) && (num_htvl_lists_check < htlvsLists->NumLists) )
  {
    num_htvl_lists_check++;
    read_values = read_htlv_values_list->HTLVsArray;
    if(read_values->NumValues >0)
    {
      for(values_check = 0; values_check < read_values->NumValues; values_check++ )
      {
        value = &(read_values->values[values_check]);
        if(value->Value[0] == VB_ADDINFO_1)
        {
          if(value->ValueLength == (VB_ADDINFO1_VALUE_SIZE + 1))
          {
            additional_info1_value = (t_VB_AdditionalInfo1_Value *)&(value->Value[1]);

            domain = VbDatamodelDmFind(additional_info1_value->MAC);
            if(domain != NULL)
            {
              VbLogPrint( VB_LOG_INFO, "DM MAC %s", additional_info1_value->fwVersion);

              strncpy((char *)domain->dm.addInfo1.fwVersion, (char *)additional_info1_value->fwVersion, VB_FW_VERSION_LENGTH);
              domain->dm.addInfo1.fwVersion[VB_FW_VERSION_LENGTH - 1] = '\0';

              domain->dm.addInfo1.qosRate = additional_info1_value->qosRate;
              domain->dm.addInfo1.maxLengthTxop = _ntohs_ghn(additional_info1_value->maxLengthTxop);
            }
            else
            {
              for(num_domains = 0 ; num_domains < vbDatamodelDomains->NumDomains ; num_domains++)
              {
                domain = &vbDatamodelDomains->domainsArray[num_domains];
                ep = VbDatamodelEpFind(additional_info1_value->MAC, &(domain->eps));
                if(ep != NULL)
                {
                  VbLogPrint( VB_LOG_INFO, "EP MAC %s", additional_info1_value->fwVersion);

                  strncpy((char *)ep->addInfo1.fwVersion, (char *)additional_info1_value->fwVersion, VB_FW_VERSION_LENGTH);
                  ep->addInfo1.fwVersion[VB_FW_VERSION_LENGTH - 1] = '\0';
                }
              }
            }
          }
        }
      }
    }
    read_htlv_values_list = read_htlv_values_list->nextList;
  }

  return result;
}

/*******************************************************************/

static void VbDatamodelMacDomainsListInit( void )
{

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  bzero(vbDatamodelDomainsList, sizeof(vbDatamodelDomainsList));
  vbDatamodelDomains = &(vbDatamodelDomainsList[0]);
  vbDatamodelDomainsNew = &(vbDatamodelDomainsList[1]);

  pthread_mutex_unlock( &vbDatamodelMutexDomains );
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

/*******************************************************************/

t_VB_comErrorCode VbDatamodelHtlvListAdd( const t_ValuesArray *valuesArray,
    BOOL sendAck,  BOOL markTimeStamp, const INT8U *srcMac, t_HTLVsLists  **htlvList)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  t_HTLVValuesList *new_list;

  if((valuesArray != NULL) && (srcMac != NULL))
  {
    if((*htlvList) == NULL)
    {
      (*htlvList) = VbDatamodelHtlvListCreate();
    }
    if((*htlvList) != NULL)
    {
      new_list = (t_HTLVValuesList *)calloc(1, sizeof(t_HTLVValuesList));

      if(new_list != NULL)
      {
        bzero(new_list, sizeof(t_HTLVValuesList));
        new_list->HTLVsArray = (t_ValuesArray *)valuesArray;
        new_list->nextList = NULL;
        new_list->ack = sendAck;
        if(srcMac != NULL)
        {
          memcpy(new_list->srcMAC,srcMac,ETH_ALEN);
        }
        else
        {
          memset(new_list->srcMAC,0,ETH_ALEN);
        }
        if(markTimeStamp)
        {
          clock_gettime(CLOCK_MONOTONIC, &new_list->timeStamp);
        }
        VbDatamodelHtlvListInsert( *htlvList, new_list );
      }
      else
      {
        result =  VB_COM_ERROR_MALLOC;
      }
    }
    else
    {
      result =  VB_COM_ERROR_MALLOC;
    }
  }
  else
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }
  return result;
}

/********************************************************************/

t_VB_comErrorCode VbDatamodelHTLVsArrayCopy( const t_ValuesArray *srcArray, t_ValuesArray **destArray )
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  INT16U value_to_copy;

  if((srcArray == NULL) || (destArray == NULL))
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }
  else
  {
    if((*destArray) != NULL)
    {
      VbDatamodelHTLVsArrayDestroy(destArray);
    }
    (*destArray) = (t_ValuesArray *)calloc(1, sizeof(t_ValuesArray));
    if((*destArray) == NULL)
    {
      result = VB_COM_ERROR_MALLOC;
    }
    else
    {
      (*destArray)->NumValues = 0;
      (*destArray)->values = (t_Values *)calloc(1, sizeof(t_Values) * (srcArray->NumValues));
      if((*destArray)->values == NULL)
      {
        result = VB_COM_ERROR_MALLOC;
      }
      else
      {
        (*destArray)->NumValues = srcArray->NumValues;
        for(value_to_copy = 0 ; value_to_copy < srcArray->NumValues ; value_to_copy++)
        {
          (*destArray)->values[value_to_copy].ValueLength = srcArray->values[value_to_copy].ValueLength;
          (*destArray)->values[value_to_copy].Value =
              (INT8U *)calloc(1, (*destArray)->values[value_to_copy].ValueLength);
          if((*destArray)->values[value_to_copy].Value == NULL)
          {
            result = VB_COM_ERROR_MALLOC;
            break;
          }
          else
          {
            memcpy((*destArray)->values[value_to_copy].Value,
                srcArray->values[value_to_copy].Value,srcArray->values[value_to_copy].ValueLength);
          }
        }
      }
    }
  }
  return result;
}

/********************************************************************/

t_VB_comErrorCode VbDatamodelHTLVsArrayDestroy( t_ValuesArray **HTLVsArray )
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  INT16U values_check;

  if(HTLVsArray == NULL)
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }
  else
  {
    if((*HTLVsArray) != NULL)
    {
      if((*HTLVsArray)->values != NULL)
      {
        for(values_check = 0 ; values_check < (*HTLVsArray)->NumValues; values_check++)
        {
          if((*HTLVsArray)->values[values_check].Value != NULL)
          {
            free((*HTLVsArray)->values[values_check].Value);
            (*HTLVsArray)->values[values_check].Value = NULL;
          }
          (*HTLVsArray)->values[values_check].ValueLength = 0;
        }
        free((*HTLVsArray)->values);
        (*HTLVsArray)->values = NULL;
      }
      (*HTLVsArray)->NumValues = 0;
      free((*HTLVsArray));
      (*HTLVsArray) = NULL;
    }
  }
  return result;
}

/********************************************************************/

t_VB_comErrorCode VbDatamodelHtlvsListValueDestroy( t_HTLVsLists **htlvsLists )
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  t_HTLVValuesList *next_list;
  t_HTLVValuesList *this_list;
  INT16U num_htvl_lists_check;

  if(htlvsLists!= NULL)
  {
    if((*htlvsLists) != NULL)
    {
      (*htlvsLists)->tail = NULL;
      this_list = (*htlvsLists)->head;
      (*htlvsLists)->head = NULL;
      num_htvl_lists_check = 0;

      if((this_list != NULL) && ((*htlvsLists)->NumLists == 0))
      {
        VbLogPrint(VB_LOG_ERROR, "this_list != NULL and numlist == 0");
      }
      while( (this_list != NULL) && (num_htvl_lists_check < (*htlvsLists)->NumLists) )
      {
        num_htvl_lists_check++;
        next_list = this_list->nextList;
        result = VbDatamodelHTLVsArrayDestroy( &(this_list->HTLVsArray) );
        free(this_list);
        this_list = next_list;
      }
      free((*htlvsLists));
      (*htlvsLists) = NULL;
    }
  }
  return result;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelGetActiveMacsArray( INT16U *numNodes, INT8U** arrayNodesMac, BOOLEAN onlyDMs )
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  INT16U num_domains_check;
  INT16U num_eps_check;
  INT16U num_macs;
  t_EpsList *eps;
  t_Domains *dms;
  BOOL  duplicated_mac;

  if( (numNodes == NULL) || (arrayNodesMac == NULL) )
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }
  else
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );
    if( (vbDatamodelDomains != NULL) && (vbDatamodelDomains->NumDomains>0) )
    {
      num_macs = 0;
      for(num_domains_check = 0 ; num_domains_check < vbDatamodelDomains->NumDomains;
          num_domains_check++)
      {
        dms = &vbDatamodelDomains->domainsArray[num_domains_check];
        if (VbDatamodelNodeIsActive(&(dms->dm)) == TRUE)
        {
          num_macs++;
          if (onlyDMs == FALSE)
          {
            eps = &(dms->eps);
            for (num_eps_check = 0 ; num_eps_check < eps->numEps ; num_eps_check++)
            {
              if (VbDatamodelNodeIsActive(&(eps->epsArray[num_eps_check])) == TRUE)
              {
                num_macs++;
              }
            }
          }
        }
      }
      VbLogPrint( VB_LOG_DEBUG, "Total macs %d", num_macs);
      if (num_macs != 0)
      {
        (*arrayNodesMac) = (INT8U *)calloc(1, num_macs*ETH_ALEN);
        if((*arrayNodesMac) == NULL)
        {
          result = VB_COM_ERROR_MALLOC;
        }
        else
        {
          num_macs = 0;
          for(num_domains_check = 0 ; num_domains_check < vbDatamodelDomains->NumDomains;
              num_domains_check++)
          {
            dms = &vbDatamodelDomains->domainsArray[num_domains_check];
            if (VbDatamodelNodeIsActive(&(dms->dm)) == TRUE)
            {
              duplicated_mac = VbDatamodelLookforMacinMacsArray( dms->dm.MAC, (*arrayNodesMac), num_macs );
              if(!duplicated_mac)
              {
                VbLogPrint( VB_LOG_DEBUG, "DM Node %02X:%02X:%02X:%02X:%02X:%02X active",
                    dms->dm.MAC[0],
                    dms->dm.MAC[1],
                    dms->dm.MAC[2],
                    dms->dm.MAC[3],
                    dms->dm.MAC[4],
                    dms->dm.MAC[5]);
                memcpy(&(*arrayNodesMac)[num_macs*ETH_ALEN],dms->dm.MAC,ETH_ALEN);
                num_macs++;
              }

              if (onlyDMs == FALSE)
              {
                eps = &(dms->eps);
                for (num_eps_check = 0 ; num_eps_check < eps->numEps ; num_eps_check++)
                {
                  if (VbDatamodelNodeIsActive(&(eps->epsArray[num_eps_check])) == TRUE)
                  {
                    duplicated_mac = VbDatamodelLookforMacinMacsArray( eps->epsArray[num_eps_check].MAC, (*arrayNodesMac), num_macs );
                    if(!duplicated_mac)
                    {
                      VbLogPrint( VB_LOG_DEBUG, "EP Node %02X:%02X:%02X:%02X:%02X:%02X active",
                          eps->epsArray[num_eps_check].MAC[0],
                          eps->epsArray[num_eps_check].MAC[1],
                          eps->epsArray[num_eps_check].MAC[2],
                          eps->epsArray[num_eps_check].MAC[3],
                          eps->epsArray[num_eps_check].MAC[4],
                          eps->epsArray[num_eps_check].MAC[5]);
                      memcpy(&(*arrayNodesMac)[num_macs*ETH_ALEN], eps->epsArray[num_eps_check].MAC,ETH_ALEN);
                      num_macs++;
                    }
                  }
                }
              }
            }
          }
        }
      }
      (*numNodes) = num_macs;
      VbLogPrint( VB_LOG_DEBUG, "Not duplicated macs %d", num_macs);
    }
    else
    {
      (*numNodes) = 0;
    }
    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  return result;
}

/*******************************************************************/

INT16U VbDatamodelNmDomainsActiveGet(BOOLEAN grabMutex)
{
  INT16U num_nodes = 0;
  INT16U num_domains_check;
  t_Domains *dms;

  if (grabMutex)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );
  }

  if (vbDatamodelDomains != NULL)
  {
    for (num_domains_check = 0 ; num_domains_check < vbDatamodelDomains->NumDomains; num_domains_check++)
    {
      dms = &vbDatamodelDomains->domainsArray[num_domains_check];

      if (VbDatamodelNodeIsActive(&(dms->dm)) == TRUE)
      {
        num_nodes++;
      }
    }
  }

  if (grabMutex)
  {
    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  return num_nodes;
}

/*******************************************************************/

INT16U VbDatamodelNmDomainsAliveGet( void )
{
  INT16U num_nodes = 0;

  pthread_mutex_lock( &vbDatamodelMutexDomains );
  if(vbDatamodelDomains != NULL)
  {
    num_nodes = vbDatamodelDomains->NumDomains;
  }
  pthread_mutex_unlock( &vbDatamodelMutexDomains );
  return num_nodes;
}
/*******************************************************************/

t_VB_comErrorCode VbDatamodelDomainDiscover(BOOL *netChange, INT32U *numDomains, INT32U transactionId, BOOL firstTime)
{
  t_VB_comErrorCode    result;
  t_HGF_LCMP_ErrorCode lcmp_err;
  t_lcmpComParams      lcmp_params;
  INT8U                parameters[1];
  t_ValuesArray       *values_to_read = NULL;
  t_HTLVsLists        *read_htlvs = NULL;

  parameters[0] = (INT8U)VB_DOMAINMACS;

  result = VbDatamodelValueToArrayAdd( 1, (INT8U *)parameters, &values_to_read);

  if (result == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      result = VB_COM_ERROR_LCMP;
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = MULTICAST_DMS;
    lcmp_params.paramIdReq      = VB_DOMAINMACS;
    lcmp_params.reqValues       = values_to_read;
    lcmp_params.rspValuesList   = &read_htlvs;
 //   lcmp_params.transactionId   = transactionId;

    lcmp_err = VbLcmpRead(&lcmp_params);

    if (lcmp_err == HGF_LCMP_ERROR_NO_RESPONSE )
    {
      result = VB_COM_ERROR_RECEIVE_TIMEOUT;
    }
    else if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending DiscoverRead.req", lcmp_err);

      result = VB_COM_ERROR_LCMP_READ;
    }
  }

  pthread_mutex_lock( &vbDatamodelMutexDomainsDiscover );

  vbDatamodelDomainsChanges.numDMsAdded = 0;
  vbDatamodelDomainsChanges.numEPsAdded = 0;
  vbDatamodelDomainsChanges.numEPsRem   = 0;
  vbDatamodelDomainsChanges.numDMsRem   = 0;

  if (result == VB_COM_ERROR_RECEIVE_TIMEOUT)
  {
    // No domains discovered
    if (VbDatamodelNmDomainsAliveGet() != 0)
    {
      result = VbDatamodelNoTLVDomainTimeOutProcess(netChange);
      if(result == VB_COM_ERROR_NONE)
      {
        result = VB_COM_ERROR_RECEIVE_TIMEOUT;
      }
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    result = VbDatamodelTLVDomainProcess(read_htlvs, netChange, firstTime);
  }

  pthread_mutex_unlock( &vbDatamodelMutexDomainsDiscover );

  if ((result == VB_COM_ERROR_NONE) || (result == VB_COM_ERROR_RECEIVE_TIMEOUT))
  {
    if (numDomains != NULL)
    {
      *numDomains = VbDatamodelNmDomainsAliveGet();
    }
  }

  VbDatamodelHTLVsArrayDestroy(&values_to_read);
  VbDatamodelHtlvsListValueDestroy(&read_htlvs);

  return result;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelAddInfo1Get(void)
{
  t_VB_comErrorCode    result;
  t_HGF_LCMP_ErrorCode lcmp_err;
  t_lcmpComParams      lcmp_params;
  INT8U                parameters[1];
  t_ValuesArray       *values_to_read = NULL;
  t_HTLVsLists        *read_htlvs = NULL;

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  parameters[0] = (INT8U)VB_ADDINFO_1;

  result = VbDatamodelValueToArrayAdd( 1, (INT8U *)parameters, &values_to_read);

  if (result == VB_COM_ERROR_NONE)
  {
    // Init LCMP params
    lcmp_err = VbLcmpParamsInit(&lcmp_params);

    if (lcmp_err != HGF_LCMP_ERROR_NONE)
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d initializing LCMP parameters", lcmp_err);
      result = VB_COM_ERROR_LCMP;
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    // Configure LCMP params
    lcmp_params.transmisionType = MULTICAST;
    lcmp_params.paramIdReq      = VB_ADDINFO_1;
    lcmp_params.reqValues       = values_to_read;
    lcmp_params.rspValuesList   = &read_htlvs;

    lcmp_err = VbLcmpRead(&lcmp_params);

    if ((lcmp_err != HGF_LCMP_ERROR_NONE) && (lcmp_err != HGF_LCMP_ERROR_NO_RESPONSE ))
    {
      VbLogPrint(VB_LOG_ERROR, "Error %d sending AddInfo1.req", lcmp_err);

      result = VB_COM_ERROR_LCMP_READ;
    }
    else if (lcmp_err == HGF_LCMP_ERROR_NO_RESPONSE )
    {
      result = VB_COM_ERROR_RECEIVE_TIMEOUT;
    }
  }

  if (result == VB_COM_ERROR_NONE)
  {
    result = VbDatamodelTLVAddInfo1Process(read_htlvs);
  }

  if (result == VB_COM_ERROR_NONE)
  {
    result = VbDatamodelAllNodesCapabilitiesUpdate();
  }

  VbDatamodelHTLVsArrayDestroy(&values_to_read);
  VbDatamodelHtlvsListValueDestroy(&read_htlvs);

  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return result;
}

/*******************************************************************/

void VbDatamodelInit(void)
{
  VbDatamodelMacDomainsListInit();
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelValueToArrayAdd( INT16U length, const INT8U* value, t_ValuesArray **values_array)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  t_Values *temp_values_old = NULL;
  t_Values *temp_values_new = NULL;

  if((length == 0) || (value == NULL) || (values_array == NULL))
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }
  else
  {
    if((*values_array) == NULL)
    {
      (*values_array) = (t_ValuesArray *)calloc(1, sizeof(t_ValuesArray));
      if((*values_array) == NULL)
      {
        result = VB_COM_ERROR_MALLOC;
      }
      else
      {
        (*values_array)->NumValues = 0;
        (*values_array)->values = NULL;
      }
    }

    if(result == VB_COM_ERROR_NONE)
    {
      if((*values_array)->NumValues > 0)
      {
        temp_values_old = (*values_array)->values;
      }
      temp_values_new = (t_Values *)calloc(1, sizeof(t_Values) * ((*values_array)->NumValues + 1));
      if(temp_values_new == NULL)
      {
        result = VB_COM_ERROR_MALLOC;
      }
      else
      {
        if((*values_array)->NumValues > 0)
        {
          memcpy(temp_values_new, temp_values_old, sizeof(t_Values) * (*values_array)->NumValues);
          free(temp_values_old);
          temp_values_old = NULL;
        }
        temp_values_new[(*values_array)->NumValues].Value = (INT8U *)calloc(1, length);

        if(temp_values_new[(*values_array)->NumValues].Value == NULL)
        {
          // Release previously allocated memory
          free(temp_values_new);
          temp_values_new = NULL;

          result = VB_COM_ERROR_MALLOC;
        }
        else
        {
          memcpy(temp_values_new[(*values_array)->NumValues].Value, value, length);
          temp_values_new[(*values_array)->NumValues].ValueLength = length;
          (*values_array)->values = temp_values_new;
          (*values_array)->NumValues++;
        }
      }
    }
  }
  return result;
}

/*******************************************************************/

void VbDatamodelAllDomainsListDestroy( void )
{
  VbLogPrint(VB_LOG_INFO, "Releasing datamodel...");
  VbDatamodelDomainsListDestroy(vbDatamodelDomains);
  VbDatamodelDomainsListDestroy(vbDatamodelDomainsNew);
  VbLogPrint(VB_LOG_INFO, "Released datamodel!");
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelTrafficReportAdd(const INT8U *macAddr, t_IngressTraffic *report, t_bpsBandTrafficReport *bps_report)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_node           *node = NULL;
  INT8U i;

  if (macAddr == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    // Look for given node
    node = VbDatamodelActiveNodeFind(macAddr);

    if (node == NULL)
    {
      // Device not found
      ret = VB_COM_ERROR_NODEVICE;
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      // copy traffic report
      node->ingressTraffic.channelCapacity = report->channelCapacity;
      node->ingressTraffic.desiredCapacity = report->desiredCapacity;
      node->ingressTraffic.effectiveCapacity = report->effectiveCapacity;
      node->ingressTraffic.macEff = report->macEff;
      for( i = 0 ; i < VECTORBOOST_INGRESS_TRAFFIC_PRIORITIES_NUMBER  ; i++ )
      {
        node->ingressTraffic.maxBuff[i] = report->maxBuff[i];
        node->ingressTraffic.traffic[i] = report->traffic[i];
      }

      node->ingressTraffic.bpsBandsInfo.nBands = bps_report->nBands;
      for( i = 0 ; ((i < bps_report->nBands) && (i< VB_PSD_NUM_BANDS)); i++ )
      {
        node->ingressTraffic.bpsBandsInfo.bpsBand[i] = bps_report->bpsBand[i];
      }

      node->ingressTraffic.newReportReceived = TRUE;
      node->ingressTraffic.rxReports++;

      // Update timestamp
      ret = VbDatamodelTrafficTimeStampInit(&(node->ingressTraffic));
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelGetAllTrafficReports(t_vbEAMsg **msg)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx = 0;
  t_Domains        *domain_ptr = NULL;
  INT32U            num_reports;
  INT32U            num_bands = 0;
  INT8U            *payload_pointer;

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  if (vbDatamodelDomains != NULL)
  {
    num_reports = 0;
    for (domain_idx = 0; domain_idx < vbDatamodelDomains->NumDomains; domain_idx++)
    {
      domain_ptr = &(vbDatamodelDomains->domainsArray[domain_idx]);

      if (domain_ptr->dm.ingressTraffic.newReportReceived)
      {
        num_reports++;
        num_bands += domain_ptr->dm.ingressTraffic.bpsBandsInfo.nBands;
      }

      // Check if there is an EP with pending traffic reports
      if ((VbDatamodelNodeIsActive(&(domain_ptr->dm)) == TRUE) &&
          (domain_ptr->dm.linkedNode->ingressTraffic.newReportReceived == TRUE))
      {
        num_reports++;
        num_bands += domain_ptr->dm.linkedNode->ingressTraffic.bpsBandsInfo.nBands;
      }
    }

    if (num_reports > 0)
    {
      ret = VbEATrafficAwarenessMultiFrameCreate(num_reports, num_bands, msg);
    }
    else
    {
      ret = VB_COM_ERROR_NOT_FOUND;
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      // Get payload pointer and apply offset
      payload_pointer  = (*msg)->eaPayload.msg;
      payload_pointer += VB_EA_TRAFFIC_REPORT_HDR_SIZE;

      for (domain_idx = 0;
          (domain_idx < vbDatamodelDomains->NumDomains) && (ret == VB_COM_ERROR_NONE);
          domain_idx++)
      {
        domain_ptr = &(vbDatamodelDomains->domainsArray[domain_idx]);

        // DM
        if (domain_ptr->dm.ingressTraffic.newReportReceived)
        {
          // Fill traffic awareness message
          ret = VbEATrafficAwarenessFillMsg(domain_ptr->dm.MAC, &(domain_ptr->dm.ingressTraffic), payload_pointer);

          // Update pointer to write
          payload_pointer += VB_EA_TRAFFIC_REPORT_RSP_SIZE + domain_ptr->dm.ingressTraffic.bpsBandsInfo.nBands*sizeof(INT16U) + sizeof(INT16U);

          // Mark
          domain_ptr->dm.ingressTraffic.newReportReceived = FALSE;
        }

        // EP (if present)
        if ((VbDatamodelNodeIsActive(&(domain_ptr->dm)) == TRUE) &&
            (domain_ptr->dm.linkedNode->ingressTraffic.newReportReceived == TRUE))
        {
          // Fill traffic awareness message
          ret = VbEATrafficAwarenessFillMsg(domain_ptr->dm.linkedNode->MAC, &(domain_ptr->dm.linkedNode->ingressTraffic), payload_pointer);

          // Update pointer to write
          payload_pointer += VB_EA_TRAFFIC_REPORT_RSP_SIZE + + domain_ptr->dm.linkedNode->ingressTraffic.bpsBandsInfo.nBands*sizeof(INT16U) + sizeof(INT16U);

          // Mark
          domain_ptr->dm.linkedNode->ingressTraffic.newReportReceived = FALSE;
        }
      }
    }
  }
  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDomainAlignInfoUpdate(const INT8U *macAddr, INT16U seqNum, INT32U macClock, t_syncDetInfo *syncAllDids)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_Domains        *domain_master = NULL;

  if ((macAddr == NULL) || (syncAllDids == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  pthread_mutex_lock( &vbDatamodelMutexDomains );
  if (ret == VB_COM_ERROR_NONE)
  {
    // Look for given DM
    domain_master = VbDatamodelDmFind(macAddr);

    if (domain_master == NULL)
    {
      // Device not found
      ret = VB_COM_ERROR_NODEVICE;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    INT32U i;

    // Update sequence number and MAC clock
    domain_master->alignInfo.seqNum = seqNum;
    domain_master->alignInfo.macClock = macClock;

    for (i = 0; i < VB_ALIGN_GHN_MAX_TX_NODES; i++)
    {
      domain_master->alignInfo.syncDetsInfo[i] = syncAllDids[i];
    }
  }
  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDomainAlignChangeUpdate(const INT8U *macAddr, INT16U seqNumOffset, BOOLEAN clockEdge)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_Domains        *domain_master = NULL;

  if (macAddr == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  pthread_mutex_lock( &vbDatamodelMutexDomains );
  if (ret == VB_COM_ERROR_NONE)
  {
    // Look for given DM
    domain_master = VbDatamodelDmFind(macAddr);

    if (domain_master == NULL)
    {
      // Device not found
      ret = VB_COM_ERROR_NODEVICE;
    }
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    // Update sequence number and MAC clock
    domain_master->alignChange.seqNumOffset = seqNumOffset;
    domain_master->alignChange.clockEdge    = clockEdge;
    domain_master->alignChange.updated      = TRUE;
  }
  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return ret;
}

/*******************************************************************/

BOOLEAN VbDatamodelNodeIsActive(t_node *node)
{
  BOOLEAN ret = FALSE;

  if ((node != NULL) && (node->linkedNode != NULL))
  {
    // A node is considered "active" when its partner node is present
    ret = node->linkedNode->used;
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDomainsDataGet( INT16U *numDomains, t_DomainData **domainData, BOOLEAN fullInfo, BOOLEAN all)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  INT16U            domain_idx;
  INT16U            num_elements;
  t_Domains        *dms;
  t_DomainData     *this_domain_data;

  if ((numDomains == NULL) || (domainData == NULL))
  {
    result = VB_COM_ERROR_BAD_ARGS;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    if (vbDatamodelDomains == NULL)
    {
      result = VB_COM_ERROR_DATAMODEL;
    }

    if (result == VB_COM_ERROR_NONE)
    {
      if (all == TRUE)
      {
        (*numDomains) = vbDatamodelDomains->NumDomains;
      }
      else
      {
        (*numDomains) = VbDatamodelNmDomainsActiveGet(FALSE);
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      if (*numDomains > 0)
      {
        (*domainData) = (t_DomainData *)calloc(1, (*numDomains)*sizeof(t_DomainData));

        if ((*domainData) == NULL)
        {
          result = VB_COM_ERROR_MALLOC;
        }
      }
      else
      {
        (*domainData) = NULL;
      }
    }

    if ((result == VB_COM_ERROR_NONE) && (*numDomains > 0))
    {
      num_elements = 0;

      for (domain_idx = 0 ; domain_idx < vbDatamodelDomains->NumDomains; domain_idx++)
      {
        dms = &(vbDatamodelDomains->domainsArray[domain_idx]);

        if ((all == TRUE) || (VbDatamodelNodeIsActive(&(dms->dm)) == TRUE))
        {
          this_domain_data = &((*domainData)[num_elements]);

          MACAddrClone(this_domain_data->mac, dms->dm.MAC);
          this_domain_data->devID = dms->dm.devID;

          if (fullInfo == TRUE)
          {
            this_domain_data->extSeed = dms->dm.addInfo1.extSeed;
            this_domain_data->qosRate = dms->dm.addInfo1.qosRate;
            this_domain_data->maxLengthTxop = dms->dm.addInfo1.maxLengthTxop;
            memcpy(this_domain_data->fwVersion, dms->dm.addInfo1.fwVersion, VB_FW_VERSION_LENGTH);
            this_domain_data->cap = dms->dm.cap;
          }

          this_domain_data->numEps = dms->eps.numEps;

          num_elements++;
        }
      }
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  return result;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDomainsReportDataGet(INT8U * reportType, INT16U *numDMAdded, INT16U *numEPAdded, INT16U *numDMRem, INT16U *numEPRem, INT8U **pld)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;

  if ((numDMAdded == NULL) || (numEPAdded == NULL) || (numDMRem == NULL) ||  (numEPRem == NULL))
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    *reportType = vbDatamodelDomainsChanges.reportType;
    *numDMAdded = vbDatamodelDomainsChanges.numDMsAdded;
    *numEPAdded = vbDatamodelDomainsChanges.numEPsAdded;
    *numDMRem   = vbDatamodelDomainsChanges.numDMsRem;
    *numEPRem   = vbDatamodelDomainsChanges.numEPsRem;
    *pld        = vbDatamodelDomainsChanges.pInfo;

    pthread_mutex_unlock( &vbDatamodelMutexDomains );

  }

  return result;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDomainsReportDataFree(void)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  if(vbDatamodelDomainsChanges.pInfo != NULL)
  {
    free(vbDatamodelDomainsChanges.pInfo);
    vbDatamodelDomainsChanges.pInfo = NULL;
  }

  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return result;
}
/*******************************************************************/

t_VB_comErrorCode VbDatamodelEpsDataGet(const INT8U *dmMac, INT16U *numEps, t_EpData **epData)
{
  t_VB_comErrorCode result = VB_COM_ERROR_NONE;
  INT16U            ep_idx;
  t_node           *ep;
  t_Domains        *domain;
  t_EpData         *this_ep_data;

  if ((dmMac == NULL) || (numEps == NULL) || (epData == NULL))
  {
    result = VB_COM_ERROR_PARAM_ERROR;
  }

  if (result == VB_COM_ERROR_NONE)
  {
    //Protected zone
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    if (vbDatamodelDomains == NULL)
    {
      result = VB_COM_ERROR_DATAMODEL;
    }

    if (result == VB_COM_ERROR_NONE)
    {
      // Look for dm
      domain = VbDatamodelDmFind(dmMac);

      if (domain == NULL)
      {
        result = VB_COM_ERROR_NOT_FOUND;
      }
    }

    if (result == VB_COM_ERROR_NONE)
    {
      // DM found

      // Get MAC of active EPs
      (*numEps) = domain->eps.numEps;

      if ((*numEps) == 0)
      {
        (*epData) = NULL;
      }
      else
      {
        (*epData) = (t_EpData *)calloc(1, (*numEps)*sizeof(t_EpData));

        if ((*epData) == NULL)
        {
          result = VB_COM_ERROR_MALLOC;
        }
      }

      if ((result == VB_COM_ERROR_NONE) && (*numEps > 0))
      {
        for (ep_idx = 0; ep_idx < (*numEps); ep_idx++)
        {
          ep = &(domain->eps.epsArray[ep_idx]);

          this_ep_data = &(*epData)[ep_idx];

          MACAddrClone(this_ep_data->mac, ep->MAC);
          memcpy(this_ep_data->fwVersion, ep->addInfo1.fwVersion, VB_FW_VERSION_LENGTH);
          this_ep_data->devID = ep->devID;
          this_ep_data->DevState = ep->state;
          this_ep_data->cap = ep->cap;
        }
      }
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  return result;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelAllDomainsTrafficTimeStampUpdate(void)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx = 0;
  t_Domains        *domain_ptr = NULL;
  t_node           *ep_ptr = NULL;
  INT16U            end_point = 0;

  pthread_mutex_lock( &vbDatamodelMutexDomains );

  if (vbDatamodelDomains != NULL)
  {
    for (domain_idx = 0; domain_idx < vbDatamodelDomains->NumDomains; domain_idx++)
    {
      // Init DM timestamp

      domain_ptr = &(vbDatamodelDomains->domainsArray[domain_idx]);

      ret = VbDatamodelTrafficTimeStampInit(&(domain_ptr->dm.ingressTraffic));

      if (ret == VB_COM_ERROR_NONE)
      {
        // Init EPs timestamp

        for (end_point = 0; end_point < domain_ptr->eps.numEps; end_point++)
        {
          ep_ptr = &(domain_ptr->eps.epsArray[end_point]);

          ret = VbDatamodelTrafficTimeStampInit(&(ep_ptr->ingressTraffic));

          if (ret != VB_COM_ERROR_NONE)
          {
            VbLogPrint(VB_LOG_ERROR, "Error updating traffic info timestamp (err %d)", ret);
            break;
          }
        }
      }

      if (ret != VB_COM_ERROR_NONE)
      {
        VbLogPrint(VB_LOG_ERROR, "Error updating traffic info timestamp (err %d)", ret);
        break;
      }
    }
  }

  pthread_mutex_unlock( &vbDatamodelMutexDomains );

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelNodeCapTrafficReportSet(const INT8U *mac, BOOLEAN enable)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_node           *node = NULL;
  CHAR              mac_str[MAC_STR_LEN];

  if (mac == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    // Search given node
    node = VbDatamodelActiveNodeFind(mac);

    if (node != NULL)
    {
      node->cap.trafficReports = enable;
    }
    else
    {
      ret = VB_COM_ERROR_NODEVICE;
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_INFO, "Node %s -> trafficReport cap = %s", node->MACStr, enable?"YES":"NO");
  }
  else
  {
    MACAddrMem2str(mac_str, mac);
    VbLogPrint(VB_LOG_ERROR, "Error %d updating capability", ret, mac_str);
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelActiveNodeLoop(t_nodeLoopCb loopCb, BOOLEAN lock, void *args)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx = 0;
  INT32U            ep_idx = 0;
  t_Domains        *domain = NULL;

  if (loopCb == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (lock == TRUE)
    {
      pthread_mutex_lock( &vbDatamodelMutexDomains );
    }

    if (vbDatamodelDomains != NULL)
    {
      // Loop through active domains
      for (domain_idx = 0; (domain_idx < vbDatamodelDomains->NumDomains) && (ret == VB_COM_ERROR_NONE); domain_idx++)
      {
        domain = &(vbDatamodelDomains->domainsArray[domain_idx]);

        if (VbDatamodelNodeIsActive(&(domain->dm)) == TRUE)
        {
          // Call callback for DM
          ret = loopCb(&domain->dm, args);

          // Loop through active eps
          for (ep_idx = 0; (ep_idx < domain->eps.numEps) && (ret == VB_COM_ERROR_NONE); ep_idx++)
          {
            if (VbDatamodelNodeIsActive(&(domain->eps.epsArray[ep_idx])) == TRUE)
            {
              // Call callback for EPs
              ret = loopCb(&(domain->eps.epsArray[ep_idx]), args);
            }
          }
        }
      }
    }

    if (lock == TRUE)
    {
      pthread_mutex_unlock( &vbDatamodelMutexDomains );
    }
  }

  if (ret == VB_COM_ERROR_EXIT_LOOP_OK)
  {
    // Expected error code to stop loop, change to VB_COM_ERROR_NONE
    ret = VB_COM_ERROR_NONE;
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDomainsLoop(t_domainLoopCb loopCb, void *args)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  INT32U            domain_idx = 0;
  t_Domains        *domain = NULL;

  if (loopCb == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    if (vbDatamodelDomains != NULL)
    {
      // Loop through domains
      for (domain_idx = 0; (domain_idx < vbDatamodelDomains->NumDomains) && (ret == VB_COM_ERROR_NONE); domain_idx++)
      {
        domain = &(vbDatamodelDomains->domainsArray[domain_idx]);

        // Call callback for domain
        ret = loopCb(domain, args);
      }
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDMMACGet(INT32U domainIdx, INT8U *mac)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_Domains        *domain = NULL;

  if (mac == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    if (vbDatamodelDomains == NULL)
    {
      ret = VB_COM_ERROR_DATAMODEL;
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      if (domainIdx >= vbDatamodelDomains->NumDomains)
      {
        ret = VB_COM_ERROR_NOT_FOUND;
      }
    }

    if (ret == VB_COM_ERROR_NONE)
    {
      domain = &(vbDatamodelDomains->domainsArray[domainIdx]);
      MACAddrClone(mac, domain->dm.MAC);
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  return ret;
}

/*******************************************************************/

t_vbDriverCountersIndex VbDatamodelEvToCounter(t_driverEvent event)
{
  t_vbDriverCountersIndex counter = (t_vbDriverCountersIndex)DRIVER_EV_START;

  if (event >= DRIVER_EV_LAST)
  {
    VbLogPrint(VB_LOG_ERROR, "Invalid event to translate (%u)", event);
  }
  else
  {
    // Counters are located in same positions as defined in t_driverEvent
    counter = (t_vbDriverCountersIndex)event;
  }

  return counter;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelNodeSeedIndexSet(const INT8U *mac, INT16U seedIndex, INT16U did)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_node           *node = NULL;

  if (mac == NULL)
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    // Search given node
    node = VbDatamodelActiveNodeFind(mac);

    if (node != NULL)
    {
      node->addInfo1.extSeed = seedIndex;
      node->devID = did;
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    if (node != NULL)
    {
      VbLogPrint(VB_LOG_INFO, "Updating Node %s -> seed = %u", node->MACStr, seedIndex);
    }
  }
  else
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d updating seed", ret);
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelDMAddInfo1Read(const INT8U *mac, INT8U *fwVersion, INT16U *qosRate, INT16U *maxTxopLength)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_Domains *domain = NULL;

  if ((mac == NULL) || (fwVersion == NULL) || (qosRate == NULL) || (maxTxopLength == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    // Search given node
    domain = VbDatamodelDmFind(mac);
    if (domain != NULL)
    {
      strncpy((char *)fwVersion, (char *)domain->dm.addInfo1.fwVersion, VB_FW_VERSION_LENGTH);
      *qosRate = domain->dm.addInfo1.qosRate;
      *maxTxopLength = domain->dm.addInfo1.maxLengthTxop;
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  if (ret != VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d reading DM AddInfo1", ret);
  }

  return ret;
}

/*******************************************************************/

t_VB_comErrorCode VbDatamodelEPAddInfo1Read(const INT8U *epMac, const INT8U *dmMac, INT8U *fwVersion)
{
  t_VB_comErrorCode ret = VB_COM_ERROR_NONE;
  t_node           *node = NULL;
  t_Domains        *domain = NULL;

  if ((epMac == NULL) || (dmMac == NULL) || (fwVersion == NULL))
  {
    ret = VB_COM_ERROR_BAD_ARGS;
  }

  if (ret == VB_COM_ERROR_NONE)
  {
    pthread_mutex_lock( &vbDatamodelMutexDomains );

    domain = VbDatamodelDmFind(dmMac);
    if(domain != NULL)
    {
      // Search given node
      node = VbDatamodelEpFind(epMac, &domain->eps);
      if (node != NULL)
      {
        strncpy((char *)fwVersion, (char *)node->addInfo1.fwVersion, VB_FW_VERSION_LENGTH);
      }
    }

    pthread_mutex_unlock( &vbDatamodelMutexDomains );
  }

  if (ret != VB_COM_ERROR_NONE)
  {
    VbLogPrint(VB_LOG_ERROR, "Error %d reading EP AddInfo1", ret);
  }

  return ret;
}


/*******************************************************************/

t_HTLVsLists *VbDatamodelHtlvListCreate( void )
{
  t_HTLVsLists *new_htlvs_list;

  new_htlvs_list = (t_HTLVsLists *)calloc(1, sizeof(t_HTLVsLists));
  if(new_htlvs_list != NULL)
  {
    bzero(new_htlvs_list, sizeof(t_HTLVsLists));
    new_htlvs_list->NumLists = 0;
    new_htlvs_list->head = NULL;
    new_htlvs_list->tail = NULL;
  }
  return new_htlvs_list;
}

/*******************************************************************/

void VbDatamodelHtlvListInsert( t_HTLVsLists *htlvsLists, t_HTLVValuesList *htlvValuesList )
{
  if((htlvsLists != NULL) && (htlvValuesList != NULL))
  {
    if(htlvsLists->tail != NULL)
    {
      htlvsLists->tail->nextList = htlvValuesList;
    }
    htlvsLists->tail = htlvValuesList;
    if(htlvsLists->head == NULL)
    {
      htlvsLists->head = htlvValuesList;
    }
    htlvsLists->NumLists++;
  }
}

/*******************************************************************/

/**
 * @}
 **/
