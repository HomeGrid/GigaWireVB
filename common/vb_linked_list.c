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
 * @file vb_linked_list.c
 * @brief Functions used to handle linked lists
 *
 * @internal
 *
 * @author pablom
 * @date Feb 17, 2017
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include <stdlib.h>
#include <string.h>

#include "types.h"
#include "vb_linked_list.h"

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/*******************************************************************/

BOOL GetElementByMAC(t_linkedMACElement *head, t_linkedMACElement **searched, INT8U *mac)
{
  t_linkedElement *current;
  t_linkedMACElement *curr_MAC;
  BOOL found = FALSE;

  if((head != NULL) && (searched != NULL))
  {
    // Search the given mac
    LIST_FOREACH((t_linkedElement *)head,current)
    {
      curr_MAC = (t_linkedMACElement *)current;
      if(memcmp(mac, curr_MAC->mac, 6) == 0)
      {
        *searched = curr_MAC;
        found = TRUE;
        break;
      }
    }
  }

  return found;
}

/*******************************************************************/

BOOL GetElementByIndex(t_linkedElement *head, t_linkedElement **searched, INT32U index)
{
  t_linkedElement *current;
  BOOL found = FALSE;
  INT32U counter = 0;

  if((head != NULL) && (searched != NULL))
  {
    // Search the given mac
    LIST_FOREACH(head,current)
    {
      if(counter++ == index)
      {
        *searched = current;
        found = TRUE;
        break;
      }
    }
  }

  return found;
}

/*******************************************************************/

void AppendElement(t_linkedElement **head, t_linkedElement *newEl)
{
  t_linkedElement *current;

  if(newEl != NULL)
  {
    // The list was empty?
    if(*head == NULL)
    {
      *head = newEl;
      newEl->index = 1;
    }
    // Append to the end of the list
    else
    {
      current = *head;

      // Go to the end of the list
      while (current->next != NULL)
      {
       current = current->next;
      }
      current->next = newEl;
      newEl->index = current->index + 1;
    }
    newEl->next = NULL;
  }
}

/*******************************************************************/

void RemoveElement(t_linkedElement **head, t_linkedElement *elem)
{
  t_linkedElement *current;
  t_linkedElement *previous;

  if(elem != NULL)
  {
    // The list was empty?
    if(*head != NULL)
    {
      current  = *head;
      previous = *head;

      // Perform the search
      while (current->next != NULL && current != elem)
      {
        previous = current;
        current  = current->next;
      }

      // Remove from the list
      if(current == *head)
      {
        *head = current->next;
      }
      else
      {
        previous->next = current->next;
      }
    }
    elem->next = NULL;
  }
}

/*******************************************************************/

void ClearList(t_linkedElement *head)
{
  t_linkedElement *current, *next;

  current = head;
  while (current != NULL)
  {
    next = current->next;
    free(current);
    current = next;
  }
}

/*******************************************************************/

/**
 * @}
 **/
