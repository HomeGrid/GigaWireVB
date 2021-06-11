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
 * @file vb_linked_list.h
 * @brief Functions used to handle linked lists
 *
 * @internal
 *
 * @author pablom
 * @date Feb 17, 2017
 *
 **/

#ifndef VB_LINKED_LIST_H_
#define VB_LINKED_LIST_H_

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */

#include "vb_types.h"

/*
 ************************************************************************
 ** Public type definitions
 ************************************************************************
 */

// This struct has to be inherited in other structs to build the linked lists
typedef struct linkedElement
{
  struct linkedElement *next;
  INT32U index;
}t_linkedElement;

// We use this struct to search an element with its MAC
typedef struct linkedMACElement
{
  struct linkedElement l;
  INT8U mac[ETH_ALEN];
}t_linkedMACElement;

/*
 ************************************************************************
 ** Linked list MACROS
 ************************************************************************
 */

#define LIST_COUNT(head,el,counter)                      \
do {                                                     \
  (counter) = 0;                                         \
  LIST_FOREACH(head,el) { ++(counter); }                 \
} while (0)


#define LIST_FOREACH(head,el)                            \
    for ((el) = (head); el; (el) = (el)->next)


#define LIST_GOTO_END(head,el)                           \
do {                                                     \
  for ((el) = (head); el->next; (el) = (el)->next){}     \
} while (0)

/*
 ************************************************************************
 ** Public functions
 ************************************************************************
 */

/**
 * @brief Find an element on the list by its MAC
 * @param[in] head first element of the list
 * @param[out] searched pointer to the pointer that will point the searched element
 * @return TRUE if the element is on the list
 **/
BOOL GetElementByMAC(t_linkedMACElement *head, t_linkedMACElement **searched, INT8U *mac);

/**
 * @brief Find an element on the list by its MAC
 * @param[in] head first element of the list
 * @param[out] searched pointer to the pointer that will point the searched element
 * @return TRUE if the element is on the list
 **/
BOOL GetElementByIndex(t_linkedElement *head, t_linkedElement **searched, INT32U index);

/**
 * @brief Append one element at the end of the list
 * @param[in] head pointer to the first element of the list
 * @param[in] newEl Element to be appended
 **/
void AppendElement(t_linkedElement **head, t_linkedElement *new);

/**
 * @brief Append one element at the end of the list
 * @param[in] head pointer to the first element of the list
 * @param[in] newEl Element to be appended
 **/
void RemoveElement(t_linkedElement **head, t_linkedElement *elem);

/**
 * @brief Free all memory allocated by the list elements
 * @param[in] head first element of the list
 **/
void ClearList(t_linkedElement *head);

#endif /* VB_LINKED_LIST_H_ */

/**
 * @}
 **/
