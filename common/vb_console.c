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
 * @file vb_console.c
 * @brief Console to test the Vector Boost
 *
 * @internal
 *
 * @author
 * @date 22/12/2014
 *
 **/

/*
 ************************************************************************
 ** Included files
 ************************************************************************
 */
#include "types.h"

#include <sys/socket.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <netinet/in.h>
#include <errno.h>
#include <pthread.h>

#include "vb_log.h"
#include "vb_console.h"
#include "vb_thread.h"
#include "vb_priorities.h"

/*
 ************************************************************************
 ** Private constants
 ************************************************************************
 */

#define VB_CONSOLE_THREAD_NAME        ("Console")
#define VB_CONSOLE_COMMANDS_MAX       (20)
#define VB_CONSOLE_NAME_MAX_SIZE      (21)

/*
 ************************************************************************
 ** Private type definitions
 ************************************************************************
 */

typedef struct
{
  CHAR *commandName;
  BOOL (*callbackFun)(void *arg, void (*write_fun)(const char *fmt, ...), char **cmd);
  void *arg;
} t_consoleCommand;


/*
 ************************************************************************
 ** Private variables
 ************************************************************************
 */

static pthread_t       vbConsoleThread;
static pthread_mutex_t vbConsoleMutex;
static BOOL            vbConsoleThreadRunning;

static INT32S          vbConsoleSockFd;
static INT32S          vbConsoleConnFd;
static INT16U          vbConsolePort;
static char           vbConsoleName[VB_CONSOLE_NAME_MAX_SIZE] = { 0 };

static t_consoleCommand consoleCommands[VB_CONSOLE_COMMANDS_MAX] = { 0 };

/*
 ************************************************************************
 ** Private function declaration
 ************************************************************************
 */

// Console thread entry point
static void *VbThreadConsole(void *arg);

// Function in charge of finding the appropiate callback to execute for a
// specific command
static BOOL vbConsoleExecuteCommand(INT32S sk, CHAR* command);

// Function that sends data to the user
static void VbConsoleWrite(const char *fmt, ...);

/*
 ************************************************************************
 ** Private function implementation
 ************************************************************************
 */

/**
 * @brief Adapts the initial string to be processed
 *
 * @param str
 **/
static void consoleRemoveExtraChars(char *str)
{
  char           *tmp;
  int             spacesHeading;

  //remove all blank spaces at the beginning
  spacesHeading = strspn(str, " ");

  if (spacesHeading != 0)
  {
    strcpy(str, (str + spacesHeading));
  }
  //remove \n and/or \r
  tmp = strchr(str, '\r');

  if (tmp)
  {
    *tmp = '\0';
  }

  tmp = strchr(str, '\n');

  if (tmp)
  {
    *tmp = '\0';
  }
}

/*******************************************************************/

/*
 * Internal implementation of "isblank" function.
 * The implementation provided by "ctype.h" interface is not
 * working fine in MIPS arch.
 */
static int isblank(int c)
{
  int ret = 0;

  if ((c == ' ') ||
      (c == '\t'))
  {
    ret = 1;
  }

  return ret;
}

/*******************************************************************/

/**
 * @brief  Divides string into sub-strings
 *
 * Divides the input string into up to @ref CONSOLE_PARAMS_MAXNUM seperate
 * sub-strings separated by blank characters. Each sub-string can have up to
 * @ref CONSOLE_PARAMS_MAXNUM characters.
 *
 * The resulting sub-strings are stored in a static variable. Therefore it is
 * not necessary to allocate memory for the sub-strings, only for the pointers
 * to them. On the other hand, this function is @b not thread-save!
 *
 * A non-bracking space in the input string mut be preceded with a backslash
 * '\'. The blackslash is @b not copied directly into the sub-string:
 *
 * <tt>
 * "a b"  => "a" + "b"
 * "a\ b" => "a b"
 * "a\b"  => "ab"
 * "a\\b" => "a\b"
 * </tt>
 *
 * If there are less than @ref CONSOLE_PARAMS_MAXNUM sub-strings, the pointer
 * after the last string is set to NULL.
 *
 * @param[in]  str  String to be split.
 * @param[out] cmd  Array of strings with at least @ref CONSOLE_PARAMS_MAXNUM
 *                  elements.
 *
 **/
static void consoleStr2words(const CHAR *str, CHAR **cmd)
{
  static          CHAR command[VB_CONSOLE_BUFFER_SIZE];
  INT32U          index = 0;
  const CHAR     *max = command + sizeof(command);
  INT32U          offset;
  CHAR           *ptr = command;
  BOOLEAN         stop;
  enum
  { BLANK, ESCAPE, NOBLANK } type = BLANK;

  /*
   * We need at least a valid output pointer.
   */
  if (cmd == NULL)
  {
  }
  /*
   * Without input, no output ...
   */
  else if (str == NULL)
  {
    cmd[0] = NULL;
  }
  else
  {
    /*
     * Loop over input string until any stop condition is reached.
     */
    for (offset = 0, stop = FALSE; stop == FALSE; offset++)
    {
      /*
       * If the input string is longer that the memory available in the command
       * buffer, end processing the input string end terminate the command
       * buffer with a null character.
       * Set a checkpoint if we were not able to process the complete input
       * string.
       */
      if (ptr == max)
      {
        command[sizeof(command) - 1] = '\0';
        stop = TRUE;
        cmd[index] = NULL;
      }
      /*
       * Terminate command buffer with the null character if we reached the end
       * of the input string and end processing the string.
       */
      else if (str[offset] == '\0')
      {
        *ptr = '\0';
        stop = TRUE;
        cmd[index] = NULL;
      }
      /*
       * If the last character was a backslash, just copy the character from the
       * input string to the command buffer.
       */
      else if (type == ESCAPE)
      {
        *(ptr++) = str[offset];
        type = NOBLANK;
      }
      /*
       * In case of a non-escaped blank, terminate a command-line parameter with
       * the null character, if the previous character was not a blank.
       */
      else if (isblank((int)str[offset]) == 1)
      {
        if (type == NOBLANK)
        {
          *(ptr++) = '\0';
          type = BLANK;
        }
      }
      /*
       * Non-escaped, non-blank character.
       */
      else
      {
        /*
         * If last character has been a space (or begin of string), "create" a
         * new element in the command buffer.
         */
        if (type == BLANK)
        {
          if (index == VB_CONSOLE_PARAMS_MAXNUM)
          {
            stop = TRUE;
          }
          else
          {
            cmd[index++] = ptr;
          }
        }
        /*
         * There is still an entry for a new element in the command buffer.
         */
        if (stop == FALSE)
        {
          /*
           * A blackslash is not copied into the output string (unless it is
           * preceided by another backslash).
           */
          if (str[offset] == '\\')
          {
            type = ESCAPE;
          }
          /*
           * Copy any un-escaped, non blank character to command-buffer.
           */
          else
          {
            *(ptr++) = str[offset];
            type = NOBLANK;
          }
        }
      }
    }
  }
}

/*******************************************************************/

static void VbConsoleStateSet(BOOLEAN running)
{
  vbConsoleThreadRunning = running;
}

/*******************************************************************/

static BOOLEAN VbConsoleStateGet(void)
{
  return vbConsoleThreadRunning;
}

/*******************************************************************/

static void *VbThreadConsole(void *arg)
{
  int reuseaddr = 1;
  struct sockaddr_in6 serv_addr, cli_addr;
  socklen_t clilen = sizeof(cli_addr);

  CHAR buffer_storage[VB_CONSOLE_BUFFER_SIZE];
  CHAR *buffer = NULL;

  // "SO_REUSEADDR" will let us reuse the same address (for the server) as in
  // a previous (crashed) execution of this same process.
  //
  // More info here:
  //
  //   http://stackoverflow.com/questions/14388706/socket-options-so-reuseaddr-and-so-reuseport-how-do-they-differ-do-they-mean-t
  //
  if (setsockopt(vbConsoleSockFd,SOL_SOCKET,SO_REUSEADDR,&reuseaddr,sizeof(reuseaddr))==-1)
  {
     VbLogPrint(VB_LOG_ERROR, "Error configuring socket (SO_REUSEADDR) [%s]",strerror(errno));
  }
  else
  {
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin6_family = AF_INET6;
    serv_addr.sin6_addr   = in6addr_any;
    serv_addr.sin6_port   = htons(vbConsolePort);

    if (bind(vbConsoleSockFd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    {
      VbLogPrint(VB_LOG_ERROR, "Error on binding [%s]",strerror(errno));
    }
    else
    {
      if (listen(vbConsoleSockFd,1),0)
      {
        VbLogPrint(VB_LOG_ERROR, "Error on listen [%s]",strerror(errno));
      }
      else
      {
        while (VbConsoleStateGet() == TRUE)
        {
          INT32S console_conn = -1;

          VbLogPrint(VB_LOG_DEBUG, "Waiting for new connection...");

          bzero((char *) &cli_addr, sizeof(cli_addr));
          console_conn = accept(vbConsoleSockFd, (struct sockaddr *) &cli_addr, &clilen);

          pthread_mutex_lock(&vbConsoleMutex);
          vbConsoleConnFd = console_conn;
          pthread_mutex_unlock(&vbConsoleMutex);

          if (VbConsoleStateGet() == TRUE)
          {
            VbLogPrint(VB_LOG_DEBUG, "New console connection established!");

            if (console_conn < 0)
            {
              VbLogPrint(VB_LOG_ERROR, "Error on accept [%s]", strerror(errno));
            }
            else
            {
              // New connection received from the client.  Let's keep
              // processing messages as they are received.
              //
              BOOL vbConsoleClientConnected = TRUE;

              while(vbConsoleClientConnected)
              {
                int buffer_offset      = 0; // total bytes (from the current
                // message) received
                ssize_t bytes_received = 0; // bytes received in the last
                // call to "recv()"
                INT32U payload_length  = 0; // length of the current message
                // payload (initially unknown)

                bzero(buffer_storage, VB_CONSOLE_BUFFER_SIZE);
                buffer = buffer_storage;

                //Show prompt
                //
                VbConsoleWrite("%s#", vbConsoleName);

                // Wait until a newline character is received.
                //
                do
                {
                  bytes_received = recv(console_conn, buffer+buffer_offset, VB_CONSOLE_BUFFER_SIZE - buffer_offset, 0);
                  if (bytes_received < 0)
                  {
                    // Socket error
                    //
                    VbLogPrint(VB_LOG_ERROR, "Error reading from socket [%s]",strerror(errno));
                    vbConsoleClientConnected = FALSE;
                    break;
                  }
                  else if (bytes_received == 0)
                  {
                    // Socket was orderly closed
                    //
                    VbLogPrint(VB_LOG_DEBUG,"Socket was remotely closed");
                    vbConsoleClientConnected = FALSE;
                    break;
                  }
                  else
                  {
                    // Data was received
                    //
                    buffer_offset += bytes_received;
                  }
                } while (vbConsoleClientConnected && !strchr(buffer_storage, '\r'));

                if (vbConsoleClientConnected)
                {
                  payload_length = strchr(buffer_storage, '\r') - buffer_storage;
                  buffer_storage[payload_length] = 0x0;

                  // Execute the just received command, by invoking the (previously
                  // registered) corresponding handler for such command.
                  //
                  vbConsoleClientConnected = vbConsoleExecuteCommand(console_conn, buffer_storage);
                }
              } // while(vbConsoleClientConnected)
            }
          }

          if (console_conn != -1)
          {
            shutdown(console_conn, SHUT_RDWR);
            close(console_conn);
          }
        } // while(vbConsoleThreadRunning)
      } // listen()
    } // bind()
  } // setsockopt(SO_REUSEADDR)

  if (vbConsoleSockFd != -1)
  {
    shutdown(vbConsoleSockFd, SHUT_RDWR);
    close(vbConsoleSockFd);
  }

  VbLogPrint(VB_LOG_INFO, "Console thread finished!");

  return NULL;
}

/*******************************************************************/

static BOOL vbConsoleExecuteCommand(INT32S sk, CHAR* command)
{
  int         i;
  BOOL        return_value = TRUE;
  BOOL        return_command = TRUE;
  BOOL        copy_command = TRUE;
  CHAR       *args[VB_CONSOLE_PARAMS_MAXNUM];
  static CHAR latest_valid_command[VB_CONSOLE_BUFFER_SIZE];

  VbLogPrint(VB_LOG_DEBUG,"Processing console command: >>%s<<", command);

  if (strlen(command) == 0)
  {
    command = latest_valid_command;
    copy_command = FALSE;
    VbLogPrint(VB_LOG_DEBUG,"Empty command. Latest command will be executed instead: >>%s<<", command);
  }
  else
  {
    consoleRemoveExtraChars(command);
  }

  // Splits lastCommand in words
  consoleStr2words(command, args);

  if (args[0] != NULL)
  {
    if ((!strcmp(args[0], "help")) || (!strcmp(args[0], "?")))
    {
      VbConsoleWrite("Available commands:\n\n\thelp\n\tquit\n");
      if(copy_command == TRUE)
      {
        strncpy(latest_valid_command, command, VB_CONSOLE_BUFFER_SIZE);
        latest_valid_command[VB_CONSOLE_BUFFER_SIZE - 1] = '\0';
      }

      for (i=0; i<VB_CONSOLE_COMMANDS_MAX; i++)
      {
        if (consoleCommands[i].commandName != NULL)
        {
          VbConsoleWrite("\t%s\n", consoleCommands[i].commandName);
        }
      }
      VbConsoleWrite("\nOK\n");

      return_value = TRUE;
    }
    else if (!strcmp(args[0], "quit"))
    {
      //VbConsoleWrite("Console connection closed.");
      VbConsoleWrite("OK\n");
      return_value = FALSE;
    }
    else
    {
      pthread_mutex_lock(&vbConsoleMutex);

      for (i=0; i<VB_CONSOLE_COMMANDS_MAX; i++)
      {
        if (consoleCommands[i].commandName != NULL)
        {
          if (!strcmp(args[0], consoleCommands[i].commandName))
          {
            pthread_mutex_unlock(&vbConsoleMutex);

            if(copy_command == TRUE)
            {
              strncpy(latest_valid_command, command, VB_CONSOLE_BUFFER_SIZE);
              latest_valid_command[VB_CONSOLE_BUFFER_SIZE - 1] = '\0';
            }

            return_command = consoleCommands[i].callbackFun(consoleCommands[i].arg, VbConsoleWrite, args);

            if(return_command)
            {
              VbConsoleWrite("OK\n");
            }
            else
            {
              VbConsoleWrite("KO\n");
            }
            break;
          }
        }
      }

      if (VB_CONSOLE_COMMANDS_MAX == i)
      {
        pthread_mutex_unlock(&vbConsoleMutex);
        //VbConsoleWrite("Unknown command. Type 'help' to obtain the list of available commands.\n");
        VbConsoleWrite("KO\n");
      }
    }
  }

  return return_value;
}

/*******************************************************************/

static void VbConsoleWrite(const char *fmt, ...)
{
  va_list args;
  char    output_buffer[VB_CONSOLE_BUFFER_SIZE] = { 0 };

  va_start(args, fmt);
  vsnprintf(output_buffer, VB_CONSOLE_BUFFER_SIZE, fmt, args);
  va_end(args);

  output_buffer[VB_CONSOLE_BUFFER_SIZE-1] = 0x0;

  pthread_mutex_lock(&vbConsoleMutex);

  if (vbConsoleConnFd)
  {
    send(vbConsoleConnFd, output_buffer, strlen(output_buffer), MSG_NOSIGNAL );
  }

  pthread_mutex_unlock(&vbConsoleMutex);
}

/*******************************************************************/

/*
 ************************************************************************
 ** Public function implementation
 ************************************************************************
 */

void VbConsoleInit(int console_port, const char* name)
{
  vbConsoleThread         = 0;
  vbConsolePort           = console_port;
  VbConsoleStateSet(FALSE);

  pthread_mutex_init(&vbConsoleMutex, NULL);

  strncpy(vbConsoleName, name, VB_CONSOLE_NAME_MAX_SIZE);
  vbConsoleName[VB_CONSOLE_NAME_MAX_SIZE - 1] = '\0';

  bzero(consoleCommands, sizeof(consoleCommands));

  vbConsoleSockFd = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

  if (vbConsoleSockFd < 0)
  {
    VbLogPrint(VB_LOG_ERROR, "Error opening socket [%s]", strerror(errno));
  }
}

/*******************************************************************/

void VbConsoleStart(void)
{
  VbConsoleStop();

  VbLogPrint(VB_LOG_INFO, "Starting %s thread", VB_CONSOLE_THREAD_NAME);

  VbConsoleStateSet(TRUE);

  if (FALSE == VbThreadCreate(VB_CONSOLE_THREAD_NAME, VbThreadConsole, NULL, VB_CONSOLE_THREAD_PRIORITY, &vbConsoleThread))
  {
    VbLogPrint(VB_LOG_ERROR,"Can't create %s thread", VB_CONSOLE_THREAD_NAME);
    VbConsoleStateSet(FALSE);
  }
}

/*******************************************************************/

void VbConsoleStop( void )
{
  if (VbConsoleStateGet() == TRUE)
  {
    VbLogPrint(VB_LOG_INFO, "Stopping %s thread...", VB_CONSOLE_THREAD_NAME);

    VbConsoleStateSet(FALSE);

    pthread_mutex_lock(&vbConsoleMutex);

    if (vbConsoleConnFd != -1)
    {
      shutdown(vbConsoleConnFd, SHUT_RDWR);
    }

    if (vbConsoleSockFd != -1)
    {
      shutdown(vbConsoleSockFd, SHUT_RDWR);
    }

    pthread_mutex_unlock(&vbConsoleMutex);

    VbThreadJoin(vbConsoleThread , VB_CONSOLE_THREAD_NAME);

    // Release previously allocated memory
    VbConsoleDestroy();

    VbLogPrint(VB_LOG_INFO, "Stopped %s thread!", VB_CONSOLE_THREAD_NAME);
  }
}

/*******************************************************************/

void VbConsoleDestroy( void )
{
  INT32U i;

  pthread_mutex_lock(&vbConsoleMutex);

  for (i=0; i<VB_CONSOLE_COMMANDS_MAX; i++)
  {
    if (consoleCommands[i].commandName != NULL)
    {
      free(consoleCommands[i].commandName);
      consoleCommands[i].commandName = NULL;
      consoleCommands[i].callbackFun = NULL;
      consoleCommands[i].arg         = NULL;
    }
  }

  pthread_mutex_unlock(&vbConsoleMutex);
}

/*******************************************************************/

void VbConsoleCommandRegister(const CHAR* command_name, BOOL (*f)(void *arg, t_writeFun writeFun, char **cmd), void *arg)
{
  int i;

  pthread_mutex_lock(&vbConsoleMutex);

  for (i=0; i<VB_CONSOLE_COMMANDS_MAX; i++)
  {
    if (consoleCommands[i].commandName == NULL)
    {
      break;
    }
  }

  if (VB_CONSOLE_COMMANDS_MAX == i)
  {
    VbLogPrint(VB_LOG_ERROR, "There is no space left to register more console commands ('%s')", command_name);
  }
  else
  {
    consoleCommands[i].commandName  = strdup(command_name);
    consoleCommands[i].callbackFun  = f;
    consoleCommands[i].arg          = arg;
  }

  pthread_mutex_unlock(&vbConsoleMutex);
}

/*******************************************************************/

/**
 * @}
 **/
