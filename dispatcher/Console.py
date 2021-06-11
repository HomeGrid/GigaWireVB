###############################################################################
#
#
#  <legal_notice>
#  * BSD License 2.0
#  *
#  * Copyright (c) 2021, MaxLinear, Inc.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions are met:
#  * 1. Redistributions of source code must retain the above copyright notice, 
#  *    this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright notice, 
#  *    this list of conditions and the following disclaimer in the documentation 
#  *    and/or other materials provided with the distribution.
#  * 3. Neither the name of the copyright holder nor the names of its contributors 
#  *    may be used to endorse or promote products derived from this software 
#  *    without specific prior written permission.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
#  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
#  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
#  * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
#  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  * OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
#  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT \(INCLUDING NEGLIGENCE OR OTHERWISE\) 
#  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
#  * POSSIBILITY OF SUCH DAMAGE.
#  </legal_notice>
#
#
###############################################################################
import socket

import Config
import threading
import Log
from Singleton import singleton
from RemoteConsole import CRemoteConsole

# DEFAULT_CONSOLE_PORT = 61000

class CConsole(object):
    __metaclass__ = singleton

    def __init__(self):
        self.__running = False
        self.__consoleThread = None
        self.__listenSocket = None
        self.__connSocket = None
        self.__connected = False
        self.__consoleCommands = {}
        self.__consoleMutex = threading.Lock()
        self.__remoteConsole = CRemoteConsole()

    def start(self, port):
        result = True

        if self.__running:
            result = False

        self.command_register("quit", self.close_connection, "Exit the console")
        self.command_register("help", self.show_help, "Show this help")

        if result:
            try:
                # Open socket
                self.__running = True
                self.__listenSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Bind the socket to the port
                server_address = ("", port)  # to connect from outside the machine
                self.__listenSocket.bind(server_address)
                Log.logger.info('starting up on %s port %s' % server_address)
            except socket.error, v:
                Log.logger.error('Exception starting console, %s' % v.strerror)
                result = False

        if result:
            try:
                self.__running = True
                self.__consoleThread = threading.Thread(target=self.console_thread, args=())
                self.__consoleThread.start()
            except StandardError:
                result = False
                Log.logger.error("Error start console")

        return result

    def stop(self):
        try:
            self.__running = False
            if self.__connSocket is not None:
                self.__connSocket.shutdown(socket.SHUT_RDWR)
                self.__connSocket.close()
            if self.__listenSocket is not None:
                self.__listenSocket.shutdown(socket.SHUT_RDWR)
                self.__listenSocket.close()
        except socket.error, v:
            Log.logger.error("Error closing Console sockets. " + v.strerror)

    def command_register(self, command_name, callback_func, description):
        if command_name not in self.__consoleCommands:
            self.__consoleCommands[command_name] = (callback_func, description)

    def console_thread(self):
        if not self.__running:
            return

        try:
            # Listen for incoming connections
            self.__listenSocket.listen(1)
        except socket.error, v:
            Log.logger.error('Socket error listening' + v.strerror)

        Log.logger.info('Console initialized')
        while self.__running:

            try:
                # Wait for a connection
                self.__connSocket, client_address = self.__listenSocket.accept()
                Log.logger.post(Log.LogType.Debug, "new console connection")
                self.__connected = True

                self.handle_connection()

            except socket.error:
                Log.logger.error('Exception accepting new connection')
                self.__running = False

    def handle_connection(self):
        while self.__connected:
            self.write("\ndispatcher$ ")
            try:
                rx_data = str(self.__connSocket.recv(4096))
                rx_data = rx_data.rstrip()

                if self.__running:
                    if self.__remoteConsole.connected:
                        if rx_data == "quit":
                            self.__remoteConsole.stop()
                        else:
                            rx_data = rx_data+"\r"
                            self.__remoteConsole.send(rx_data)
                    else:
                        args = rx_data.split()
                        if len(args) >= 1:
                            self.execute_command(args)

                if not self.__connected or not self.__running:
                    self.__connSocket.shutdown(socket.SHUT_RDWR)
                    self.__connSocket.close()
                    self.__connected = False

            except socket.error:
                self.__connected = False
                Log.logger.error("Exception receiving data")

    def write(self, data):
        if not self.__running or not self.__connected:
            return

        try:
            self.__consoleMutex.acquire()
            if self.__connSocket.sendall(data) is not None:
                Log.logger.error("Error sending data to console")
        except socket.error:
            self.__connected = False
            Log.logger.error('Exception sending data to console')
        finally:
            self.__consoleMutex.release()
    
    def execute_command(self, args):
        # print "command \"%s\" len %d type %s" % (command_name, len(command_name), type(command_name))
        command_name = args[0]
        if command_name in self.__consoleCommands:
            (func, desc) = self.__consoleCommands[command_name]
            #execute function
            func(args)
            self.write("\nOK")
        elif command_name == "h":
            self.show_help()
            self.write("\nOK")
        else:
            #send error
            self.write("\nKO")

    def show_help(self, args):
        self.write("\nList of commands:")
        for command in self.__consoleCommands:
            self.write("\n\t%s\t- %s" % (command, self.__consoleCommands[command][1]))

    def close_connection(self, args):
        self.__connected = False

    def open_remote_console(self, ip, port):
        return self.__remoteConsole.start(ip, port)

