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

import threading
import Console
import Log


class CRemoteConsole(object):
# Used to connect the engine console with the dispatcher console
# This class opens a connection with the engine console and redirects
# the data comming from the dispatcher console and the engine console

    def __init__(self):
        self.__running = False
        self.__connSocket = None
        self.connected = False
        self.__consoleThread = None
        self.dataRXCallback = None

    def start(self, ip, port):
        result = True

        if self.__running:
            result = False

        if result:
            try:
                # Open socket
                self.__running = True
                self.__connSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Bind the socket to the port
                server_address = ("127.0.0.1", port)  # ip and port of engine
                self.__connSocket.connect(server_address)
                Log.logger.info('Remote console connected to %s port %s' % server_address)
                self.connected = True
            except socket.error, v:
                Log.logger.error('Exception connecting to remote engine console, %s' % v.strerror)
                result = False

        if result:
            try:
                self.__running = True
                self.__consoleThread = threading.Thread(target=self.remote_console_thread, args=())
                self.__consoleThread.start()
            except StandardError:
                result = False
                Log.logger.error("Error start console")

        return result

    def stop(self):
        try:
            self.__running = False
            self.connected = False
            self.__connSocket.shutdown(socket.SHUT_RDWR)
            self.__connSocket.close()
        except socket.error:
            Log.logger.error("Error closing Console sockets")

    def remote_console_thread(self):
        if not self.__running:
            return

        console = Console.CConsole()

        while self.__running:

            try:
                rx_data = str(self.__connSocket.recv(4096))
                # rx_data = rx_data.rstrip()

                if self.__running:
                    console.write(rx_data)
                    # if self.dataRXCallback is not None:
                    #     self.dataRXCallback(rx_data)

            except socket.error:
                Log.logger.error('Exception accepting new connection')
                self.__running = False

    def send(self, data):
        if not self.__running:
            return

        try:
            if self.__connSocket.sendall(data) is not None:
                Log.logger.info("Error sending data to remote console")
        except socket.error:
            self.connected = False
            Log.logger.error('Exception sending data to console')
