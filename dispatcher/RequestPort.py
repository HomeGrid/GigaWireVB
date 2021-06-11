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
from time import sleep
import Log
import re

MAX_RESPONSE_READ_ATTEMPTS = 5
MESSAGE_HEADER_LENGTH = 4

class CRequestsPort(object):
# This class receives connection requests from the drivers and handles the
# communication with them to get the driver_id and return the new ip and port
# of the engine to connect to

    # Callback
    getEngineAddress = None

    def __init__(self, port):
        self.__status = False
        self.listenPort = port
        self.__listenSocket = None
        self.__connSocket = None
        self.__listenThread = None

    def start(self):
        result = True

        if self.__status:
            result = False

        if result:
            try:
                self.__status = True
                self.__listenSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                # Bind the socket to the port
                server_address = ("", self.listenPort)  # to connect from outside the machine
                self.__listenSocket.bind(server_address)
                Log.logger.info('Starting request port on port %s' % server_address[1])
            except socket.error, v:
                Log.logger.error('Error starting Request Port on %s port %s. %s' % (server_address[0], server_address[1], v.strerror))
                result = False
        if result:
            __listenThread = threading.Thread(target=self.listen_thread, args=())
            __listenThread.start()
        return result

    def stop(self):
        self.__status = False
        if self.__listenSocket is not None:
            self.__listenSocket.shutdown(socket.SHUT_RDWR)
        self.__listenSocket.close()

    def listen_thread(self):

        if not self.__status:
            return

        try:
            # Listen for incoming connections
            self.__listenSocket.listen(1)
        except socket.error, v:
            Log.logger.error('Socket error listening', v.strerror)

        while self.__status:

            try:
                Log.logger.info('waiting for driver connection')
                # Wait for a connection
                self.__connSocket, client_address = self.__listenSocket.accept()

                Log.logger.post(Log.LogType.Debug, 'New Connection from ' + client_address[0])

                # Get driver_id from remote driver
                (version_str, driver_id) = self.request_driver_id()
                #Comprobar version y driver id
                version_arr = re.findall(r'\d+', version_str) # 2.0 r165

                if version_arr[2] >= 165:
                    # Get local engine address
                    (ip_address, port) = self.getEngineAddress(driver_id)

                    if not all((ip_address, port)):
                        Log.logger.post(Log.LogType.Debug, "Error, driver id not registered")
                    else:
                        #Send address
                        self.send_engine_address(ip_address, port, driver_id)
                        sleep(1)
                else:
                    Log.logger.error('waiting for driver connection')

                self.__connSocket.shutdown(socket.SHUT_RDWR)
                self.__connSocket.close()

            except socket.error:
                Log.logger.error('Exception accepting new connection')

    def receive_message(self):

        rx_complete = False
        counter = 0
        header = bytearray()
        while not rx_complete:
            try:
                rx_bytes = bytearray(self.__connSocket.recv(MESSAGE_HEADER_LENGTH))

                counter += len(rx_bytes)
                header.extend(rx_bytes)
                if counter >= 4:
                    rx_complete = True
            except socket.error:
                break

        if not rx_complete:
            return None

        ea_code_flag = int(header[0])
        payload_length = socket.ntohs(header[2] * 256 + header[1])
        op_code = int(header[3])

        # print "Header: flag %s length %d opcode %s" % (hex(ea_code_flag), payload_length, hex(op_code))

        if ea_code_flag != 0xa5:
            return None

        payload = ""
        if payload_length == 0:
            rx_complete = True
        else:
            rx_complete = False

        while not rx_complete:
            try:
                payload += self.__connSocket.recv(payload_length)

                if len(payload) >= payload_length:
                    rx_complete = True
            except socket.error:
                break

        if not rx_complete:
            return None

        return op_code, payload

    def request_driver_id(self):
        version_request_msg = "\xa5\x00\x00\x0e"
        version = None
        driver_id = None

        try:
            self.__connSocket.settimeout(20)
            self.__connSocket.sendall(version_request_msg)
            correct_ans = False
            attempts = 0

            while not correct_ans and attempts < MAX_RESPONSE_READ_ATTEMPTS:

                (op_code, response) = self.receive_message()

                if not all((op_code, response)):
                    continue
                # Expected op_code
                if op_code != 0x0f:
                    continue

                version = response[0:31].rstrip('\0')  # CHAR version[VB_EA_VERSION_MAX_SIZE]; 31
                driver_id = response[31:51].rstrip('\0')  # CHAR driverId[VB_EA_DRIVER_ID_MAX_SIZE]; 21

                # 2.0   r165
                correct_ans = True

            if correct_ans:
                Log.logger.post(Log.LogType.Debug, "Version %s DriverId %s " % (version, driver_id))
            elif attempts >= MAX_RESPONSE_READ_ATTEMPTS:
                Log.logger.error("No response from driver received")

        except socket.error:
            Log.logger.error("Data send exception")

        return version, driver_id

    def send_engine_address(self, ip_addr, port, driver_id): # connection -> atributo de la clase
        #Build message
        header_msg = "\xa5\x00\x2B\x1f"

        # Length has to be 16 + 6 + 21 = 43 (0x2B)

        ip_addr_str = "{:15}".format(ip_addr) + "\x00" # length 16 'aaa.bbb.ccc.ddd\0'
        port_str    = "{:5}".format(port) + "\x00"    # length 6 '65535\0'
        driver_str  = driver_id + ("\x00" * (21 - len(driver_id))) # Pad with zeroes

        payload = ip_addr_str + port_str + driver_str

        message = header_msg + payload

        #print "Sending message %s length %d payload [%d]" % (message, len(message), len(payload))
        try:
            self.__connSocket.sendall(message)
        except socket.error:
            Log.logger.error("Error sending mesage")
