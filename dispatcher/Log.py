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
import sys

import inspect
import threading
from Singleton import singleton
import mutex
import Queue


class LogType(object):
    Debug = 0
    Info = 1
    Error = 2
    description = ["Debug", " Info", "Error"]


class LogMessage(object):
    def __init__(self):
        self.type = LogType.Info
        self.text = ""
        self.caller = ""


class CLog:
    __metaclass__ = singleton

    def __init__(self):
        self.__running = False
        self.__logThread = None
        self.__logMutex = mutex.mutex()
        self.__logsQueue = Queue.Queue()
        self.logLevel = LogType.Debug

    def start(self, log_level):
        result = True

        if log_level == 0:
            self.logLevel = LogType.Debug
        elif log_level == 1:
            self.logLevel = LogType.Info
        elif log_level == 2:
            self.logLevel = LogType.Error

        if self.__running:
            result = False

        if result:
            try:
                self.__running = True
                self.__logThread = threading.Thread(target=self.log_thread, args=())
                self.__logThread.start()
            except StandardError:
                result = False
        return result

    def stop(self):
        self.__running = False
        self.post(LogType.Debug, "Close Log")

    def log_thread(self):
        print "Start Log"
        while self.__running:
            try:
                msg = self.__logsQueue.get()

                print_msg = False

                if msg is not None:
                    if msg.type >= self.logLevel:
                        print_msg = True

                if msg.type == LogType.Error:
                    out_file = sys.stderr
                else:
                    out_file = sys.stdout

                if print_msg:
                    print >> out_file, "[%20s][%s] %s" % (msg.caller, LogType.description[msg.type], msg.text)

            except StandardError, v:
                print "Error reading msg from log queue %s" % v.message

    def post(self, log_type, text):
        msg = LogMessage()
        msg.type = log_type
        msg.text = text
        msg.caller = inspect.stack()[1][3]

        self.__logsQueue.put(msg)

    def info(self, text):
        msg = LogMessage()
        msg.type = LogType.Info
        msg.text = text
        msg.caller = inspect.stack()[1][3]

        self.__logsQueue.put(msg)

    def error(self, text):
        msg = LogMessage()
        msg.type = LogType.Error
        msg.text = text
        msg.caller = inspect.stack()[1][3]

        self.__logsQueue.put(msg)

# Global object
logger = CLog()
