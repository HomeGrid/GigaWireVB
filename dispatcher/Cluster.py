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
import os

import subprocess
import Log
import Config

DEFAULT_INI_FILE = "vb_engine.ini"
DEFAULT_STDOUT_FILE = "engine_stdout.txt"
ENGINE_TEMPLATE_INI_FILE = "engine_template.ini"

class EngineStatus(object):
    Unknown = 0
    Stopped = 1
    Running = 2


class CCluster(object):

    def __init__(self, cluster_id, ip_address, port, engine_id, console_port):
        self.__clusterId = cluster_id
        self.__engineRunning = EngineStatus.Stopped
        self.__engineIPAddress = ip_address
        self.__engineServerPort = port
        self.__engineId = engine_id
        self.__engineConsolePort = console_port
        self.__basePath = Config.params.get("BASE_PATH_NAME") + str(self.__clusterId) + "/"
        self.__program = None
        self.__stdoutFile = None

    def engine_status_get(self):
        if self.__program is not None:
            if self.__program.poll() is None:
                self.__engineRunning = True
            else:
                self.__engineRunning = False
        else:
            self.__engineRunning = False

        return self.__engineRunning

    def engine_launch(self):
        if self.engine_status_get():
            return True

        ret = True
        # generate ini file
        if not self.generate_ini_file():
            ret = False
        else:
            try:
                self.__stdoutFile = open(self.__basePath + DEFAULT_STDOUT_FILE, "w+")
                # call instance
                self.__program = subprocess.Popen(["sudo", os.getcwd()+"/vector_boost_engine", "-f",
                                                os.getcwd()+"/"+self.__basePath + DEFAULT_INI_FILE],
                                                shell=False,
                                                cwd=self.__basePath,
                                                stdout=self.__stdoutFile,
                                                stderr=subprocess.STDOUT)
            except OSError, v:
                Log.logger.error("Error running vector_boost_engine. Error %s" % v.strerror)
                ret = False
        return ret

    def engine_terminate(self):
        if self.engine_status_get():
            self.__program.terminate()

    def engine_output_log_get(self):

        output = ""
        try:
            if self.__program is not None:
                out_file = open(self.__basePath + DEFAULT_STDOUT_FILE, "r")
                output = out_file.read()
        except OSError, v:
            Log.logger.error("Error reading engine output log. Error %s" % v.strerror)

        return output

    def engine_address_port_get(self):
        return self.__engineIPAddress, self.__engineServerPort

    def engine_console_port_get(self):
        return self.__engineIPAddress, self.__engineConsolePort

    def generate_ini_file(self):
        # Read in the file
        try:
            with open(Config.params.get("ENGINE_TEMPLATE_INI_FILE"), 'r') as pattern_file:
                file_data = pattern_file.read()
        except (OSError, IOError), v:
            Log.logger.error("Error opening template ini file" + v.strerror)
            return False

        # Replace the target string
        file_data = file_data.replace('{engineid}', self.__engineId)
        file_data = file_data.replace('{port}', str(self.__engineServerPort))
        file_data = file_data.replace('{iface}', Config.params.get("SERVER_NETW_IFACE"))
        file_data = file_data.replace('{consolePort}', str(self.__engineConsolePort))

        try:

            # Create a separate directory for each instance
            if not os.path.exists(self.__basePath):
                os.makedirs(self.__basePath)

            # Write the file out again
            # Overwrite the file if already exists
            with open(self.__basePath + DEFAULT_INI_FILE, 'w') as dest_file:
                dest_file.write(file_data)
        except (OSError, IOError), v:
            Log.logger.error("Error writing ini file." + v.strerror)
            return False

        return True

