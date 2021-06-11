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
import xml.etree.ElementTree as ET

from Singleton import singleton

# DEFAULT_REQUEST_PORT = 40011
# SERVER_NETW_IFACE = "eth0"
# BASE_PATH_NAME = "cluster"
# DEFAULT_STDOUT_FILE = "stdout.log"
# ENGINE_TEMPLATE_INI_FILE = "engine_template.ini"
# DATABASE_FILE = 'test.db'
# ENGINE_CONSOLE_BASE_PORT = 60000
# DEFAULT_CONSOLE_PORT = 61000
# DEFAULT_LOG_LEVEL = LogType.Debug


class CConfig(object):
    __metaclass__ = singleton

    def __init__(self):
        self.__configParamsList = {}

    def read_config_file(self, file):
        ret = True
        try:
            tree = ET.parse(file).getroot()
            for elem in tree:
                if elem.attrib['name'] not in self.__configParamsList:
                    self.__configParamsList[elem.attrib['name']] = elem.text
                # print "param %s = %s" % (elem.attrib['name'], elem.text)
        except ET.ParseError:
            ret = False

        return ret

    def get(self, name):
        if name in self.__configParamsList:
            return self.__configParamsList[name]
        else:
            return None

params = CConfig()
