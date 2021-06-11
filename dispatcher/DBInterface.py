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
import sqlite3

import os.path
import Log


class CDBInterface(object):

    def init(self, file):
        self.__dbFile = file
        if os.path.isfile(self.__dbFile):
            res = True
        else:
            res = False

        return res

    def get_cluster_info_from_driver_id(self, driver_id):

        query = "SELECT clusters.cluster_id, clusters.engine_ip, clusters.engine_port FROM driver_cluster INNER JOIN" \
                " clusters on clusters.cluster_id = driver_cluster.cluster_id WHERE driver_cluster.driver_id = '%s'" % driver_id

        try:
            dbconnection = sqlite3.connect(self.__dbFile)
            cursor = dbconnection.cursor()
            cursor.execute(query)
            cluster = cursor.fetchone()
        except sqlite3.Error, v:
            Log.logger.error("Error getting cluster info from driver id. " + v.message)
            cluster = None

        if cluster is not None:
            cluster_id = cluster[0]
            ip_addr = cluster[1]
            port = cluster[2]
            result = (cluster_id, ip_addr, port)
        else:
            result = None

        return result

    def get_cluster_info(self, id):
    # Driver id = 0 -> select all clusters.

        if id > 0:
            query = "SELECT * FROM clusters WHERE clusters.cluster_id = '%s'" % id
        else:
            query = "SELECT * FROM clusters"
        try:
            dbconnection = sqlite3.connect(self.__dbFile)
            cursor = dbconnection.cursor()
            cursor.execute(query)
            data = cursor.fetchall()
        except sqlite3.Error, v:
            Log.logger.error("Error getting cluster info from driver id. " + v.message)
            data = None

        return data

