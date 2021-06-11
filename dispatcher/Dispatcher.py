#!/usr/bin/env python
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

import Config
from RequestPort import CRequestsPort
from Cluster import CCluster
from Cluster import EngineStatus
import DBInterface
from Console import CConsole
import Log
import sys


class Dispatcher(object):
# Contains a list of cluster objects and handles the requests comming from the drivers.
# Initially the clusters list is empty and as the drivers send their requests, new cluster
# objects are added to the list.

    def __init__(self):
        self.__clustersList = {}
        self.requestsPortInterface = CRequestsPort(int(Config.params.get("DEFAULT_REQUEST_PORT")))
        self.requestsPortInterface.getEngineAddress = self.new_driver_connection_callback

        self.dbInterface = DBInterface.CDBInterface()
        # register console commands
        console = CConsole()
        console.command_register("cluster", self.cluster_command, "Get information/actions from cluster")
        console.command_register("report", self.console_report, "Show clusters list")
        console.command_register("kill", self.console_kill, "Terminate program")

    def start(self):
    # Init the requests port and db interface to attend drivers requests
        result = True
        if not self.requestsPortInterface.start():
            result = False
        if result and not self.dbInterface.init(Config.params.get("DATABASE_FILE")):
            result = False
        if result:
            clusters = self.dbInterface.get_cluster_info(0)
            for data in clusters:
                cluster_id = data[0]
                ip_address = data[3]
                conf_port = data[4]
                console_base_port = int(Config.params.get("ENGINE_CONSOLE_BASE_PORT"))
                self.create_cluster(cluster_id, ip_address, conf_port, ("engine_%d" % cluster_id),
                                    console_base_port + cluster_id)
        return result
        # Check returns

    def stop(self):
    # Stop listening at requests port
        self.requestsPortInterface.stop()
        for cluster_id in self.__clustersList:
            self.__clustersList[cluster_id].engine_terminate()
        Log.logger.info("Stop request port and clusters")

    def check_engine_status(self, cluster_id):
        if cluster_id in self.__clustersList:
            res = self.__clustersList[cluster_id].engine_status_get()
        else:
            res = EngineStatus.Unknown
        return res

    def create_cluster(self, cluster_id, ip_address, port, engine_id, console_port):
    # type: (object, object, object, object, object) -> object
    # Add a new cluster object to the clusters list
        if cluster_id in self.__clustersList:
            return

        # cluster_id, ip_address, port, engine_id, console_port
        self.__clustersList[cluster_id] = CCluster(cluster_id, ip_address, port, engine_id, console_port)
        Log.logger.info("Create cluster %d addr %s : %d" % (cluster_id, ip_address, port))

    def destroy_all_cluster(self):
        for cluster in self.__clustersList:
            self.__clustersList[cluster].engine_terminate()
            del self.__clustersList[cluster]

    def new_driver_connection_callback(self, driver_id):
        # get corresponding cluster_id
        cluster_info = self.dbInterface.get_cluster_info_from_driver_id(driver_id)

        if cluster_info is None:
            return None

        Log.logger.info("New driver connection, driver id %s" % driver_id)
        cluster_id = cluster_info[0]
        ip_address = cluster_info[1]
        conf_port = cluster_info[2]

        # check engine status
        if cluster_info not in self.__clustersList:
            console_base_port = int(Config.params.get("ENGINE_CONSOLE_BASE_PORT"))
            # Add instance
            self.create_cluster(cluster_id, ip_address, conf_port, ("engine_%d" % cluster_id),
                                console_base_port + cluster_id)

        if self.check_engine_status(cluster_id) != EngineStatus.Running:
            self.__clustersList[cluster_id].engine_launch()

        return ip_address, conf_port

    def console_report(self, args):
        console = CConsole()

        clusters = self.dbInterface.get_cluster_info(0)
        console.write("\nClusters Report")
        console.write("\n=========================================================================")
        console.write("\n|Cluster Id| Cluster Name |    Engine IP    |Engine port| Engine status |")
        console.write("\n=========================================================================")

        for data in clusters:
            console.write("\n|    %2d    | %11s  | %15s | %9d |" % (data[0], data[1], data[3], data[4]))
            if data[0] in self.__clustersList:
                if self.__clustersList[data[0]].engine_status_get():
                    console.write("   running     |")
                else:
                    console.write("   stopped     |")
            else:
                console.write("   stopped     |")

        console.write("\n=========================================================================")

    def console_kill(self, args):
        console = CConsole()

        console.write("\nKill")
        self.stop()
        console.stop()
        Log.logger.stop()

    def cluster_command(self, args):
        console = CConsole()

        if len(args) < 3:
            console.write("\nNo cluster specified")
            console.write("\nusage: cluster [id] [option], where option:")
            console.write("\n\tlog\t-engine output log")
            console.write("\n\tconsole\t-connect to engine console")
            console.write("\n\tstart\t-launch engine")
            console.write("\n\tstop\t-stop engine")
            return False

        key = int(args[1])
        cmd = args[2]

        if key not in self.__clustersList:
            console.write("\nInvalid Cluster id")
            return False
        else:
            if cmd == "log":
                log = self.__clustersList[key].engine_output_log_get()
                console.write(log)
            elif cmd == "console":
                (ip, port) = self.__clustersList[key].engine_console_port_get()
                Log.logger.info("Open remote console with %s : %d" % (ip, port))
                if console.open_remote_console(ip, port):
                    console.write("\nEngine console open. To exit type \"quit\"")
                else:
                    console.write("\nError opening engine console.")
            elif cmd == "start":
                if self.__clustersList[key].engine_launch():
                    console.write("\nEngine running")
                else:
                    console.write("\nError launching engine")
            elif cmd == "stop":
                self.__clustersList[key].engine_terminate()
                console.write("\nEngine stopped")


if __name__ == "__main__":

    error = False
    if not Config.params.read_config_file("config.xml"):
        print >> sys.stderr, "Error reading config.xml"
        error = True
    else:
        disp_console = CConsole()
        dispatcher = Dispatcher()

    if not error:
        if not Log.logger.start(Log.LogType.description.index(Config.params.get("DEFAULT_LOG_LEVEL"))):
            error = True
            Log.logger.stop()
            print >> sys.stderr, "Error starting Log"

    if not error:
        if not disp_console.start(int(Config.params.get("DISPATCHER_CONSOLE_PORT"))):
            error = True
            print >> sys.stderr, "Error starting Console"
            disp_console.stop()
            Log.logger.stop()

    if not error:
        if not dispatcher.start():
            error = True
            print >> sys.stderr, "Error starting Dispatcher"
            dispatcher.stop()
            disp_console.stop()
            Log.logger.stop()

    if not error:
        Log.logger.info("Dispatcher started")
