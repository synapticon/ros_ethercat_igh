
/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Vectioneer B.V.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Vectioneer nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Alexey Zakharov
* Author: Viatcheslav Tretyakov
*********************************************************************/

#include "MainControlLoop.h"

#include <mcx/mcx_core.h>

#include <ros/package.h>
#include "ros/ros.h"

using namespace mcx;

#define NUM_OF_DRIVES 2
#define NUM_OD_DIOS 0

void link(parameter_server::Parameter *ps) {


    ps->link("root/Control/controlword", "root/cia402/driveCommand");
    ps->link("root/Control/opmode", "root/cia402/driveMode");
    ps->link("root/cia402/driveState", "root/Control/statusword");

    ps->link("root/Control/read_sdo", "root/EtherCAT/Domain1/read_sdo");

}

void run(const utils::CommandLineArgs &cmd_args) {

// sets cycle time of the controls
    const int rt_dt_micro_s = 1000;

// sets configuration paths
    std::string path_control_dir = cmd_args.config_path + "/control";
    std::string path_control = path_control_dir + "/control.xml";
    std::string path_ethercat = cmd_args.config_path + "/io";
    std::string path_log = cmd_args.log_path;

    parameter_server::Parameter param_server;

// creates root of the parameter tree
    param_server.create("root", nullptr);

// creates log output to a file
    mcx::log::Module logger(path_log);
    logger.create("logger", &param_server, rt_dt_micro_s);
// create and configure log output task
    mcx::container::Task logger_task("Logger_task", &param_server);
    logger_task.add(&logger);
    logger_task.configure();

// prints system configuration
    utils::printSystemConfig(cmd_args, "test_master");

// creates main control loop
    MainControlLoop main_control_loop(NUM_OF_DRIVES, NUM_OD_DIOS);
    main_control_loop.create("Control", &param_server, rt_dt_micro_s);

// creates cia402 driver
    mcx::drive::Module cia402(cmd_args.system_mode, mcx::drive::DriveType::CiA402, NUM_OF_DRIVES);
    cia402.create("cia402", &param_server, rt_dt_micro_s);

// creates and configure control task
    container::Task controls_task("Control_task", &param_server);
// adds main control loop module to the control task
    controls_task.add(&main_control_loop);
// adds cia402 drier to the control task
    controls_task.add(&cia402);
    controls_task.configure();

// creates EtherCAT master
    auto master = ecat::createMaster(cmd_args.system_mode);
// loads a list of the domains from {cmd_args.config_path}/io/pdo.xml
    ecat::Domain domain("Domain1", path_ethercat, "pdo.xml");
// adds domains to the master
    ecat::Module ethercat(master, domain);
    ethercat.create("EtherCAT", &param_server, rt_dt_micro_s);

// creates and configure EtherCAT task
    container::Task ethercat_task("EtherCAT_task", &param_server);
    ethercat_task.add(&ethercat);
    ethercat_task.configure();

// creates req/rep server
    comm::RequestReply reqrep;
// creates publisher module
    comm::Publisher publisher(reqrep, cmd_args.conn_data);
    publisher.create("ParamPub", &param_server, rt_dt_micro_s);
// creates and configure publisher task
    container::Task comm_task("Comm_task", &param_server);
    comm_task.add(&publisher);
    comm_task.configure();

// when all modules are configured req/rep caches the parameter tree
    reqrep.configure((&param_server), path_control_dir);

// loads configuration from control.xml
    parameter_server::load(path_control, &param_server);


// linking modules
    link(&param_server);

// starts logger and communication with non-realtime scheduler
    logger_task.start(rt_dt_micro_s, container::TaskSched::NORMAL);
    comm_task.start(rt_dt_micro_s, container::TaskSched::NORMAL);
// starts control and ethercat with realtime scheduler,
// attaches to isolated CPUs 0 and 1, sets priorities
    controls_task.start(rt_dt_micro_s, container::TaskSched::REALTIME, {0}, 80);
    ethercat_task.start(rt_dt_micro_s, container::TaskSched::REALTIME, {1}, 80);

// starts req/rep server
    bool is_connected = reqrep.start(cmd_args.conn_data);
    ASSERT(is_connected, "Failed to start Req/Rep server");

// running until terminate signal is received
    while (utils::running() && ros::ok()) {
        reqrep.iterate();
    }

// stops all the tasks
    reqrep.stop();
    comm_task.stop();
    ethercat_task.stop();
    controls_task.stop();
    logger_task.stop();

// clears all allocated resources
    param_server.destroy();

}

int main(int argc, char **argv) {

    // ros node initialization
    ros::init(argc, argv, "ethercat_master_server");
    std::string ros_package_path = ros::package::getPath("ethercat_master");

    using namespace utils;

// default settings
    utils::CommandLineArgs command_line_args;
    command_line_args.config_path = ros_package_path + "/src/config";
    command_line_args.log_path = "/var/log/motorcortex";
    command_line_args.system_type = 0;
    command_line_args.system_mode = SystemMode::PRODUCTION;
    command_line_args.error_level = 3;
    command_line_args.app_version = std::string("0.1");
    command_line_args.lib_version = std::string(utils::version());
    command_line_args.conn_data = {.direction =
    comm::ConnectionDir::BIND_TO_LOCAL,
            .transport = "ws", .address = "*", .req_port = "5558", .pub_port
            = "5557"};
// parse settings from command line
    parseCmdLine(argc, argv, &command_line_args);

// starts low latency, isolates CPU 0 and 1
    utils::startRealTime({0, 1});
// runs awesome controls
    run(command_line_args);

// stops low latency, removes CPU isolation
    utils::stopRealTime();

    return 0;
}