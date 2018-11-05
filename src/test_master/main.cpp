/*
* Add Comment here
*/

#include "MainControlLoop.h"

#include <mcx/mcx_core.h>

using namespace mcx;

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
    MainControlLoop main_control_loop;
    main_control_loop.create("Control", &param_server, rt_dt_micro_s);

// creates cia402 driver
    mcx::drive::Module cia402(cmd_args.system_mode, mcx::drive::DriveType::CiA402, 2);
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
    while (utils::running()) {
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

    using namespace utils;

// default settings
    utils::CommandLineArgs command_line_args;
    command_line_args.config_path = "../config";
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
// runs awsome controls
    run(command_line_args);
// stops low latency, removes CPU isolation
    utils::stopRealTime();

    return 0;
}