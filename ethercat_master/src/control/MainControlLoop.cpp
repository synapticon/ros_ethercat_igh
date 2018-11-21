/*
* Add Comment here
*/

#include "MainControlLoop.h"
#include <cstdio>

using namespace mcx;

MainControlLoop::MainControlLoop(unsigned int number_of_drives, unsigned int number_of_ios) : number_of_drives_{
        number_of_drives}, number_of_ios_{number_of_ios} {

    opmode_ = new int[number_of_drives]{};
    controlword_ = new int[number_of_drives]{};
    statusword_ = new int[number_of_drives]{};
    drives_ = new Drive[number_of_drives];
    dio_devices_ = new DigitalIO[number_of_ios];

}

MainControlLoop::~MainControlLoop() {

    delete[] opmode_;
    delete[] controlword_;
    delete[] statusword_;
    delete[] drives_;
    delete[] dio_devices_;
}

void MainControlLoop::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {
    sub_drives_ = nh_.subscribe<motorcortex_msgs::DriveOutList>("/drive_control", 1,
                                                                &MainControlLoop::drivesControlCallback, this);
    sub_dios_ = nh_.subscribe<motorcortex_msgs::DigitalOutputsList>("/digital_outputs", 1,
                                                                    &MainControlLoop::diosControlCallback, this);

    drive_feedback_pub_ = nh_.advertise<motorcortex_msgs::DriveInList>("/drive_feedback", 1);
    digital_inputs_pub_ = nh_.advertise<motorcortex_msgs::DigitalInputsList>("/digital_inputs", 1);

    service_get_sdo_ = nh_.advertiseService("get_sdo_config", &MainControlLoop::getSDOSrv, this);
    service_set_sdo_ = nh_.advertiseService("set_sdo_config", &MainControlLoop::setSDOSrv, this);

    std::string axisName = "axis";
    for (unsigned int i = 0; i < number_of_drives_; i++) {
        std::string indexedName = axisName + std::to_string(i + 1);
        createSubmodule(&drives_[i], indexedName.c_str());
    }

    std::string deviceName = "device";
    for (unsigned int i = 0; i < number_of_ios_; i++) {
        std::string indexedName = deviceName + std::to_string(i + 1);
        createSubmodule(&dio_devices_[i], indexedName.c_str());
    }

}

bool MainControlLoop::initPhase1_() {
    // user input
    addParameter("controlword", mcx::parameter_server::ParameterType::OUTPUT, controlword_, number_of_drives_);
    addParameter("opmode", mcx::parameter_server::ParameterType::OUTPUT, opmode_, number_of_drives_);
    addParameter("statusword", mcx::parameter_server::ParameterType::INPUT, statusword_, number_of_drives_);

    return true;
}

bool MainControlLoop::initPhase2_() {
    return true;
}

bool MainControlLoop::startOp_() {
    return true;
}

bool MainControlLoop::stopOp_() {
    return true;
}

bool MainControlLoop::iterateOp_(const container::TaskTime &system_time, container::UserTime *user_time) {

    ros::spinOnce();

    motorcortex_msgs::DriveInList driveFeedbackList;

    for (unsigned int i = 0; i < number_of_drives_; i++) {
        drives_[i].iterate(system_time, user_time);
        controlword_[i] = drives_[i].getControlWord();
        opmode_[i] = drives_[i].getOpMode();
        drives_[i].setStatusWord(statusword_[i]);
        driveFeedbackList.drives_feedback.push_back(drives_[i].getDriveFeedback());
    }

    motorcortex_msgs::DigitalInputsList digitalInputsList;

    for (unsigned int i = 0; i < number_of_ios_; i++) {
        dio_devices_[i].iterate(system_time, user_time);
        digitalInputsList.devices_feedback.push_back(dio_devices_[i].getDIOFeedback());
    }

    drive_feedback_pub_.publish(driveFeedbackList);
    digital_inputs_pub_.publish(digitalInputsList);

    return true;
}

void MainControlLoop::drivesControlCallback(const motorcortex_msgs::DriveOutList::ConstPtr &drives_command_msg) {
    unsigned int max_counter = std::min(number_of_drives_, drives_command_msg->drive_command.size());
    for (unsigned int i = 0; i < max_counter; i++) {
        drives_[i].setDriveCommand(drives_command_msg->drive_command[i]);
    }
}

void MainControlLoop::diosControlCallback(const motorcortex_msgs::DigitalOutputsList::ConstPtr &dios_command_msg) {
    unsigned int max_counter = std::min(number_of_ios_, dios_command_msg->devices_command.size());
    for (unsigned int i = 0; i < max_counter; i++) {
        dio_devices_[i].setDigitalOutputs(dios_command_msg->devices_command[i]);
    }
}

bool MainControlLoop::getSDOSrv(motorcortex_msgs::GetSDOCfg::Request &req,
                                motorcortex_msgs::GetSDOCfg::Response &res) {
    unsigned int max_counter = std::min(number_of_drives_, req.read_cfg.size());

    for (unsigned int i = 0; i < max_counter; i++) {
        //ToDO: request an update
        drives_[i].requestSDOUpdate(req.read_cfg[i]);

        //ToDo: receive the updated data
        Drive::SDOCfg SDOCfg = drives_[i].getSDOCfg();
        res.torque_controller_cfg.push_back(SDOCfg.torqueControllerCfg);
        res.velocity_controller_cfg.push_back(SDOCfg.velocityControllerCfg);
        res.position_controller_cfg.push_back(SDOCfg.positionControllerCfg);
    }
    return true;
}

bool MainControlLoop::setSDOSrv(motorcortex_msgs::SetSDOCfg::Request &req,
                                motorcortex_msgs::SetSDOCfg::Response &res) {
    for (unsigned int i = 0; i < number_of_drives_; i++) {
        Drive::SDOCfg sdoCfg;
        if (i < req.torque_controller_cfg.size()) {
            sdoCfg.torqueControllerCfg = req.torque_controller_cfg[i];
        }
        if (i < req.velocity_controller_cfg.size()) {
            sdoCfg.velocityControllerCfg = req.velocity_controller_cfg[i];
        }
        if (i < req.velocity_controller_cfg.size()) {
            sdoCfg.positionControllerCfg = req.position_controller_cfg[i];
        }
        //ToDo: set the configuration
        drives_[i].setSDOCfg(sdoCfg);

        //ToDo: notify of success or failure
        res.success.push_back(true);
    }
}