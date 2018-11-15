/*
* Add Comment here
*/

#include "MainControlLoop.h"
#include <cstdio>

using namespace mcx;

void MainControlLoop::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {
    sub_drives_ = nh_.subscribe<motorcortex_msgs::MotorcortexOutList>("/motorcortex_control", 1,
                                                                      &MainControlLoop::drivesControlCallback, this);
    sub_dios_ = nh_.subscribe<motorcortex_msgs::DigitalOutputsList>("/digital_outputs", 1,
                                                                    &MainControlLoop::diosControlCallback, this);
    drive_feedback_pub_ = nh_.advertise<motorcortex_msgs::MotorcortexInList>("/motorcortex_feedback", 1);
    digital_inputs_pub_ = nh_.advertise<motorcortex_msgs::DigitalInputsList>("/digital_inputs", 1);

    std::string axisName = "axis";
    int counter = 0;
    for (auto &drive : drives_) {
        std::string indexedName = axisName + std::to_string(++counter);
        createSubmodule(&drive, indexedName.c_str());
    }

    std::string deviceName = "device";
    counter = 0;
    for (auto &dio_device : dio_devices_) {
        std::string indexedName = deviceName + std::to_string(++counter);
        createSubmodule(&dio_device, indexedName.c_str());
    }

}

bool MainControlLoop::initPhase1_() {

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

    motorcortex_msgs::MotorcortexInList driveFeedbackList;

    for (auto &drive : drives_) {
        drive.iterate(system_time, user_time);
        driveFeedbackList.drives_feedback.push_back(drive.getDriveFeedback());
    }

    motorcortex_msgs::DigitalInputsList digitalInputsList;

    for (auto &dio_device : dio_devices_) {
        dio_device.iterate(system_time, user_time);
        digitalInputsList.devices_feedback.push_back(dio_device.getDIOFeedback());
    }

    drive_feedback_pub_.publish(driveFeedbackList);
    digital_inputs_pub_.publish(digitalInputsList);

    return true;
}

void MainControlLoop::drivesControlCallback(const motorcortex_msgs::MotorcortexOutList::ConstPtr &drives_command_msg) {
    unsigned int max_counter = std::min(drives_.size(), drives_command_msg->drive_command.size());
    for (unsigned int i = 0; i < max_counter; i++) {
        drives_[i].setDriveCommand(drives_command_msg->drive_command[i]);
    }
}

void MainControlLoop::diosControlCallback(const motorcortex_msgs::DigitalOutputsList::ConstPtr &dios_command_msg) {
    unsigned int max_counter = std::min(dio_devices_.size(), dios_command_msg->devices_command.size());
    for (unsigned int i = 0; i < max_counter; i++) {
        dio_devices_[i].setDigitalOutputs(dios_command_msg->devices_command[i]);
    }
}