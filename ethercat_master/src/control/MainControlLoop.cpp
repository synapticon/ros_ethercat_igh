/*
* Add Comment here
*/

#include "MainControlLoop.h"
#include <cstdio>

using namespace mcx;

void MainControlLoop::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {
    sub_ = nh_.subscribe<motorcortex_msgs::MotorcortexOutList>("/motorcortex_control", 1, &MainControlLoop::controlCallback, this);
    pub_ = nh_.advertise<motorcortex_msgs::MotorcortexInList>("/motorcortex_feedback", 1);

    std::string axisName = "axis";
    int counter = 0;
    for (auto &drive : drives_) {
        std::string indexedName = axisName + std::to_string(counter++);
        createSubmodule(&drive, indexedName.c_str());
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

    motorcortex_msgs::MotorcortexInList driveFeedbackList;

    for (auto &drive : drives_) {
        drive.iterate(system_time, user_time);
        driveFeedbackList.drives_feedback.push_back(drive.getDriveFeedback());
    }

    pub_.publish(driveFeedbackList);

    ros::spinOnce();

    return true;
}

void MainControlLoop::controlCallback(const motorcortex_msgs::MotorcortexOutList::ConstPtr &command_msg) {
    unsigned int max_counter = std::min(drives_.size(), command_msg->drive_command.size());
    for (unsigned int i = 0; i < max_counter; i++) {
        drives_[i].setDriveCommand(command_msg->drive_command[i]);
    }
}