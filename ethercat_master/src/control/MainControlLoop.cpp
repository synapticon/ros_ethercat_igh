/*
* Add Comment here
*/

#include "MainControlLoop.h"
#include <cstdio>

using namespace mcx;

void MainControlLoop::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {
    sub_ = nh_.subscribe<std_msgs::String>("/command", 10, &MainControlLoop::controlCallback, this);
    pub_ = nh_.advertise<motion_control::MotorcortexInList>("/motorcortex_feedback", 1);

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

    motion_control::MotorcortexInList driveFeedbackList;

    for (auto &drive : drives_) {
        drive.iterate(system_time, user_time);
        driveFeedbackList.drives_feedback.push_back(drive.getDriveFeedback());
    }

    pub_.publish(driveFeedbackList);

    ros::spinOnce();

    return true;
}

void MainControlLoop::controlCallback(const std_msgs::String::ConstPtr &command_msg) {
    std::cout << command_msg->data << std::endl;
}