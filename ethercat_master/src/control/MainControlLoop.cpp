/*
* Add Comment here
*/

#include "MainControlLoop.h"
#include <stdio.h>

using namespace mcx;

void MainControlLoop::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {
    sub_ = nh_.subscribe<std_msgs::String>("/command", 10, &MainControlLoop::controlCallback, this);
    pub_  = nh_.advertise<std_msgs::String>("/feedback", 1);
    count_ = 0;
}

bool MainControlLoop::initPhase1_() {

    addParameter("inputVelocity", mcx::parameter_server::ParameterType::INPUT, &input_velocity_);
    addParameter("outputVelocity", mcx::parameter_server::ParameterType::OUTPUT, &output_velocity_);
    addParameter("gain", mcx::parameter_server::ParameterType::PARAMETER, &gain_);

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

    output_velocity_ = input_velocity_ *gain_;

    std_msgs::String msg;
    std::stringstream ss;
    ss << "running " << count_;
    msg.data = ss.str();

    pub_.publish(msg);
    count_++;

    ros::spinOnce();

    return true;
}

void MainControlLoop::controlCallback(const std_msgs::String::ConstPtr& command_msg) {
    std::cout << command_msg->data << std::endl;
}