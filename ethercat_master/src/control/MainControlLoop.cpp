/*
* Add Comment here
*/

#include "MainControlLoop.h"
#include <stdio.h>

using namespace mcx;

void MainControlLoop::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {
    sub_ = nh_.subscribe<std_msgs::String>("/command", 10, &MainControlLoop::controlCallback, this);
    pub_ = nh_.advertise<motion_control::MotorcortexIn>("/motorcortex_feedback", 1);
}

bool MainControlLoop::initPhase1_() {


    for (int i = 0; i < 4; i++) {
        driveFeedback_.analog_inputs.push_back(0);
        driveFeedback_.digital_inputs.push_back(0);
    }

    //addParameter("", mcx::parameter_server::ParameterType::INPUT, &);
    addParameter("statusword", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.statusword);
    addParameter("driveEnabled", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.drive_enabled);
    addParameter("driveErrorCode", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.drive_error_code);
    addParameter("slaveTimestamp", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.slave_timestamp);
    addParameter("positionValue", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.position_value);
    addParameter("velocityValue", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.velocity_value);
    addParameter("torqueValue", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.torque_value);
    addParameter("secondaryPositionValue", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.secondary_position_value);
    addParameter("secondaryVelocityValue", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.secondary_velocity_value);
    addParameter("analogInput1", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.analog_inputs[0]);
    addParameter("analogInput2", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.analog_inputs[1]);
    addParameter("analogInput3", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.analog_inputs[2]);
    addParameter("analogInput4", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.analog_inputs[3]);
    addParameter("digitalInput1", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.digital_inputs[0]);
    addParameter("digitalInput2", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.digital_inputs[1]);
    addParameter("digitalInput3", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.digital_inputs[2]);
    addParameter("digitalInput4", mcx::parameter_server::ParameterType::INPUT, &driveFeedback_.digital_inputs[3]);

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

    pub_.publish(driveFeedback_);

    ros::spinOnce();

    return true;
}

void MainControlLoop::controlCallback(const std_msgs::String::ConstPtr& command_msg) {
    std::cout << command_msg->data << std::endl;
}