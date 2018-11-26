/*
* Add Comment here
*/

#include "Drive.h"

using namespace mcx;


void Drive::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {

    createSubmodule(&drive_sdo_read_, "SDORead");
    createSubmodule(&drive_sdo_write_, "SDOWrite");

}

bool Drive::initPhase1_() {

    using namespace mcx::parameter_server;

    driveFeedback_.analog_inputs.resize(4, 0);
    driveFeedback_.digital_inputs.resize(4, 0);
    driveCommand_.digital_outputs.resize(4, 0);

    addParameter("driveErrorCode", ParameterType::INPUT, &driveFeedback_.drive_error_code);
    addParameter("slaveTimestamp", ParameterType::INPUT, &driveFeedback_.slave_timestamp);
    addParameter("positionValue", ParameterType::INPUT, &driveFeedback_.position_value);
    addParameter("velocityValue", ParameterType::INPUT, &driveFeedback_.velocity_value);
    addParameter("torqueValue", ParameterType::INPUT, &driveFeedback_.torque_value);
    addParameter("secondaryPositionValue", ParameterType::INPUT, &driveFeedback_.secondary_position_value);
    addParameter("secondaryVelocityValue", ParameterType::INPUT, &driveFeedback_.secondary_velocity_value);
    addParameter("analogInput1", ParameterType::INPUT, &driveFeedback_.analog_inputs[0]);
    addParameter("analogInput2", ParameterType::INPUT, &driveFeedback_.analog_inputs[1]);
    addParameter("analogInput3", ParameterType::INPUT, &driveFeedback_.analog_inputs[2]);
    addParameter("analogInput4", ParameterType::INPUT, &driveFeedback_.analog_inputs[3]);
    addParameter("digitalInput1", ParameterType::INPUT, &driveFeedback_.digital_inputs[0]);
    addParameter("digitalInput2", ParameterType::INPUT, &driveFeedback_.digital_inputs[1]);
    addParameter("digitalInput3", ParameterType::INPUT, &driveFeedback_.digital_inputs[2]);
    addParameter("digitalInput4", ParameterType::INPUT, &driveFeedback_.digital_inputs[3]);

    addParameter("targetPosition", ParameterType::OUTPUT, &driveCommand_.target_position);
    addParameter("targetVelocity", ParameterType::OUTPUT, &driveCommand_.target_velocity);
    addParameter("targetTorque", ParameterType::OUTPUT, &driveCommand_.target_torque);
    addParameter("torqueOffset", ParameterType::OUTPUT, &driveCommand_.torque_offset);
    addParameter("digitalOutput1", ParameterType::OUTPUT, &driveCommand_.digital_outputs[0]);
    addParameter("digitalOutput2", ParameterType::OUTPUT, &driveCommand_.digital_outputs[1]);
    addParameter("digitalOutput3", ParameterType::OUTPUT, &driveCommand_.digital_outputs[2]);
    addParameter("digitalOutput4", ParameterType::OUTPUT, &driveCommand_.digital_outputs[3]);

    return true;
}

bool Drive::initPhase2_() {
    return true;
}

bool Drive::startOp_() {
    return true;
}

bool Drive::stopOp_() {
    return true;
}

bool Drive::iterateOp_(const container::TaskTime &system_time, container::UserTime *user_time) {

    drive_sdo_write_.iterate(system_time, user_time);
    drive_sdo_read_.iterate(system_time, user_time);

    return true;
}