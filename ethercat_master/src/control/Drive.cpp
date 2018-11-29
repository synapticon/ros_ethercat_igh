
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