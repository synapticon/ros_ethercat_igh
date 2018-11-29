
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

#include "DriveSdo.h"
#include <cstdio>

using namespace mcx;

void DriveSdo::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {

}

bool DriveSdo::initPhase1_() {

    using namespace mcx::parameter_server;

    //SDOs
    auto param = addParameter("torqueControllerKp", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.controller_Kp);
    handles_.push_back(param);
    param = addParameter("torqueControllerKi", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.controller_Ki);
    handles_.push_back(param);
    param = addParameter("torqueControllerKd", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.controller_Kd);
    handles_.push_back(param);
    param = addParameter("fieldWeakeningEnable", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.field_weakening_enable);
    handles_.push_back(param);
    param = addParameter("fieldWeakeningPercentage", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.field_weakening_percentage);
    handles_.push_back(param);
    param = addParameter("fieldWeakeningStartingSpeed", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.field_weakening_starting_speed);
    handles_.push_back(param);
    param = addParameter("fieldWeakeningEndingSpeed", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.field_weakening_ending_speed);
    handles_.push_back(param);
    param = addParameter("commutationAngleMeasurementDelay", ParameterType::PARAMETER, &sdoCfg_.torqueControllerCfg.commutation_angle_measurement_delay);
    handles_.push_back(param);

    param = addParameter("velocityControllerKp", ParameterType::PARAMETER, &sdoCfg_.velocityControllerCfg.controller_Kp);
    handles_.push_back(param);
    param = addParameter("velocityControllerKi", ParameterType::PARAMETER, &sdoCfg_.velocityControllerCfg.controller_Ki);
    handles_.push_back(param);
    param = addParameter("velocityControllerKd", ParameterType::PARAMETER, &sdoCfg_.velocityControllerCfg.controller_Kd);
    handles_.push_back(param);
    param = addParameter("velocityControllerIntegralLimit", ParameterType::PARAMETER, &sdoCfg_.velocityControllerCfg.controller_integral_limit);
    handles_.push_back(param);

    param = addParameter("positionControllerPositionLoopKp", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.position_loop_Kp);
    handles_.push_back(param);
    param = addParameter("positionControllerPositionLoopKi", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.position_loop_Ki);
    handles_.push_back(param);
    param = addParameter("positionControllerPositionLoopKd", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.position_loop_Kd);
    handles_.push_back(param);
    param = addParameter("positionControllerPositionLoopIntegralLimit", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.position_loop_integral_limit);
    handles_.push_back(param);
    param = addParameter("positionControllerVelocityLoopKp", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.velocity_loop_Kp);
    handles_.push_back(param);
    param = addParameter("positionControllerVelocityLoopKi", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.velocity_loop_Ki);
    handles_.push_back(param);
    param = addParameter("positionControllerVelocityLoopKd", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.velocity_loop_Kd);
    handles_.push_back(param);
    param = addParameter("positionControllerVelocityLoopIntegralLimit", ParameterType::PARAMETER, &sdoCfg_.positionControllerCfg.velocity_loop_integral_limit);
    handles_.push_back(param);

    for (auto& handle : handles_) {
        handle.updateOutput(false);
    }

    param = addParameter("saveAllParameters", ParameterType::PARAMETER, &save_cfg_);
    save_params_handler_ = param;
    save_params_handler_.updateOutput(false);

    param = addParameter("restoreAllParameters", ParameterType::PARAMETER, &restore_cfg_);
    restore_params_handler_ = param;
    restore_params_handler_.updateOutput(false);

    return true;
}

bool DriveSdo::initPhase2_() {
    return true;
}

bool DriveSdo::startOp_() {
    return true;
}

bool DriveSdo::stopOp_() {
    return true;
}

bool DriveSdo::iterateOp_(const container::TaskTime &system_time, container::UserTime *user_time) {
    return true;
}