
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

#include "DigitalIO.h"
#include <cstdio>

using namespace mcx;

void DigitalIO::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {

}

bool DigitalIO::initPhase1_() {

    digitalInputs_.digital_inputs.resize(12, 0);
    digitalOutputs_.digital_outputs.resize(12, 0);

    std::string inputParam = "digitalInput";
    std::string outputParam = "digitalOutput";

    for (int i = 0; i < 12; i++) {
        std::string inputParamName = inputParam + std::to_string(i+1);
        std::string outputParamName = outputParam + std::to_string(i+1);
        addParameter(inputParamName.c_str(), mcx::parameter_server::ParameterType::INPUT, &digitalInputs_.digital_inputs[i]);
        addParameter(outputParamName.c_str(), mcx::parameter_server::ParameterType::OUTPUT, &digitalOutputs_.digital_outputs[i]);
    }

    return true;
}

bool DigitalIO::initPhase2_() {
    return true;
}

bool DigitalIO::startOp_() {
    return true;
}

bool DigitalIO::stopOp_() {
    return true;
}

bool DigitalIO::iterateOp_(const container::TaskTime &system_time, container::UserTime *user_time) {
    return true;
}