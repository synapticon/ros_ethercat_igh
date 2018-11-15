/*
* Add Comment here
*/

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