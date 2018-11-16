/*
* Add Comment here
*/

#include "Drive.h"
#include "Cia402FsmImpl.h"

using namespace mcx;

Drive::Drive() : sm_transition_{sm_data_} {

}

void Drive::create_(const char *name, parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) {
    // Create common transition state
    // This state is a collection of the common behaviours
    // Other states use its events implementation.

    // Transition state needs to have a pointer to the state machine to
    // be able to execute state transitions.
    sm_transition_.setMachine(&sm_);
    // add a pointer to Transition to a shared data that states can use it
    sm_data_.transition = &sm_transition_;
    // Create state machine
    sm_.setName(name);
    sm_.createState(Init(sm_data_));
    sm_.createState(NotReadyToSwitchOn(sm_data_));
    sm_.createState(SwitchOnDisabled(sm_data_));
    sm_.createState(ReadyToSwitchOn(sm_data_));
    sm_.createState(SwitchOn(sm_data_));
    sm_.createState(OperationEnable(sm_data_));
    sm_.createState(QuickStopActive(sm_data_));
    sm_.createState(FaultReset(sm_data_));
    sm_.createState(Fault(sm_data_));

    sm_.setActiveState<Init>();
    sm_.addEvent(&Cia402FsmBase::faultAcknowledge);

}

bool Drive::initPhase1_() {

    driveFeedback_.analog_inputs.resize(4, 0);
    driveFeedback_.digital_inputs.resize(4, 0);
    driveCommand_.digital_outputs.resize(4, 0);

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

    addParameter("targetPosition", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.target_position);
    addParameter("targetVelocity", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.target_velocity);
    addParameter("targetTorque", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.target_torque);
    addParameter("torqueOffset", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.torque_offset);
    addParameter("digitalOutput1", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.digital_outputs[0]);
    addParameter("digitalOutput2", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.digital_outputs[1]);
    addParameter("digitalOutput3", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.digital_outputs[2]);
    addParameter("digitalOutput4", mcx::parameter_server::ParameterType::OUTPUT, &driveCommand_.digital_outputs[3]);

    // drive commands
    addParameter("cia402CmdOut", mcx::parameter_server::ParameterType::OUTPUT, &sm_data_.cia402_cmd);
    addParameter("cia402OpmodeOut", mcx::parameter_server::ParameterType::OUTPUT, &sm_data_.cia402_opmode);

    addParameter("cia402StateIn", mcx::parameter_server::ParameterType::INPUT, &sm_data_.cia402_status);

    // user input
    addParameter("controlword", mcx::parameter_server::ParameterType::INPUT, &driveCommand_.controlword);
    addParameter("opmode", mcx::parameter_server::ParameterType::INPUT, &driveCommand_.opmode);

    addParameter("statusword", mcx::parameter_server::ParameterType::OUTPUT, &driveFeedback_.statusword);

    // error handle
    addParameter("upload", mcx::parameter_server::ParameterType::OUTPUT, &upload_);

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

    sm_data_.cia402_opmode = driveCommand_.opmode;

    if (driveCommand_.controlword != oldControlWord) {
        switch (driveCommand_.controlword) {
            case DriveCommand::DRIVE_CMD_OFF:
                sm_.executeEvent(&Cia402FsmBase::gotoOff);
                break;
            case DriveCommand::DRIVE_CMD_DISENGAGE:
                sm_.executeEvent(&Cia402FsmBase::gotoDisengage);
                break;
            case DriveCommand::DRIVE_CMD_ENGAGE:
                sm_.executeEvent(&Cia402FsmBase::gotoEngage);
                break;
            case DriveCommand::DRIVE_CMD_FAULT_ACK:
                sm_.executeEvent(&Cia402FsmBase::faultAcknowledge);
                driveCommand_.controlword = 0;
                break;
            case DriveCommand::DRIVE_CMD_QUICK_STOP:
                sm_.executeEvent(&Cia402FsmBase::gotoQuickStop);
                break;
            default:
                break;
        }
        oldControlWord = driveCommand_.controlword;
    }

    sm_.iterate(getDtSec());

    driveFeedback_.statusword = sm_data_.drive_state;

    if (sm_data_.drive_state == DriveState::DRIVE_STATUS_FAULT) {
        upload_ = true;
    } else {
        upload_ = false;
    }

    return true;
}