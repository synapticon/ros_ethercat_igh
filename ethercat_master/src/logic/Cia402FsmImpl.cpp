//
// Created by alexey on 5-11-18.
//

#include "Cia402FsmImpl.h"
#include "Cia402FsmData.h"

using namespace mcx::state_machine;

//
Init::Init(Cia402FsmData& data) : Cia402FsmBase(data) {
  fsm_data_.cia402_cmd = 0;
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_OFF;
}

void Init::enter() {

}

EventStatus Init::faultAcknowledge() {
  setActiveState<NotReadyToSwitchOn>();
  addEvent(&Cia402FsmBase::resetting, fsm_data_.timeout_sec.fault_ack);
  return EventStatus::EVENT_DONE;
}

//
NotReadyToSwitchOn::NotReadyToSwitchOn(Cia402FsmData& data) : Cia402FsmBase(data) {
}

void NotReadyToSwitchOn::enter() {
  fsm_data_.cia402_cmd = 0;
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_OFF;
}

EventStatus NotReadyToSwitchOn::resetting() {
  return fsm_data_.transition->resetting();
}

EventStatus NotReadyToSwitchOn::terminateEvent() {
  return fsm_data_.transition->terminateEvent();
}

//
SwitchOnDisabled::SwitchOnDisabled(Cia402FsmData& data) : Cia402FsmBase(data) {
}

void SwitchOnDisabled::enter() {
  fsm_data_.cia402_cmd = 0;
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_OFF;
}

EventStatus SwitchOnDisabled::gotoDisengage() {
  addEvent(&Cia402FsmBase::shutdown, fsm_data_.timeout_sec.shutdown);
  addEvent(&Cia402FsmBase::switchOn, fsm_data_.timeout_sec.switch_on);
  return EventStatus::EVENT_DONE;
}

EventStatus SwitchOnDisabled::gotoEngage() {
  gotoDisengage();
  addEvent(&Cia402FsmBase::enableOperation, fsm_data_.timeout_sec.enable_operation);
  return EventStatus::EVENT_DONE;
}

EventStatus SwitchOnDisabled::shutdown() {
  return fsm_data_.transition->shutdown();
}

EventStatus SwitchOnDisabled::terminateEvent() {
  return fsm_data_.transition->terminateEvent();
}

//
ReadyToSwitchOn::ReadyToSwitchOn(Cia402FsmData& data) : Cia402FsmBase(data) {
}

void ReadyToSwitchOn::enter() {
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_OFF;
}

// switch on
EventStatus ReadyToSwitchOn::switchOn() {
  return fsm_data_.transition->switchOn();
}

// switch off
EventStatus ReadyToSwitchOn::quickStop() {
  return fsm_data_.transition->quickStop();
}

EventStatus ReadyToSwitchOn::disableVoltage() {
  return fsm_data_.transition->disableVoltage();
}

EventStatus ReadyToSwitchOn::terminateEvent() {
  return fsm_data_.transition->terminateEvent();
}

//
SwitchOn::SwitchOn(Cia402FsmData& data) : Cia402FsmBase(data) {
}

void SwitchOn::enter() {
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_DISENGAGED;
}

EventStatus SwitchOn::gotoOff() {
  addEvent(&Cia402FsmBase::shutdown, fsm_data_.timeout_sec.shutdown);
  addEvent(&Cia402FsmBase::disableVoltage, fsm_data_.timeout_sec.disable_voltage);
  return EventStatus::EVENT_DONE;
}

EventStatus SwitchOn::gotoEngage() {
  addEvent(&Cia402FsmBase::enableOperation, fsm_data_.timeout_sec.enable_operation);
  return EventStatus::EVENT_DONE;
}

// switch on
EventStatus SwitchOn::enableOperation() {
  return fsm_data_.transition->enableOperation();
}

// switch off
EventStatus SwitchOn::shutdown() {
  return fsm_data_.transition->shutdown();
}

EventStatus SwitchOn::quickStop() {
  return fsm_data_.transition->quickStop();
}

EventStatus SwitchOn::disableVoltage() {
  return fsm_data_.transition->disableVoltage();
}

EventStatus SwitchOn::terminateEvent() {
  return fsm_data_.transition->terminateEvent();
}

//
OperationEnable::OperationEnable(Cia402FsmData& data) : Cia402FsmBase(data) {
}

void OperationEnable::enter() {
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_ENGAGED;
}

void OperationEnable::iterate(double dt) {
  if (bitIsSet(fsm_data_.cia402_cmd, CIA402_CMD_ENABLE_OPERATION) && bitIsCleared(fsm_data_.cia402_status, Cia402_STATUS_OPERATION_ENABLED)) {
    LOG_ERROR("Drive suddenly left Operation Enable state");
    setActiveState<Fault>();
  }
}

EventStatus OperationEnable::gotoOff() {
  addEvent(&Cia402FsmBase::quickStop, fsm_data_.timeout_sec.quick_stop);
  addEvent(&Cia402FsmBase::disableVoltage, fsm_data_.timeout_sec.disable_voltage);
  return EventStatus::EVENT_DONE;
}

EventStatus OperationEnable::gotoDisengage() {
  addEvent(&Cia402FsmBase::disableOperation, fsm_data_.timeout_sec.disable_operation);
  return EventStatus::EVENT_DONE;
}

EventStatus OperationEnable::gotoQuickStop() {
  addEvent(&Cia402FsmBase::quickStop, fsm_data_.timeout_sec.quick_stop);
  return EventStatus::EVENT_DONE;
}

// quick stop
EventStatus OperationEnable::quickStop() {
  /*
   * Bit 2 is canceled (Quick Stop command)
   */
  fsm_data_.cia402_cmd = clearBit(fsm_data_.cia402_cmd, CIA402_CMD_QUICK_STOP);
  if (bitIsCleared(fsm_data_.cia402_status, Cia402_STATUS_QUICK_STOP_NOT_ACTIVE)) {
    setActiveState<QuickStopActive>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

// switch off
EventStatus OperationEnable::disableOperation() {
  /*
   * Bit 3 is canceled (Disable Operation command)
   */
  fsm_data_.cia402_cmd = clearBit(fsm_data_.cia402_cmd, CIA402_CMD_ENABLE_OPERATION);
  if (bitIsCleared(fsm_data_.cia402_status, Cia402_STATUS_OPERATION_ENABLED)) {
    setActiveState<SwitchOn>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

EventStatus OperationEnable::shutdown() {
  return fsm_data_.transition->shutdown();
}

EventStatus OperationEnable::disableVoltage() {
  return fsm_data_.transition->disableVoltage();
}

EventStatus OperationEnable::terminateEvent() {
  return fsm_data_.transition->terminateEvent();
}

//
QuickStopActive::QuickStopActive(Cia402FsmData& data) : Cia402FsmBase(data) {
}

void QuickStopActive::enter() {
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_QUICK_STOP_ACTIVE;
}

void QuickStopActive::iterate(double dt) {
  if (bitIsCleared(fsm_data_.cia402_cmd, DRIVE_CMD_QUICK_STOP) &&
      bitIsSet(fsm_data_.cia402_status, DRIVE_STATUS_QUICK_STOP_ACTIVE)) {
    LOG_ERROR("Drive suddenly left Quick Stop state");
    setActiveState<Fault>();
  }
}

EventStatus QuickStopActive::gotoOff() {
  addEvent(&Cia402FsmBase::disableVoltage, fsm_data_.timeout_sec.disable_voltage);
  return EventStatus::EVENT_DONE;
}

EventStatus QuickStopActive::gotoDisengage() {
  addEvent(&Cia402FsmBase::disableVoltage, fsm_data_.timeout_sec.disable_voltage);
  addEvent(&Cia402FsmBase::shutdown, fsm_data_.timeout_sec.shutdown);
  addEvent(&Cia402FsmBase::switchOn, fsm_data_.timeout_sec.switch_on);
  return EventStatus::EVENT_DONE;
}

EventStatus QuickStopActive::gotoEngage() {
  addEvent(&Cia402FsmBase::enableOperation, fsm_data_.timeout_sec.enable_operation);
  return EventStatus::EVENT_DONE;
}

// switch on
EventStatus QuickStopActive::enableOperation() {
  /*
 * Bit 2 is set (Enable operation)
 */
  fsm_data_.cia402_cmd = setBit(fsm_data_.cia402_cmd, CIA402_CMD_QUICK_STOP);
  if (bitIsSet(fsm_data_.cia402_status, Cia402_STATUS_QUICK_STOP_NOT_ACTIVE, Cia402_STATUS_OPERATION_ENABLED)) {
    setActiveState<OperationEnable>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

// switch off
EventStatus QuickStopActive::disableVoltage() {
  return fsm_data_.transition->disableVoltage();
}

EventStatus QuickStopActive::terminateEvent() {
  return fsm_data_.transition->terminateEvent();
}

//
Fault::Fault(Cia402FsmData& data) : Cia402FsmBase(data) {
}

void Fault::enter() {
  fsm_data_.drive_state = DriveState::DRIVE_STATUS_FAULT;
  fsm_data_.cia402_cmd = 0;
}

EventStatus Fault::faultAcknowledge() {
  setActiveState<FaultReset>();
  addEvent(&Cia402FsmBase::resetting, fsm_data_.timeout_sec.fault_ack);
  return EventStatus::EVENT_DONE;
}

//
FaultReset::FaultReset(Cia402FsmData& data) : Cia402FsmBase(data) {
}

EventStatus FaultReset::resetting() {
  return fsm_data_.transition->resetting();
}

EventStatus FaultReset::terminateEvent() {
  return fsm_data_.transition->terminateEvent();
}
