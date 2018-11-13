//
// Created by alexey on 5-11-18.
//

#include "Cia402FsmBase.h"

using namespace mcx::state_machine;

Cia402FsmBase::Cia402FsmBase(Cia402FsmData& data) : fsm_data_(data) {

}

void Cia402FsmBase::registerUserEvents() {
  addEventName(&Cia402FsmBase::shutdown, "Shutdown");
  addEventName(&Cia402FsmBase::switchOn, "Switch On");
  addEventName(&Cia402FsmBase::disableVoltage, "Disable Voltage");
  addEventName(&Cia402FsmBase::quickStop, "Quick Stop");
  addEventName(&Cia402FsmBase::disableOperation, "Disable Operation");
  addEventName(&Cia402FsmBase::enableOperation, "Enable Operation");
  addEventName(&Cia402FsmBase::resetting, "Fault Reset");

  addEventName(&Cia402FsmBase::gotoOff, "Goto Off");
  addEventName(&Cia402FsmBase::gotoDisengage, "Goto Disengage");
  addEventName(&Cia402FsmBase::gotoEngage, "Goto Engage");
  addEventName(&Cia402FsmBase::gotoQuickStop, "Goto Quick Stop");

  addEventName(&Cia402FsmBase::faultAcknowledge, "Fault Acknowledge");
}

EventStatus Cia402FsmBase::shutdown() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::switchOn() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::disableVoltage() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::quickStop() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::disableOperation() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::enableOperation() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::resetting() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::gotoOff() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::gotoDisengage() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::gotoEngage() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::gotoQuickStop() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::faultAcknowledge() {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::warning_(const Error& error) {
 return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::forcedDisengaged_(const Error& error) {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::shutdown_(const Error& error) {
  return EventStatus::EVENT_NONE;
}

EventStatus Cia402FsmBase::emergencyStop_(const Error& error) {
  return EventStatus::EVENT_NONE;
}