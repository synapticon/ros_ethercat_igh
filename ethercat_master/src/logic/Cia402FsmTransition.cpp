//
// Created by alexey on 5-11-18.
//

#include "Cia402FsmTransition.h"
#include "Cia402FsmData.h"
#include "Cia402FsmImpl.h"

using namespace mcx::state_machine;

Transition::Transition(Cia402FsmData& data) : Cia402FsmBase(data) {

}

EventStatus Transition::shutdown() {
  /*
   * Bit 0 is canceled (Shutdown command)
   * Bit 1 Disable Voltage and Bit 2
   * Quick Stop are set in the control word (Shutdown command).
   * DC-link voltage may be present.
   */
  fsm_data_.cia402_cmd = clearBit(fsm_data_.cia402_cmd, CIA402_CMD_SWITCH_ON);
  fsm_data_.cia402_cmd = setBit(fsm_data_.cia402_cmd, CIA402_CMD_DISABLE_VOLTAGE, CIA402_CMD_QUICK_STOP);
  if (bitIsSet(fsm_data_.cia402_status, Cia402_STATUS_READY_TO_SWITCH_ON)) {
    setActiveState<ReadyToSwitchOn>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

EventStatus Transition::switchOn() {
  /*
 * Bit 0 is also set (Switch On command)
 */
  fsm_data_.cia402_cmd = setBit(fsm_data_.cia402_cmd, CIA402_CMD_SWITCH_ON);
  if (bitIsSet(fsm_data_.cia402_status, Cia402_STATUS_SWITCHED_ON)) {
    setActiveState<SwitchOn>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

mcx::state_machine::EventStatus Transition::enableOperation() {
  /*
   * Bit 3 is also set (Enable Operation command)
   */
  fsm_data_.cia402_cmd = setBit(fsm_data_.cia402_cmd, CIA402_CMD_ENABLE_OPERATION);
  if (bitIsSet(fsm_data_.cia402_status, Cia402_STATUS_OPERATION_ENABLED)) {
    setActiveState<OperationEnable>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

EventStatus Transition::quickStop() {
  /*
 * Bits ½ are canceled (Quick Stop / Disable Voltage command)
 */
  fsm_data_.cia402_cmd = clearBit(fsm_data_.cia402_cmd, CIA402_CMD_QUICK_STOP);
  if (bitIsSet(fsm_data_.cia402_status, Cia402_STATUS_SWITCH_ON_DISABLED)) {
    setActiveState<SwitchOnDisabled>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

EventStatus Transition::disableVoltage() {
  /*
 * Bits ½ are canceled (Quick Stop / Disable Voltage command
 */
  fsm_data_.cia402_cmd = clearBit(fsm_data_.cia402_cmd, CIA402_CMD_DISABLE_VOLTAGE);
  if (bitIsSet(fsm_data_.cia402_status, Cia402_STATUS_SWITCH_ON_DISABLED)) {
    setActiveState<SwitchOnDisabled>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

mcx::state_machine::EventStatus Transition::resetting() {
  /*
   * Reset errors
   */
  fsm_data_.cia402_cmd = setBit(fsm_data_.cia402_cmd, CIA402_CMD_FAULT_REST);
  if (bitIsCleared(fsm_data_.cia402_status, Cia402_STATUS_FAULT) &&
      bitIsSet(fsm_data_.cia402_status, Cia402_STATUS_SWITCH_ON_DISABLED)) {
    setActiveState<SwitchOnDisabled>();
    return EventStatus::EVENT_DONE;
  }
  return EventStatus::EVENT_REPEAT;
}

mcx::state_machine::EventStatus Transition::terminateEvent() {
  /*
   * Activate Fault
   */
  setActiveState<Fault>();
  return EventStatus::EVENT_DONE;
}
