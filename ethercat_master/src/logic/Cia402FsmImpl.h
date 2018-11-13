//
// Created by alexey on 5-11-18.
//

#ifndef CIA402_CIA402FSMIMPL_H
#define CIA402_CIA402FSMIMPL_H

#include "Cia402FsmBase.h"
#include "Cia402FsmTransition.h"

struct Cia402FsmData;

/*
 * Init State
 */
class Init : public Cia402FsmBase {
public:
  explicit Init(Cia402FsmData& data);

  void enter() override;

  // user input commands
  mcx::state_machine::EventStatus faultAcknowledge() override;
};

/*
 * Drive not ready to switch on, there is no operational readiness
 * (BTB/RTO) signaled from the controller program.
 */
class NotReadyToSwitchOn : public Cia402FsmBase {
public:
  explicit NotReadyToSwitchOn(Cia402FsmData& data);

  void enter() override;

  mcx::state_machine::EventStatus resetting() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

/*
 * Drive is ready to switch on, parameters can be transferred,
 * the DC-link voltage can be switched on, motion functions cannot
 * be carried out yet.
 */
class SwitchOnDisabled : public Cia402FsmBase {
public:
  explicit SwitchOnDisabled(Cia402FsmData& data);

  void enter() override;

  // user input commands
  mcx::state_machine::EventStatus gotoDisengage() override;

  mcx::state_machine::EventStatus gotoEngage() override;

  // internal commands
  // switch on
  mcx::state_machine::EventStatus shutdown() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

/*
 * DC-link voltage may be switched on, parameters can be transferred,
 * motion functions cannot be carried out yet.
 */
class ReadyToSwitchOn : public Cia402FsmBase {
public:
  explicit ReadyToSwitchOn(Cia402FsmData& data);

  void enter() override;

  // internal commands (transition state)
  // switch on
  mcx::state_machine::EventStatus switchOn() override;

  // switch off
  mcx::state_machine::EventStatus quickStop() override;

  mcx::state_machine::EventStatus disableVoltage() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

/*
 * DC-link voltage must be switched on, parameters can be transferred, motion
 * functions cannot be carried out yet, output stage is switched on (enabled).
 */
class SwitchOn : public Cia402FsmBase {
public:
  explicit SwitchOn(Cia402FsmData& data);

  void enter() override;

  // user input commands
  mcx::state_machine::EventStatus gotoOff() override;

  mcx::state_machine::EventStatus gotoEngage() override;

  // internal commands
  // switch on
  mcx::state_machine::EventStatus enableOperation() override;

  // switch off
  mcx::state_machine::EventStatus shutdown() override;

  mcx::state_machine::EventStatus quickStop() override;

  mcx::state_machine::EventStatus disableVoltage() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

/*
 * No fault present, output stage is enabled, motion functions are enabled.
 */
class OperationEnable : public Cia402FsmBase {
public:
  explicit OperationEnable(Cia402FsmData& data);

  void enter() override;

  void iterate(double dt) override;

  // user input commands
  mcx::state_machine::EventStatus gotoOff() override;

  mcx::state_machine::EventStatus gotoDisengage() override;

  mcx::state_machine::EventStatus gotoQuickStop() override;

  // user input commands
  // quick stop
  mcx::state_machine::EventStatus quickStop() override;

  // switch off
  mcx::state_machine::EventStatus disableOperation() override;

  mcx::state_machine::EventStatus shutdown() override;

  mcx::state_machine::EventStatus disableVoltage() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

/*
 * Drive has been stopped with the emergency ramp, output stage is enabled, motion
 * functions are enabled, response depends on Object 605Ah ( 6.3)
 */
class QuickStopActive : public Cia402FsmBase {
public:
  explicit QuickStopActive(Cia402FsmData& data);

  void enter() override;

  void iterate(double dt) override;

  // user input commands
  mcx::state_machine::EventStatus gotoOff() override;

  mcx::state_machine::EventStatus gotoDisengage() override;

  mcx::state_machine::EventStatus gotoEngage() override;

  // user input commands
  // switch on
  mcx::state_machine::EventStatus enableOperation() override;

  // switch off
  mcx::state_machine::EventStatus disableVoltage() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

/*
 * Drive error is active, DC-Voltage is disabled
 */
class Fault : public Cia402FsmBase {
public:
  explicit Fault(Cia402FsmData& data);

  void enter() override;

  mcx::state_machine::EventStatus faultAcknowledge() override;
};


/*
 * Resetting drive error
 */
class FaultReset : public Cia402FsmBase {
public:
  explicit FaultReset(Cia402FsmData& data);

  mcx::state_machine::EventStatus resetting() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

#endif //CIA402_CIA402FSMIMPL_H
