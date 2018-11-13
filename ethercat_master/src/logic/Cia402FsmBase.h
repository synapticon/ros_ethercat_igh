//
// Created by alexey on 5-11-18.
//

#ifndef CIA402_CIA402FSMBASE_H
#define CIA402_CIA402FSMBASE_H

#include <mcx/sm_statemachine.h>

struct Cia402FsmData;

class Cia402FsmBase : public mcx::state_machine::State<Cia402FsmBase> {
public:
  Cia402FsmBase(Cia402FsmData&);
  ~Cia402FsmBase() = default;

  void registerUserEvents();

  virtual mcx::state_machine::EventStatus shutdown();
  virtual mcx::state_machine::EventStatus switchOn();
  virtual mcx::state_machine::EventStatus disableVoltage();
  virtual mcx::state_machine::EventStatus quickStop();
  virtual mcx::state_machine::EventStatus disableOperation();
  virtual mcx::state_machine::EventStatus enableOperation();
  virtual mcx::state_machine::EventStatus resetting();

  virtual mcx::state_machine::EventStatus gotoOff();
  virtual mcx::state_machine::EventStatus gotoDisengage();
  virtual mcx::state_machine::EventStatus gotoEngage();
  virtual mcx::state_machine::EventStatus gotoQuickStop();
  virtual mcx::state_machine::EventStatus faultAcknowledge();

  virtual mcx::state_machine::EventStatus warning_(const mcx::state_machine::Error& error);
  virtual mcx::state_machine::EventStatus forcedDisengaged_(const mcx::state_machine::Error& error);
  virtual mcx::state_machine::EventStatus shutdown_(const mcx::state_machine::Error& error);
  virtual mcx::state_machine::EventStatus emergencyStop_(const mcx::state_machine::Error& error);

protected:
  Cia402FsmData& fsm_data_;
};


#endif //CIA402_CIA402FSMBASE_H
