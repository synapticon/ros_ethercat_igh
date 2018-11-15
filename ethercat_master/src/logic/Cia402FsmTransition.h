//
// Created by alexey on 5-11-18.
//

#ifndef CIA402_CIA402FSMTRANSITION_H
#define CIA402_CIA402FSMTRANSITION_H

#include "Cia402FsmBase.h"

// common implementations which appear more than once in the code
class Transition : public Cia402FsmBase {
public:
  explicit Transition(Cia402FsmData& data);
  mcx::state_machine::EventStatus shutdown() override;
  mcx::state_machine::EventStatus switchOn() override;
  mcx::state_machine::EventStatus enableOperation() override;
  mcx::state_machine::EventStatus quickStop() override;
  mcx::state_machine::EventStatus disableVoltage() override;
  mcx::state_machine::EventStatus resetting() override;
  mcx::state_machine::EventStatus terminateEvent() override;
};

#endif //CIA402_CIA402FSMTRANSITION_H
