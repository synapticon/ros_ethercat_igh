/*
* Add Comment here
*/

#ifndef DRIVE_H
#define DRIVE_H

#include <mcx/mcx_core.h>
#include "motion_control/MotorcortexIn.h"


class Drive : public mcx::container::Module {
public:

  Drive() = default;

  ~Drive() override = default;

  const motion_control::MotorcortexIn& getDriveFeedback() const {
      return driveFeedback_;
  }

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameter_server, uint64_t dt_micro_s) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& system_time, mcx::container::UserTime* user_time) override;

private:
    motion_control::MotorcortexIn driveFeedback_;

};

#endif /* DRIVE_H */
