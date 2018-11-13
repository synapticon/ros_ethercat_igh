/*
* Add Comment here
*/

#ifndef DRIVE_H
#define DRIVE_H

#include <mcx/mcx_core.h>

#include "Cia402FsmBase.h"
#include "Cia402FsmData.h"
#include "Cia402FsmTransition.h"
#include "DriveDef.h"

#include "motion_control/MotorcortexIn.h"
#include "motion_control/MotorcortexOut.h"


class Drive : public mcx::container::Module {
    using Cia402StateMachine = mcx::state_machine::StateMachine<Cia402FsmBase>;
public:

    Drive();

    ~Drive() override = default;

    const motion_control::MotorcortexIn &getDriveFeedback() const {
        return driveFeedback_;
    }

    void setDriveCommand(const motion_control::MotorcortexOut& driveCommand) {
        driveCommand_ = driveCommand;
    }

private:
    void create_(const char *name, mcx::parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) override;

    bool initPhase1_() override;

    bool initPhase2_() override;

    bool startOp_() override;

    bool stopOp_() override;

    bool iterateOp_(const mcx::container::TaskTime &system_time, mcx::container::UserTime *user_time) override;

    motion_control::MotorcortexIn driveFeedback_;

    motion_control::MotorcortexOut driveCommand_;
    decltype(driveCommand_.controlword) oldControlWord{};
    decltype(driveCommand_.opmode) opmode{};


    Cia402StateMachine sm_;
    Cia402FsmData sm_data_{};
    Transition sm_transition_;
    mcx::parameter_server::ParamHandle drive_cmd_handle_;

};

#endif /* DRIVE_H */
