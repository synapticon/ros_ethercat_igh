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

#include "motorcortex_msgs/DriveIn.h"
#include "motorcortex_msgs/DriveOut.h"
#include "motorcortex_msgs/TorqueControllerCfg.h"
#include "motorcortex_msgs/VelocityControllerCfg.h"
#include "motorcortex_msgs/PositionControllerCfg.h"
#include <tuple>


class Drive : public mcx::container::Module {
    using Cia402StateMachine = mcx::state_machine::StateMachine<Cia402FsmBase>;

public:

    Drive();

    ~Drive() override = default;

    struct SDOCfg {
        motorcortex_msgs::TorqueControllerCfg torqueControllerCfg;
        motorcortex_msgs::VelocityControllerCfg velocityControllerCfg;
        motorcortex_msgs::PositionControllerCfg positionControllerCfg;
    };

    const motorcortex_msgs::DriveIn &getDriveFeedback() const {
        return driveFeedback_;
    }

    void setDriveCommand(const motorcortex_msgs::DriveOut& driveCommand) {
        driveCommand_ = driveCommand;
    }

    void setSDOCfg(const SDOCfg& sdoCfg) {
        sdoCfg_ = sdoCfg;
    }

    const SDOCfg &getSDOCfg() const {
        return sdoCfg_;
    }

    void requestSDOUpdate(bool req) {
        request_update_ = req;
    }

private:
    void create_(const char *name, mcx::parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) override;

    bool initPhase1_() override;

    bool initPhase2_() override;

    bool startOp_() override;

    bool stopOp_() override;

    bool iterateOp_(const mcx::container::TaskTime &system_time, mcx::container::UserTime *user_time) override;

    motorcortex_msgs::DriveIn driveFeedback_;

    motorcortex_msgs::DriveOut driveCommand_;
    decltype(driveCommand_.controlword) oldControlWord{};
    decltype(driveCommand_.opmode) opmode{};

    SDOCfg sdoCfg_;

    bool upload_{};
    bool request_update_{};
    Cia402StateMachine sm_;
    Cia402FsmData sm_data_{};
    Transition sm_transition_;
    mcx::parameter_server::ParamHandle drive_cmd_handle_;

};

#endif /* DRIVE_H */
