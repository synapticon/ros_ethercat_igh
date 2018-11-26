/*
* Add Comment here
*/

#ifndef DRIVE_H
#define DRIVE_H

#include <mcx/mcx_core.h>
#include "DriveSdo.h"
#include "motorcortex_msgs/DriveIn.h"
#include "motorcortex_msgs/DriveOut.h"


class Drive : public mcx::container::Module {
public:

    Drive() = default;
    ~Drive() override = default;


    const motorcortex_msgs::DriveIn &getDriveFeedback() const {
        return driveFeedback_;
    }

    int getOpMode() const {
        return driveCommand_.opmode;
    }

    int getControlWord() const {
        return driveCommand_.controlword;
    }

    void setStatusWord(int statusword) {
        driveFeedback_.statusword = statusword;
    }

    void setDriveCommand(const motorcortex_msgs::DriveOut& driveCommand) {
        driveCommand_ = driveCommand;
    }

    void setSDOCfg(const DriveSdo::SDOCfg& sdoCfg) {
        drive_sdo_write_.setSDOCfg(sdoCfg);
    }

    const DriveSdo::SDOCfg &getSDOCfg() const {
        return drive_sdo_read_.getSDOCfg();
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

    bool request_update_{};

    DriveSdo drive_sdo_read_;
    DriveSdo drive_sdo_write_;

};

#endif /* DRIVE_H */
