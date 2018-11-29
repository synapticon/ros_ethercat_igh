
/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Vectioneer B.V.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Vectioneer nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Alexey Zakharov
* Author: Viatcheslav Tretyakov
*********************************************************************/

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

    void saveAllCfg(const uint32_t& save_cfg) {
        drive_sdo_write_.saveAllCfg(save_cfg);
    }

    void restoreAllCfg(const uint32_t& restore_cfg) {
        drive_sdo_write_.restoreAllCfg(restore_cfg);
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
