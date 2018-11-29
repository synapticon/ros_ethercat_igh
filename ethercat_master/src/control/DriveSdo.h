
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

#ifndef DRIVESDO_H
#define DRIVESDO_H

#include <mcx/mcx_core.h>
#include "motorcortex_msgs/TorqueControllerCfg.h"
#include "motorcortex_msgs/VelocityControllerCfg.h"
#include "motorcortex_msgs/PositionControllerCfg.h"

class DriveSdo : public mcx::container::Module {
public:

    DriveSdo() = default;

    ~DriveSdo() override = default;

    struct SDOCfg {
        motorcortex_msgs::TorqueControllerCfg torqueControllerCfg;
        motorcortex_msgs::VelocityControllerCfg velocityControllerCfg;
        motorcortex_msgs::PositionControllerCfg positionControllerCfg;
    };

    void setSDOCfg(const SDOCfg& sdoCfg) {
        sdoCfg_ = sdoCfg;
        for (auto& handle : handles_) {
            handle.updateOutputOnce();
        }
        //LOG_INFO("kp: %f", sdoCfg_.velocityControllerCfg.controller_Kp);
        //LOG_INFO("All params updated");
    }

    const SDOCfg &getSDOCfg() const {
        return sdoCfg_;
    }

    void saveAllCfg(const uint32_t& save_cfg) {
        save_cfg_ = save_cfg;
        save_params_handler_.updateOutputOnce();
    }

    void restoreAllCfg(const uint32_t& restore_cfg) {
        restore_cfg_ = restore_cfg;
        restore_params_handler_.updateOutputOnce();
    }

private:
    void create_(const char *name, mcx::parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) override;

    bool initPhase1_() override;

    bool initPhase2_() override;

    bool startOp_() override;

    bool stopOp_() override;

    bool iterateOp_(const mcx::container::TaskTime &system_time, mcx::container::UserTime *user_time) override;

    SDOCfg sdoCfg_;
    std::vector<mcx::parameter_server::ParamHandle> handles_;
    mcx::parameter_server::ParamHandle save_params_handler_;
    mcx::parameter_server::ParamHandle restore_params_handler_;
    uint32_t save_cfg_{};
    uint32_t restore_cfg_{};

};

#endif /* DRIVESDO_H */
