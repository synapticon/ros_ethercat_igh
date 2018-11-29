
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

#ifndef DIGITALIO_H
#define DIGITALIO_H

#include <mcx/mcx_core.h>
#include "motorcortex_msgs/DigitalInputs.h"
#include "motorcortex_msgs/DigitalOutputs.h"

class DigitalIO : public mcx::container::Module {
public:

    DigitalIO() = default;

    ~DigitalIO() override = default;

    const motorcortex_msgs::DigitalInputs &getDIOFeedback() const {
        return digitalInputs_;
    }

    void setDigitalOutputs(const motorcortex_msgs::DigitalOutputs& digitalOutputsCommand) {
        digitalOutputs_ = digitalOutputsCommand;
    }

private:
    void create_(const char *name, mcx::parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) override;

    bool initPhase1_() override;

    bool initPhase2_() override;

    bool startOp_() override;

    bool stopOp_() override;

    bool iterateOp_(const mcx::container::TaskTime &system_time, mcx::container::UserTime *user_time) override;

    motorcortex_msgs::DigitalInputs digitalInputs_;
    motorcortex_msgs::DigitalOutputs digitalOutputs_;
};

#endif /* DIGITALIO_H */
