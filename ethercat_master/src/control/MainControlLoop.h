
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

#ifndef MAINCONTROLLOOP_H
#define MAINCONTROLLOOP_H

#include <mcx/mcx_core.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Drive.h"
#include "DigitalIO.h"
#include "motorcortex_msgs/DriveInList.h"
#include "motorcortex_msgs/DriveOutList.h"
#include "motorcortex_msgs/DigitalInputsList.h"
#include "motorcortex_msgs/DigitalOutputsList.h"
#include "motorcortex_msgs/GetSDOCfg.h"
#include "motorcortex_msgs/SetSDOCfg.h"
#include "motorcortex_msgs/SaveCfgParams.h"
#include "motorcortex_msgs/RestoreCfgParams.h"

class MainControlLoop : public mcx::container::Module {

public:

    MainControlLoop(unsigned int number_of_drives, unsigned int number_of_ios);

    ~MainControlLoop() override;

    void drivesControlCallback(const motorcortex_msgs::DriveOutList::ConstPtr &drives_command_msg);

    void diosControlCallback(const motorcortex_msgs::DigitalOutputsList::ConstPtr &dios_command_msg);

    bool getSDOSrv(motorcortex_msgs::GetSDOCfg::Request &req, motorcortex_msgs::GetSDOCfg::Response &res);

    bool setSDOSrv(motorcortex_msgs::SetSDOCfg::Request &req, motorcortex_msgs::SetSDOCfg::Response &res);

    bool saveCfgParamsSrv(motorcortex_msgs::SaveCfgParams::Request &req, motorcortex_msgs::SaveCfgParams::Response &res);

    bool restoreCfgParamsSrv(motorcortex_msgs::RestoreCfgParams::Request &req, motorcortex_msgs::RestoreCfgParams::Response &res);

private:
    void create_(const char *name, mcx::parameter_server::Parameter *parameter_server, uint64_t dt_micro_s) override;

    bool initPhase1_() override;

    bool initPhase2_() override;

    bool startOp_() override;

    bool stopOp_() override;

    bool iterateOp_(const mcx::container::TaskTime &system_time, mcx::container::UserTime *user_time) override;

    ros::NodeHandle nh_;
    ros::Subscriber sub_drives_;
    ros::Subscriber sub_dios_;
    ros::Publisher drive_feedback_pub_;
    ros::Publisher digital_inputs_pub_;
    ros::ServiceServer service_get_sdo_;
    ros::ServiceServer service_set_sdo_;
    ros::ServiceServer service_save_cfg_;
    ros::ServiceServer service_restore_cfg_;


    size_t number_of_drives_;
    size_t number_of_ios_;
    int* opmode_;
    int* controlword_;
    int* statusword_;
    Drive* drives_;
    DigitalIO* dio_devices_;
    bool read_sdo_;
    double read_sdo_time_max_sec_{0.5};
    double read_sdo_time_sec_{};


};

#endif /* MAINCONTROLLOOP_H */