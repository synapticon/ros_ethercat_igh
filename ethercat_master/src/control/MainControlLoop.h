/*
* Add Comment here
*/

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

class MainControlLoop : public mcx::container::Module {
public:

    MainControlLoop() = default;

    ~MainControlLoop() override = default;

    void drivesControlCallback(const motorcortex_msgs::DriveOutList::ConstPtr &drives_command_msg);

    void diosControlCallback(const motorcortex_msgs::DigitalOutputsList::ConstPtr &dios_command_msg);

    bool getSDOSrv(motorcortex_msgs::GetSDOCfg::Request &req, motorcortex_msgs::GetSDOCfg::Response &res);

    bool setSDOSrv(motorcortex_msgs::SetSDOCfg::Request &req, motorcortex_msgs::SetSDOCfg::Response &res);

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
    std::array<Drive, 2> drives_;
    std::array<DigitalIO, 1> dio_devices_;

};

#endif /* MAINCONTROLLOOP_H */