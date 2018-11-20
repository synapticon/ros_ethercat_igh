#!/usr/bin/env python

import rospy
from motorcortex_msgs.srv import *

def read_cfg_client(request_drives):
    rospy.wait_for_service('get_sdo_config')
    try:
        read_cfg = rospy.ServiceProxy('get_sdo_config', GetSDOCfg)
        cfgs = read_cfg(request_drives)
        return cfgs.torque_controller_cfg, cfgs.velocity_controller_cfg, cfgs.position_controller_cfg
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    request_drives = [True, True]
    torques, velocities, positions = read_cfg_client(request_drives)
    for torque, velocity, position in zip(torques, velocities, positions):
        print ("********************************************")
        print (torque)
        print ("--------------------------------------------")
        print (velocity)
        print("--------------------------------------------")
        print (position)
        print("********************************************")