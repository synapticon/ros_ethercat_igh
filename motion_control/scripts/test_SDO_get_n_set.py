#!/usr/bin/env python

import rospy
from motorcortex_msgs.srv import *

def read_cfg_client(request_drives):
    rospy.wait_for_service('get_sdo_config')
    try:
        read_cfg = rospy.ServiceProxy('get_sdo_config', GetSDOCfg)
        cfgs = read_cfg(request_drives)
        return cfgs.torque_controller_cfg, cfgs.velocity_controller_cfg, cfgs.position_controller_cfg
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


def write_cfg_client(torqueControllerCfgs, velocityControllerCfgs, positionControllerCfgs):
    rospy.wait_for_service('set_sdo_config')
    try:
        write_cfg = rospy.ServiceProxy('set_sdo_config', SetSDOCfg)
        successes = write_cfg(torqueControllerCfgs, velocityControllerCfgs, positionControllerCfgs)
        return successes
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e


if __name__ == "__main__":

    rospy.init_node('SDO_updater', anonymous=True)
    rate = rospy.Rate(2)  # 2hz

    while not rospy.is_shutdown():

        torqueControllerCfgs = []
        velocityControllerCfgs = []
        positionControllerCfgs = []

        request_drives = [True, True]
        torques, velocities, positions = read_cfg_client(request_drives)
        for torque, velocity, position in zip(torques, velocities, positions):

            print ("******************* Torque *********************")
            print (torque)
            print ("------------------ Velocity --------------------")
            print (velocity)
            print("------------------- Position --------------------")
            print (position)
            print("********************************************")

            #change some parameters for a test purpose
            velocity.controller_Kp = velocity.controller_Kp + 0.000001

            torqueControllerCfgs.append(torque)
            velocityControllerCfgs.append(velocity)
            positionControllerCfgs.append(position)

        successes = write_cfg_client(torqueControllerCfgs, velocityControllerCfgs, positionControllerCfgs)

        print(successes)

        rate.sleep()