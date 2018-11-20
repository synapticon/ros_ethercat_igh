#!/usr/bin/env python

import rospy
from motorcortex_msgs.srv import *
from motorcortex_msgs.msg import TorqueControllerCfg, VelocityControllerCfg, PositionControllerCfg

def write_cfg_client(torqueControllerCfgs, velocityControllerCfgs, positionControllerCfgs):
    rospy.wait_for_service('set_sdo_config')
    try:
        write_cfg = rospy.ServiceProxy('set_sdo_config', SetSDOCfg)
        successes = write_cfg(torqueControllerCfgs, velocityControllerCfgs, positionControllerCfgs)
        return successes
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    torqueControllerCfgs = [TorqueControllerCfg(), TorqueControllerCfg()]
    velocityControllerCfgs = [VelocityControllerCfg(), VelocityControllerCfg()]
    positionControllerCfgs = [PositionControllerCfg(), PositionControllerCfg()]

    successes = write_cfg_client(torqueControllerCfgs, velocityControllerCfgs, positionControllerCfgs)

    print (successes)
