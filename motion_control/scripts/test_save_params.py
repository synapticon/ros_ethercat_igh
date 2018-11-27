#!/usr/bin/env python

import rospy
from motorcortex_msgs.srv import *

def save_cfg_client(save_on_slaves):
    rospy.wait_for_service('save_sdo_config')
    try:
        save_cfg = rospy.ServiceProxy('save_sdo_config', SaveCfgParams)
        successes = save_cfg(save_on_slaves)
        return successes
    except rospy.ServiceException as e:
        print "Store service call failed: %s"%e


if __name__ == "__main__":

    save = [True, True]

    successes = save_cfg_client(save)

    print (successes)
