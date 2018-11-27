#!/usr/bin/env python

import rospy
from motorcortex_msgs.srv import *

def reset_cfg_client(restore_on_slaves):
    rospy.wait_for_service('restore_default_sdo_config')
    try:
        restore_cfg = rospy.ServiceProxy('restore_default_sdo_config', RestoreCfgParams)
        successes = restore_cfg(restore_on_slaves)
        return successes
    except rospy.ServiceException as e:
        print "Store service call failed: %s"%e


if __name__ == "__main__":
    restore = [True, True]

    successes = reset_cfg_client(restore)

    print (successes)
