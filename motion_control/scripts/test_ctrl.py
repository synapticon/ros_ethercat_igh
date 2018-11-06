#!/usr/bin/env python

from __future__ import print_function, division

import rospy
from drive import Drive

from motion_control.msg import MotorcortexOutList, MotorcortexInList
from time import sleep


drives = [Drive(0), Drive(1)]
def feedback_callback(motorcortex_feedback):
    for drive in drives:
        drive.update(motorcortex_feedback.drives_feedback[drive.getId()])

pub = rospy.Publisher('/motorcortex_control', MotorcortexOutList, queue_size=1)
rospy.Subscriber("/motorcortex_feedback", MotorcortexInList, feedback_callback)


def controller():

    rospy.init_node('motorcortex_control', anonymous=True)
    r = rospy.Rate(100)#Hz
    pos = 0;


    positionIncrement = 1000

    # switch on here
    while not rospy.is_shutdown():
        # receive here
        controlMsg = MotorcortexOutList()
        for drive in drives:
            if drive.hasError():
                print("drive %d has error"%drive.getId())
                drive.resetError()
            else:
                drive.switchOn()

            if drive.isEnabled():
                drive.setPosition(drive.getPosition() + positionIncrement)
            else:
                drive.setPosition(drive.getPosition())

            controlMsg.drive_command.append(drive.encode())

        pub.publish(controlMsg)
        # send here
        r.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException: pass