#!/usr/bin/env python

from __future__ import print_function, division

import rospy
from drive import Drive, OpModesCiA402
from dio import DIO

from motorcortex_msgs.msg import MotorcortexOutList, MotorcortexInList, DigitalInputsList, DigitalOutputsList

# list your slave devices here
drives = [Drive(0), Drive(1)]
dios = [DIO(0)]

def drives_feedback_callback(motorcortex_feedback):
    for drive in drives:
        drive.update(motorcortex_feedback.drives_feedback[drive.getId()])

def digital_inputs_callback(digital_inputs):
    for dio in dios:
        dio.update(digital_inputs.devices_feedback[dio.getId()])

drives_pub = rospy.Publisher('/motorcortex_control', MotorcortexOutList, queue_size = 1)
dios_pub = rospy.Publisher('/digital_outputs', DigitalOutputsList, queue_size = 1)
rospy.Subscriber("/motorcortex_feedback", MotorcortexInList, drives_feedback_callback)
rospy.Subscriber("/digital_inputs", DigitalInputsList, digital_inputs_callback)


def safe_off():
    drivesControlMsg = MotorcortexOutList()
    for drive in drives:
        drive.switchOff()
        drivesControlMsg.drive_command.append(drive.encode())

    digitalOutputsControlMsg = DigitalOutputsList()
    for dio in dios:
        digital_outputs = [False] * 12
        dio.setDigitalOutputs(digital_outputs)
        digitalOutputsControlMsg.devices_command.append(dio.encode())

    drives_pub.publish(drivesControlMsg)
    dios_pub.publish(digitalOutputsControlMsg)

def controller():

    rospy.init_node('motorcortex_control', anonymous=True)
    rospy.on_shutdown(safe_off)
    r = rospy.Rate(100)#Hz

    # set your test parameters here
    opMode = OpModesCiA402.CSV.value
    positionIncrement = 1000
    referenceVelocity = 500
    referenceTorque = 50

    counter = 0

    # switch on here
    while not rospy.is_shutdown():
        # receive here
        drivesControlMsg = MotorcortexOutList()
        for drive in drives:
            if drive.hasError():
                print("drive %d has error"%drive.getId())
                drive.resetError()
            else:
                drive.setMode(opMode)
                drive.switchOn()

            if drive.isEnabled():
                if opMode == OpModesCiA402.CSP.value:
                    drive.setPosition(drive.getPosition() + positionIncrement)
                elif opMode == OpModesCiA402.CSV.value:
                    drive.setVelocity(referenceVelocity)
                elif opMode == OpModesCiA402.CST.value:
                    drive.setTorque(referenceTorque)
            else:
                drive.setPosition(drive.getPosition())
                drive.setVelocity(0)
                drive.setTorque(0)

            drivesControlMsg.drive_command.append(drive.encode())

        digitalOutputsControlMsg = DigitalOutputsList()
        for dio in dios:
            # blink
            if counter < 100:
                output_state = True
            else:
                output_state = False

            digital_outputs = [output_state] * 12
            dio.setDigitalOutputs(digital_outputs)
            digitalOutputsControlMsg.devices_command.append(dio.encode())

        # send here
        drives_pub.publish(drivesControlMsg)
        dios_pub.publish(digitalOutputsControlMsg)

        counter += 1
        if counter > 200:
            counter = 0

        r.sleep()



if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException: pass