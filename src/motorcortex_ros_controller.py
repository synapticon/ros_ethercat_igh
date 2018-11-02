#!/usr/bin/env python

from __future__ import print_function, division

import rospy

from motorcortex_ros.msg import MotorcortexIn, MotorcortexInList, MotorcortexOut, MotorcortexOutList
import motorcortex
import time

SERVER = 'localhost'


class Watchdog(object):
    start_time = time.time()
    def update(self):
        self.start_time = time.time()

    def check(self, timeout_sec):
        if (time.time() - self.start_time) < timeout_sec:
            return True
        return False

class Control(object):

    def __init__(self):

        # watchdog
        self.watchdog = Watchdog()

        # set loop rate
        rate = rospy.Rate(1000)  # Hz
        rospy.Subscriber("/motorcortex_control", MotorcortexOutList, self.ctrl_callback)
        feedback_pub = rospy.Publisher('/motorcortex_feedback', MotorcortexInList,
                                       queue_size=1)

        # Loading protobuf types and hashes
        motorcortex_types = motorcortex.MessageTypes()
        self.motorcortex_msg, = motorcortex_types.load(
            [{'proto': './motorcortex_api/motorcortex-msg/motorcortex_pb2.py', 'hash': './motorcortex_api/motorcortex-msg/motorcortex_hash.json'}])
        motorcortex_msg = self.motorcortex_msg

        # Open request connection
        parameter_tree = motorcortex.ParameterTree()
        self.request = motorcortex.Request(motorcortex_types, parameter_tree)
        req = self.request
        if req.connect("ws://%s:5558" % SERVER):
            print("Request connection is etablished")
        else:
            print("Failed to establish Request connection")

        # Login
        reply = req.login("operator", "operat")
        result = reply.get()
        if result.status == motorcortex_msg.OK:
            print("Login successfull")
        else:
            print("Failed to login")

        # Request a parameter tree from the server
        reply = req.getParameterTree()
        result = reply.get()
        parameter_tree.load(result)
        if result.status == motorcortex_msg.OK:
            print("Got parameter tree")
        else:
            print("Failed to get parameter tree")

        # Open subscription
        sub = motorcortex.Subscribe(req, motorcortex_types)
        if sub.connect("ws://%s:5557" % SERVER):
            print("Subscribe connection is etablished")
        else:
            print("Failed to establish Subscribe connection")

        subscription = sub.subscribe(['root/EtherCAT/Domain1/axis1/In/Statusword',
                                      'root/EtherCAT/Domain1/axis1/In/Position Value',
                                      'root/EtherCAT/Domain1/axis2/In/Statusword',
                                      'root/EtherCAT/Domain1/axis2/In/Position Value',
                                      'root/cia402/driveEnabled',
                                      'root/cia402/driveErrorCode'], 'group1')

        self.main(feedback_pub, rate, subscription)

        req.close()
        sub.close()

    def ctrl_callback(self, motorcortex_control):

        self.watchdog.update()

        drive1 = motorcortex_control.drive_command[0]
        drive2 = motorcortex_control.drive_command[1]

        #control word and opmode are commanded to cia402 to a single input
        replyHandle = self.request.setParameterList([{'path': 'root/cia402/actualState', 'value': drive1.controlword},
                                       {'path': 'root/cia402/actualMode', 'value': drive1.opmode},
                                       {'path': 'root/EtherCAT/Domain1/axis1/Out/Target Position',
                                        'value': drive1.target_position},
                                       {'path': 'root/EtherCAT/Domain1/axis2/Out/Target Position',
                                        'value': drive2.target_position}])

        # replyValue = replyHandle.get()  # Waiting for status reply
        # if replyValue.status != self.motorcortex_msg.OK:
        #     print(motorcortex.statusToStr(self.motorcortex_msg, replyValue.status))

    def main(self, feedback_pub, rate, subscription):
        # run control loop
        while not rospy.is_shutdown():

            # check watchdog with timeout 1 sec
            if not self.watchdog.check(1):
                # if watchdog failed, switch off drives
                print("Watchdog active!")
                replyHandle = self.request.setParameter('root/cia402/actualState', 1)
                replyHandle.get()

            mcxMessage = subscription.read()
            if mcxMessage:
                # axis 1
                feedback_msg1 = MotorcortexIn()
                feedback_msg1.statusword = mcxMessage[0].value[0]
                feedback_msg1.position_value = mcxMessage[1].value[0]
                feedback_msg1.drive_enabled = mcxMessage[4].value[0]
                feedback_msg1.drive_error_code = mcxMessage[5].value[0]
                # axis 2
                feedback_msg2 = MotorcortexIn()
                feedback_msg2.statusword = mcxMessage[2].value[0]
                feedback_msg2.position_value = mcxMessage[3].value[0]
                feedback_msg2.drive_enabled = mcxMessage[4].value[1]
                feedback_msg2.drive_error_code = mcxMessage[5].value[1]

                feedback_msg_list = MotorcortexInList()
                feedback_msg_list.drives_feedback = [feedback_msg1, feedback_msg2]
                feedback_pub.publish(feedback_msg_list)
                
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('motorcortex_control', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        drive = Control()
    except rospy.ROSInterruptException:
        pass
