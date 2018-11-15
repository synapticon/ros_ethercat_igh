#!/usr/bin/env python

from motorcortex_msgs.msg import DigitalOutputs

class DIO(object):

    def __init__(self, id):
        self.id = id
        self.digitalOutputs = [False] * 12

    def update(self, feedbackMsg):
        self.digitalInputs = feedbackMsg.digital_inputs

    def getId(self):
        return self.id

    def getDigitalInputs(self):
        return self.digitalInputs

    def setDigitalOutputs(self, digital_outputs):
        self.digitalOutputs = digital_outputs

    def encode(self):
        ctrl_msg = DigitalOutputs()
        ctrl_msg.digital_outputs = self.digitalOutputs

        return ctrl_msg