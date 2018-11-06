#!/usr/bin/env python

from motion_control.msg import MotorcortexOut

DRIVE_OFF = 2
DRIVE_ON = 4
RESET_ERROR = 701
POSITION_MODE = 1

class Drive(object):
    enabled = False
    actualPosition = 0
    targetPosition = 0
    status = 0
    errorCode = 0
    command = DRIVE_OFF
    mode = POSITION_MODE

    def __init__(self, id):
        self.id = id

    def update(self, statusMsg):
        self.enabled = statusMsg.drive_enabled
        self.actualPosition = statusMsg.position_value
        self.status = statusMsg.statusword
        self.errorCode = statusMsg.drive_error_code

    def getId(self):
        return self.id

    def isEnabled(self):
        return self.enabled

    def setPosition(self, position):
        self.targetPosition = position

    def getPosition(self):
        return self.actualPosition

    def setMode(self, mode):
        self.mode = mode

    def switchOn(self):
        self.command = DRIVE_ON

    def switchOff(self):
        self.command = DRIVE_OFF

    def resetError(self):
        self.command = RESET_ERROR

    def hasError(self):
        return self.errorCode != 0

    def encode(self):
        ctrl_msg = MotorcortexOut()
        ctrl_msg.controlword = self.command
        ctrl_msg.opmode = self.mode
        ctrl_msg.target_position = self.targetPosition

        return ctrl_msg
