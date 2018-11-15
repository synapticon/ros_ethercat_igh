#!/usr/bin/env python

from motorcortex_msgs.msg import MotorcortexOut
from enum import Enum

class DriveCommand(Enum):
    DRIVE_CMD_OFF = 0x0
    DRIVE_CMD_DISENGAGE = 0x1
    DRIVE_CMD_ENGAGE = 0x2
    DRIVE_CMD_FAULT_ACK = 0x3
    DRIVE_CMD_QUICK_STOP = 0x4

class DriveState(Enum):
    DRIVE_STATUS_OFF = 0x0
    DRIVE_STATUS_DISENGAGED = 0x1
    DRIVE_STATUS_ENGAGED = 0x2
    DRIVE_STATUS_FAULT = 0x3
    DRIVE_STATUS_QUICK_STOP_ACTIVE = 0x4

class OpModesCiA402(Enum):
    CSP = 8
    CSV = 9
    CST = 10
    NONE = 0

class Drive(object):
    enabled = False
    actualPosition = 0
    targetPosition = 0
    status = 0
    errorCode = 0
    command = DriveCommand.DRIVE_CMD_OFF.value
    mode = OpModesCiA402.NONE.value

    def __init__(self, id):
        self.id = id
        self.digitalOutputs = [False,False,False,False]
        self.targetPosition = None
        self.targetVelocity = 0
        self.targetTorque = 0
        self.torqueOffset = 0


    def update(self, statusMsg):
        self.status = statusMsg.statusword
        self.errorCode = statusMsg.drive_error_code
        self.slaveTimestamp = statusMsg.slave_timestamp
        self.actualPosition = statusMsg.position_value
        self.actualVelocity = statusMsg.velocity_value
        self.actualTorque = statusMsg.torque_value
        self.actualSecondaryPosition = statusMsg.secondary_position_value
        self.actualSecondaryVelocity = statusMsg.secondary_velocity_value
        self.analogInputs = statusMsg.analog_inputs
        self.digitalInputs = statusMsg.digital_inputs

    def getId(self):
        return self.id

    def isEnabled(self):
        return self.status == DriveState.DRIVE_STATUS_ENGAGED.value

    def setPosition(self, position):
        self.targetPosition = position

    def setVelocity(self, velocity):
        self.targetVelocity = velocity

    def setTorque(self, torque):
        self.targetTorque = torque

    def setTorqueOffset(self, torque_offset):
        self.torqueOffset = torque_offset

    def setDigitalOutputs(self, digital_outputs):
        self.digitalOutputs = digital_outputs

    def getTimestamp(self):
        return self.slaveTimestamp

    def getPosition(self):
        return self.actualPosition

    def getVelocity(self):
        return self.actualVelocity

    def getTorque(self):
        return self.actualTorque

    def getSecondaryPosition(self):
        return self.actualSecondaryPosition

    def getSecondaryVelocity(self):
        return self.actualSecondaryVelocity

    def getAnalogInputs(self):
        return self.analogInputs

    def getDigitalInputs(self):
        return self.digitalInputs

    def setMode(self, mode):
        self.mode = mode

    def switchOn(self):
        self.command = DriveCommand.DRIVE_CMD_ENGAGE.value

    def switchOff(self):
        self.command = DriveCommand.DRIVE_CMD_OFF.value

    def resetError(self):
        self.command = DriveCommand.DRIVE_CMD_FAULT_ACK.value

    def hasError(self):
        return self.status == DriveState.DRIVE_STATUS_FAULT.value

    def encode(self):
        ctrl_msg = MotorcortexOut()
        ctrl_msg.controlword = self.command
        ctrl_msg.opmode = self.mode
        ctrl_msg.target_position = self.targetPosition
        ctrl_msg.target_velocity = self.targetVelocity
        ctrl_msg.target_torque = self.targetTorque
        ctrl_msg.torque_offset = self.torqueOffset
        ctrl_msg.digital_outputs = self.digitalOutputs

        return ctrl_msg
