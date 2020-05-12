'''
Project: Simple MotionV2 based drive controller for Carla

@file </drive.py>

@author Prajankya Sonar - <prajankya@gmail.com>

MIT License
Copyright (c) 2020 Prajankya Sonar
'''

from smv2_drive import DriveController

import time


class MyController():
    _isMotorConnected = False
    _positionFeedback = 0
    _feedbackAngle = 0

    ###################### Init #######################
    def __init__(self):
        self._controller = DriveController("/dev/ttyUSB0", 1, False)

        ##### Register Callbacks #####
        self._controller.logCallback = self._logCallback
        self._controller.errorCallback = self._errorCallback
        self._controller.readingCallback = self._readingCallback
        self._controller.connectedCallback = self._connectedCallback

        # Connect and start
        self._controller.connect()

        try:
            print("WAITING TO CONNECT......")
            while True:
                time.sleep(0.2)
                if self._isMotorConnected:
                    print("Connected !")
                    break

            # Motor is connected
            self._controller.setAddedConstantTorque(150)

            while True:
                print("Move to 90")
                self._goToAngle(90, 2000, 1.6, 0.1)
                time.sleep(5)

                # print("Move to - 90")
                # self._goToAngle(-90, 2000, 1.6, 0.1)
                # time.sleep(5)

                print("Disable tracking")
                #                            Resistive torque, gain
                self._controller.disableSetpointTracking(1000, 0.008)
                time.sleep(3)

                self._goToAngle(-90, 2000, 1.6, 0.1)
                time.sleep(6)

                print("Enable tracking")
                self._controller.enableSetpointTracking()
                time.sleep(6)

            self.__del__()

        except KeyboardInterrupt as _:
            self.__del__()

    def _goToAngle(self, pos, max_torque, kp, kd):
        self._controller.setAbsoluteSetpoint(
            int((pos/360)*10000), max_torque, kp, kd)

    ################### Destructor ###################
    def __del__(self):
        ######################
        # nullptr ALL CALLBACKS before calling desctructor
        # Very very important to mitigate deadlock while exiting
        ######################
        self._controller.logCallback = None
        self._controller.errorCallback = None
        self._controller.readingCallback = None
        self._controller.connectedCallback = None

        print("Quitting....")

    ################# Event Callbacks #################
    def _logCallback(self, obj):
        # logType
        # message
        print("LOG>", obj.message)
        pass

    def _connectedCallback(self, obj):
        # isConnected
        print("CON>", obj.isConnected)
        self._isMotorConnected = obj.isConnected

    def _errorCallback(self, obj):
        # bool trackingError,
        # bool driveFault
        print("Error>", obj.trackingError, obj.driveFault)

    def _readingCallback(self, obj):
        # int posSetpoint
        # int posFeedback
        # int velSetpoint
        # print("Reading >", obj.posSetpoint, obj.posFeedback, obj.velSetpoint)

        self._positionFeedback = obj.posFeedback
        self._feedbackAngle = format((obj.posFeedback / 10000)*360, '.2f')
        # print("Reading>", self._positionFeedback, self._feedbackAngle)


if __name__ == "__main__":
    MyController()
