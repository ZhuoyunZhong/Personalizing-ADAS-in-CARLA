'''
Project: Simple MotionV2 based drive controller for Carla

@file </test.py>

@author Prajankya Sonar - <prajankya@gmail.com>

MIT License
Copyright (c) 2020 Prajankya Sonar
'''


from smv2_drive import DriveController

import time
import threading


class MyController():
    _isMotorConnected = False

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

        print("WAITING TO CONNECT......")

        try:
            while True:
                if self._isMotorConnected:
                    # self._controller.incrementSetpoint(-3000)
                    time.sleep(1)
                    self._controller.goToPosition(2000)
                    print("MOVING")
                    # self._controller.incrementSetpoint(3000)
                    # self._controller.incrementSetpoint(100)
                    time.sleep(10)
                    self._controller.goToPosition(-2000)
                    # self._controller.incrementSetpoint(100)
                    time.sleep(10)

                    self._controller.setEnabled(False)
                    time.sleep(10)
                    self._controller.setEnabled(True)

                    # i = 1000
                    # while i > 0:
                    #     self._controller.goToPosition(self.posFeedback)
                    #     time.sleep(0.01)
                    #     i -= 1
                    #     print(i)

                # time.sleep(0.1)
        except KeyboardInterrupt as _:
            self.__del__()

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
        self.posFeedback = obj.posFeedback
        pass


if __name__ == "__main__":
    MyController()
