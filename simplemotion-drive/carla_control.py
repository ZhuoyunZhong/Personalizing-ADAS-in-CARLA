#!/usr/bin/env python3

'''
Project: Simple MotionV2 based drive controller for Carla

@file </carla_control.py>

@author Prajankya Sonar - <prajankya@gmail.com>

MIT License
Copyright (c) 2020 Prajankya Sonar
'''

from __future__ import print_function

from smv2_drive import DriveController

import argparse
import pygame
import logging
import carla
import time
from carla_utils.HUD import HUD
from carla_utils.World import World
from carla_utils.KeyboardControl import KeyboardControl

import sys
sys.path.append('/opt/carla/PythonAPI/carla')

try:
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.roaming_agent import RoamingAgent
except ImportError as err:
    print("cannot Import agents module")
    sys.exit(1)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------
# ==============================================================================


class MyController():
    _isSteeringWheelConnected = False
    _positionFeedback = 0
    _feedbackAngle = 0
    _log_message = None

    world = None

    ###################### Init #######################
    def __init__(self, args):
        pygame.init()
        pygame.font.init()

        self._controller = DriveController("/dev/ttyUSB0", 1, False)

        ##### Register Callbacks #####
        self._controller.logCallback = self._logCallback
        self._controller.errorCallback = self._errorCallback
        self._controller.readingCallback = self._readingCallback
        self._controller.connectedCallback = self._connectedCallback

        try:
            # initialize carla client
            client = carla.Client(args.host, args.port)
            client.set_timeout(4.0)

            self._display = pygame.display.set_mode(
                (args.width, args.height),
                pygame.HWSURFACE | pygame.DOUBLEBUF)

            self._hud = HUD(args.width, args.height)
            self._world = World(client.get_world(), self._hud, args.filter)
            self._keyboardController = KeyboardControl(self._world)

            if args.agent == "Roaming":
                self._agent = RoamingAgent(self._world.player)
            else:
                self._agent = BasicAgent(self._world.player)
                spawn_point = self._world.map.get_spawn_points()[0]
                self._agent.set_destination((spawn_point.location.x,
                                             spawn_point.location.y,
                                             spawn_point.location.z))

            self._clock = pygame.time.Clock()

            # Steering Wheel is connected
            self._setupSteeringWheel(args)

            # run loop logic
            while not self._keyboardController.parse_events():
                self._runLoop()
        finally:
            self.__del__()

    def _setupSteeringWheel(self, args):
        self._clock.tick(10)  # run at 10 fps

        font = pygame.font.Font(pygame.font.get_default_font(), 34)

        # Few variables
        self.steeringWheelSetupStage = 0

        isSettingUp = True

        while isSettingUp:
            self._display.fill((0, 0, 0))

            # Setup stages
            # stage - to connect
            if self.steeringWheelSetupStage == 0:
                # Connect and start
                self._controller.connect()
                self.steeringWheelSetupStage += 1

            # stage - connecting
            elif self.steeringWheelSetupStage == 1:
                infoText = font.render(
                    'Connecting to Steering Wheel %s' % (
                        '.'*(int(pygame.time.get_ticks()/1000) % 4)), True, (255, 255, 255))
                infoTextRect = infoText.get_rect()
                infoTextRect.center = (args.width // 2, args.height // 4)

                self._display.blit(infoText, infoTextRect)

                if self._isSteeringWheelConnected:
                    print("Connected !")
                    self.steeringWheelSetupStage += 1

            # stage - set steering to zero
            elif self.steeringWheelSetupStage == 2:
                self._display.blit(infoText, infoTextRect)

                infoText = font.render(
                    'Move the Steering wheel to zero position and press \'s\' key', True, (255, 255, 255))
                infoTextRect = infoText.get_rect()
                infoTextRect.center = (args.width // 2, args.height // 4)

                self._display.blit(infoText, infoTextRect)

                text = font.render('Steering angle : %lf' % self._feedbackAngle, True,
                                   (255, 0, 0))
                textRect = text.get_rect()
                textRect.center = (args.width // 2, args.height // 2)

                self._display.blit(text, textRect)


            #
            # finished stages logic

            if self._log_message is not None:
                if self._log_message == "Opening SM bus failed":
                    self._display.fill((0, 0, 0))
                    logText = font.render(
                        "Cannot connect to Steering wheel, please check and try again", True, (255, 100, 100))
                    logTextRect = logText.get_rect()
                    logTextRect.center = (args.width // 2, args.height // 2)

                    self._display.blit(logText, logTextRect)

                else:  # show any log message
                    logText = pygame.font.Font(pygame.font.get_default_font(), 24).render(
                        self._log_message, True, (250, 255, 250))
                    self._display.blit(logText, (20, args.height - 40))

            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit(0)
                elif event.type == pygame.KEYUP:
                    # exit application
                    if (event.key == pygame.K_ESCAPE) or (event.key == pygame.K_q and pygame.key.get_mods() & pygame.KMOD_CTRL):
                        sys.exit(0)
                    # If s key is pressed
                    if event.key == pygame.K_s:
                        self._controller.setZero()
                        time.sleep(1)
                        isSettingUp = False

        # Setup other settings for the steering wheel
        self._controller.setAddedConstantTorque(150)
        # self._controller.disableSetpointTracking(500, 0.001)

    def _runLoop(self):

        # as soon as the server is ready continue!
        self._world.world.wait_for_tick(10.0)

        self._world.tick(self._clock)
        self._world.render(self._display)
        pygame.display.flip()

        # skip traffic lights
        if self._world.player.is_at_traffic_light():
            traffic_light = self._world.player.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                self._hud.notification("Traffic light changed!")
                traffic_light.set_state(carla.TrafficLightState.Green)

        control = self._agent.run_step()
        # control.brake = False
        # control.throttle = 1
        # control.hand_brake = False
        control.manual_gear_shift = False

        # Move steering wheel
        steer = int(control.steer*90)

        self._goToAngle(steer, 1000, 3, 0.05)

        # print(steer - self._feedbackAngle)

        control.steer = self._feedbackSteer * 2

        self._world.player.apply_control(control)

    def _goToAngle(self, pos, max_torque=2000, Kp=1.6, Kd=0.1):
        self._controller.setAbsoluteSetpoint(
            int((pos/360)*10000), max_torque, Kp, Kd)

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
        try:
            if self._world is not None:
                self._world.destroy()
        finally:
            pass

        pygame.quit()
        print("Quitting....")

    ################# Event Callbacks #################
    def _logCallback(self, obj):
        # logType
        # message
        print("LOG>", obj.message)
        self._hud.notification(obj.message)
        self._log_message = obj.message

    def _connectedCallback(self, obj):
        # isConnected
        print("CON>", obj.isConnected)
        self._isSteeringWheelConnected = obj.isConnected

    def _errorCallback(self, obj):
        # bool trackingError,
        # bool driveFault
        print("Error>", obj.trackingError, obj.driveFault)

    def _readingCallback(self, obj):
        # int posSetpoint
        # int posFeedback
        # int torqueSetpoint
        # print("Reading >", obj.posSetpoint,
        #       obj.posFeedback, obj.torqueSetpoint)

        self._positionFeedback = obj.posFeedback
        self._feedbackAngle = (obj.posFeedback / 10000)*360
        self._feedbackSteer = (self._feedbackAngle / 360)

        # print("Reading>", self._positionFeedback, self._feedbackAngle)


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.audi.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument("-a", "--agent", type=str,
                           choices=["Roaming", "Basic"],
                           help="select which agent to run",
                           default="Basic")
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:

        MyController(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
