#!/usr/bin/env python

import argparse
import glob
import logging
import os
import random
import re
import sys

try:
    sys.path.append(glob.glob('../CARLA_Simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

try:
    sys.path.append('../CARLA_Simulator/PythonAPI/carla')
except IndexError:
    pass

from agents.navigation.roaming_agent import RoamingAgent
from agents.navigation.basic_agent import BasicAgent

import pygame

from manual_control import *
from sensors import *
from hud import *


class World(object):
    def __init__(self, carla_world, hud, actor_filter):
        self.world = carla_world
        self.map = self.world.get_map()
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = None
        self.find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'ego')
        blueprint.set_attribute('color', '0, 0, 0')
        # Spawn the player.
        if self.player is not None:
            spawn_point = carla.Transform(carla.Location(x=50, y=7.5, z=0.5))
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_point = carla.Transform(carla.Location(x=50, y=7.5, z=0.5))
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def find_weather_presets(self):
        rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
        name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
        presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
        self._weather_presets = [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# Main loop
def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args.filter)
        controller = KeyboardControl(world)

        if args.agent == "Roaming":
            agent = RoamingAgent(world.player)
        else:
            agent = BasicAgent(world.player)
            # Destination Setting
            agent.set_destination((230, 39, 0))

        clock = pygame.time.Clock()
        while True:
            if controller.parse_events():
                return

            # as soon as the server is ready continue!
            world.world.wait_for_tick(10.0)

            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            control = agent.run_step()
            control.manual_gear_shift = False
            world.player.apply_control(control)

    finally:
        if world is not None:
            world.destroy()

        pygame.quit()


def main():
    # Arguments
    argparser = argparse.ArgumentParser(description='CARLA Ego Vehicle Client')
    argparser.add_argument('-v', '--verbose', action='store_true', dest='debug',
                           help='print debug information')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--res', metavar='WIDTH x HEIGHT', default='1920x1080',
                           help='window resolution')
    argparser.add_argument('--filter', metavar='PATTERN', default='vehicle.tesla.*',
                           help='actor filter (default: "vehicle.tesla.*")')
    argparser.add_argument("-a", "--agent", type=str, choices=["Roaming", "Basic"], default="Basic",
                           help="select which agent to run")
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]

    # Logging process
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)
    print(__doc__)

    # Start the loop
    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
