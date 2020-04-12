#!/usr/bin/env python

import glob
import os
import sys
try:
    sys.path.append(glob.glob('../../CARLA_Simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import pygame
import random


def spawn_surrounding_vehicles(world, scene):
    # Scenes
    transform_list = []
    if scene == "1":
        transform_curr_front = carla.Transform(carla.Location(x=65, y=7.8, z=0.1))
        transform_list = [transform_curr_front]

    if scene == "2":
        transform_curr_front = carla.Transform(carla.Location(x=65, y=7.8, z=0.1))
        transform_curr_back = carla.Transform(carla.Location(x=40, y=7.3, z=0.1))
        transform_side_front = carla.Transform(carla.Location(x=65, y=3.8, z=0.1))
        transform_side_back = carla.Transform(carla.Location(x=40, y=3.5, z=0.1))
        transform_list = [transform_curr_front, transform_curr_back, transform_side_front, transform_side_back]

    # Set vehicles selection
    blueprint_library = world.get_blueprint_library()
    bp_lib = []
    bp_lib.extend(blueprint_library.filter('vehicle.nissan.*'))
    bp_lib.extend(blueprint_library.filter('vehicle.audi.*'))
    bp_lib.extend(blueprint_library.filter('vehicle.tesla.model3'))
    
    # Spawn vehicles
    vehicle_list = []
    for vehicle_i in range(len(transform_list)):
        # Vehicle attribute
        bp = random.choice(bp_lib)
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        # Vehicle location
        transform = transform_list[vehicle_i]

        # Spawn the vehicle
        vehicle = world.try_spawn_actor(bp, transform)
        if vehicle is not None:
            vehicle_list.append(vehicle)
            print('created %s' % vehicle.type_id)
        else:
            print('location for %s occupied' % vehicle.type_id)

    pygame.time.wait(500)
    for vehicle in vehicle_list:
        vehicle.set_autopilot(True)
        print("%s set to Autopilot mode." % vehicle.type_id)

    return vehicle_list
