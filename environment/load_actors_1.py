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

import random
import time


def spawn_surrounding_vehicles(world, blueprint_library):
    # Create a certain amount of vehicles in autopilot mode
    vehicle_list = []

    # Set vehicles selection
    bp_lib = []
    bp_lib.extend(blueprint_library.filter('vehicle.nissan.*'))
    bp_lib.extend(blueprint_library.filter('vehicle.audi.*'))
    bp_lib.extend(blueprint_library.filter('vehicle.tesla.*'))

    # Set vehicles spawn location
    transform_front = carla.Transform(carla.Location(x=65, y=7.8, z=0.1))
    transform_list = [transform_front]

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

    return vehicle_list


def load_actors():
    # Initialize client
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    # Actor list
    vehicle_list = []

    try:
        # Get the world and blueprints
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # Spawn actors
        vehicle_list = spawn_surrounding_vehicles(world, blueprint_library)

        time.sleep(0.5)
        for v in vehicle_list:
            v.set_autopilot(True)

        # Keep the actors running
        while True:
            time.sleep(1)

    finally:
        # File terminated
        client.apply_batch([carla.command.DestroyActor(v) for v in vehicle_list])
        print('All actors destroyed.')


if __name__ == "__main__":
    try:
        load_actors()
    except KeyboardInterrupt:
        pass
