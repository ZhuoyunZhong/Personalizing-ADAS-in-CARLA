#!/usr/bin/env python
""" Module with auxiliary / helper functions. """

import math
import numpy as np

import carla


def get_poly_y(x, param):
    """
    Compute polynomial y given x and polynomial parameters
    :param x: a numpy vector of x
    :param param: polynomial parameters
    :return: a numpy vector of y
    """
    t_m = np.array([np.power(x, 0), np.power(x, 1), np.power(x, 2),
                    np.power(x, 3), np.power(x, 4), np.power(x, 5)])
    y = np.matmul(param, t_m)
    return y


def transform_to_frame(frame, points, inverse=False):
    """
    Perform a transform from local frame to world frame
    :param frame: new frame
    :param points: numpy matrix of points (3xn)
    :param inverse: is inverse?
    :return: a matrix of points (3xn)
    """
    if points.ndim > 1:
        length = points.shape[1]
    else:
        length = 1
        points = np.reshape(points, (3, 1))
    points_m = np.vstack( (points[0:3, :], np.ones((1, length))) )  

    rotation = frame.rotation
    translation = frame.location

    matrix = np.eye(4)

    cy = math.cos(np.radians(rotation.yaw))
    sy = math.sin(np.radians(rotation.yaw))
    cr = math.cos(np.radians(rotation.roll))
    sr = math.sin(np.radians(rotation.roll))
    cp = math.cos(np.radians(rotation.pitch))
    sp = math.sin(np.radians(rotation.pitch))
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    matrix[0, 0] = cp * cy
    matrix[0, 1] = cy * sp * sr - sy * cr
    matrix[0, 2] = -(cy * sp * cr + sy * sr)
    matrix[1, 0] = sy * cp
    matrix[1, 1] = sy * sp * sr + cy * cr
    matrix[1, 2] = cy * sr - sy * sp * cr
    matrix[2, 0] = sp
    matrix[2, 1] = -(cp * sr)
    matrix[2, 2] = cp * cr

    if not inverse:
        matrix = np.linalg.inv(matrix)

    return np.matmul(matrix, points_m)[0:3,:]


def transform_to_world(local_frame, vector, inverse=False):
    """
    Perform a transform from local frame to world frame
    :param local_frame: local frame
    :param vector: point coordinate with respect to origin_frame
    :param inverse: is inverse?
    :return: a CarlaVector3
    """
    matrix = np.eye(4)
    vector4 = np.array([vector.x, vector.y, vector.z, 1])

    rotation = local_frame.rotation
    translation = local_frame.location

    cy = math.cos(np.radians(rotation.yaw))
    sy = math.sin(np.radians(rotation.yaw))
    cr = math.cos(np.radians(rotation.roll))
    sr = math.sin(np.radians(rotation.roll))
    cp = math.cos(np.radians(rotation.pitch))
    sp = math.sin(np.radians(rotation.pitch))
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    matrix[0, 0] = cp * cy
    matrix[0, 1] = cy * sp * sr - sy * cr
    matrix[0, 2] = -(cy * sp * cr + sy * sr)
    matrix[1, 0] = sy * cp
    matrix[1, 1] = sy * sp * sr + cy * cr
    matrix[1, 2] = cy * sr - sy * sp * cr
    matrix[2, 0] = sp
    matrix[2, 1] = -(cp * sr)
    matrix[2, 2] = cp * cr

    if inverse:
        matrix = np.linalg.inv(matrix)

    vector_new = np.matmul(matrix, vector4)
    return carla.Vector3D(x=vector_new[0], y=vector_new[1], z=vector_new[2])


def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

    :param world: carla.world object
    :param waypoints: list or iterable container with the waypoints to draw
    :param z: height in meters
    :return:
    """
    for p in waypoints:
        t = p.transform
        begin = t.location + carla.Location(z=z)
        size = 0.1
        angle = math.radians(t.rotation.yaw)
        end = begin + carla.Location(x=5*size*math.cos(angle), y=5*size*math.sin(angle))
        # world.debug.draw_point(begin, size=size, color=carla.Color(255,0,0), life_time=0.1)
        world.debug.draw_arrow(begin, end, arrow_size=size, life_time=0.1)


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Kmh
    :param vehicle: the vehicle for which speed is calculated
    :return: speed as a float in Kmh
    """
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


def is_within_distance_ahead(target_transform, current_transform, max_distance):
    """
    Check if a target object is within a certain distance in front of a reference object.

    :param target_transform: location of the target object
    :param current_transform: location of the reference object
    :param orientation: orientation of the reference object
    :param max_distance: maximum allowed distance
    :return: True if target object is within max_distance ahead of the reference object
    """
    target_vector = np.array([target_transform.location.x - current_transform.location.x, target_transform.location.y - current_transform.location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    fwd = current_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle < 90.0


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

    :param target_location: location of the target object
    :param current_location: location of the reference object
    :param orientation: orientation of the reference object
    :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return (norm_target, d_angle)


def distance_vehicle(waypoint, vehicle_transform):
    loc = vehicle_transform.location
    dx = waypoint.transform.location.x - loc.x
    dy = waypoint.transform.location.y - loc.y

    return math.sqrt(dx * dx + dy * dy)


def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2
    location_1, location_2:   carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]
