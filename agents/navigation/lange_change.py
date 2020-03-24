#!/usr/bin/env python

import numpy as np
import math
from scipy.interpolate import splrep, splev
from agents.navigation.local_planner import RoadOption
from agents.navigation.local_waypoint import LocalWaypoint
from agents.tools.misc import get_poly_y


# This class generate lateral and longitudinal quintic polynomial
# trajectory s(t) and generate waypoints according to target_speed
class PolyLaneChange:
    def __init__(self, world, param):
        self._world_obj = world
        self._map = self._world_obj.world.get_map()

        self._lon_dis = param['lon_dis']
        self._lat_dis = param['lat_dis']
        self._lon_param = param['lon_param']
        self._lat_param = param['lat_param']
        self._dt = param["dt"]

        self._npts = 15

    # Return lane change waypoints
    def get_waypoints(self, ref):
        lane_change_plan = []
        x_ref = ref[0]
        y_ref = ref[1]
        yaw = ref[2]
        sy = math.sin(math.radians(yaw))
        cy = math.cos(math.radians(yaw))

        # Get points
        t = np.linspace(0, self._dt, self._npts)
        x = get_poly_y(t, self._lon_param)
        y = get_poly_y(t, self._lat_param)

        # Transform to world coordinate
        R = np.array([[cy, -sy], [sy, cy]])
        coord = np.matmul(R, np.stack((x, y))) + np.array([[x_ref], [y_ref]])

        # Store waypoints
        for i in range(self._npts):
            waypoint = LocalWaypoint(coord[0][i], coord[1][i], 0)
            lane_change_plan.append((waypoint, RoadOption.CHANGELANELEFT))

        return lane_change_plan


# This class generate lateral and longitudinal quintic polynomial
# trajectory s(t) and generate waypoints according to target_speed
class SplineLaneChange:
    def __init__(self, world, param):
        self._world_obj = world
        self._map = self._world_obj.world.get_map()

        self._tck = param['tck']
        self._lon_dis = param["lon_dis"]
        self._lat_dis = param["lat_dis"]

        self._npts = 15

    # Return lane change waypoints
    def get_waypoints(self, target_speed, ref, extras=None):
        lane_change_plan = []
        tck = self._tck

        x_ref = ref[0]
        y_ref = ref[1]
        yaw = ref[2]
        sy = math.sin(math.radians(yaw))
        cy = math.cos(math.radians(yaw))

        dt = math.sqrt(self._lat_dis ** 2 + self._lon_dis ** 2) / target_speed  # For future
        increment_x = self._lon_dis / self._npts

        # If extras exists, refit tck with extra points
        if extras:
            xs = []
            ys = []
            # Add new points
            for (x_coord, y_coord) in extras:
                x = (cy * x_coord + sy * y_coord) + (-cy * x_ref - sy * y_ref)
                y = (-sy * x_coord + cy * y_coord) + (sy * x_ref - cy * y_ref)
                xs.append(x)
                ys.append(y)
            # Add points from previous spline
            x = -increment_x
            for _ in range(self._npts):
                # Get longitudinal position
                x += increment_x
                # Get lateral position
                y = float(splev(x, self._tck))
                xs.append(x)
                ys.append(y)
            xs.sort()
            ys.sort(reverse=True)
            tck = splrep(xs, ys, s=0.1)

        # Generate points for lane changing
        x = -increment_x
        for _ in range(self._npts):
            # Get longitudinal position
            x += increment_x
            # Get lateral position
            y = float(splev(x, tck))

            # Transform to world coordinate
            x_coord = cy * x - sy * y + x_ref
            y_coord = sy * x + cy * y + y_ref

            # Store waypoints
            waypoint = LocalWaypoint(x_coord, y_coord, 0)
            lane_change_plan.append((waypoint, RoadOption.CHANGELANELEFT))

        return lane_change_plan
