#!/usr/bin/env python

import math
from agents.navigation.local_planner import RoadOption
from agents.navigation.local_waypoint import LocalWaypoint


# This class generate lateral quintic polynomial trajectory s(t)
# and generate waypoints and plan according to target_speed
class PolyLaneChange:
    def __init__(self, world, param):
        self._world_obj = world
        self._map = self._world_obj.world.get_map()

        self._param_lon = param['param_lon']
        self._param_lat = param['param_lat']
        self._dt = param["dt"]

        self._npts = 15

    # Generate longitudinal quintic polynomial trajectory
    def get_x_from_curve(self, t):
        a5, a4, a3, a2, a1, a0 = self._param_lon
        return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

    # Generate lateral quintic polynomial trajectory
    def get_y_from_curve(self, t):
        a5, a4, a3, a2, a1, a0 = self._param_lat
        return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

    # Return lane change waypoints
    def get_waypoints(self, target_speed, ref):
        lane_change_plan = []
        x_ref = ref[0]
        y_ref = ref[1]
        yaw = ref[2]
        sy = math.sin(math.radians(yaw))
        cy = math.cos(math.radians(yaw))

        increment_t = self._dt / self._npts
        t = -increment_t
        for _ in range(self._npts):
            # Get time and longitudinal position
            t += increment_t
            x = self.get_x_from_curve(t)
            # Get lateral position
            y = self.get_y_from_curve(t)

            # Transform to world coordinate
            x_coord = cy * x - sy * y + x_ref
            y_coord = sy * x + cy * y + y_ref

            # Store waypoints
            waypoint = LocalWaypoint(x_coord, y_coord, 0)
            lane_change_plan.append((waypoint, RoadOption.CHANGELANELEFT))

        return lane_change_plan
