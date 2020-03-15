#!/usr/bin/env python

import carla


# Carla does not provide method to create own waypoint
# at a certain location in the map.
# Thus this LocalWaypointclass is used for local planning
class LocalWaypoint:
    def __init__(self, x, y, z):
        self.transform = carla.Transform(carla.Location(x=x, y=y, z=z))
