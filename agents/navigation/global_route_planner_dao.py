import numpy as np


class GlobalRoutePlannerDAO(object):
    """
    This class is the data access layer for
    fetching data from the carla server instance
    for GlobalRoutePlanner
    """

    def __init__(self, wmap, sampling_resolution=1):
        self._sampling_resolution = sampling_resolution
        self._wmap = wmap  # Carla world map object

    def get_topology(self):
        """
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects.

        return: list of dictionary objects with the following attributes
                entry   -   waypoint of entry point of road segment
                entryxyz-   (x,y,z) of entry point of road segment
                exit    -   waypoint of exit point of road segment
                exitxyz -   (x,y,z) of exit point of road segment
                path    -   list of waypoints separated by 1m from entry
                            to exit
        """
        # To return the world's topology list
        topology = []

        # Retrieving waypoints to construct a detailed topology
        # Each segment: tuple(carla.Waypoint, carla.Waypoint)
        # representing the begin or end of a road
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            p1, p2 = wp1.transform.location, wp2.transform.location
            # Rounding off to avoid floating point imprecision
            x1, y1, z1, x2, y2, z2 = np.round([p1.x, p1.y, p1.z, p2.x, p2.y, p2.z], 0)
            wp1.transform.location, wp2.transform.location = p1, p2
            # Save as a dictionary
            seg_dict = dict()
            seg_dict['entry'], seg_dict['exit'] = wp1, wp2
            seg_dict['entryxyz'], seg_dict['exitxyz'] = (x1, y1, z1), (x2, y2, z2)
            seg_dict['path'] = []
            # Get path way points from begin to end according to sampling resolution
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                w = wp1.next(self._sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self._sampling_resolution:
                    seg_dict['path'].append(w)
                    w = w.next(self._sampling_resolution)[0]
            else:
                seg_dict['path'].append(wp1.next(self._sampling_resolution/2.0)[0])
            topology.append(seg_dict)
        return topology

    def get_waypoint(self, location):
        """
        Returns a waypoint that can be located in an exact location
        or translated to the center of the nearest lane
        """
        return self._wmap.get_waypoint(location)

    def get_resolution(self):
        """Get sampling resolution"""
        return self._sampling_resolution
