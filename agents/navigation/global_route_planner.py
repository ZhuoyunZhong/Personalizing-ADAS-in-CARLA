#!/usr/bin/env python

import math

import numpy as np
import networkx as nx

import carla
from agents.navigation.local_planner import RoadOption
from agents.tools.misc import vector


class GlobalRoutePlanner(object):
    """
    This class provides a very high level route plan. (Global Planar)
    1, To build a graph with
    _build_graph(), _find_loose_ends(), _lane_change_link()
    2, Finding a path and decision plan
    _path_search(), _turn_decision
    3, Global Planar
    trace_route()
    """

    def __init__(self, dao):
        """Instantiate with a GlobalRoutePlannerDAO object"""
        self._dao = dao
        self._topology = None
        self._graph = None
        self._id_map = None
        self._road_id_to_edge = None
        self._intersection_end_node = -1
        self._previous_decision = RoadOption.VOID

    def setup(self):
        """
        Performs initial server data lookup for world detailed topology
        and builds graph representation of the world map.
        """
        self._topology = self._dao.get_topology()
        self._graph, self._id_map, self._road_id_to_edge = self._build_graph()
        self._find_loose_ends()
        self._lane_change_link()

    def _build_graph(self):
        """
        This function builds a NetworkX graph representation of topology.
        The topology is read from self._topology.
        graph node properties:
            vertex   -   (x,y,z) position in world map
        graph edge properties:
            entry_vector    -   unit vector along tangent at entry point
            exit_vector     -   unit vector along tangent at exit point
            net_vector      -   unit vector of the chord from entry to exit
            intersection    -   boolean indicating if the edge belongs to an
                                intersection
        return      :   graph -> NetworkX graph representing the world map,
                        id_map-> mapping from (x,y,z) to node id
                        road_id_to_edge-> map from road id to edge in the graph
        """
        # Create NetworkX directed graph
        graph = nx.DiGraph()
        # Map with structure {wp_coordinate: id, ... }  wp_coordinate: (x, y, z)
        id_map = dict()
        # Map with structure {road_id: {section_id: {lane_id: edge, ...}, ... }, ... },  edge: (wp1_id_1, wp2_id)
        road_id_to_edge = dict()

        # Each segment: tuple(carla.Waypoint, carla.Waypoint)
        for segment in self._topology:
            entry_xyz, exit_xyz = segment['entryxyz'], segment['exitxyz']
            path = segment['path']
            entry_wp, exit_wp = segment['entry'], segment['exit']
            intersection = entry_wp.is_junction
            # id depends on entry waypoint (not exit one)
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id

            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map
                if vertex not in id_map:
                    new_id = len(id_map)
                    id_map[vertex] = new_id  # id of id_map
                    graph.add_node(new_id, vertex=vertex)
            # Adding edge to road_id_to_edge
            id1 = id_map[entry_xyz]
            id2 = id_map[exit_xyz]
            if road_id not in road_id_to_edge:
                road_id_to_edge[road_id] = dict()
            if section_id not in road_id_to_edge[road_id]:
                road_id_to_edge[road_id][section_id] = dict()
            road_id_to_edge[road_id][section_id][lane_id] = (id1, id2)

            # Adding edge with attributes to NetworkX graph
            entry_carla_vector = entry_wp.transform.rotation.get_forward_vector()
            exit_carla_vector = exit_wp.transform.rotation.get_forward_vector()
            graph.add_edge(
                id1, id2,
                length=len(path) + 1, path=path,
                entry_waypoint=entry_wp, exit_waypoint=exit_wp,
                entry_vector=np.array(
                    [entry_carla_vector.x, entry_carla_vector.y, entry_carla_vector.z]),
                exit_vector=np.array(
                    [exit_carla_vector.x, exit_carla_vector.y, exit_carla_vector.z]),
                net_vector=vector(entry_wp.transform.location, exit_wp.transform.location),
                intersection=intersection, type=RoadOption.LANEFOLLOW)

        return graph, id_map, road_id_to_edge

    def _find_loose_ends(self):
        """
        This method finds road segments that have an unconnected end and (why?)
        adds them to the internal graph representation
        """
        count_loose_ends = 0
        hop_resolution = self._dao.get_resolution()
        for segment in self._topology:
            end_wp = segment['exit']
            exit_xyz = segment['exitxyz']
            road_id, section_id, lane_id = end_wp.road_id, end_wp.section_id, end_wp.lane_id
            if road_id in self._road_id_to_edge and \
                    section_id in self._road_id_to_edge[road_id] and \
                    lane_id in self._road_id_to_edge[road_id][section_id]:
                pass

            else:
                count_loose_ends += 1
                if road_id not in self._road_id_to_edge:
                    self._road_id_to_edge[road_id] = dict()
                if section_id not in self._road_id_to_edge[road_id]:
                    self._road_id_to_edge[road_id][section_id] = dict()
                id1 = self._id_map[exit_xyz]
                id2 = -1 * count_loose_ends
                self._road_id_to_edge[road_id][section_id][lane_id] = (id1, id2)
                next_wp = end_wp.next(hop_resolution)
                path = []
                while next_wp is not None and next_wp and \
                        next_wp[0].road_id == road_id and \
                        next_wp[0].section_id == section_id and \
                        next_wp[0].lane_id == lane_id:
                    path.append(next_wp[0])
                    next_wp = next_wp[0].next(hop_resolution)
                if path:
                    id2_xyz = (path[-1].transform.location.x,
                               path[-1].transform.location.y,
                               path[-1].transform.location.z)
                    self._graph.add_node(id2, vertex=id2_xyz)
                    self._graph.add_edge(
                        id1, id2,
                        length=len(path) + 1, path=path,
                        entry_waypoint=end_wp, exit_waypoint=path[-1],
                        entry_vector=None, exit_vector=None, net_vector=None,
                        intersection=end_wp.is_intersection, type=RoadOption.LANEFOLLOW)

    def _localize(self, location):
        """
        This function finds the edge in graph closest to given location
        location        :   carla.Location to be localized in the graph
        return          :   pair node ids representing an edge in the graph
        """
        waypoint = self._dao.get_waypoint(location)
        edge = None
        try:
            edge = self._road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        except KeyError:
            print(
                "Failed to localize! : ",
                "Road id : ", waypoint.road_id,
                "Section id : ", waypoint.section_id,
                "Lane id : ", waypoint.lane_id,
                "Location : ", waypoint.transform.location.x,
                waypoint.transform.location.y)
        return edge

    def _lane_change_link(self):
        """
        This method places zero cost links in the topology graph
        representing availability of lane changes.
        """
        for segment in self._topology:
            left_found, right_found = False, False

            for waypoint in segment['path']:
                if not segment['entry'].is_junction:
                    next_waypoint, next_road_option, next_segment = None, None, None

                    if bool(waypoint.lane_change & carla.LaneChange.Right) and not right_found:
                        next_waypoint = waypoint.get_right_lane()
                        # if next waypoint exists && allowed driving lane && same road
                        if next_waypoint is not None and \
                                next_waypoint.lane_type == carla.LaneType.Driving and \
                                waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANERIGHT
                            # add this edge to graph
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                # length = 0
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=segment['entry'],
                                    exit_waypoint=self._graph.edges[next_segment[0], next_segment[1]]['entry_waypoint'],
                                    path=[], length=0, type=next_road_option, change_waypoint=waypoint)
                                right_found = True

                    if bool(waypoint.lane_change & carla.LaneChange.Left) and not left_found:
                        next_waypoint = waypoint.get_left_lane()
                        # if next waypoint exists && allowed driving lane && same road
                        if next_waypoint is not None and \
                                next_waypoint.lane_type == carla.LaneType.Driving and \
                                waypoint.road_id == next_waypoint.road_id:
                            next_road_option = RoadOption.CHANGELANELEFT
                            # add this edge to graph
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                # length = 0
                                self._graph.add_edge(
                                    self._id_map[segment['entryxyz']], next_segment[0], entry_waypoint=segment['entry'],
                                    exit_waypoint=self._graph.edges[next_segment[0], next_segment[1]]['entry_waypoint'],
                                    path=[], length=0, type=next_road_option, change_waypoint=waypoint)
                                left_found = True

                if left_found and right_found:
                    break

    def _distance_heuristic(self, id1, id2):
        """
        Distance heuristic calculator for path searching
        in self._graph
        """
        l1 = np.array(self._graph.nodes[id1]['vertex'])
        l2 = np.array(self._graph.nodes[id2]['vertex'])
        return np.linalg.norm(l1 - l2)

    def _path_search(self, origin, destination):
        """
        This function finds the shortest path connecting origin and destination
        using A* search with distance heuristic.
        return a path as list of node ids (as int) of the graph self._graph
        """
        start_edge, end_edge = self._localize(origin), self._localize(destination)

        # Use A* to find path
        route = nx.astar_path(self._graph, source=start_edge[0], target=end_edge[0],
                              heuristic=self._distance_heuristic, weight='length')
        route.append(end_edge[1])  # append end of end_edge
        return route

    def _successive_last_intersection_edge(self, index, route):
        """
        This method returns the last successive intersection edge
        from a starting index on the route.

        This helps moving past tiny intersection edges to calculate
        proper turn decisions.
        """
        last_intersection_edge = None
        last_node = None
        for node1, node2 in [(route[i], route[i + 1]) for i in range(index, len(route) - 1)]:
            candidate_edge = self._graph.edges[node1, node2]
            if node1 == route[index]:
                last_intersection_edge = candidate_edge
            if candidate_edge['type'] == RoadOption.LANEFOLLOW and \
                    candidate_edge['intersection']:
                last_intersection_edge = candidate_edge
                last_node = node2
            else:
                break

        return last_node, last_intersection_edge

    def _turn_decision(self, index, route, threshold=math.radians(5)):
        """
        This method returns the turn decision (RoadOption) for pair of edges
        around current index of route list
        """
        decision = None
        previous_node = route[index - 1]
        current_node = route[index]
        next_node = route[index + 1]
        next_edge = self._graph.edges[current_node, next_node]
        if index > 0:
            # if during intersection
            if self._previous_decision != RoadOption.VOID and \
                    self._intersection_end_node > 0 and \
                    self._intersection_end_node != previous_node and \
                    next_edge['type'] == RoadOption.LANEFOLLOW and \
                    next_edge['intersection']:
                decision = self._previous_decision
            else:
                self._intersection_end_node = -1
                current_edge = self._graph.edges[previous_node, current_node]
                calculate_turn = current_edge['type'].value == RoadOption.LANEFOLLOW.value and \
                                 not current_edge['intersection'] and \
                                 next_edge['type'].value == RoadOption.LANEFOLLOW.value and \
                                 next_edge['intersection']
                # if start intersection
                if calculate_turn:
                    last_node, tail_edge = self._successive_last_intersection_edge(index, route)
                    self._intersection_end_node = last_node
                    if tail_edge is not None:
                        next_edge = tail_edge
                    cv, nv = current_edge['exit_vector'], next_edge['net_vector']
                    cross_list = []
                    for neighbor in self._graph.successors(current_node):
                        select_edge = self._graph.edges[current_node, neighbor]
                        if select_edge['type'].value == RoadOption.LANEFOLLOW.value:
                            if neighbor != route[index + 1]:
                                sv = select_edge['net_vector']
                                cross_list.append(np.cross(cv, sv)[2])
                    next_cross = np.cross(cv, nv)[2]
                    deviation = math.acos(np.clip(
                        np.dot(cv, nv) / (np.linalg.norm(cv) * np.linalg.norm(nv)), -1.0, 1.0))
                    if not cross_list:
                        cross_list.append(0)
                    # decide plan according to angle deviation
                    if deviation < threshold:
                        decision = RoadOption.STRAIGHT
                    elif cross_list and next_cross < min(cross_list):
                        decision = RoadOption.LEFT
                    elif cross_list and next_cross > max(cross_list):
                        decision = RoadOption.RIGHT
                    elif next_cross < 0:
                        decision = RoadOption.LEFT
                    elif next_cross > 0:
                        decision = RoadOption.RIGHT
                else:
                    decision = next_edge['type']
        else:
            decision = next_edge['type']
        self._previous_decision = decision

        return decision

    def abstract_route_plan(self, origin, destination):
        """
        The following function generates the route plan/decision
        return a list of
            node ids (as int) of the graph self._graph
        and a list of turn by turn navigation decisions as
            agents.navigation.local_planner.RoadOption elements
            Possible values are STRAIGHT, LEFT, RIGHT, LANEFOLLOW, VOID
                                CHANGELANELEFT, CHANGELANERIGHT
        """
        route = self._path_search(origin, destination)
        plan = []

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            plan.append(road_option)

        return route, plan

    def _find_closest_in_list(self, current_waypoint, waypoint_list):
        min_distance = float('inf')
        closest_index = -1
        for i, waypoint in enumerate(waypoint_list):
            distance = waypoint.transform.location.distance(current_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def trace_route(self, origin, destination):
        """
        This method returns list of (carla.Waypoint, RoadOption)
        from origin (carla.Location) to destination (carla.Location)
        """
        route_trace = []
        current_waypoint = self._dao.get_waypoint(origin)
        destination_waypoint = self._dao.get_waypoint(destination)
        resolution = self._dao.get_resolution()

        route, plan = self.abstract_route_plan(origin, destination)
        for i in range(len(route) - 1):
            road_option = plan[i]
            edge = self._graph.edges[route[i], route[i + 1]]
            path = []

            # left current edge:
            # STRAIGHT, LEFT, RIGHT, LANECHANEGLEFT, LANECHANGERIGHT
            if edge['type'].value != RoadOption.LANEFOLLOW.value and \
                    edge['type'].value != RoadOption.VOID.value:
                route_trace.append((current_waypoint, road_option))
                exit_wp = edge['exit_waypoint']
                id1, id2 = self._road_id_to_edge[exit_wp.road_id][exit_wp.section_id][exit_wp.lane_id]
                next_edge = self._graph.edges[id1, id2]
                if next_edge['path']:
                    closest_index = self._find_closest_in_list(current_waypoint, next_edge['path'])
                    closest_index = min(len(next_edge['path']) - 1, closest_index + 5)
                    current_waypoint = next_edge['path'][closest_index]
                else:
                    current_waypoint = next_edge['exit_waypoint']
                route_trace.append((current_waypoint, road_option))

            # stay in current edge: LANEFOLLOW
            else:
                path = path + [edge['entry_waypoint']] + edge['path'] + [edge['exit_waypoint']]
                closest_index = self._find_closest_in_list(current_waypoint, path)
                for waypoint in path[closest_index:]:
                    current_waypoint = waypoint
                    route_trace.append((current_waypoint, road_option))
                    # stop in advance
                    if len(route) - i <= 2 and \
                            waypoint.transform.location.distance(destination) < 2 * resolution:
                        break
                    elif len(route) - i <= 2 and \
                            current_waypoint.road_id == destination_waypoint.road_id and \
                            current_waypoint.section_id == destination_waypoint.section_id and \
                            current_waypoint.lane_id == destination_waypoint.lane_id:
                        destination_index = self._find_closest_in_list(destination_waypoint, path)
                        if closest_index > destination_index:
                            break
        return route_trace
