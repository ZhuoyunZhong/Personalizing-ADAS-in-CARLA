#!/usr/bin/env python

import pygame
import carla
import math
import numpy as np

from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.lange_change import PolyLaneChange, SinLaneChange, SplineLaneChange
from agents.learning.model import Model
from agents.tools.misc import transform_to_frame


class LearningAgent(Agent):
    """
    BasicAgent implements a basic agent that navigates scenes to reach a given
    target destination. This agent respects traffic lights and other vehicles.
    """

    def __init__(self, world):
        """
        :param vehicle: actor to apply to local planner logic onto
        """
        super(LearningAgent, self).__init__(world.player)
        self._world_obj = world
        # Learning Model
        self._model = Model()
        self._safe_distance = None
        self._target_speed = None
        self._sin_param = None
        self._poly_param = None
        self._spline_param = None
        # Local plannar
        self._local_planner = LocalPlanner(world.player)
        self.update_parameters()
        # Global plannar
        self._proximity_threshold = 10.0  # meter
        self._state = AgentState.NAVIGATING


        self._hop_resolution = 0.1


        self._path_seperation_hop = 2
        self._path_seperation_threshold = 0.5
        self._grp = None  # global route planar
        # Behavior planning
        self._hazard_detected = False
        self._perform_lane_change = False
        self._front_r = []
        self._left_front_r = []
        self._left_back_r = []

    # Update personalized parameters from model
    def update_parameters(self):
        self._safe_distance = self._model.get_parameter("safe_distance")
        self._target_speed = self._model.get_parameter("target_speed")
        self._sin_param = self._model.get_parameter("sin_param")
        self._poly_param = self._model.get_parameter("poly_param")
        self._spline_param = self._model.get_parameter("spline_param")
        args_lateral_dict = {'K_P': 1.0, 'K_I': 0.4, 'K_D': 0.01}
        args_longitudinal_dict = {'K_P': 0.3, 'K_I': 0.2, 'K_D': 0.002}
        self._local_planner.init_controller(opt_dict={'target_speed': self._target_speed,
                                                      'lateral_control_dict': args_lateral_dict,
                                                      'longitudinal_control_dict': args_longitudinal_dict})

    # Start learning by collecting data
    def collect(self):
        dict_param = {}
        # Collect points while driving
        dict_param.update({"points": [self._vehicle.get_location().x,
                                      self._vehicle.get_location().y,
                                      self._vehicle.get_location().z,
                                      self._vehicle.get_transform().rotation.yaw,
                                      pygame.time.get_ticks()]})

        # Collect radar information while driving
        dict_param.update({"radars": [self._front_r,
                                      self._left_front_r,
                                      self._left_back_r]})

        # Collect speed while driving
        v = self._world_obj.player.get_velocity()
        dict_param.update({"velocity": v})

        # Collect distance to obstacle in front
        if self._front_r:
            dict_param.update({"distance": self._front_r[1][0]})

        self._model.collect(dict_param)

    # End learning mode
    def end_collect(self):
        self._model.end_collect()
        self.update_parameters()

    # Set global destination and get global waypoints
    def set_destination(self, location):
        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router
        """
        start_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        end_waypoint = self._map.get_waypoint(carla.Location(location[0], location[1], location[2]))

        route_trace = self._trace_route(start_waypoint, end_waypoint)
        assert route_trace

        self._local_planner.set_global_plan(route_trace)

    # Get global waypoints
    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the optimal route
        from start_waypoint to end_waypoint
        """
        # Setting up global router
        if self._grp is None:
            dao = GlobalRoutePlannerDAO(self._vehicle.get_world().get_map(), self._hop_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp

        # Obtain route plan
        route = self._grp.trace_route(start_waypoint.transform.location, end_waypoint.transform.location)

        return route

    # Run step
    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """

        # Check all the radars
        if self._world_obj.front_radar.detected:
            if abs(self._world_obj.front_radar.rel_pos[1]) < 1:
                self._front_r = [pygame.time.get_ticks(), self._world_obj.front_radar.rel_pos, 
                                                        self._world_obj.front_radar.rel_vel]
            self._world_obj.front_radar.detected = False                                        
        if self._world_obj.left_front_radar.detected:
            if self._world_obj.front_radar.rel_pos[1] < -1:
                self._left_front_r =[pygame.time.get_ticks(), self._world_obj.left_front_radar.rel_pos, 
                                                            self._world_obj.left_front_radar.rel_vel]
            self._world_obj.left_front_radar.detected = False
        if self._world_obj.left_back_radar.detected:
            if self._world_obj.front_radar.rel_pos[1] < -1:
                self._left_back_r = [pygame.time.get_ticks(), self._world_obj.left_back_radar.rel_pos, 
                                                            self._world_obj.left_back_radar.rel_vel]
            self._world_obj.left_back_radar.detected = False
        # Remove radar data if not detected again in 0.25 second
        if self._front_r and (pygame.time.get_ticks() - self._front_r[0] > 5000):
            self._front_r = []
        if self._left_front_r and (pygame.time.get_ticks() - self._left_front_r[0] > 5000):
            self._left_front_r = []
        if self._left_back_r and (pygame.time.get_ticks() - self._left_back_r[0] > 5000):
            self._left_back_r = []
        
        # Detect vehicles in front
        self._hazard_detected = False
        if self._front_r and (self._front_r[1][0] < self._safe_distance):
            self._hazard_detected = True

        '''                          
        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        actor_list = self._world.get_actors()
        lights_list = actor_list.filter("*traffic_light*")
        # check for the state of the traffic lights
        light_state, traffic_light = self._is_light_red(lights_list)
        if light_state:
            if debug:
                print('=== RED LIGHT AHEAD [{}])'.format(traffic_light.id))
            self._state = AgentState.BLOCKED_RED_LIGHT
            self._hazard_detected = True
        '''
        
        #print(self._front_r)
        #print(self._left_front_r)
        #print(self._left_back_r)
        #print(self._state)

        # Finite State Machine
        # 1, Navigating
        if self._state == AgentState.NAVIGATING:
            if self._hazard_detected:
                self._state = AgentState.BLOCKED_BY_VEHICLE
                # The vehicle is driving at a certain speed
                # There is enough space
                if self._vehicle.get_velocity().x > 5 and \
                self._vehicle.get_location().y > 7:
                    self._state = AgentState.PREPARE_LANE_CHANGING

        # 2, Blocked by Vehicle
        elif self._state == AgentState.BLOCKED_BY_VEHICLE:
            if not self._hazard_detected:
                self._state = AgentState.NAVIGATING

        # 4, Prepare Lane Change
        elif self._state == AgentState.PREPARE_LANE_CHANGING:
            if  not (self._front_r and self._front_r[1][0] < self._safe_distance) and \
                not (self._left_front_r and self._left_front_r[1][0] < self._safe_distance) and \
                not (self._left_back_r and self._left_back_r[1][0] > -10):
                    print(self._front_r)
                    print(self._left_front_r)
                    print(self._left_back_r)
                    self._state = AgentState.LANE_CHANGING
                    self._perform_lane_change = True

        # 5, Lane Change
        elif self._state == AgentState.LANE_CHANGING:
            if abs(self._vehicle.get_velocity().y) < 0.5 and \
               self._vehicle.get_location().y < 7.0:
                self._state = AgentState.NAVIGATING

        # Standard local planner behavior
        if self._state == AgentState.NAVIGATING or self._state == AgentState.LANE_CHANGING:
            control = self._local_planner.run_step(debug=debug)
        elif self._state == AgentState.PREPARE_LANE_CHANGING:
            if self._left_front_r and self._left_front_r[1][0] < self._safe_distance or \
               self._front_r and self._front_r[1][0] < self._safe_distance:
                control = self._local_planner.empty_control(debug=debug)
            else:
                control = self._local_planner.run_step(debug=debug)
        elif self._state == AgentState.BLOCKED_BY_VEHICLE or self._state == AgentState.BLOCKED_RED_LIGHT:
            control = self._local_planner.empty_control(debug=debug)

        # When performing a lane change
        if self._perform_lane_change:
            # Record original destination
            destination = self._local_planner.get_global_destination()
            # Get lane change start location
            ref_location = self._world_obj.player.get_location()
            ref_yaw = self._world_obj.player.get_transform().rotation.yaw

            if self._local_planner.waypoint_buffer:
                waypoint = self._local_planner.waypoint_buffer[-1][0]
                ref_location = waypoint.transform.location
            
            wait_dist = 2.0  # need some time to plan
            ref = [ref_location.x + wait_dist, ref_location.y, ref_yaw]

            # Replace current plan with a lane change plan
            DL = self._left_front_r[1][0] if self._left_front_r else 100
            DH = self._left_back_r[1][0] if self._left_back_r else 100
            GMM_v = [[self._vehicle.get_velocity().x, self._sin_param["lat_dis"], DL, DH]]
            
            lane_changer = SinLaneChange(self._world_obj, self._sin_param, np.array(GMM_v))
            lane_change_plan = lane_changer.get_waypoints(ref)
            self._local_planner.set_local_plan(lane_change_plan)
            '''
            lane_changer = PolyLaneChange(self._world_obj, self._poly_param)
            lane_change_plan = lane_changer.get_waypoints(ref)
            self._local_planner.set_local_plan(lane_change_plan)
            '''
            '''
            lane_changer = SplineLaneChange(self._world_obj, self._spline_param)
            # Plan first time without extra point
            lane_change_plan = lane_changer.get_waypoints(self._target_speed, ref)
            # Find global waypoint closest to the end of the plan
            end_point = self._map.get_waypoint(lane_change_plan[-1][0].transform.location)
            extras = [[end_point.transform.location.x, end_point.transform.location.y]]
            # Plan second time with extra global waypoint
            lane_change_plan = lane_changer.get_waypoints(self._target_speed, ref, extras)
            self._local_planner.set_local_plan(lane_change_plan)
            '''
            # replan globally with new vehicle position after lane changing
            new_start = self._map.get_waypoint(lane_change_plan[-1][0].transform.location)
            route_trace = self._trace_route(new_start, destination)
            assert route_trace
            self._local_planner.add_global_plan(route_trace)

            self._perform_lane_change = False
            print("perform lane change")

        return control

    def done(self):
        """
        Check whether the agent has reached its destination.
        :return bool
        """
        return self._local_planner.done()
