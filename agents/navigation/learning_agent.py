#!/usr/bin/env python

import pygame
import carla
import math
import numpy as np

from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.lange_change import PolyLaneChange, SinLaneChange
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
        self._THW = None
        self._target_speed = None
        self._sin_param = None
        self._poly_param = None
        # Local plannar
        self._local_planner = LocalPlanner(world.player)
        self.update_parameters()
        # Global plannar
        self._proximity_threshold = 10.0  # meter
        self._state = AgentState.NAVIGATING
        self._hop_resolution = 0.2
        self._path_seperation_hop = 2
        self._path_seperation_threshold = 0.5
        self._grp = None  # global route planar
        # Behavior planning
        self._hazard_detected = False
        self._blocked_time = None
        self._perform_lane_change = False
        self._front_r = []
        self._left_front_r = []
        self._left_back_r = []

    # Update personalized parameters from model
    def update_parameters(self):
        self._THW = self._model.get_parameter("safe_distance")["THW"]
        self._target_speed = self._model.get_parameter("target_speed")
        self._sin_param = self._model.get_parameter("sin_param")
        self._poly_param = self._model.get_parameter("poly_param")

        CONTROLLER_TYPE = 'PID' # options:MPC, PID, STANLEY
        args_lateral_dict = {'K_P': 1.0, 'K_I': 0.4, 'K_D': 0.01, 'control_type': CONTROLLER_TYPE}
        args_longitudinal_dict = {'K_P': 0.3, 'K_I': 0.2, 'K_D': 0.002}
        self._local_planner.init_controller(opt_dict={'target_speed': self._target_speed,
                                                      'lateral_control_dict': args_lateral_dict,
                                                      'longitudinal_control_dict': args_longitudinal_dict})

    # Start learning by collecting data
    def collect(self):
        # State for each step
        personalization_param = []

        # Time stamp
        personalization_param.extend([pygame.time.get_ticks()])

        # Collect vehicle position
        t = self._vehicle.get_transform()
        personalization_param.extend([t.location.x,
                                      t.location.y,
                                      t.location.z,
                                      t.rotation.yaw])

        # Collect vehicle velocity and speed
        v = self._vehicle.get_velocity()
        personalization_param.extend([v.x, v.y, v.z, self._get_speed()])                    

        # Collect radar information
        front_dis = 100
        front_vel = 50
        left_front_dis = 100
        left_front_vel = 50
        left_back_dis = -100
        left_back_vel = 0
        if self._front_r:
            front_dis = self._front_r[1][0]
            front_vel = self._front_r[2][0]
        if self._left_front_r:
            left_front_dis = self._left_front_r[1][0]
            left_front_vel = self._left_front_r[2][0]
        if self._left_back_r:
            left_back_dis = self._left_back_r[1][0]
            left_back_vel = self._left_back_r[2][0]
        personalization_param.extend([front_dis, left_front_dis, left_back_dis, 
                                      front_vel, left_front_vel, left_back_vel])

        self._model.collect(personalization_param)

    # End collection
    def end_collect(self):
        self._model.end_collect()

    # Train model
    def train_model(self):
        self._model.train_new_model()

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

    # Get vehicle speed
    def _get_speed(self):
        v = self._vehicle.get_velocity()
        ego_speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        return ego_speed

    # Run step
    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """
        ## Update Environment ##
        # Check all the radars
        if self._world_obj.front_radar.detected:
            if abs(self._world_obj.front_radar.rel_pos[1]) < 1:
                self._front_r = [pygame.time.get_ticks(), self._world_obj.front_radar.rel_pos, 
                                                        self._world_obj.front_radar.rel_vel]
            self._world_obj.front_radar.detected = False                                        
        if self._world_obj.left_front_radar.detected:
            if self._world_obj.left_front_radar.rel_pos[1] < -1:
                self._left_front_r =[pygame.time.get_ticks(), self._world_obj.left_front_radar.rel_pos, 
                                                            self._world_obj.left_front_radar.rel_vel]
            self._world_obj.left_front_radar.detected = False
        if self._world_obj.left_back_radar.detected:
            if self._world_obj.left_back_radar.rel_pos[1] < -1:
                self._left_back_r = [pygame.time.get_ticks(), self._world_obj.left_back_radar.rel_pos, 
                                                            self._world_obj.left_back_radar.rel_vel]
            self._world_obj.left_back_radar.detected = False
        # Remove radar data if not detected again in 0.5 second
        if self._front_r and (pygame.time.get_ticks() - self._front_r[0] > 5000):
            self._front_r = []
        if self._left_front_r and (pygame.time.get_ticks() - self._left_front_r[0] > 5000):
            self._left_front_r = []
        if self._left_back_r and (pygame.time.get_ticks() - self._left_back_r[0] > 5000):
            self._left_back_r = []
        
        # Detect vehicles in front
        self._hazard_detected = False
        if self._front_r and (self._front_r[1][0] < 20.0):
            self._hazard_detected = True
        # update hazard existing time
        if self._hazard_detected:
            if self._blocked_time is None:
                self._blocked_time = pygame.time.get_ticks()
                hazard_time = 0
            else:
                hazard_time = pygame.time.get_ticks() - self._blocked_time
        else:
            self._blocked_time = None

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
        #print(self._state)

        # Finite State Machine
        # 1, Navigating
        if self._state == AgentState.NAVIGATING:
            if self._hazard_detected:
                self._state = AgentState.BLOCKED_BY_VEHICLE

        # 2, Blocked by Vehicle
        elif self._state == AgentState.BLOCKED_BY_VEHICLE:
            if not self._hazard_detected:
                self._state = AgentState.NAVIGATING
            # The vehicle is driving at a certain speed
            # There is enough space
            else:
                if hazard_time > 5000 and \
                    190 > self._vehicle.get_location().x > 10 and \
                    10 > self._vehicle.get_location().y > 7:
                    self._state = AgentState.PREPARE_LANE_CHANGING

        # 4, Prepare Lane Change
        elif self._state == AgentState.PREPARE_LANE_CHANGING:
            safe_distance = self._THW * self._get_speed()
            if  not (self._front_r and self._front_r[1][0] < safe_distance) and \
                not (self._left_front_r and self._left_front_r[1][0] < safe_distance) and \
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

        # 6, Emergency Brake
        emergency_distance = 5.0
        if self._front_r and self._front_r[1][0] < emergency_distance:
            self._state = AgentState.EMERGENCY_BRAKE


        # Local Planner Behavior according to states
        if self._state == AgentState.NAVIGATING or self._state == AgentState.LANE_CHANGING:
            control = self._local_planner.run_step(debug=debug)

        elif self._state == AgentState.PREPARE_LANE_CHANGING:
            safe_distance = self._THW * self._get_speed()
            if self._left_front_r and self._left_front_r[1][0] < safe_distance or \
               self._front_r and self._front_r[1][0] < safe_distance:
                control = self._local_planner.empty_control(debug=debug)
            else:
                control = self._local_planner.run_step(debug=debug)

        elif self._state == AgentState.BLOCKED_BY_VEHICLE:
            # ACC
            front_dis = self._front_r[1][0]
            front_vel = self._front_r[2][0]
            ego_speed = self._get_speed()
            desired_speed = front_vel - (ego_speed-front_vel)/front_dis
            if ego_speed > 1:
                desired_speed += 2*(front_dis/ego_speed - self._THW)
            control = self._local_planner.run_step(debug=debug, target_speed=desired_speed*3.6)

        elif self._state == AgentState.EMERGENCY_BRAKE:
            control = self._local_planner.soft_stop()
            if self._front_r[1][0] >= emergency_distance:
                self._state = AgentState.NAVIGATING

        elif self._state == AgentState.BLOCKED_RED_LIGHT:
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
            
            wait_dist = 0.0  # need some time to plan
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
