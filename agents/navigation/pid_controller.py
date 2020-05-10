#!/usr/bin/env python

import carla

import math
import numpy as np
from collections import deque
from agents.tools.misc import get_speed

from casadi import *
import time

class VehiclePIDController:
    """
    VehiclePIDController is the combination of two PID controllers (lateral and longitudinal)
    """

    def __init__(self, vehicle, args_lateral=None, args_longitudinal=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        :param args_longitudinal: dictionary of arguments to set the longitudinal PID controller
        """
        if not args_lateral:
            args_lateral = {'K_P': 0.4, 'K_I': 0.2, 'K_D': 0.4, 'dt': 0.05, 'control_type': 'PID'}
        if not args_longitudinal:
            args_longitudinal = {'K_P': 1.0, 'K_I': 0.2, 'K_D': 0.6, 'dt': 0.05}

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._lon_controller = PIDLongitudinalController(self._vehicle, **args_longitudinal)
        self._lat_controller = PIDLateralController(self._vehicle, **args_lateral)

    def run_step(self, target_speed, waypoints, target_waypoint, current_waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal PID controllers to reach a target waypoint
        at a given target_speed.

        :param target_speed: desired vehicle speed
        :param waypoint: target location encoded as a waypoint
        :return: Carla.VehicleControl() instance
        """
        throttle = self._lon_controller.run_step(target_speed)
        steering = self._lat_controller.run_step(waypoints, target_waypoint, current_waypoint)

        # throttle, steering = self._mpc.run_step(target_speed, waypoints)        

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = 0.0
        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudinalController:
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    Speed longitudinal controller (Position longitudinal controller preferred)
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.5, K_I=0.5, dt=0.05):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=30)

    def run_step(self, target_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

        :param target_speed: target speed in Km/h
        :return: throttle control in the range [0, 1]
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)


    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
                 when it is [-1, 0], it becomes brake control
        """
        # speed error
        _e = (target_speed - current_speed)
        self._e_buffer.append(_e)

        # d, i term of error
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0
        # control signal
        return np.clip((self._K_P * _e) + (self._K_D * _de / self._dt) + (self._K_I * _ie * self._dt), 0.0, 1.0)


class PIDLateralController:
    """
    PIDLateralController implements lateral control using a PID.
    Heading lateral controller (Stanley lateral controller preferred)
    """

    def __init__(self, vehicle, K_P=0.5, K_D=0.5, K_I=0.2, dt=0.05, control_type='PID'):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=10)
        self._control_type = control_type 

    def run_step(self, waypoints, target_waypoint, current_waypoint):
        """
        Execute one step of lateral control to steer the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control in the range [-1, 1] where:
            -1 represent maximum steering to left
            +1 maximum steering to right
        """

        if self._control_type=='PID':
            return self._pid_control(target_waypoint, self._vehicle.get_transform())
        else:    
            return self._stanley_control(target_waypoint, current_waypoint, self._vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """
        # print(" ")
        # print("================= PID Control ======================")

        v_begin = vehicle_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))
        
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x -
                          v_begin.x, waypoint.transform.location.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        # _dot should range from -pi to pi
        if _dot > 1.5708:
            _dot = -(math.pi - _dot)
        elif _dot < -1.5708:
            _dot = math.pi + _dot

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._K_P * _dot) + (self._K_D * _de /
                                             self._dt) + (self._K_I * _ie * self._dt), -1.0, 1.0)

    def _stanley_control(self, target_waypoint, current_waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param vehicle_transform: current transform of the vehicle
        :return: steering control in the range [-1, 1]
        """

        # heading error
        # print(" ")
        # print("================= Stanley ======================")
        yaw_path = np.arctan2(target_waypoint.transform.location.y-current_waypoint.transform.location.y, target_waypoint.transform.location.x - current_waypoint.transform.location.x)
        
        v_begin = vehicle_transform.location

        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        # vehicle heading vector
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        
        yaw_vehicle = np.arctan2(v_vec[1], v_vec[0])

        yaw_diff = yaw_path - yaw_vehicle

        # Wrapping the yaw_diff
        if yaw_diff > np.pi:
            yaw_diff -= 2 * np.pi
        if yaw_diff < - np.pi:
            yaw_diff += 2 * np.pi

        # Calculate cross-track error
        cross_err_current = (v_begin.x - current_waypoint.transform.location.x)**2 +  (v_begin.y - current_waypoint.transform.location.y)**2 
        cross_err_target = (v_begin.x - target_waypoint.transform.location.x)**2 +  (v_begin.y - target_waypoint.transform.location.y)**2 
        crosstrack_error = np.min([cross_err_current, cross_err_target]) 


        yaw_cross_track = np.arctan2(v_begin.y-target_waypoint.transform.location.y, v_begin.x-target_waypoint.transform.location.x)

        yaw_path2ct = yaw_path - yaw_cross_track

        if yaw_path2ct > np.pi:
            yaw_path2ct -= 2 * np.pi
        if yaw_path2ct < - np.pi:
            yaw_path2ct += 2 * np.pi
        if yaw_path2ct > 0:
            crosstrack_error = abs(crosstrack_error)
        else:
            crosstrack_error = -abs(crosstrack_error)

        v = get_speed(self._vehicle)

        k_e = 3
        k_v = 1

        #print("crosstrack_error: ", crosstrack_error)

        yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (k_v + v))

        steer_expect = yaw_diff + yaw_diff_crosstrack
        steer_expect = min(2, steer_expect)
        steer_expect = max(-2, steer_expect)

        if steer_expect > np.pi:
            steer_expect -= 2 * np.pi
        if steer_expect < - np.pi:
            steer_expect += 2 * np.pi
        #print("steer expect: ", steer_expect)
        
        return steer_expect
