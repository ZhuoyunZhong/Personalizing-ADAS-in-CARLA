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
        self._control_type =control_type 

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
        print(" ")
        print("================= PID Control ======================")

        v_begin = vehicle_transform.location

        print("v_begin:", v_begin)
        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))
        print("v_end:", v_end)
        
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([waypoint.transform.location.x -
                          v_begin.x, waypoint.transform.location.y -
                          v_begin.y, 0.0])
        
        print("waypoint:", waypoint)
        print("v_vec:", v_vec)
        print("w_vec:", w_vec)

        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))

        print("_dot:", _dot)

        _cross = np.cross(v_vec, w_vec)

        print("_cross:", _cross)

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
        print(" ")
        print("================= Stanley ======================")
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

class MPC:
    
    def __init__(self, vehicle):
        self._vehicle = vehicle

    def run_step(self, target_speed, waypoints, target_waypoint = None, current_waypoint = None):
        throttle, brake, steering = self._model_predictive_control(target_speed, waypoints, self._vehicle.get_transform())        

        control = carla.VehicleControl()
        control.steer = steering[0]
        control.throttle = throttle[0]
        control.brake = 0
        control.hand_brake = False
        control.manual_gear_shift = False
        return control

    def get_cross_track_error(self, current_xy, waypoints):
        
        squared_terms = (current_xy - waypoints[:, :2])**2
        crosstrack_error = np.min(squared_terms[:, 0] + squared_terms[:, 1])

        yaw_cross_track = np.arctan2(current_xy[1] - waypoints[0][1], current_xy[0] - waypoints[0][0])
        yaw_path = np.arctan2(waypoints[-1][1] - waypoints[0][1], waypoints[-1][0] - waypoints[0][0])
        
        yaw_path2ct = yaw_path - yaw_cross_track
               
        if yaw_path2ct > np.pi:
            yaw_path2ct -= 2 * np.pi
        if yaw_path2ct < - np.pi:
            yaw_path2ct += 2 * np.pi
        if yaw_path2ct > 0:
            crosstrack_error = abs(crosstrack_error)
        else:
            crosstrack_error = - abs(crosstrack_error)

        return crosstrack_error

    def get_psi(self, vehicle_transform):

        v_begin = vehicle_transform.location

        v_end = v_begin + carla.Location(x=math.cos(math.radians(vehicle_transform.rotation.yaw)),
                                         y=math.sin(math.radians(vehicle_transform.rotation.yaw)))

        # vehicle heading vector
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        
        yaw_vehicle = np.arctan2(v_vec[1], v_vec[0])

        return yaw_vehicle

    def get_epsi(self, vehicle_transform, waypoints):
        # heading error
        yaw_path = np.arctan2(waypoints[-1][1] - waypoints[0][1], waypoints[-1][0] - waypoints[0][0])
        
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

        return yaw_diff    

    def get_coeffs(self, waypoints_xy):

        x = np.array(waypoints_xy)[:,0]
        y = np.array(waypoints_xy)[:,1]

        coeffs = np.flip(np.polyfit(x, y, 3))

        return coeffs

    def _model_predictive_control(self, target_speed, waypoints, vehicle_transform):

        # Transform points from world frame to car frame
        _x = vehicle_transform.location.x
        _y = vehicle_transform.location.y
        
        # _psi = vehicle_transform.rotation.yaw
        _psi = self.get_psi(vehicle_transform)

        R = np.array([[cos(-_psi), -sin(-_psi)],[sin(-_psi), cos(-_psi)]])
        t = np.array([[_x],[_y]])

        waypoints_world = np.array(waypoints)
        waypoints_car = np.array(waypoints)

        waypoints_car[:, 0:2] = np.dot(R, np.array([waypoints_world[:, 0], waypoints_world[:, 1]]) - t).T

        N = 10
        dt = 0.1
        Lf = 2.67
        ref_v = target_speed

        # Define var start positions 
        x_start = 0
        y_start = x_start + N
        psi_start = y_start + N
        v_start = psi_start + N
        cte_start = v_start + N
        epsi_start = cte_start + N
        delta_start = epsi_start + N
        a_start = delta_start + N - 1


        # State
        x = 0;
        y = 0;
        psi = 0;
        v = get_speed(self._vehicle)
        cte = self.get_cross_track_error([x,y], waypoints_car);
        epsi = self.get_epsi(vehicle_transform, waypoints_car);

        coeffs = self.get_coeffs(waypoints_car)

        # number of model variables 
        # For example: If the [state] is a 4 element vector, the [actuators] is a 2   
        # element vector and there are 10 timesteps. The number of variables is:
        n_vars = N*6 + (N-1)*2
        
        # Set the number of constraints
        n_constraints = N*6
        
        # NLP variable vector
        vars =  MX.sym('x',n_vars)
        vars_init = np.zeros(n_vars)
        
        # set initial variables values
        vars_init[x_start] = x;
        vars_init[y_start] = y;
        vars_init[psi_start] = psi;
        vars_init[v_start] = v;
        vars_init[cte_start] = cte;
        vars_init[epsi_start] = epsi;
        
        # upperbound and lowerbound vectors for vars
        vars_upperbound = np.zeros(n_vars)
        vars_lowerbound = np.zeros(n_vars)
        
        # Set all non-actuators upper and lowerlimits
        # to the max negative and positive values.
        vars_upperbound[:delta_start] =  1.0e9
        vars_lowerbound[:delta_start] = -1.0e9
        
        # Set the upper and lower limits of delta as -25 and 25 degrees
        vars_upperbound[delta_start:a_start] =  0.7
        vars_lowerbound[delta_start:a_start] = -0.7
        
        # Set the upper and lower limits of accelerations as -1 and 1
        vars_upperbound[a_start:] =  0.7
        vars_lowerbound[a_start:] = -0.7

        # Lower and upper limits for the constraints 
        # Should be 0 besides initial state.
        constraints_upperbound = np.zeros(n_constraints)
        constraints_lowerbound = np.zeros(n_constraints)
        
        constraints_lowerbound[x_start] = x
        constraints_lowerbound[y_start] = y
        constraints_lowerbound[psi_start] = psi
        constraints_lowerbound[v_start] = v
        constraints_lowerbound[cte_start] = cte
        constraints_lowerbound[epsi_start] = epsi
        
        constraints_upperbound[x_start] = x
        constraints_upperbound[y_start] = y
        constraints_upperbound[psi_start] = psi
        constraints_upperbound[v_start] = v
        constraints_upperbound[cte_start] = cte
        constraints_upperbound[epsi_start] = epsi
        
        # Object for defining objective and constraints
        f, g = self.operator(vars, coeffs, n_constraints, N, dt, ref_v, Lf)

        # NLP
        nlp = {'x':vars, 'f':f, 'g':vertcat(*g)}
        # print("g shape:", vertcat(*g).shape) 
        
        ## ----
        ## SOLVE THE NLP
        ## ----

        # Set options
        opts = {}
        opts["expand"] = True
        #opts["ipopt.max_iter"] = 4
        opts["ipopt.linear_solver"] = 'ma27'
        opts["ipopt.print_level"] = 0

        # Allocate an NLP solver
        solver = nlpsol("solver", "ipopt", nlp, opts)
        arg = {}

        # Initial condition
        arg["x0"] = vars_init
        # print("x0 shape: ", vars_init.shape)
        
        # Bounds on x
        arg["lbx"] = vars_lowerbound
        arg["ubx"] = vars_upperbound
        # print("lbx shape: ", vars_lowerbound.shape)
        
        # Bounds on g
        arg["lbg"] = constraints_lowerbound
        arg["ubg"] = constraints_upperbound
        # print("ubg: ", constraints_upperbound.shape)

        # Solve the problem
        res = solver(**arg)
        vars_opt = np.array(res["x"])

        x_mpc = vars_opt[x_start:y_start]
        y_mpc = vars_opt[y_start:psi_start]

        steering = vars_opt[delta_start:a_start]
        accelerations = vars_opt[a_start:]

        # print("steering: ", steering)
        # print("accelerations: ", accelerations)
        
        throttle_output = accelerations*0
        brake_output = accelerations*0

        for i in range(N-1):
            if accelerations[i]>0:
                throttle_output[i] = accelerations[i] 
                brake_output[i] = 0
            
            else:   
                throttle_output[i] = 0
                brake_output[i] = -accelerations[i]

        steer_output = steering
        
        print("================= MPC ======================")
        print("steer_output: ", steer_output[0])
        print("throttle_output: ", throttle_output[0])
        print("brake_output: ", brake_output[0])
        print("============================================")
        print("   ")

        return throttle_output[0], brake_output[0], steer_output[0]
    
    def operator(self, vars, coeffs, n_constraints, N, dt, ref_v, Lf):
        # Define var start positions 
        x_start = 0
        y_start = x_start + N
        psi_start = y_start + N
        v_start = psi_start + N
        cte_start = v_start + N
        epsi_start = cte_start + N
        delta_start = epsi_start + N
        a_start = delta_start + N - 1


        # fg = np.zeros(self.n_vars)
        f = MX.zeros(1)
        g = [0]*n_constraints
        
        # Add Cross track error, Heading error and velocity error to Cost Function
        for i in range(N):
            f[0] += 10000*vars[cte_start + i]**2
            f[0] += 10000*(vars[epsi_start + i])**2        ## 15
            f[0] += 1000*(vars[v_start + i] - ref_v)**2  ## 10,000    
                    
        # Add control signal regulaization term to the Cost function
        for i in range(N-1):
            f[0] += 50*vars[delta_start + i]**2     # 100
            f[0] += 50*vars[a_start + i]**2     # 100
            # f[0] += 700*(vars[delta_start + i] * vars[v_start+i])**2
        
        # # Add cost for drastically changing controls 
        for i in range(N-2):
            f[0] += 250000*(vars[delta_start + i + 1] - vars[delta_start + i])**2
            f[0] += 200000*(vars[a_start + i + 1] - vars[a_start + i])**2
        
        # Add contraints
        g[x_start] = vars[x_start]
        g[y_start] = vars[y_start]
        g[psi_start] = vars[psi_start]
        g[v_start] = vars[v_start]
        g[cte_start] = vars[cte_start]
        g[epsi_start] = vars[epsi_start]

        for i in range(1,N):
            x1 = vars[x_start + i]
            y1 = vars[y_start + i]
            psi1 = vars[psi_start + i]
            v1 = vars[v_start + i]
            cte1 = vars[cte_start + i]
            epsi1 = vars[epsi_start + i]

            x0 = vars[x_start + i - 1]
            y0 = vars[y_start + i - 1]
            psi0 = vars[psi_start + i - 1]
            v0 = vars[v_start + i - 1]
            cte0 = vars[cte_start + i - 1]
            epsi0 = vars[epsi_start + i - 1]
            
            delta0 = vars[delta_start + i - 1]
            a0 = vars[a_start + i - 1]
                    
            f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0**2 + coeffs[3] * x0 **3
            psides0 = atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0**2)                

            g[x_start + i] = x1 - (x0 + v0 * cos(psi0) * dt);
            g[y_start + i] = y1 - (y0 + v0 * sin(psi0) * dt);
            g[psi_start + i] = psi1 - (psi0 + (v0/Lf) * delta0 * dt);
            g[v_start + i] = v1 - (v0 + a0 * dt);
            g[cte_start + i] = cte1 - ((f0 - y0) + (v0 * sin(epsi0) * dt));
            g[epsi_start + i] = epsi1 - ((psi0 - psides0) + (v0/Lf) * delta0 * dt);
            
        return f, g   


