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
        x = 0 
        y = 0 
        psi = 0 
        v = get_speed(self._vehicle)
        cte = self.get_cross_track_error([x,y], waypoints_car) 
        epsi = self.get_epsi(vehicle_transform, waypoints_car) 

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
        vars_init[x_start] = x 
        vars_init[y_start] = y 
        vars_init[psi_start] = psi 
        vars_init[v_start] = v 
        vars_init[cte_start] = cte 
        vars_init[epsi_start] = epsi 
        
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
        
        '''
        print("================= MPC ======================")
        print("steer_output: ", steer_output[0])
        print("throttle_output: ", throttle_output[0])
        print("brake_output: ", brake_output[0])
        print("============================================")
        print("   ")
        '''

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

            g[x_start + i] = x1 - (x0 + v0 * cos(psi0) * dt) 
            g[y_start + i] = y1 - (y0 + v0 * sin(psi0) * dt) 
            g[psi_start + i] = psi1 - (psi0 + (v0/Lf) * delta0 * dt) 
            g[v_start + i] = v1 - (v0 + a0 * dt) 
            g[cte_start + i] = cte1 - ((f0 - y0) + (v0 * sin(epsi0) * dt)) 
            g[epsi_start + i] = epsi1 - ((psi0 - psides0) + (v0/Lf) * delta0 * dt) 
            
        return f, g   
