#!/usr/bin/env python
# coding=utf-8

import carla

import pickle
from os import path
from os.path import dirname, abspath

from scipy.interpolate import splrep
from scipy.stats import norm
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt
import numpy as np
import math
from agents.tools.misc import get_poly_y, transform_to_frame
from GMM import GMM


# A Model class to store personalized parameters
class Model:
    def __init__(self):
        # Path to store model
        self._data_folder = path.join(dirname(dirname(dirname(abspath(__file__)))), "data")
        self._model_path = path.join(self._data_folder, "model.pickle")
        # Model
        self._model = None
        self.load_model()

        # Learning data container
        self._state_list = []

    def load_model(self):
        # if file doesn't exist, create one
        if not path.exists(self._model_path):
            # Default model
            model = {"safe_distance": {"THW": 2.0}, 
                     "target_speed": 28.0,
                     "poly_param": {"lon_dis": 30.0, "lat_dis": -3.5, "dt": 4.0,
                                    "lon_param": np.array([0.0, 7.0, 0.0, 0.3125, -0.1172, 0.0117]),
                                    "lat_param": np.array([0.0, 0.0, 0.0, -0.5469, 0.2051, -0.0205])},
                     "sin_param": {"lon_vel": 5.0, "lat_dis": -3.5, "dt": 4.0}}
            with open(self._model_path, 'wb') as f:
                pickle.dump(model, f)

        # load model from file
        with open(self._model_path, 'rb') as f:
            model = pickle.load(f)
        self._model = model

    # Collect data and 
    def collect(self, param):
        self._state_list.append(param)
    
    # End collecting and save data in files
    def end_collect(self):
        self.update_target_speed()
        self.update_safe_distance()
        self.update_sin_param()
        self.update_poly_param()
        
        # Temporary
        self._state_list.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.save_data(np.array(self._state_list), "states.csv")

    # Save data in a file
    def save_data(self, data, file_name):
        train_path = path.join(self._data_folder, file_name)
        data = np.atleast_2d(data)
        if path.exists(train_path):
            with open(train_path, 'rb') as f:
                train_data = np.loadtxt(f, delimiter=",")
                train_data = np.atleast_2d(train_data)
                train_data = np.append(train_data, data, axis=0)
        else:
            train_data = data

        with open(train_path, 'wb') as f:
            np.savetxt(f, train_data, delimiter=",")
        
    # Load data from a file
    def load_data(self, file_name):
        train_path = path.join(self._data_folder, file_name)
        train_data = None
        if path.exists(train_path):
            with open(train_path, 'rb') as f:
                train_data = np.loadtxt(f, delimiter=",")
                train_data = np.atleast_2d(train_data)
        else:
            print("%s does not exist."% file_name)

        return train_data

    # Return a certain parameter value
    def get_parameter(self, keyword):
        return self._model[keyword]

    @staticmethod
    # Return points indicating lane changing process
    def get_lane_changing_points(points):
        points = np.atleast_2d(points)
        t_ref, x_ref, y_ref, z_ref, yaw_ref = points[0,:]

        # In matrix form
        points_m = np.transpose(points)

        # Transform back to (0, 0)
        ref_frame = carla.Transform(carla.Location(x=x_ref, y=y_ref, z=z_ref),
                                    carla.Rotation(yaw=yaw_ref, pitch=0, roll=0))
        points_v = transform_to_frame(ref_frame, points_m[1:4, :])

        # Get points (x, t) and (y, t)
        t_v = points_m[0,:] / 1000.0
        x_v = points_v[0,:]
        y_v = points_v[1,:]
        z_v = points_v[2,:]
        
        # Find out which part is lane changing
        e_y = y_v[1:] - y_v[0:-1]
        index = e_y < -0.02  # Left
        start_index = -1
        x_v = x_v[1:][index]
        y_v = y_v[1:][index]
        z_v = z_v[1:][index]
        t_v = t_v[1:][index]

        if x_v.shape[0] > 0:
            x_v -= x_v[0]
            y_v -= y_v[0]
            z_v -= z_v[0]
            t_v -= t_v[0]
            for i, val in enumerate(index):
                if val:
                    start_index = i
                    break

        #Temp Not using z_v for now
        return x_v, y_v, t_v, start_index


    ## Learn from collected data to update value ##
    # Helper function for polynomial
    @staticmethod
    def get_2d_poly_param(lon_dis, lat_dis, dt):
        desired_lon_speed = lon_dis/dt

        # Calculate new polynomial parameters
        M = np.array([[1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 2, 0, 0, 0],
                    [1, dt, dt ** 2, dt ** 3, dt ** 4, dt ** 5],
                    [0, 1, 2 * dt, 3 * dt ** 2, 4 * dt ** 3, 5 * dt ** 4],
                    [0, 0, 2, 6 * dt, 12 * dt ** 2, 20 * dt ** 3]])
        state_lon = np.array([0, desired_lon_speed, 0, lon_dis, desired_lon_speed, 0])
        state_lat = np.array([0, 0, 0, lat_dis, 0, 0])

        lon_param = np.matmul(np.linalg.inv(M), state_lon)
        lat_param = np.matmul(np.linalg.inv(M), state_lat)

        return lon_param, lat_param
    
    def update_poly_param(self, debug=False):
        """
        1, Polynomial curve definition comes from:
        S. Ammoun, F. Nashashibi and C. Laurgeau,
        "An analysis of the lane changing manoeuvre on roads :
        the contribution of inter-vehicle cooperation via communication," 2007
        IEEE Intelligent Vehicles Symposium, Istanbul, 2007, pp. 1095-1100.
        2, Learning method comes from:
        Yao, Wen & Zhao, Huijing & Davoine, Franck & Zha, Hongbin. (2012).
        Learning lane change trajectories from on-road driving data.
        IEEE Intelligent Vehicles Symposium, Proceedings. 885-890. 10.1109/IVS.2012.6232190.
        """
        states = np.array(self._state_list)
        points = states[:,0:5]
        
        # Get points while lane changing
        x_v, y_v, t_v, start_index = self.get_lane_changing_points(points)

        if x_v.size > 20:
            # New lane changing parameters
            dt = t_v[-1] - t_v[0]
            lon_dis = x_v[-1] - x_v[0]
            lat_dis = y_v[-1] - y_v[0]

            if debug:
                lon_param, lat_param = self.get_2d_poly_param(lon_dis, lat_dis, dt)
                plt.figure(figsize=(3, 9))
                plt.subplot(312)
                plt.xlabel('time (t)')
                plt.ylabel('Longitudinal (m)')
                t_after = np.linspace(0, dt, 100)
                x_after = get_poly_y(t_after, lon_param)
                plt.plot(t_after, x_after, color='g')
                plt.scatter(t_v, x_v, marker='x', color='k', s=5)
                plt.subplot(313)
                plt.xlabel('time (t)')
                plt.ylabel('Lateral (m)')
                y_after = get_poly_y(t_after, lat_param)
                plt.plot(t_after, y_after, color='g')
                plt.scatter(t_v, y_v, marker='x', color='k', s=5)
                plt.subplot(311)
                plt.xlabel('Longitudinal (m)')
                plt.ylabel('Lateral (m)')
                plt.plot(x_after, y_after, color='g')
                plt.scatter(x_v, y_v, marker='x', color='k', s=5)
                plt.show()

            # Save data
            self.save_data(np.array([lon_dis, lat_dis, dt]), "poly_train_data.csv")
            print("Polynomial Parameters Data Saved")
        else:
            print("No availiable polynomial data")

    # Train polynomial parameters
    def train_poly_param(self):
        data = self.load_data("GMM_train_data.csv")
        if data is not None and data.size != 0:
            lon_dis = np.mean(data[:,0])
            lat_dis = np.mean(data[:,1])
            dt = np.mean(data[:,2])
            lon_param, lat_param = self.get_2d_poly_param(lon_dis, lat_dis, dt)

            # Update model
            poly_param = {"lat_dis": lat_dis, "lon_dis": lon_dis, "dt": dt,
                        "lat_param": lat_param, "lon_param": lon_param}
            self._model["poly_param"] = poly_param
            
            print("New lon_dis: %f, lat_dis: %f, dt: %f"% (lon_dis, lat_dis, dt))
            print("Polynomial Parameters Updated")
        else:
            print("No poly parameters data")


    def update_sin_param(self, debug=True):
        """
        Sinusoidal Method comes from:
        V. A. Butakov and P. Ioannou,
        "Personalized Driver/Vehicle Lane Change Models for ADAS,"
        in IEEE Transactions on Vehicular Technology, vol. 64, no. 10, pp. 4422-4431, Oct. 2015.
        """
        states = np.array(self._state_list)
        points = states[:,0:5]
        
        # Get points while lane changing
        x_v, y_v, t_v, start_index = self.get_lane_changing_points(points)

        if x_v.size > 20:
            # New lane changing parameters
            new_dt = t_v[-1] - t_v[0]
            new_lon_dis = x_v[-1] - x_v[0]
            new_lat_dis = y_v[-1] - y_v[0]

            # For GMM
            # velocity of ego
            V = self._state_list[start_index][8]
            # lateral distance
            H = new_lat_dis
            # get radar information when lane change happens
            DL = self._state_list[start_index][10]
            DF = self._state_list[start_index][11]

            if debug:
                t_after = np.linspace(0, new_dt, 100)
                plt.figure(figsize=(3, 9))
                plt.subplot(312)
                plt.xlabel('time (t)')
                plt.ylabel('Longitudinal (m)')
                x_after = np.linspace(0, new_lon_dis, 100)
                plt.plot(t_after, x_after, color='g')
                plt.scatter(t_v, x_v, marker='x', color='k', s=5)
                plt.subplot(313)
                plt.xlabel('time (t)')
                plt.ylabel('Lateral (m)')
                y_after = -new_lat_dis/(2*math.pi) * np.sin(2*math.pi * t_after/new_dt) + \
                            new_lat_dis * t_after/new_dt
                plt.plot(t_after, y_after, color='g')
                plt.scatter(t_v, y_v, marker='x', color='k', s=5)
                plt.subplot(311)
                plt.xlabel('Longitudinal (m)')
                plt.ylabel('Lateral (m)')
                plt.plot(x_after, y_after, color='g')
                plt.scatter(x_v, y_v, marker='x', color='k', s=5)
                plt.show()

            # Save data for GMM
            self.save_data(np.array([V, H, DL, DF, new_dt, new_lon_dis/new_dt]), "GMM_train_data.csv")
            print("Sin Parameters Data Saved")
        else:
            print("No availiable sin data")

    # Train sin parameters
    def train_sin_param(self):
        data = self.load_data("GMM_train_data.csv")
        if data is not None and data.size != 0:
            gmm = GMM()
            gmm.train(data[:,0:5])

            if gmm.GMM_model is not None:
                # Update GMM model
                gmm.save_model()

                # Update longitudinal distance
                long_vel = np.mean(data[:,5]/data[:,4]) * 3.0
                self._model["sin_param"]["lon_vel"] = long_vel
                
                print("New Longitudinal Velocity: %f"% long_vel)
                print("Sinusoidal Parameters Updated")
            else:
                print("Training GMM failed")
        else:
            print("No sin parameters data")


    def update_safe_distance(self, debug=False):
        """
        Personalized ACC Method comes from:
        J. Wang, L. Zhang, D. Zhang and K. Li, 
        "An Adaptive Longitudinal Driving Assistance System Based on Driver Characteristics," 
        in IEEE Transactions on Intelligent Transportation Systems, vol. 14, no. 1, pp. 1-12, 
        March 2013, doi: 10.1109/TITS.2012.2205143.
        """
        # Get velocity and acceleration of the ego vehicle
        states = np.array(self._state_list)
        time_stamp = states[:,0]/1000.0
        speed = states[:,8]
        acceleration = (speed[1:]-speed[0:-1]) / (time_stamp[1:]-time_stamp[0:-1])
        time_stamp = time_stamp[1:]
        speed = speed[1:]
        # Get distance and velocity of the lead vehicle
        front_distance = states[1:,9]
        front_speed = states[1:,12]
        rel_speed = speed - front_speed
        
        # get speed while there is a vehicle ahead
        index = (front_distance != 100) & (speed != 0) & (front_distance != 0)
        # remove approaching state
        current_index = 0
        current_time = time_stamp[0]
        accelrating_flag = acceleration[0] > 0
        for i in range(time_stamp.size):
            if acceleration[i] > 0:
                if not accelrating_flag:
                    accelrating_flag = True
                    current_time = time_stamp[i]
                    current_index = i
            else:
                if accelrating_flag:
                    accelrating_flag = False
                    if time_stamp[i] - current_time > 3.0:
                        index[current_index:i] = False

        time_stamp = time_stamp[index]
        speed = speed[index]
        rel_speed = rel_speed[index]
        acceleration = acceleration[index]
        front_distance = front_distance[index]

        # Compute THW and TTCi
        if speed.size > 50:
            THW = front_distance / speed
            TTCi = rel_speed / front_distance

            # Select stable vehicle following
            steady_index = abs(TTCi) < 0.05
            THW = THW[steady_index]
            TTCi = TTCi[steady_index]

            # get new value
            THW_mean, THW_std = norm.fit(THW)
            THW_covar = THW_std**2

            if debug:
                fig = plt.figure()
                ax1 = fig.add_subplot(221)
                ax1.hist(THW, bins=THW.size/3, ec='red', alpha=0.5, density=True)
                THW = np.sort(THW)
                x = np.linspace(THW[0], THW[-1], THW.size).reshape(-1,1)
                y = norm.pdf(x, THW_mean, THW_std).ravel()
                ax1.plot(x, y, c='red')
                ax2 = fig.add_subplot(222)
                ax2.hist(TTCi, bins=TTCi.size/3, ec='red', alpha=0.5)
                ax3 = fig.add_subplot(212)
                ax3.scatter(time_stamp, speed, c='r')
                ax3.scatter(time_stamp, rel_speed, c='g')
                ax3.scatter(time_stamp, acceleration, c='b')
                ax3.scatter(time_stamp, front_distance, c='k')
                
                plt.show()
            
            self.save_data(np.array([THW_mean, THW_covar]), "safe_distance_train_data.csv")
            print("Safe Distance Data Saved")
        else:
            print("No availiable safe distance data")

    def train_safe_distance(self):
        data = self.load_data("safe_distance_train_data.csv")
        if data is not None and data.size != 0:
            # Update THW
            means = data[:, 0]
            covars = data[:, 1]
            target_THW = means[0]
            THW_covar = covars[0]

            for i in range(means.size-1):
                u1 = target_THW
                s1 = THW_covar
                u2 = means[i+1]
                s2 = covars[i+1]

                target_THW = (s1*u2+s2*u1) / (s1+s2)
                THW_covar = s1*s2 / (s1+s2)

            self._model["target_speed"] = {"THW": target_THW}
            print("Safe Distance Trained")
            print("New THW: %f"% target_THW)
        else:
            print("No safe distance data")


    def update_target_speed(self, debug=False):
        # Get acceleration and speed
        states = np.array(self._state_list)
        time_stamp = states[:,0]
        speed = states[:,8]

        acceleration = (speed[1:]-speed[0:-1]) / ((time_stamp[1:]-time_stamp[0:-1])/1000.0)
        time_stamp = time_stamp[1:]
        speed = speed[1:]

        # get speed while it is stable and no vehicle ahead
        index = (abs(acceleration) < 1) & (states[1:, 9] == 100)
        stable_speed = speed[index]
        # select half of the highest speed
        stable_speed = np.sort(stable_speed)[::-1]
        stable_speed = stable_speed[0:stable_speed.size/4]

        if stable_speed.size > 50:
            gmm = GaussianMixture(n_components=3, covariance_type='diag')
            result = gmm.fit(stable_speed[:, np.newaxis])
            weights, means, covars = result.weights_, result.means_, result.covariances_

            # Select the gaussian distribution with the maximum mean
            max_index = np.argmax(means)
            mean, covar = means[max_index][0], covars[max_index][0]

            if debug:
                plt.hist(stable_speed, bins = stable_speed.size, ec='red', alpha=0.5, density=True)
                x = np.linspace(stable_speed[0], stable_speed[-1], stable_speed.size).reshape(-1,1)
                y1 = weights[0] * norm.pdf(x, means[0], np.sqrt(covars[0])).ravel()
                y2 = weights[1] * norm.pdf(x, means[1], np.sqrt(covars[1])).ravel()
                y3 = weights[2] * norm.pdf(x, means[2], np.sqrt(covars[2])).ravel()
                plt.plot(x, y1+y2+y3, c='black')
                plt.plot(x, y1, c='red')
                plt.plot(x, y2, c='green')
                plt.plot(x, y3, c='blue')
                plt.show()

            # get new value
            self.save_data(np.array([mean, covar]), "target_speed_train_data.csv")
            print("Target Speed Data Saved")
        else:
            print("No availiable speed data")

    # Train target speed
    def train_target_speed(self):
        # Train target speed
        data = self.load_data("target_speed_train_data.csv")
        if data is not None and data.size != 0:
            # Update mean and covariance
            means = data[:, 0]
            covars = data[:, 1]
            target_speed = means[0]
            speed_covar = covars[0]

            for i in range(means.size-1):
                u1 = target_speed
                s1 = speed_covar
                u2 = means[i+1]
                s2 = covars[i+1]

                target_speed = (s1*u2+s2*u1) / (s1+s2)
                speed_covar = s1*s2 / (s1+s2)

            self._model["target_speed"] = target_speed
            print("Target Speed Trained")
            print("New Target Speed: %f"% target_speed)
        else:
            print("No target speed data")


    # Train the model according to saved driver's data
    def train_new_model(self):
        self.train_target_speed()
        self.train_safe_distance()
        self.train_sin_param()
        self.train_poly_param()
        self.store_new_model()

    # Store learned result
    def store_new_model(self):
        with open(self._model_path, 'wb') as f:
            pickle.dump(self._model, f)
