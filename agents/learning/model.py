#!/usr/bin/env python

import pickle
from os import path
from os.path import dirname, abspath

import numpy as np
import math
import matplotlib.pyplot as plt


# A Model class to store personalized parameters
class Model:
    def __init__(self):
        # Path to store model
        self.path = path.join(dirname(dirname(dirname(abspath(__file__)))), "data/model.pickle")
        # Model
        self._model = None
        self.load_model()
        # Learning data container
        self._distance_list = []
        self._speed_list = []
        self._poly_points = []

    def load_model(self):
        # if file doesn't exist, create one
        if not path.exists(self.path):
            model = {"safe_distance": 12.0, "target_speed": 28.0,
                     "poly_param": {"param_lat": [-0.0205, 0.2051, -0.5469, 0.0, 0.0, 0.0],
                                    "param_lon": [0.0, 0.0, 0.0, 0.0, 20.0/3.6, 0.0], "dt": 4.0}}
            with open(self.path, 'wb') as f:
                pickle.dump(model, f)

        # load model from file
        with open(self.path, 'rb') as f:
            model = pickle.load(f)
        self._model = model

    # Collect data
    def collect(self, dict_param):
        if dict_param:
            if "speed" in dict_param:
                self._speed_list.append(dict_param["speed"])
            if "distance" in dict_param:
                self._distance_list.append(dict_param["distance"])
            if "poly_points" in dict_param:
                self._poly_points.append(dict_param["poly_points"])

    # Return a certain parameter value
    def get_parameter(self, keyword):
        return self._model[keyword]

    # Learn from collected data to update value
    def update_poly_param(self, debug=True):
        current_poly_param = self.get_parameter("poly_param")
        current_lon = current_poly_param["param_lon"]
        current_lat = current_poly_param["param_lat"]
        current_dt = current_poly_param["dt"]

        if len(self._poly_points) < 50:
            return

        # Transform back to (0, 0)
        x_ref, y_ref, yaw_ref, t_ref = self._poly_points[0]
        sy = math.sin(math.radians(yaw_ref))
        cy = math.cos(math.radians(yaw_ref))

        # Get points (x, t) and (y, t)
        point_list_x = []
        point_list_y = []
        point_list_t = []
        for point_x, point_y, _, point_t in self._poly_points:
            x = (cy * point_x + sy * point_y) + (-cy * x_ref - sy * y_ref)
            y = (-sy * point_x + cy * point_y) + (sy * x_ref - cy * y_ref)
            t = (point_t - t_ref) / 1e3
            point_list_x.append(x)
            point_list_y.append(y)
            point_list_t.append(t)
        x_v = np.array(point_list_x)
        y_v = np.array(point_list_y)
        t_v = np.array(point_list_t)

        # Find out which part is lane changing
        e_y = y_v[1:] - y_v[0:-1]
        index = e_y < -0.02  # Left
        x_v = x_v[1:][index]
        y_v = y_v[1:][index]
        t_v = t_v[1:][index]
        x_v -= x_v[0]
        y_v -= y_v[0]
        t_v -= t_v[0]
        # Time span of lane changing
        new_dt = t_v[-1] - t_v[0]
        dt = self.change_param(current_dt, new_dt)

        # Add some points to get a more smooth fit
        n_pts = 5
        x_speed = (x_v[-1] - x_v[-2]) / (t_v[-1] - t_v[-2])
        t_v = np.concatenate((np.linspace(t_v[0] - n_pts / 20.0, t_v[0], num=n_pts), t_v,
                              np.linspace(t_v[-1], t_v[-1] + n_pts / 20.0, num=n_pts)))
        x_v = np.concatenate((np.linspace(x_v[0] - n_pts / 20.0 * x_speed, x_v[0], num=n_pts), x_v,
                              np.linspace(x_v[-1], x_v[-1] + n_pts / 20.0 * x_speed, num=n_pts)))
        y_v = np.concatenate((np.linspace(y_v[0], y_v[0], num=n_pts), y_v,
                              np.linspace(y_v[-1], y_v[-1], num=n_pts)))

        # Generate 5 times more points using current poly curve
        t_curr = np.linspace(0.0, current_dt, 5*len(t_v))
        x_curr = self.get_poly_y(t_curr, current_lon)
        y_curr = self.get_poly_y(t_curr, current_lat)

        # Fit all the points combined to get new poly parameters
        fit_t = np.concatenate((t_v, t_curr))
        fit_x = np.concatenate((x_v, x_curr))
        fit_y = np.concatenate((y_v, y_curr))
        param_lon = np.polyfit(fit_t, fit_x, 5)
        param_lat = np.polyfit(fit_t, fit_y, 5)

        if debug:
            # print(np.polyfit(t_v, x_v, 5))
            # print(np.polyfit(t_v, y_v, 5))
            plt.figure(figsize=(3, 9))
            plt.subplot(311)
            plt.plot(x_v, y_v)
            plt.subplot(312)
            t_after = np.linspace(0, dt, 5*len(t_v))
            x_after = self.get_poly_y(t_after, param_lon)
            plt.plot(t_after, x_after, color='g')
            plt.plot(t_curr, x_curr, color='y')
            plt.scatter(t_v, x_v, color='k')
            plt.subplot(313)
            y_after = self.get_poly_y(t_after, param_lat)
            plt.plot(t_after, y_after, color='g')
            plt.plot(t_curr, y_curr, color='y')
            plt.scatter(t_v, y_v, color='k')
            plt.show()

        poly_param = {"param_lat": param_lat, "param_lon": param_lon, "dt": dt}
        self._model["poly_param"] = poly_param
        self._poly_points = []

    def update_safe_distance(self):
        current_dist = self.get_parameter("safe_distance")
        # Only consider the lowest 10% of the distance value list
        percentage = 0.2
        self._distance_list.sort()
        length = int(percentage * len(self._distance_list))
        if length <= 0:
            return
        new_dist = sum(self._distance_list[0:length]) / length

        safe_distance = self.change_param(current_dist, new_dist)

        self._model["safe_distance"] = safe_distance
        self._distance_list = []

    def update_target_speed(self):
        current_speed = self.get_parameter("target_speed")
        # Only consider the largest 10% of the speed value list
        percentage = 0.2
        self._speed_list.sort(reverse=True)
        length = int(percentage * len(self._speed_list))
        if length <= 0:
            return
        new_speed = sum(self._speed_list[0:length]) / length

        target_speed = self.change_param(current_speed, new_speed)

        self._model["target_speed"] = target_speed
        self._speed_list = []

    # Store learned result
    def store_new_model(self):
        with open(self.path, 'wb') as f:
            pickle.dump(self._model, f)

    # End collecting data
    def end_collect(self):
        print("Before")
        print(self._model)
        self.update_poly_param()
        self.update_safe_distance()
        self.update_target_speed()
        self.store_new_model()
        print("After")
        print(self._model)

    # update parameters according to found new one
    @staticmethod
    def change_param(current, new, max_change_rate=0.1):
        if abs(new - current) < max_change_rate * current:
            result = new
        else:
            if new > current:
                result = current + max_change_rate * current
            else:
                result = current - max_change_rate * current
        return result

    @staticmethod
    # get points given a polynomial curve
    def get_poly_y(x, param):
        t_m = np.array([np.power(x, 5), np.power(x, 4), np.power(x, 3),
                        np.power(x, 2), np.power(x, 1), np.power(x, 0)])
        y = np.matmul(param, t_m)
        return y