#!/usr/bin/env python

import pickle
from os import path
from os.path import dirname, abspath


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

    def load_model(self):
        # if file doesn't exist, create one
        if not path.exists(self.path):
            model = {"safe_distance": 15.0, "target_speed": 20.0}
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

    # Return a certain parameter value
    def get_parameter(self, keyword):
        return self._model[keyword]

    # Learn from collected data to update value
    def update_safe_distance(self):
        current_dist = self.get_parameter("safe_distance")
        # Only consider the lowest 10% of the distance value list
        percentage = 0.1
        self._distance_list.sort()
        length = int(percentage * len(self._distance_list))
        if length <= 0:
            return
        new_dist = sum(self._distance_list[0:length]) / length
        # Change slowly, not bigger than max_change_rate of the origin
        max_change_rate = 0.2
        if abs(new_dist-current_dist) < max_change_rate * current_dist:
            safe_distance = new_dist
        else:
            if new_dist > current_dist:
                safe_distance = current_dist * (1+max_change_rate)
            else:
                safe_distance = current_dist * (1-max_change_rate)
        
        self._model["safe_distance"] = safe_distance
        self._distance_list = []

    def update_target_speed(self):
        current_speed = self.get_parameter("target_speed")
        # Only consider the largest 10% of the speed value list
        percentage = 0.1
        self._speed_list.sort(reverse=True)
        length = int(percentage * len(self._speed_list))
        if length <= 0:
            return
        new_speed = sum(self._speed_list[0:length]) / length
        # Change slowly, not bigger than max_change_rate of the origin
        max_change_rate = 0.2
        if abs(new_speed - current_speed) < max_change_rate * current_speed:
            target_speed = new_speed
        else:
            if new_speed > current_speed:
                target_speed = current_speed * (1 + max_change_rate)
            else:
                target_speed = current_speed * (1 - max_change_rate)

        self._model["target_speed"] = target_speed
        self._speed_list = []

    # Store learned result
    def store_new_model(self):
        with open(self.path, 'wb') as f:
            pickle.dump(self._model, f)

    # End collecting data
    def end_collect(self):
        self.update_safe_distance()
        self.update_target_speed()
        self.store_new_model()
        print(self._model)
