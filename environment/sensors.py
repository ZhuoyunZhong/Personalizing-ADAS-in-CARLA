#!/usr/bin/env python

# Sensor class

import glob
import os
import sys
import weakref
import collections

try:
    sys.path.append(glob.glob('../../CARLA_Simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
from carla import ColorConverter as cc

import pygame
import numpy as np
import math

from lane_detection import *


# Obstacle sensor (ultrasonic)
class ObstacleSensor(object):
    def __init__(self, parent_actor, hud):
        self._parent = parent_actor
        world = self._parent.get_world()
        obs_sensor_bp = world.get_blueprint_library().find('sensor.other.obstacle')
        obs_sensor_bp.set_attribute('distance', '20')
        self.sensor = world.try_spawn_actor(obs_sensor_bp, carla.Transform(carla.Location(x=1, z=1)),
                                            attach_to=self._parent)
        self.obstacle = None
        self.distance_to_obstacle = None
        self.close_to_obstacle = False

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: ObstacleSensor._on_detect(weak_self, event))

    @staticmethod
    def _on_detect(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.obstacle = event.other_actor
        self.distance_to_obstacle = event.distance
        if self.distance_to_obstacle < 10:
            self.close_to_obstacle = True
        else:
            self.close_to_obstacle = False


# CollisionSensor
class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        name = ' '.join(event.other_actor.type_id.replace('_', '.').title().split('.')[1:])
        truncate = 250
        actor_type = (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# GnssSensor
class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)),
                                        attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# CameraManager
class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=0.5, z=1.5)),
            carla.Transform(carla.Location(x=0.0, z=2.0)),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))]
        self.transform_index = 0
        # Define sensor list
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        # Append blueprint in the end of each sensor
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('channels', '8')
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None
        # For the main rgb camera
        self.process_rate = 10
        self.counter = 10
        self.lanes = None
        self.left_lane = Lane()
        self.right_lane = Lane()
        self.curvature = None
        self.offset = None
        self.inverse_mat = None
        self.lane_image = None

    def set_sensor(self, index, notify=True, display_camera=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            if display_camera:
                self.sensor.listen(lambda img: CameraManager._parse_image(weak_self, img))
            else:
                self.sensor.listen(lambda img: CameraManager._process_image(weak_self, img))

        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _process_image(weak_self, image):
        self = weak_self()
        if not self:
            return

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.camera'):
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]

            '''
            # Process received image
            if self.counter >= self.process_rate:
                lane_image, curvature, offset, inverse_mat = lane_detection(array, self.left_lane, self.right_lane)
                self.curvature = curvature
                self.offset = offset
                self.inverse_mat = inverse_mat
                self.lane_image = lane_image
                self.counter = 0
            else:
                self.counter += 1

            if self.lane_image is not None:
                new_image = unwarp_found_region(array, self.lane_image, self.inverse_mat, self.curvature, self.offset)
                # Display the result
                self.surface = pygame.surfarray.make_surface(new_image.swapaxes(0, 1))
            else:
                self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            '''
            # new_image, curvature, offset = lane_detection(array, self.left_lane, self.right_lane)
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        else:
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)
