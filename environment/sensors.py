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
try:
    sys.path.append('../')
except IndexError:
    pass
from agents.tools.misc import *


# Radar
class RadarSensor(object):
    def __init__(self, parent_actor, hud, x=2.5, y=0.0, z=1.0, yaw=0.0):
        self._parent = parent_actor
        world = self._parent.get_world()
        radar_sensor_bp = world.get_blueprint_library().find('sensor.other.radar')
        radar_sensor_bp.set_attribute('range', '25')
        radar_sensor_bp.set_attribute('horizontal_fov', '30')
        radar_sensor_bp.set_attribute('points_per_second', '120')
        
        self._location = carla.Location(x=x, y=y, z=z)
        self._rotation = carla.Rotation(yaw=yaw)
        self._transform = carla.Transform(location=self._location, rotation=self._rotation)
        self.sensor = world.try_spawn_actor(radar_sensor_bp, self._transform,
                                            attach_to=self._parent)
        self.points = []
        self.detected = False
        self.rel_pos = None
        self.rel_vel = None
        self.velocity_range = 15
        self.debug = world.debug

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: RadarSensor._on_detect(weak_self, event))

    @staticmethod
    def clamp(min_v, max_v, value):
        return max(min_v, min(value, max_v))
    
    @staticmethod
    def _on_detect(weak_self, event, debug=False):
        self = weak_self()
        if not self:
            return
        
        if debug:
            # Get radar transform
            radar_pos = event.transform.location
            radar_rot = event.transform.rotation

        # Get vehicle transform
        vv = self._parent.get_velocity()
        veh_vel = carla.Vector3D(x=vv.x, y=vv.y, z=vv.z)
        v2w_pos_transform = self._parent.get_transform()

        # Ignore points with radar velocity caused by moving vehicles
        # Which are basically static points
        ## TODO This method still cannot avoid all the static points ###
        vel_transform = carla.Transform(carla.Location(), self._rotation)
        moving_vel = transform_to_world(vel_transform, veh_vel, inverse=True)
        
        self.detected = False
        # For each point
        for detect in event:
            # Calculate actual velocity of the point
            act_vel = detect.velocity + moving_vel.x
            if abs(act_vel) < 3:
                continue
            
            # Get point transform
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            p2v_transform = carla.Transform(self._location, carla.Rotation(
                                            pitch=self._rotation.pitch + alt, 
                                            yaw=self._rotation.yaw + azi,
                                            roll=self._rotation.roll))
            # Get point position in vehicle coordinate
            pos_vec = carla.Vector3D(x=detect.depth)
            rel_pos = transform_to_world(p2v_transform, pos_vec)

            # Store point
            self.points.append([rel_pos.x, rel_pos.y, rel_pos.z, act_vel])
            
            # Draw point
            if debug:
                p2w_transform = carla.Transform(radar_pos, carla.Rotation(
                                                pitch=radar_rot.pitch + alt, yaw=radar_rot.yaw + azi,
                                                roll=radar_rot.roll))
                draw_vec = transform_to_world(p2w_transform, carla.Vector3D(x=detect.depth-0.25))
                norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
                r = int(self.clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                g = int(self.clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                b = int(abs(self.clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                self.debug.draw_point(draw_vec, size=0.075, 
                    life_time=0.06,persistent_lines=False, color=carla.Color(r, g, b))
            
        # Store detected object
        if len(self.points) > 1:
            rel = np.mean(np.array(self.points), axis=0)
            self.rel_pos = rel[0:3]
            self.rel_vel = rel[3:4]
            self.detected = True
            self.points = []

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
        if self.distance_to_obstacle < 20:
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

    # Change camera position
    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    # Change camera
    def next_sensor(self):
        self.set_sensor(self.index + 1, display_camera=True)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    # TODO
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

            # Disabled for now, for not able to handle vehicle-in-front cases
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
