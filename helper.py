def process_obstacle_sig(obs):
    print(obs.distance)


def process_img(image):
    pass


def spawn_self(world, blueprint_library):
    # store vehicles and sensors
    self_actors = []

    # Spawn a black tesla as test vehicle
    bp_self = random.choice(blueprint_library.filter('vehicle.tesla.*'))
    transform_self = carla.Transform(carla.Location(x=50, y=7.5, z=0.5))
    vehicle = world.try_spawn_actor(bp_self, transform_self)
    if vehicle is not None:
        self_actors.append(vehicle)
        print('created %s' % vehicle.type_id)

        # Spawn sensors
        # RGB camera
        rgb_camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=0.8, z=1.5))
        rgb_camera = world.try_spawn_actor(rgb_camera_bp, camera_transform, attach_to=vehicle)
        if rgb_camera is not None:
            self_actors.append(rgb_camera)
            print('created %s' % rgb_camera.type_id)
        else:
            print('rgb_camera position invalid')

        # Obstacle sensors (ultrasonic)
        obs_detect_bp = blueprint_library.find('sensor.other.obstacle')
        obs_detect_bp.set_attribute('distance', '20')
        obs_detect_transform = carla.Transform(carla.Location(x=1.6, z=1))
        obs_detect = world.try_spawn_actor(obs_detect_bp, obs_detect_transform, attach_to=vehicle)
        if obs_detect is not None:
            self_actors.append(obs_detect)
            print("created %s" % obs_detect.type_id)
        else:
            print('obstacle detection position invalid')

    else:
        "Self vehicle spawn location occupied."

    return self_actors


def sensor(world, blueprint_library):
    self_actors = []

    try:
        # Spawn self with sensors
        self_actors = spawn_self(world, blueprint_library)
        self_v = self_actors[0]
        self_rgb = self_actors[1]
        self_obs = self_actors[2]

        # Acquire and process sensor data
        self_obs.listen(lambda obs: process_obstacle_sig(obs))
        self_rgb.listen(lambda image: image.save_to_disk('rgb_cam_out/%06d.png' % image.frame))

        # Start Simulation
        self_v.apply_control(carla.VehicleControl(throttle=0.5, brake=0.0))

    finally:
        for x in self_actors:
            x.destroy()
