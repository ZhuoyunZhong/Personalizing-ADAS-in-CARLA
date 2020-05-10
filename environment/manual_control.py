import sys
import glob
import os

try:
    sys.path.append(glob.glob('../../CARLA_Simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

try:
    import pygame
    from pygame.locals import KMOD_CTRL, KMOD_SHIFT
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_F1
    from pygame.locals import K_h
    from pygame.locals import K_TAB
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_c

    from pygame.locals import K_p
    from pygame.locals import K_l
    from pygame.locals import K_t

    from pygame.locals import K_UP, K_DOWN, K_LEFT, K_RIGHT
    from pygame.locals import K_w, K_a, K_s, K_d
    from pygame.locals import K_q
    from pygame.locals import K_m
    from pygame.locals import K_COMMA, K_PERIOD
    from pygame.locals import K_SPACE

    from pygame.locals import K_r
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

"""
    F1           : toggle HUD
    H            : toggle help
    ESC          : quit
    C            : change weather (Shift+C reverse)
    TAB          : change sensor position
    `            : change sensor

    W/Up         : throttle
    S/Down       : brake
    AD/LeftRight : steer
    Q            : toggle reverse
    Space        : hand-brake
    M            : toggle manual transmission
    ,/.          : gear up/down

    P            : toggle autopilot
    L            : toggle learning mode
    T            : train the model with existing data
    Backspace    : reborn
"""


class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot=False):
        self._autopilot_enabled = start_in_autopilot
        self._learning_enabled = False
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.enable_agent(start_in_autopilot)
        else:
            raise NotImplementedError("Actor type not supported")
        '''
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        '''
        self._steer_cache = 0.0
        world.hud.notification("Press 'h' for help.", seconds=4.0)

    def parse_events(self, client, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                # World setting
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    self._learning_enabled = False
                    world.restart()
                elif event.key == K_TAB:
                    world.main_rgb_camera.toggle_camera()
                elif event.key == K_BACKQUOTE:
                    world.main_rgb_camera.next_sensor()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()

                # Hud setting
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h:
                    world.hud.help.toggle()

                '''
                # Record setting
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.main_rgb_camera.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if world.recording_enabled:
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    currentIndex = world.main_rgb_camera.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.enable_agent(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.main_rgb_camera.set_sensor(currentIndex)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % world.recording_start)
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % world.recording_start)
                '''

                # Control setting
                if isinstance(self._control, carla.VehicleControl):
                    # gear
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1

                    # autopilot mode
                    elif event.key == K_p and not (pygame.key.get_mods() & KMOD_CTRL):
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.enable_agent(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

                    # learning mode
                    elif event.key == K_l and not self._autopilot_enabled:
                        self._learning_enabled = not self._learning_enabled
                        world.enable_learning(self._learning_enabled)
                        world.hud.notification('Learning %s' % ('On' if self._learning_enabled else 'Off'))

                    # train model
                    elif event.key == K_t:
                        world.agent.train_model()
                        world.hud.notification('Training Model...')

        # send control signal
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
            '''
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time(), world)
            '''
            world.player.apply_control(self._control)

    # Vehicle Control
    def _parse_vehicle_keys(self, keys, milliseconds):
        throttle_increment = 5e-4 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(abs(self._control.throttle + throttle_increment), 0.7)
        else:
            self._control.throttle = max(self._control.throttle - throttle_increment, 0.2)
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = 0.8
            self._control.throttle = 0.0
        else:
            self._control.brake = 0.0
        self._control.hand_brake = keys[K_SPACE]

    '''
    def _parse_walker_keys(self, keys, milliseconds, world):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = world.player_max_speed_fast if pygame.key.get_mods() & KMOD_SHIFT else world.player_max_speed
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()
    '''

    @staticmethod
    def _is_quit_shortcut(key):
        return key == K_ESCAPE
