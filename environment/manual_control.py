import pygame
from pygame.locals import K_ESCAPE
from pygame.locals import K_q
from pygame.locals import KMOD_CTRL


class KeyboardControl(object):
    def __init__(self, world):
        world.hud.notification("Press 'ESC' to quit.", seconds=4.0)

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
