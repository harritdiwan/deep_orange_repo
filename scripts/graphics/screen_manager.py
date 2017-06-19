import pygame

import constants
from graphics.screens.screen_game import screengame
from graphics.screens.screen_select import screenselect


class screenmanager(object):
    def __init__(self):
        # initialize screens
        self._drawsurface = pygame.Surface(constants.SIZE)
        self._current_screen = None
        self._state = None

        # associate state with screen
        self._options = {
            0: screenselect,
            1: screengame,
        }
        self.state = constants.SCREEN_SELECT

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        try:
            data = self._current_screen.get_data()
        except:
            data = {}

        self._state = value
        new_screen = self._options[self._state]
        self._current_screen = new_screen(self, data)

    def on_event(self, event):
        if self._current_screen is not None:
            self._current_screen.on_event(event)

    def draw(self, elapsed):
        self._drawsurface.fill(constants.WHITE)
        if self._current_screen is not None:
            self._current_screen.draw(self._drawsurface, elapsed)
        return self._drawsurface

    def getsurf(self):
        return self._drawsurface
