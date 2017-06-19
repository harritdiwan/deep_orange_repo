#!/usr/bin/env python

import os
import sys

# Change working directory
script_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_path)
sys.path.append(script_path)

import pygame
from pygame.locals import *
from fsmvisualizer.fsm_screen_transitions import FsmScreenTransitions
from fsmvisualizer import constants
from graphics.utils import utils


class FsmVisualizer():
    def __init__(self):
        os.environ['SDL_VIDEO_CENTERED'] = '1'
        self._running = True
        self._screen = None
        self._rsize = {'size': constants.SIZE, 'scale': 1}
        self._rpos = (0, 0)
        self._world = None
        self.paused = False

        # windows
        if os.name == 'nt':
            self._screensize = (int(constants.SIZE[0] / 1.25), int(constants.SIZE[1] / 1.25))
        else:
            self._screensize = constants.SIZE

    def on_init(self):
        pygame.init()
        pygame.font.init()

        # create drawing surface
        self._screen = pygame.display.set_mode(self._screensize, RESIZABLE)
        pygame.display.set_caption('FSM Visualizer')
        self._world = FsmScreenTransitions()

        # frames per second setting
        self._fpsClock = pygame.time.Clock()
        self._running = True

    def on_event(self, event):
        if event.type == QUIT:
            self._running = False
        elif event.type == VIDEORESIZE:
            self._screensize = event.dict['size']
            self._screen = pygame.display.set_mode(
                self._screensize, RESIZABLE)
            self._rsize = utils.getsize(constants.SIZE, self._screensize)
            self._rpos = utils.getposition(self._rsize['size'], self._screensize)
        elif event.type in (MOUSEMOTION, MOUSEBUTTONDOWN, MOUSEBUTTONUP):
            event.dict['offset'] = self._rpos
            event.dict['scale'] = self._rsize['scale']
        self._world.on_event(event)

    def on_loop(self):
        # draw game
        elapsed = self._fpsClock.tick() / 1000.0

        if not self.paused:
            drawsurface = self._world.draw(elapsed)

            # draw screen
            self._screen.fill(constants.BLACK)
            res = pygame.transform.smoothscale(drawsurface, self._rsize['size'])
            self._screen.blit(res, self._rpos)

    def on_render(self):
        pygame.display.update()

    def on_cleanup(self):
        pygame.quit()
        sys.exit()

    def on_execute(self):
        if self.on_init() == False:
            self._running = False

        # main game loop
        while (self._running):
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()
        self.on_cleanup()


if __name__ == "__main__":
    pyApp = FsmVisualizer()
    pyApp.on_execute()
