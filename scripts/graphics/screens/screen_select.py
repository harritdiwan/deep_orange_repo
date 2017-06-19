import pygame
from pgu import gui
from pygame.locals import *

from graphics.screens.screen import screen
from graphics.utils import utils
import constants


class Separator(gui.Widget):
    def __init__(self,**params):
        gui.Widget.__init__(self,**params)


class screenselect(screen):
    def __init__(self, manager, args=None):
        screen.__init__(self, manager, args)
        self._app = gui.App()
        self._t = gui.Table(width=constants.WIDTH / 4)

        font = pygame.font.SysFont("default", 36)
        fontBig = pygame.font.SysFont("default",48)

        self._t.tr()
        label = gui.Label("DriveSim",font=fontBig)
        self._t.td(label, colspan=5, align=0)

        self._t.tr()
        self._t.td(Separator(height=20))

        self._t.tr()
        btn = gui.Button("Start!",font=font,width=100,height=40)
        btn.connect(gui.CLICK, self.cb_start)
        self._t.td(btn, col=2, align=0)

        self._d = gui.Table()
        self._c = gui.Table(width=constants.WIDTH / 4)
        self._c.tr()
        self._c.td(self._d)

        self._player1 = gui.Select(height=40)
        label = gui.Label("None", font=font)
        self._player1.add(label, 'none')
        label = gui.Label("Human", font=font)
        self._player1.add(label, 'human')
        label = gui.Label("Autonomous", font=font)
        self._player1.add(label, 'autonomous')
        label = gui.Label("CarSim", font=font)
        self._player1.add(label, 'carsim')
        label = gui.Label("Unity", font=font)
        self._player1.add(label, 'unity')
        self._player1.value = 'unity'

        self._player2 = gui.Select(height=40)
        label = gui.Label("None", font=font)
        self._player2.add(label, 'none')
        label = gui.Label("Human", font=font)
        self._player2.add(label, 'human')
        label = gui.Label("Autonomous", font=font)
        self._player2.add(label, 'autonomous')
        self._player2.value = 'none'

        self._d.tr()
        label = gui.Label("Options", font=fontBig)
        self._d.td(label, colspan=1, align=-1)
        self._d.tr()
        self._d.td(Separator(height=50))
        self._d.tr()
        label = gui.Label("Player 1", font=font)
        self._d.td(label, colspan=1, align=-1)
        self._d.tr()
        self._d.td(Separator(height=10))
        self._d.tr()
        self._d.td(self._player1, colspan=1, align=0)
        self._d.tr()
        self._d.td(Separator(height=20))
        self._d.tr()
        label = gui.Label("Player 2", font=font)
        self._d.td(label, colspan=1, align=-1)
        self._d.tr()
        self._d.td(Separator(height=10))
        self._d.tr()
        self._d.td(self._player2, colspan=1, align=0)

        self._g = gui.Table(width=constants.WIDTH * 3 / 4)  # , background=(255, 255, 255)
        self._g.tr()
        self._g.td(self._t, colspan=1, align=0)
        self._g.td(self._c, colspan=1, align=0)

        self._app.screen = manager.getsurf()
        self._app.init(self._g)

        self.backgr = pygame.image.load('data/background.jpg').convert_alpha()
        self.backgr.fill((255, 255, 255, 128), None, pygame.BLEND_RGBA_MULT)

    def get_data(self):
        players = [self._player1.value, self._player2.value]
        valid = list(filter(lambda pl: pl is not 'none', players))
        return {"players": reversed(valid)}

    def cb_start(self):
        self._manager.state = constants.SCREEN_GAME

    def on_event(self, event):
        if event.type in (MOUSEMOTION, MOUSEBUTTONDOWN, MOUSEBUTTONUP):
            event.dict['pos'] = utils.convertpoint(event.dict['pos'], event.dict['offset'], event.dict['scale'])
        self._app.event(event)

    def draw(self, surface, elapsed):
        surface.blit(self.backgr, (0, 0, constants.WIDTH, constants.HEIGHT))
        self._app.paint()
