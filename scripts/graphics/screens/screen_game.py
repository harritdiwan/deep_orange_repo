import constants
import itertools
from graphics.screens.screen import screen
from mapping.map import map
from objects.car import car
import pygame


class screengame(screen):
    def __init__(self, manager, args):
        screen.__init__(self, manager, args)
        self._map = map()
        self._lost = False

        if 'players' in args:
            self._vehicles = [car(self._map, ind + 1, brain) for ind, brain in enumerate(args['players'])]
        else:
            self._vehicles = [car(self._map, 1, 'autonomous')]

        self.backgr = pygame.image.load('data/background.jpg').convert_alpha()
        self.backgr.fill((255, 255, 255, 128), None, pygame.BLEND_RGBA_MULT)

    def on_event(self, event):
        pass

    def draw(self, surface, elapsed):
        if not self._lost:
            if self._map.loaded:
                # Don't update if too much time has passed
                if elapsed < 0.5:
                    for vehicle in self._vehicles:
                        vehicle.update(elapsed)

                # Render map and cars
                self._map.render(surface)
                for vehicle in self._vehicles:
                    vehicle.render(surface)

                # self._lost = any(self._map.checkcollision(car.points) for car in self._vehicles)
                # for pair in itertools.combinations(self._vehicles, r=2):
                    # print pygame.Rect.colliderect(pair[0].rot_rect, pair[1].rot_rect)
            else:
                surface.blit(self.backgr, (0, 0, constants.WIDTH, constants.HEIGHT))
        else:
            self._manager.state = constants.SCREEN_SELECT
