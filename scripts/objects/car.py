import pygame

import constants
from objects.arrays import *
from controller.motion.autonomous import autonomous
from controller.motion.human import human
from controller.motion.carsim import carsim
from controller.motion.unity import unity
from objects.sprite import sprite
from controller.mission.mission import mission


class car(sprite):
    def __init__(self, map, id, brain=None):
        self.map = map
        self.speed = 0
        self.acc = 0
        self.steer = radians(0)
        self.size = (5, 2.5)
        self.vmin = -5
        self.vmax = 35
        self.amax = 3.9
        self.amin = -constants.g
        self.stmax = 40
        self.id = id

        ms = mission(map)
        ms.setRandomGoal()
        self.pos = ms.waypoints[0].pos
        self.pose = atan2(ms.waypoints[1].pos[1]-ms.waypoints[0].pos[1],ms.waypoints[1].pos[0]-ms.waypoints[0].pos[0])

        self.surf = None
        self.rot_rect = (0, 0, 0, 0)
        self.points = []

        brains = {
                'human': human,
                'autonomous': autonomous,
                'carsim': carsim,
                'unity': unity
            }

        self.brain = brains.get(brain, brains['human'])(map, self)

        self.font = pygame.font.SysFont("default", 36)

    def move(self, value):
        self.pos = (self.pos[0] + value[0], self.pos[1] + value[1])

    @property
    def center(self):
         return (self.pos[0]+self.size[0]/2*cos(self.pose), self.pos[1]+self.size[0]/2*sin(self.pose))

    @center.setter
    def center(self,value):
        self.pos = (value[0]-self.size[0]/2*cos(self.pose), value[1]-self.size[0]/2*sin(self.pose))

    @property
    def front(self):
        return (self.pos[0] + self.size[0] * cos(self.pose), self.pos[1] + self.size[0] * sin(self.pose))

    @property
    def topleft(self):
        return ((self.center[0] - self.size[0] / 2, self.center[1] - self.size[1] / 2))

    def render(self, surface):
        if self.surf is None and self.map.loaded:
            self.surf = pygame.Surface(self.map.mtopx_tuple(self.size)).convert_alpha()
            pygame.draw.rect(self.surf, (131, 135, 142, 0), ((0, 0), self.map.mtopx_tuple(self.size)))
            mini = pygame.transform.scale(pygame.image.load('data/mini.png'), self.map.mtopx_tuple(self.size)).convert_alpha()
            self.surf.blit(mini, (0, 0))

        rect = self.surf.get_rect()
        rot = pygame.transform.rotate(self.surf,degrees(-self.pose))
        self.rot_rect = rot.get_rect(center=rect.center).move(self.map.mtopx_tuple(self.topleft))
        surface.blit(rot, self.rot_rect)
        self.brain.render(surface)

        label = self.font.render("Speed: " + "{0:.2f}".format(self.speed), 1, (0, 0, 0))
        surface.blit(label, (10, constants.HEIGHT - 36 * self.id))

    def update(self, elapsed):
        # if setinput return True do not update
        if not self.brain.setinput():
            self.steer = clamp(self.steer, radians(-self.stmax), radians(self.stmax))
            self.speed = clamp(self.speed, self.vmin, self.vmax)

            if self.speed > 0:
                self.acc = clamp(self.acc, self.amin, self.amax)
            else:
                self.acc = clamp(self.acc, -self.amax, -self.amin)

            self.move((self.speed*cos(self.pose)*elapsed, self.speed*sin(self.pose)*elapsed))
            self.pose = (self.pose + self.speed/self.size[1]*tan(self.steer)*elapsed)
            self.pose = atan2(sin(self.pose), cos(self.pose))
            self.speed += self.acc*elapsed

        self.points = translatept(rotatept(getrectpoints(
            pygame.Rect(self.map.mtopx_tuple((-self.size[0]/2., -self.size[1]/2.)), self.map.mtopx_tuple(self.size))),
            self.pose), self.map.mtopx_tuple(self.center))

    def map_position(self):
        return self.map.pxtomap_tuple(self.map.mtopx_tuple(self.center))

    def reached(self, waypt):
        wpt = self.map.mtopx_tuple(waypt)
        for pt in self.points:
            if (distance(wpt,pt)<=self.map.mtopx(2)):
                return True
        return False
