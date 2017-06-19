import numpy as np
from numpy.linalg import norm
from objects.arrays import *
import math
import pygame


class tracker():
    def __init__(self, car, map):
        self.car = car
        self.map = map
        self.nref = None
        self.traj = None

    def setTarget(self, nref):
        self.nref = nref

    def getInput(self, nref, traj):
        self.nref = nref
        self.traj = traj

        # heading error
        des_ang = math.atan2(nref.end.y - nref.start.y, nref.end.x - nref.start.x)
        es = math.atan2(math.sin(des_ang - self.car.pose), math.cos(des_ang - self.car.pose))
        steer_h = es * 0.5

        # lateral error
        p1 = np.array([self.traj[0][0], self.traj[1][0]])
        p2 = np.array([self.traj[0][1], self.traj[1][1]])
        p3 = np.array([self.car.front[0], self.car.front[1]])
        eg = np.cross(p2 - p1, p3 - p1) / norm(p2 - p1)

        steer_l = eg * -1
        steer = steer_h + steer_l

        # speed control
        if nref.end.speed == 0:
            ref_speed = min(nref.speed_limit, nref.end.speed + 1 * distance(self.car.center, (nref.end.x, nref.end.y)))
        else:
            ref_speed = nref.end.speed

        ev = ref_speed - self.car.speed
        acc = ev * 10

        return {"steer": steer, "acc": acc, "error": abs(eg) > 5}

    def render(self, surface):
        if self.nref:
            pygame.draw.circle(surface, (255, 255, 0), self.map.mtopx_tuple((self.nref.start.x, self.nref.start.y)), 7)
            pygame.draw.circle(surface, (51, 204, 204), self.map.mtopx_tuple((self.nref.end.x, self.nref.end.y)), 7)
            pass
        if self.traj:
            for i in xrange(0,len(self.traj[0])):
                px = self.traj[0][i]
                py = self.traj[1][i]
                pygame.draw.circle(surface, (255, 204, 204), self.map.mtopx_tuple((px, py)), 7)
