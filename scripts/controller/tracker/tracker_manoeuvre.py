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
        self.back_speed = -3
        self.reverse = False

    def getInput(self, nref, traj):
        self.nref = nref
        self.traj = traj

        # heading error
        des_ang = math.atan2(nref.end.y - nref.start.y, nref.end.x - nref.start.x)
        es = math.atan2(math.sin(des_ang - self.car.pose), math.cos(des_ang - self.car.pose))
        if not self.reverse:
            steer_h = es * 2
        else:
            steer_h = es * -2

        # lateral error
        p1 = np.array([nref.start.x, nref.start.y])
        p2 = np.array([nref.end.x, nref.end.y])

        if not self.reverse:
            p3 = np.array([self.car.front[0], self.car.front[1]])
        else:
            p3 = np.array([self.car.pos[0], self.car.pos[1]])

        eg = np.cross(p2 - p1, p3 - p1) / norm(p2 - p1)
        steer_l = eg * -1
        steer = steer_h + steer_l

        # forward/back mode selection
        p4 = np.array([self.car.center[0], self.car.center[1]])
        if np.inner(p2 - p1, p4 - p1) / norm(p2 - p1) > distance((p2[0], p2[1]), (p1[0], p1[1])):
            if abs(es) < 0.1 and abs(eg) < 0.1:
                self.reverse = True
                print "Reverse"
        else:
            self.reverse = False
            print "Forward"

        print "es", es, "eg", eg
        print "p", np.inner(p2 - p1, p3 - p1), "d", distance((p2[0], p2[1]), (p1[0], p1[1]))
        print "sh", steer_h, "sl", steer_l
        print ""

        # speed control
        if not self.reverse:
            if nref.end.speed == 0:
                ref_speed = min(nref.speed_limit, nref.end.speed + 1 * distance(self.car.center, (nref.end.x, nref.end.y)))
            else:
                ref_speed = nref.end.speed
        else:
            if nref.end.speed == 0:
                ref_speed = max(self.back_speed, nref.end.speed - 1 * distance(self.car.center, (nref.end.x, nref.end.y)))
            else:
                ref_speed = self.back_speed

        ev = ref_speed - self.car.speed
        acc = ev * 10

        return {"steer": steer, "acc": acc}

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
