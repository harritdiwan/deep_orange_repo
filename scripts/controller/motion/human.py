from math import radians

import pygame
from pygame.locals import *

from controller.motion.brain import brain


class human(brain):
    def __init__(self,map,car):
        brain.__init__(self,map,car)

    def setinput(self):
        # human player
        pressed = pygame.key.get_pressed()

        if pressed[K_LEFT]:
            self.car.steer += -radians(4)
        elif pressed[K_RIGHT]:
            self.car.steer += radians(4)
        else:
            self.car.steer = 0

        if pressed[K_UP]:
            self.car.acc += 1
        elif pressed[K_DOWN]:
            self.car.acc += -1
        else:
            self.car.acc = 0
