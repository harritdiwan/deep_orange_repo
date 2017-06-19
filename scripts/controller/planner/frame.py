import numpy as np
import math


class Frame():
    def __init__(self,ox,oy,angle):
        self.ox = ox
        self.oy = -oy # Correct opposite y direction
        self.angle = -angle # Correct clockwise angle

    def global2local(self, xg, yg):
        yg = -yg # Correct opposite y direction
        return self.__class__.g2l(xg, yg, self.ox, self.oy, self.angle)

    def local2global(self, xl, yl):
        x, y = self.__class__.l2g(xl, yl, self.ox, self.oy, self.angle)
        return (x, -y) # Correct opposite y direction

    @classmethod
    def g2l(cls, xg, yg, xc, yc, angle):
        pt = np.matrix([xg - xc, yg - yc])
        pt = pt.transpose()
        rot = np.matrix([[math.cos(-angle), -math.sin(-angle)], [math.sin(-angle), math.cos(-angle)]])
        mat = rot * pt
        return (mat[0].item(), mat[1].item())

    @classmethod
    def l2g(cls, xl, yl, xc, yc, angle):
        pt = np.matrix([xl, yl, 0, 1])
        pt = pt.transpose()
        rot = np.matrix(
            [[math.cos(angle), -math.sin(angle), 0, xc], [math.sin(angle), math.cos(angle), 0, yc], [0, 0, 1, 0],
             [0, 0, 0, 1]])
        mat = rot * pt
        return (mat[0, 0].item(), mat[1, 0].item())
