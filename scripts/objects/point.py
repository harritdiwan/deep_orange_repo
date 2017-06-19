import math


class Point:

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def distance(self, point):
        return math.hypot(self.x-point.x, self.y-point.y)

    def direction(self, point):
        dx = (point.x - self.x)
        dy = (point.y - self.y)

        # return math.pi - math.atan2(dx, dy)
        return math.atan2(dy, dx)
