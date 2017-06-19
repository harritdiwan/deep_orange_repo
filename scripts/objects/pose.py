from .point import Point


class Pose(Point):
    def __init__(self):
        Point.__init__(self)
        self.speed = 0  # [m/s]
        self.heading = 0  # rad
