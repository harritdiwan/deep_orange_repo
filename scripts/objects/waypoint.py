from .point import Point


class Waypoint(Point):

    def __init__(self, x, y, point_type="road", speed_limit=10):
        Point.__init__(self)

        self.x = x
        self.y = y

        self.type = point_type
        self.speed_limit = speed_limit

    def is_stop(self):
        if self.type == "stop":
            return True

    def is_checkpoint(self):
        if self.type == "checkpoint":
            return True

    def is_exit(self):
        if self.type == "exit":
            return True

    def is_entry(self):
        if self.type == "entry":
            return True

    def is_road(self):
        if self.type == "road":
            return True