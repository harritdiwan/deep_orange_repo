from .pose import Pose


class Target:

    def __init__(self):
        self.start = Pose()
        self.end = Pose()
        self.speed_limit = 0
