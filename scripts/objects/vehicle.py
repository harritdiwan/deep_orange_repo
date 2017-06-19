import math

import constants

from mission.mission import Mission
from .lanes import Lanes
from .pose import Pose
from .target import Target


class Vehicle(object):
    def __init__(self):
        self.pose = Pose()

        self.mission = Mission()
        self.target = Target()

        self.fault = []
        self.lanes = Lanes()
        self.objects = []

        self.miss_counter = 0
        self.mission_accomplished = False

        self.comfort_acceleration = constants.g / 2.0

    def in_comfort_acceleration_range(self):
        current = self.mission.current_waypoint()
        target = self.mission.target_waypoint()

        if target.is_checkpoint() or target.is_stop():
            delta_speed = 0 - current.speed_limit
        else:
            delta_speed = target.speed_limit - current.speed_limit

        acceleration = self.comfort_acceleration * math.copysign(1, delta_speed)
        delta_time = delta_speed / acceleration
        acceleration_distance = current.speed_limit * delta_time + 0.5 * acceleration * delta_time ** 2

        return self.pose.distance(target) < acceleration_distance

    def set_pose(self, x, y, speed, heading):
        self.pose.x = x
        self.pose.y = y
        self.pose.speed = speed
        self.pose.heading = heading

    def set_position(self, position):
        self.pose.x = position.x
        self.pose.y = position.y

    def set_speed(self, speed):
        self.pose.speed = speed

    def set_mission(self, mission):
        self.mission = mission

    def reset_mission(self):
        self.mission = Mission()

    def set_lanes(self, lane_msg):
        self.lanes = Lanes(lane_msg.right, lane_msg.right_ahead, lane_msg.left, lane_msg.left_ahead)

    def set_obstacles(self, obstacles):
        self.objects = obstacles

    def set_stop_sign(self, stop_sign):
        pass

    def set_heading(self, heading):
        self.pose.heading = heading

    def set_target(self, target):
        self.target = target

    def is_still(self):
        threshold = 0.1  # m/s

        if -threshold <= self.pose.speed <= threshold:
            return True
