from behavior.machine.template.state import State
from objects.target import Target
from objects.lanes import Lanes
import math


class LaneKeeping(State):

    def __init__(self):
        self.trajectory = Target()
        self.lanes = Lanes()
        self.ROAD_SIZE = 4

    def run(self, vehicle):

        if vehicle.lanes.are_both_visible():
            self.lanes = vehicle.lanes
        elif vehicle.lanes.is_left_visible():
            self.lanes = vehicle.lanes

            dx = (self.lanes.left.x - self.lanes.left_ahead.x)
            dy = (self.lanes.left.y - self.lanes.left_ahead.y)

            self.lanes.right.x = self.lanes.left.x + self.ROAD_SIZE*math.sin(math.atan2(dy, dx))
            self.lanes.right.y = self.lanes.left.y - self.ROAD_SIZE*math.cos(math.atan2(dy, dx))
        elif vehicle.lanes.is_right_visible():
            self.lanes = vehicle.lanes

            dx = (self.lanes.right.x - self.lanes.right_ahead.x)
            dy = (self.lanes.right.y - self.lanes.right_ahead.y)

            self.lanes.left.x = self.lanes.right.x - self.ROAD_SIZE * math.sin(math.atan2(dy, dx))
            self.lanes.left.y = self.lanes.right.y + self.ROAD_SIZE * math.cos(math.atan2(dy, dx))

        mid_x = (self.lanes.right.x + self.lanes.left.x)/2
        mid_y = (self.lanes.right.y + self.lanes.left.y)/2

        trajectory = Target()

        # send target to motion planner
        trajectory.start.x = vehicle.mission.current_waypoint().x #vehicle.pose.x  # this must be enhanced
        trajectory.start.y = vehicle.mission.current_waypoint().y #vehicle.pose.y
        trajectory.start.speed = vehicle.pose.speed
        trajectory.start.heading = vehicle.pose.heading

        # target.position = mid
        trajectory.end.x = vehicle.mission.target_waypoint().x #mid_x
        trajectory.end.y = vehicle.mission.target_waypoint().y #mid_y
        trajectory.end.heading = vehicle.mission.get_heading()

        # dx = (self.lanes.left.x - self.lanes.right.x)
        # dy = (self.lanes.left.y - self.lanes.right.y)
        # trajectory.end.heading = math.atan2(dx, dy)

        if vehicle.in_comfort_acceleration_range():
            trajectory.end.speed = vehicle.mission.target_waypoint().speed_limit
        else:
            trajectory.end.speed = vehicle.mission.current_waypoint().speed_limit

        trajectory.speed_limit = trajectory.end.speed

        vehicle.set_target(trajectory)
