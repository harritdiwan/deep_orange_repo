from behavior.machine.template.state import State
from objects.target import Target
import math


class StopDetected(State):

    def __init__(self):
        self.trajectory = []
        self.stop_approach_speed = 5

    def run(self, vehicle):

        trajectory = Target()

        # send target to motion planner
        trajectory.start.x = vehicle.pose.x  # this must be enhanced
        trajectory.start.y = vehicle.pose.y
        trajectory.start.speed = vehicle.pose.speed
        trajectory.start.heading = vehicle.pose.heading

        trajectory.end.x = vehicle.mission.target_waypoint().x  # this must be enhanced
        trajectory.end.y = vehicle.mission.target_waypoint().y
        trajectory.end.speed = 0

        # this must be enhanced
        if vehicle.lanes:
            dx = abs(vehicle.lanes.left.x - vehicle.lanes.right.x)
            dy = abs(vehicle.lanes.left.y + vehicle.lanes.right.y)
            trajectory.end.heading = math.atan2(dx, dy)
        else:
            trajectory.end.heading = vehicle.mission.get_heading()

        trajectory.speed_limit = self.stop_approach_speed

        vehicle.set_target(trajectory)
