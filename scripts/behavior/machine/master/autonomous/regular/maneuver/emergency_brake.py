import math
from behavior.machine.template.state import State
from objects.target import Target


class EmergencyBrake(State):
    def __init__(self):
        pass

    def run(self, vehicle):
        trajectory = Target()

        # send target to motion planner
        trajectory.start.x = vehicle.pose.x  # this must be enhanced
        trajectory.start.y = vehicle.pose.y
        trajectory.start.speed = vehicle.pose.speed
        trajectory.start.heading = vehicle.pose.heading

        trajectory.end.x = vehicle.pose.x + 1 * math.cos(vehicle.pose.heading)  # this must be enhanced
        trajectory.end.y = vehicle.pose.y + 1 * math.sin(vehicle.pose.heading)
        trajectory.end.speed = 0
        trajectory.end.heading = vehicle.pose.heading

        trajectory.speed_limit = 0

        vehicle.set_target(trajectory)
