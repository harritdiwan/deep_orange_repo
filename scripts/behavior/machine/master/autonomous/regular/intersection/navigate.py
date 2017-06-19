from behavior.machine.template.state import State
from objects.target import Target


class Navigate(State):
    def __init__(self):
        self.intersection_speed = 5  # m/s

    def run(self, vehicle):
        trajectory = Target()

        # send target to motion planner
        trajectory.start.x = vehicle.pose.x  # this must be enhanced
        trajectory.start.y = vehicle.pose.y
        trajectory.start.speed = vehicle.pose.speed
        trajectory.start.heading = vehicle.pose.heading

        trajectory.end.x = vehicle.mission.target_waypoint().x  # this must be enhanced
        trajectory.end.y = vehicle.mission.target_waypoint().y
        trajectory.end.speed = self.intersection_speed
        trajectory.end.heading = vehicle.mission.get_heading()

        trajectory.speed_limit = self.intersection_speed

        vehicle.set_target(trajectory)
