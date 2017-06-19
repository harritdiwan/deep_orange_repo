from behavior.machine.template.state import State
from objects.target import Target


class NavigateToLanes(State):
    def __init__(self):
        self.navigation_speed = 2  #m/s

    def run(self, vehicle):
        trajectory = Target()

        # send target to motion planner
        trajectory.start.x = vehicle.mission.current_waypoint().x #vehicle.pose.x  # this must be enhanced
        trajectory.start.y = vehicle.mission.current_waypoint().y #vehicle.pose.y
        trajectory.start.speed = vehicle.pose.speed
        trajectory.start.heading = vehicle.pose.heading

        trajectory.end.x = vehicle.mission.target_waypoint().x  # this must be enhanced
        trajectory.end.y = vehicle.mission.target_waypoint().y
        trajectory.end.speed = self.navigation_speed
        trajectory.end.heading = vehicle.mission.get_heading()

        trajectory.speed_limit = self.navigation_speed

        vehicle.set_target(trajectory)
