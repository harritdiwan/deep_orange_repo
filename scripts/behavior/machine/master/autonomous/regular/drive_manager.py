from behavior.machine.template.machine import Machine

from .drive.checkpoint_detected import CheckpointDetected
from .drive.lane_exit import LaneExit
from .drive.lane_keeping import LaneKeeping
from .drive.stop_detected import StopDetected


class DriveManager(Machine):
    def __init__(self):

        # States
        self.lane_exit = LaneExit()
        self.lane_keeping = LaneKeeping()
        self.stop_detected = StopDetected()
        self.checkpoint_detected = CheckpointDetected()

        # Transitions
        self.transitions = [
            (self.lane_exit, self.correct_lane, self.lane_keeping),

            (self.lane_keeping, self.exit_ahead, self.lane_exit),
            (self.lane_keeping, self.stop_ahead, self.stop_detected),
            (self.lane_keeping, self.checkpoint_ahead, self.checkpoint_detected)
        ]

        # Initialization
        self.initial_state = self.lane_keeping
        self.current_state = self.initial_state

    def correct_lane(self, vehicle):
        return not vehicle.mission.current_waypoint().is_exit()

    def exit_ahead(self, vehicle):
        return vehicle.mission.target_waypoint().is_exit()  # current or target?

    def obstacle_ahead_avoid(self, vehicle):
        pass

    def stop_ahead(self, vehicle):
        # This should be enhanced by considering Stop Sign perception
        target = vehicle.mission.target_waypoint()

        # If I've arrived at the stop
        if target.is_stop() and vehicle.in_comfort_acceleration_range():
            return True
        else:
            return False

    def checkpoint_ahead(self, vehicle):
        target = vehicle.mission.target_waypoint()

        # If I've arrived at the checkpoint
        if target.is_checkpoint()and vehicle.in_comfort_acceleration_range():
            return True
        else:
            return False

    def obstacle_ahead(self, vehicle):
        pass
