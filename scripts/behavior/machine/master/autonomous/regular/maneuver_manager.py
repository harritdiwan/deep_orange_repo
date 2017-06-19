from behavior.machine.template.machine import Machine

from .maneuver.emergency_brake import EmergencyBrake
from .maneuver.evaluate import Evaluate
from .maneuver.navigate_to_checkpoint import NavigateToCheckpoint
from .maneuver.navigate_to_lanes import NavigateToLanes
from .maneuver.parking import Parking


class ManeuverManager(Machine):
    def __init__(self):
        Machine.__init__(self)

        # States
        self.evaluate = Evaluate()
        self.parking = Parking()
        self.navigate_to_lanes = NavigateToLanes()
        self.navigate_to_checkpoint = NavigateToCheckpoint()
        self.emergency_brake = EmergencyBrake()

        # Transitions
        self.transitions = [
            (self.evaluate, self.no_lanes_ahead, self.emergency_brake),
            (self.evaluate, self.parking_ahead, self.parking),

            (self.parking, self.parking_accomplished, self.evaluate),

            (self.emergency_brake, self.vehicle_is_still, self.navigate_to_lanes),
            (self.navigate_to_lanes, self.checkpoint_ahead, self.navigate_to_checkpoint)
        ]

        # Initialization
        self.initial_state = self.evaluate
        self.current_state = self.initial_state

    def no_lanes_ahead(self, vehicle):
        return not vehicle.lanes.are_visible()

    def parking_ahead(self, vehicle):
        return False

    def parking_accomplished(self, vehicle):
        return True

    def vehicle_is_still(self, vehicle):
        return vehicle.is_still()

    def checkpoint_ahead(self, vehicle):
        return vehicle.mission.target_waypoint().is_checkpoint()
