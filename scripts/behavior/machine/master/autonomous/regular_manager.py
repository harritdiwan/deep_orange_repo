from behavior.machine.template.machine import Machine

from .regular.drive_manager import DriveManager
from .regular.idle_manager import IdleManager
from .regular.intersection_manager import IntersectionManager
from .regular.maneuver_manager import ManeuverManager


class RegularManager(Machine):
    def __init__(self):
        Machine.__init__(self)

        # States
        self.drive_manager = DriveManager()
        self.intersection_manager = IntersectionManager()
        self.idle_manager = IdleManager()
        self.maneuver_manager = ManeuverManager()

        # Transitions
        self.transitions = [
            (self.idle_manager, self.mission_received, self.maneuver_manager),

            (self.maneuver_manager, self.intersection_ahead, self.intersection_manager),
            (self.maneuver_manager, self.drive_ahead, self.drive_manager),
            (self.maneuver_manager, self.mission_accomplished, self.idle_manager),

            (self.drive_manager, self.intersection_ahead, self.intersection_manager),
            (self.drive_manager, self.maneuver_ahead, self.maneuver_manager),

            (self.intersection_manager, self.entry_ahead, self.drive_manager)
        ]

        # Initialization
        self.initial_state = self.idle_manager
        self.current_state = self.initial_state

    def intersection_ahead(self, vehicle):
        target = vehicle.mission.target_waypoint()

        if target.is_stop():
            distance_to_waypoint = target.distance(vehicle.pose)
            radius = 1  # meters
            return distance_to_waypoint < radius and vehicle.is_still()

        return False

    def maneuver_ahead(self, vehicle):
        target = vehicle.mission.target_waypoint()

        # If I've arrived at the checkpoint
        if target.is_checkpoint():
            distance_to_waypoint = target.distance(vehicle.pose)
            radius = 1  # meters
            if distance_to_waypoint < radius and vehicle.is_still():
                return True

        # If I can't see lanes and I am not at an intersection
        if not vehicle.lanes.are_visible() and not target.is_stop():  # TO BE ENHANCED
            return True

        return False

    def drive_ahead(self, vehicle):
        if vehicle.lanes.are_visible() and not vehicle.mission.target_waypoint().is_checkpoint():
            return True
        else:
            return False

    def entry_ahead(self, vehicle):
        return vehicle.mission.current_waypoint().is_entry() or vehicle.mission.target_waypoint().is_checkpoint()

    def mission_received(self, vehicle):
        return vehicle.mission.exists()

    def mission_accomplished(self, vehicle):
        target = vehicle.mission.target_waypoint()

        if target.is_checkpoint():
            distance_to_waypoint = target.distance(vehicle.pose)
            radius = 1  # meters

            return distance_to_waypoint < radius and vehicle.is_still()

        return False
