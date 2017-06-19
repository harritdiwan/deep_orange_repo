import time

from behavior.machine.template.machine import Machine

from .intersection.navigate import Navigate
from .intersection.wait_clearance import WaitClearance
from .intersection.wait_precedence import WaitPrecedence


class IntersectionManager(Machine):
    def __init__(self):
        Machine.__init__(self)

        # States
        self.wait_clearance = WaitClearance()
        self.wait_precedence = WaitPrecedence()
        self.navigate = Navigate()

        # Transitions
        self.transitions = [
            (self.wait_precedence, self.right_of_way, self.navigate),
            (self.navigate, self.obstacle_ahead, self.wait_clearance),
            (self.wait_clearance, self.intersection_clear, self.navigate)
        ]

        # Initialization
        self.initial_state = self.wait_precedence
        self.current_state = self.initial_state

    def right_of_way(self, vehicle):
        if time.time() - self.wait_precedence.timer > self.wait_precedence.sleep_time:
            return True  # Check for other vehicles in the intersection

    def obstacle_ahead(self, vehicle):
        return False  # Check for obstacles/vehicles in the intersection

    def intersection_clear(self, vehicle):
        return True  # Check for clearance
