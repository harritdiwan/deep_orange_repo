from behavior.machine.template.machine import Machine

from .autonomous.regular_manager import RegularManager


class AutonomousManager(Machine):

    def __init__(self):
        Machine.__init__(self)

        self.regular = RegularManager()

        self.transitions = []

        # Initialization
        self.initial_state = self.regular
        self.current_state = self.initial_state
