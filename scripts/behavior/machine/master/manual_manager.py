from behavior.machine.template.machine import Machine


class ManualManager(Machine):

    def __init__(self):
        Machine.__init__(self)

        self.regular = None

        self.transitions = []

        # Initialization
        self.initial_state = None
        self.current_state = self.initial_state
