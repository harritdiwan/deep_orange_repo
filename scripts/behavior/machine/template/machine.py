from .state import State


class Machine(State):
    def __init__(self):
        self.transitions = []
        self.initial_state = []
        self.current_state = []

    def run(self, vehicle):
        self.current_state.run(vehicle)
        self.current_state = self.__fire_transition(vehicle)

    def __fire_transition(self, vehicle):
        for source, activation, destination in self.transitions:
            if source is self.current_state:
                if activation(vehicle) is True:
                    source.on_exit(vehicle)
                    destination.on_start(vehicle)
                    return destination

        return self.current_state

    def on_start(self, vehicle):
        self.current_state = self.initial_state
        self.current_state.on_start(vehicle)
