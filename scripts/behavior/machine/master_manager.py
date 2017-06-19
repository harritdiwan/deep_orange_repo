from .master.autonomous_manager import AutonomousManager
from .template.machine import Machine

from .utils.logger import Logger


class MasterManager(Machine):

    def __init__(self):
        Machine.__init__(self)

        self.autonomous = AutonomousManager()
        self.transitions = []
        self.logger = Logger(console_log=False, ros_log=True)

        # Initialization
        self.initial_state = self.autonomous
        self.current_state = self.initial_state

    def run(self, vehicle):
        sc = self.get_state_path()
        Machine.run(self, vehicle)
        sn = self.get_state_path()
        self.logger.log(sc, sn, vehicle)

    def get_state_path(self):
        state = self.current_state
        state_path = [state.string()]

        while isinstance(state, Machine):
            state = state.current_state
            state_path.append(state.string())

        return state_path
