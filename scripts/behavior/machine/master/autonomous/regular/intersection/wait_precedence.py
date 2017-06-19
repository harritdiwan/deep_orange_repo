import time
from behavior.machine.template.state import State


class WaitPrecedence(State):
    def __init__(self):
        self.timer = 0
        self.sleep_time = 2  # seconds

    def run(self, vehicle):
        pass

    def on_start(self, vehicle):
        self.timer = time.time()