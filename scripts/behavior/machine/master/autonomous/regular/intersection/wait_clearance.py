import time

from behavior.machine.template.state import State


class WaitClearance(State):
    def __init__(self):
        pass

    def run(self, vehicle):
        sleep_time = 5 # seconds
        time.sleep(sleep_time)
