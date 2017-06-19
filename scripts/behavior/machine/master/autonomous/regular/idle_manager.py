from behavior.machine.template.state import State


class IdleManager(State):
    def __init__(self):
        State.__init__(self)
        pass

    def run(self, vehicle):
        pass

    def on_start(self, vehicle):
        vehicle.reset_mission()
        vehicle.mission_accomplished = True

    def on_exit(self, vehicle):
        vehicle.mission_accomplished = False
