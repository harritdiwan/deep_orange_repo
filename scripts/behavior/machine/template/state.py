class State(object):

    def run(self, vehicle):
        raise NotImplementedError("Run must be overridden!")

    def on_start(self, vehicle):
        pass

    def on_exit(self, vehicle):
        pass

    def string(self):
        return self.__class__.__name__
