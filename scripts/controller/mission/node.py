from math import *

class node():
    ROAD = 1
    STOP = 2
    ENTRY = 3
    CHECKPOINT = 4

    def __init__(self, position, name=None, properties=()):
        self.pos = position
        self.speed_limit = 0
        self.name = name
        self.props = properties

    def dist(self, nd):
        return sqrt((self.pos[0]-nd.pos[0])**2+(self.pos[1]-nd.pos[1])**2)
