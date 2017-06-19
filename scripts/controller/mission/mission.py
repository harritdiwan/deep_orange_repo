import random as rd

import networkx as nx

from controller.mission import astar
from controller.mission.node import node


class mission(object):
    def __init__(self,map):
        self.map = map
        self.graph = nx.read_gpickle("data/mission.pkl")
        self.start = self.goal = None
        self.waypoints = []

    def findPath(self, startind, goalind):
        start = self.getnode(startind)
        goal = self.getnode(goalind)
        return astar.findpath(start, goal, self.graph)

    def next_waypoint(self):
        if len(self.waypoints) > 0:
            waypoint = self.waypoints[0]
            self.waypoints = self.waypoints[1:]
            return waypoint
        return None

    @property
    def ended(self):
        return len(self.waypoints) <= 0

    def getnode(self, name):
        for nd in self.graph.nodes_iter():
            if (nd.name == name):
                return nd
        return None

    def setRandomGoal(self, minLength=3):
        if self.goal is not None:
            st = self.goal.name
            gl = rd.randint(0, self.graph.number_of_nodes() - 1)
            if not self.validate(st, gl, minLength):
                return self.setRandomGoal(minLength)
            return False
        else:
            st = rd.randint(0, self.graph.number_of_nodes() - 1)
            gl = rd.randint(0, self.graph.number_of_nodes() - 1)
            if not self.validate(st, gl, minLength):
                return self.setRandomGoal(minLength)
            return True

    def validate(self, st, gl, minLength):
        # Ensure mission validity
        waypoints = self.findPath(st, gl)
        if len(waypoints) < minLength \
                or not any(x in waypoints[0].props for x in [node.ROAD, node.CHECKPOINT]) \
                or not any(x in waypoints[-1].props for x in [node.ROAD, node.CHECKPOINT]):  # Only pick ROAD waypoints as start and end
            return False
        else:
            self.start = waypoints[0]
            self.goal = waypoints[-1]
            self.start.props = [node.ROAD]  # Dangerous, may not be a ROAD
            self.goal.props = [node.CHECKPOINT]
            self.waypoints = waypoints
            return True
