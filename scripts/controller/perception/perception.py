import networkx as nx
import pygame

from controller.mission.node import node
from objects.arrays import *
import itertools


class perception(object):
    def __init__(self, map, car):
        self.car = car
        self.map = map
        self.points = []
        self.surf = None
        self.viewwidth = 4
        self.viewdepth = 10
        self.lane_center = nx.read_gpickle("data/trajectory.pkl")
        self.visible_lanes = []

    # rettangolo width*depth
    # dal centro della macchina
    def setViewDepth(self, width, depth):
        self.viewwidth = width
        self.viewdepth = depth

    # dizionario:
    # lanes -> pos on map, 6 pt per lato, tipo lane
    # obstacles -> drivability map (outside road)
    def look(self):
        self.visible_lanes = self.findLanes()
        return {"obstacle": 0, "lane_center": self.lanes}

    def findLanes(self):
        lane_center = []
        self.points = translatept(rotatept(getrectpoints(
            pygame.Rect((-self.viewdepth / 2., -self.viewwidth / 2.),
                        (self.viewdepth, self.viewwidth))),
            self.car.pose), self.center)

        for wayp in self.lane_center.nodes_iter():
            if isinside(self.points, wayp.pos):
                lane_center.append(wayp)
        # lane_center.sort(key=lambda pt: distance(self.car.pos, pt.pos), reverse=False)
        lane_center.sort(key=lambda pt: pt.name, reverse=False)

        r = []
        l = []

        for st, en in zip(lane_center, lane_center[1:]):
            b = st.pos
            e = en.pos
            pose = atan2(e[1]-b[1], e[0]-b[0])
            diff = atan2(sin(pose - self.car.pose), cos(pose - self.car.pose))
            if abs(diff) < radians(57):
                r.append(node((e[0] + 2 * sin(pose), e[1] - 2 * cos(pose))))
                l.append(node((e[0] - 2 * sin(pose), e[1] + 2 * cos(pose))))
        return [r, l]

    @property
    def lanes(self):
        r = [(pt.pos[0], pt.pos[1]) for pt in self.visible_lanes[0]]
        l = [(pt.pos[0], pt.pos[1]) for pt in self.visible_lanes[1]]
        return [r, l]

    @property
    def center(self):
        return (self.car.front[0] + self.viewdepth / 2 * cos(self.car.pose),
                self.car.front[1] + self.viewdepth / 2 * sin(self.car.pose))

    @property
    def topleft(self):
        return (self.center[0] - self.viewdepth / 2,
                self.center[1] - self.viewwidth / 2)

    def render(self, surface):
        if not self.visible_lanes:
            return

        '''
        if self.surf is None and self.map.loaded:
            self.surf = pygame.Surface(self.map.mtopx_tuple((self.viewdepth, self.viewwidth))).convert_alpha()
            pygame.draw.rect(self.surf, (255, 135, 68, 100),
                             ((0, 0), self.map.mtopx_tuple((self.viewdepth, self.viewwidth))))

        rect = self.surf.get_rect()
        rot = pygame.transform.rotate(self.surf, degrees(-self.car.pose))
        rot_rect = rot.get_rect(center=rect.center).move(self.map.mtopx_tuple(self.topleft))
        surface.blit(rot, rot_rect)
        '''

        if len(self.lanes[0]) > 1:
            for point in [self.lanes[0][0], self.lanes[0][1]]:
                pygame.draw.circle(surface, (142, 142, 255), self.map.mtopx_tuple((point[0],point[1])), 7)
        if len(self.lanes[1]) > 1:
            for point in [self.lanes[1][0], self.lanes[1][1]]:
                pygame.draw.circle(surface, (142, 142, 255), self.map.mtopx_tuple((point[0],point[1])), 7)
        pass
