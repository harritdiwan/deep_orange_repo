import pygame
from pygame.locals import *
from graphics.utils import utils
from sets import Set
import graphviz as gv
import cairo
import rsvg
import array
from behavior.machine.master_manager import MasterManager
from behavior.machine.template.machine import Machine
from fsmvisualizer import constants
from PIL import Image
from xml.dom import minidom
from collections import deque

import rospy
from simulator.msg import StatusMsg
import atexit


class FsmScreenTransitions():
    def __init__(self):
        self._drawsurface = pygame.Surface(constants.SIZE)
        self.machine = MasterManager()
        self.graph_layout = None

        # Generate graph -> pygame surface
        self.current_state = deque(maxlen=5)
        self.current_state.append(self.machine.get_state_path()[-1])
        gr = self.generateGraph(self.current_state)

        # Resize surface
        sz = utils.getsize(gr.get_rect().size, constants.SIZE)
        self.pos = utils.getposition(sz['size'], constants.SIZE)
        self.graph = pygame.transform.smoothscale(gr, sz["size"]).convert()

        # Subscribe to
        self.status_topic = 'status'
        self.sub_status = rospy.Subscriber(self.status_topic, StatusMsg, self.set_status, queue_size=10)
        rospy.init_node('fsm_visualizer', anonymous=True)

        atexit.register(self.cleanup)

    def cleanup(self):
        if self.graph_layout:
            self.graph_layout.unlink()

    def listRightIndex(self, alist, value):
        return len(alist) - alist[-1::-1].index(value) - 1

    def set_status(self, status):
        # Generate graph -> pygame surface
        if status.states[-1] != self.current_state[-1]:
            self.current_state.append(status.states[-1])
            gr = self.generateGraph(self.current_state)

            # Resize surface
            sz = utils.getsize(gr.get_rect().size, constants.SIZE)
            self.pos = utils.getposition(sz['size'], constants.SIZE)
            self.graph = pygame.transform.smoothscale(gr, sz["size"]).convert()

    def on_event(self, event):
        if event.type in (MOUSEMOTION, MOUSEBUTTONDOWN, MOUSEBUTTONUP):
            event.dict['pos'] = utils.convertpoint(event.dict['pos'], event.dict['offset'], event.dict['scale'])

    def bgra_surf_to_rgba_string(self, cairo_surface):
        # We use PIL to do this
        img = Image.frombuffer(
            'RGBA', (cairo_surface.get_width(),
                     cairo_surface.get_height()),
            cairo_surface.get_data(), 'raw', 'BGRA', 0, 1)

        return img.tobytes('raw', 'RGBA', 0, 1)

    def generateGraph(self, active):
        return self.generateGraphCairo(active)
        # return self.generateGraphPng(active)

    def generateGraphPng(self, active):
        g = gv.Digraph(format='png')
        self.getTree(g, self.machine, active)
        g.render("fsmvisualizer/FSM_transitions")
        return pygame.image.load("fsmvisualizer/FSM_transitions.png")

    def generateGraphCairo(self, active):
        if not self.graph_layout:
            g = gv.Digraph(format='svg')
            self.getTree(g, self.machine, active)
            sdata = g.pipe().decode('utf-8')
            self.graph_layout = minidom.parseString(sdata)

        svg_tag = self.graph_layout.getElementsByTagName("svg")[0]
        width = int(svg_tag.getAttribute('width')[:-2]) * 1.25
        height = int(svg_tag.getAttribute('height')[:-2]) * 1.25
        g_tags = svg_tag.getElementsByTagName("g")
        for element in g_tags:
            if not element.childNodes[0].childNodes:
                continue
            if active and element.childNodes[0].childNodes[0].nodeValue in active:
                element.childNodes[2].attributes['fill'] = 'yellow'
                idx = self.listRightIndex(list(active), element.childNodes[0].childNodes[0].nodeValue)
                opacity = (idx + 1) / float(len(active))
                element.childNodes[2].attributes['fill-opacity'] = str(opacity)
                if idx == len(active) - 1:
                    element.childNodes[2].attributes['stroke'] = 'red'
                else:
                    element.childNodes[2].attributes['stroke'] = 'black'
            else:
                element.childNodes[2].attributes['fill'] = 'none'
                element.childNodes[2].attributes['stroke'] = 'black'
                element.childNodes[2].attributes['fill-opacity'] = '1'
        sdata = self.graph_layout.toxml()

        WIDTH = int(width)
        HEIGHT = int(height)
        data = array.array('c', chr(0) * WIDTH * HEIGHT * 4)
        surface = cairo.ImageSurface.create_for_data(
            data, cairo.FORMAT_ARGB32, WIDTH, HEIGHT, WIDTH * 4)

        svg = rsvg.Handle(None, sdata)
        ctx = cairo.Context(surface)
        svg.render_cairo(ctx)
        cr = self.bgra_surf_to_rgba_string(surface)

        return pygame.image.frombuffer(cr, (WIDTH, HEIGHT), "RGBA")

    def getStates(self, g, machine):
        states = Set()

        if not isinstance(machine, Machine):
            return states

        for source, activation, destination in machine.transitions:
            states.add(source)
            states.add(destination)
            g.node(source.string())
            g.node(destination.string())
            g.edge(source.string(), destination.string(), color='black', label=activation.__name__)
        states.add(machine.initial_state)

        return states

    def getTree(self, g, mc, active):
        g.node(mc.string())
        for state in self.getStates(g, mc):
            # if state.string() in active:
            #     g.node(state.string(), style='filled', fillcolor='yellow')
            # else:
            #     g.node(state.string())
            if state == mc.initial_state:
                g.edge(mc.string(), state.string(), color='red')
            self.getTree(g, state, active)

    def draw(self, elapsed):
        self._drawsurface.fill(constants.WHITE)
        self._drawsurface.blit(self.graph, self.pos)
        return self._drawsurface
