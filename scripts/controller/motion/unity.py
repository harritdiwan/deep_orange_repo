import socket
import atexit
import json
import math
import pygame
from controller.motion.brain import brain
from transforms3d import euler
import rospy
from do8autonomouscar.msg import do8Trajectory
from simulator.msg import TargetMsg
from objects.target import Target
import threading


class unity(brain):
    def __init__(self,map,car):
        brain.__init__(self,map,car)
        self.sock = socket.socket(socket.AF_INET,  # Internet
                                  socket.SOCK_DGRAM)  # UDP
        self.sock.bind(("0.0.0.0", 26001))
        self.sock.setblocking(0)
        self.json = None
        self.traj=None

        self.traj_topic = 'motionPlanner/trajectory'
        self.sub_traj = rospy.Subscriber(self.traj_topic, do8Trajectory, self.update_traj)
        self.target_topic = 'target'
        self.sub_target = rospy.Subscriber(self.target_topic, TargetMsg, self.set_target)
        self.nref=None

        self.semaphore = threading.Semaphore()    # Semaphore for mutual exclusion of the images

        atexit.register(self.cleanup)
        rospy.init_node('simulator')

    def update_traj(self,data):
        with self.semaphore:
            self.traj=data.trajectory

    def set_target(self, trg):
        target = Target()
        target.speed_limit = trg.speed_limit
        target.start.x = trg.start.x
        target.start.y = self.map.transform_y_back(trg.start.y)
        target.start.heading = trg.start.heading
        target.start.speed = trg.start.speed
        target.end.x = trg.end.x
        target.end.y = self.map.transform_y_back(trg.end.y)
        target.end.heading = trg.end.heading
        target.end.speed = trg.end.speed
        self.nref=target

    def cleanup(self):
        if self.sock:
            self.sock.close()

    def setinput(self):
        try:
            data, addr = self.sock.recvfrom(1024*1024)  # buffer size is 1024 bytes
            jd = json.loads(data)
            self.json = jd
            x = jd['pose']['position']['x']
            y = self.map.transform_y_back(jd['pose']['position']['y'])
            quat = ([jd['pose']['orientation']['x'], jd['pose']['orientation']['y'], jd['pose']['orientation']['z'], jd['pose']['orientation']['w']])
            self.car.pose = euler.quat2euler(quat)[0]+math.pi/2
            # print (self.car.pose)*180/math.pi
            self.car.center = (x, y)
            self.car.speed = jd['velocity']['x'] * 3.6
        except socket.error:
            '''no data received..'''
            '''simulate with the model?'''
            pass
        return True

    def render(self, surface):
        with self.semaphore:
            # if self.json:
            #     rmarkers = self.json['rightLaneMarkers']['lane']
            #     lmarkers = self.json['leftLaneMarkers']['lane']
            #     for marker in rmarkers:
            #         pygame.draw.circle(surface, (255, 0, 255), self.map.mtopx_tuple((marker['x'], self.map.transform_y_back(marker['y']))), 7)
            #     for marker in lmarkers:
            #         pygame.draw.circle(surface, (255, 255, 0), self.map.mtopx_tuple((marker['x'], self.map.transform_y_back(marker['y']))), 7)
            #
            #     obstacles = self.json['obstacle_info']
            #     for obstacle in obstacles:
            #         pygame.draw.circle(surface, (255, 0, 0), self.map.mtopx_tuple((obstacle['pose']['position']['x'], self.map.transform_y_back(obstacle['pose']['position']['y']))), 7)
            if self.traj:
                print self.traj
                print len(self.traj)
                print ""
                for i in range(0,len(self.traj)):
                    print i
                    # print self.traj[i]
                    # print ""
                    px = self.traj[i].x
                    py = self.traj[i].y
                    pygame.draw.circle(surface, (255, 204, 204), self.map.mtopx_tuple((px, py)), 7)
            if self.nref:
                pygame.draw.circle(surface, (255, 255, 0), self.map.mtopx_tuple((self.nref.start.x, self.nref.start.y)), 7)
                pygame.draw.circle(surface, (51, 204, 204), self.map.mtopx_tuple((self.nref.end.x, self.nref.end.y)), 7)
                pass
