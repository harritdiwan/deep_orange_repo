from math import *

import pygame
from pygame.locals import *

import constants
from behavior.machine.master_manager import MasterManager
from mission.mission import Mission
from objects.vehicle import Vehicle
from objects.lanes import Lanes
from objects.point import Point
from objects.waypoint import Waypoint
from controller.mission.mission import mission
from controller.mission.node import node
from controller.motion.brain import brain
from controller.perception.perception import perception
from controller.tracker.tracker_trajectory import tracker
from controller.planner.trajectory import *
from controller.planner.frame import Frame

import rospy
from simulator.msg import PoseMsg
from simulator.msg import LanesMsg
from simulator.msg import MissionMsg
from simulator.msg import WaypointMsg
from simulator.msg import TargetMsg
from simulator.msg import StatusMsg
from simulator.msg import MissionStatusMsg
from std_msgs.msg import String
from objects.target import Target


class autonomous(brain):
    def __init__(self,map,car):
        brain.__init__(self,map,car)
        self.mission = mission(map)
        self.perception = perception(map,car)
        self.vehicle = Vehicle()
        self.tracker = tracker(car, map)
        self.font = pygame.font.SysFont("default", 36)
        self.current_state = []

        # subscribe to
        self.target_topic = 'target'
        self.status_topic = 'status'
        self.mission_status_topic = 'mission/status'
        self.mission_mdf_topic = 'mission/mdf'

        # publish to
        self.pose_topic = 'pose'
        self.lanes_topic = 'lanes'
        self.mission_topic = 'mission/mdf'

        self.pub_pose = rospy.Publisher(self.pose_topic, PoseMsg, queue_size=10)
        self.pub_lanes = rospy.Publisher(self.lanes_topic, LanesMsg, queue_size=10)
        self.pub_mission = rospy.Publisher(self.mission_topic, MissionMsg, queue_size=10)
        self.sub_target = rospy.Subscriber(self.target_topic, TargetMsg, self.set_target)
        self.sub_status = rospy.Subscriber(self.status_topic, StatusMsg, self.set_status)
        self.sub_mission_status = rospy.Subscriber(self.mission_status_topic, MissionStatusMsg, self.mission_status)
        self.sub_mission_mdf = rospy.Subscriber(self.mission_topic, MissionMsg, self.mission_mdf)
        rospy.init_node('simulator')

        # local frame for trajectory
        self.frame = None
        self.planner = Trajectory_Planner_Blend()

    def mission_mdf(self, mission):
        if not self.vehicle.mission.exists(): # TEMP
            self.setVehicleMdf(mission)

    def mission_status(self, status):
        if status.done:
            self.vehicle.mission.waypoints = []
            self.vehicle.mission.current = 0

    def set_target(self, target):
        self.vehicle.set_target(target)

    def set_status(self, status):
        self.vehicle.mission.current = status.target
        self.current_state = status.states

    def setinput(self):
        pressed = pygame.key.get_pressed()

        view = self.perception.look()
        self.vehicle = self.buildVehicle(view)

        # Mission
        # if not self.vehicle.mission.exists():
        #     self.setVehicleMission()
        #     mission_msg = MissionMsg()
        #     for wayp in self.vehicle.mission.waypoints:
        #         wpm = WaypointMsg()
        #         wpm.x = wayp.x
        #         wpm.y = wayp.y
        #         wpm.type = wayp.type
        #         wpm.speed_limit = wayp.speed_limit
        #         mission_msg.waypoints.append(wpm)
        #     self.pub_mission.publish(mission_msg)

        # Pose
        pose_msg = PoseMsg()
        pose_msg.x = self.vehicle.pose.x
        pose_msg.y = self.map.transform_y_back(self.vehicle.pose.y)
        pose_msg.heading = -self.vehicle.pose.heading
        pose_msg.speed = self.vehicle.pose.speed
        self.pub_pose.publish(pose_msg)

        # Lanes
        lanes_msg = LanesMsg()
        right = Point()
        right_ahead = Point()
        left = Point()
        left_ahead = Point()

        if self.vehicle.lanes.are_visible():
            right.x = self.vehicle.lanes.right.x
            right.y = self.map.transform_y_back(self.vehicle.lanes.right.y)
            left.x = self.vehicle.lanes.left.x
            left.y = self.map.transform_y_back(self.vehicle.lanes.left.y)

            right_ahead.x = self.vehicle.lanes.right_ahead.x
            right_ahead.y = self.map.transform_y_back(self.vehicle.lanes.right_ahead.y)
            left_ahead.x = self.vehicle.lanes.left_ahead.x
            left_ahead.y = self.map.transform_y_back(self.vehicle.lanes.left_ahead.y)

        lanes_msg.right = right
        lanes_msg.left = left
        lanes_msg.right_ahead = right_ahead
        lanes_msg.left_ahead = left_ahead

        lanes_msg.right.z = 0
        lanes_msg.left.z = 0
        lanes_msg.right_ahead.z = 0
        lanes_msg.left_ahead.z = 0

        self.pub_lanes.publish(lanes_msg)

        if not pressed[K_q]:
            # autonomous player
            # build modified target (for path tracker)
            target = Target()
            target.speed_limit = self.vehicle.target.speed_limit
            target.start.x = self.vehicle.target.start.x
            target.start.y = self.map.transform_y_back(self.vehicle.target.start.y)
            target.start.heading = self.vehicle.target.start.heading
            target.start.speed = self.vehicle.target.start.speed
            target.end.x = self.vehicle.target.end.x
            target.end.y = self.map.transform_y_back(self.vehicle.target.end.y)
            target.end.heading = self.vehicle.target.end.heading
            target.end.speed = self.vehicle.target.end.speed

            # set local reference frame for trajectory
            # all angles constrained in [-pi/pi]
            wp_traversed = self.has_changed(target)

            if not self.frame or wp_traversed:
                # Harrit has y as car longitudinal axis
                car_heading = math.atan2(math.sin(self.car.pose + math.pi / 2), math.cos(self.car.pose + math.pi / 2))
                self.frame = Frame(target.start.x, target.start.y, car_heading)
                print "new frame:", target.start.x, target.start.y, target.end.x, target.end.y, self.car.center[0], self.car.center[1], car_heading*180/math.pi

            # convert to local frame
            current_wp_x, current_wp_y = self.frame.global2local(target.start.x, target.start.y)
            next_wp_x, next_wp_y = self.frame.global2local(target.end.x, target.end.y)
            car_x, car_y = self.frame.global2local(self.car.center[0], self.car.center[1])

            if (current_wp_x, current_wp_y) != (next_wp_x, next_wp_y) and target.start.x and target.start.y:
                target_heading = - target.end.heading - self.frame.angle  # Compensate for clockwise angle
                target_heading = math.atan2(math.sin(target_heading), math.cos(target_heading))  # Between [-pi, pi]
                target_heading = target_heading if target_heading>0 else 2*pi-target_heading  # Between [0, 2*pi]
                inst_heading = - self.car.pose - self.frame.angle # Compensate for clockwise angle
                inst_heading = math.atan2(math.sin(inst_heading), math.cos(inst_heading))  # Between [-pi, pi]
                inst_heading = inst_heading if inst_heading>0 else 2*pi-inst_heading  # Between [0, 2*pi]

                print "cwx: ", current_wp_x, "cwy:", current_wp_y, "nwx:", next_wp_x, "nwy:", next_wp_y, 5, 5, \
                    "cx:", car_x, "cy:", car_y, "s:", self.car.speed, "th: {0:.2f}".format(
                    target_heading*180/math.pi), "ih: {0:.2f}".format(
                    inst_heading*180/math.pi), "trv:", wp_traversed

                trajectory = self.planner.trajectory_planner(current_wp_x=current_wp_x, current_wp_y=current_wp_y,
                                                             next_wp_x=next_wp_x, next_wp_y=next_wp_y,
                                                             current_wp_v=target.start.speed, next_wp_v=target.end.speed,
                                                             # current_wp_v=5, next_wp_v=5,
                                                             current_x=car_x, current_y=car_y, inst_v=self.car.speed,
                                                             heading=target_heading, inst_heading=inst_heading,
                                                             wp_traversed=wp_traversed)
                print trajectory
                # Trajectory is empty, should recompute
                if len(trajectory[0]) < 2:
                    print "Empty trajectory, mission failed!"
                    self.tracker.setTarget(target)
                    return

                global_traj = [[0 for x in range(len(trajectory[0]))] for y in range(2)]
                for i in xrange(0, len(trajectory[0])):
                    (tx, ty) = self.frame.local2global(trajectory[0][i], trajectory[1][i])
                    global_traj[0][i] = tx
                    global_traj[1][i] = ty

                # if len(global_traj[0]) > 0:
                #     idx = int(min(len(global_traj[0]) - 1, max(10 * self.car.speed, 20)))
                #     target.end.x = global_traj[0][idx]
                #     target.end.y = global_traj[1][idx]

                input = self.tracker.getInput(target, global_traj)
                self.car.acc = input['acc']
                self.car.steer = input['steer']

                # Lateral error too large, should recompute
                if input['error']:
                    print "Lateral error too large, mission failed!"
                    return
        else:
            # human player
            if pressed[K_LEFT]:
                self.car.steer += -radians(4)
            elif pressed[K_RIGHT]:
                self.car.steer += radians(4)
            else:
                self.car.steer = 0

            if pressed[K_UP]:
                self.car.acc += 1
            elif pressed[K_DOWN]:
                self.car.acc += -1
            else:
                self.car.acc = 0

    def render(self, surface):
        if not self.vehicle.mission.exists():
            return

        # draw waypoints
        for wp in self.vehicle.mission.waypoints[:]:
            if wp.is_stop():
                pygame.draw.circle(surface, (255, 115, 66), self.map.mtopx_tuple((wp.x,wp.y)), 7)
            elif wp.is_checkpoint():
                pygame.draw.circle(surface, (142, 255, 142), self.map.mtopx_tuple((wp.x, wp.y)), 7)
            else:
                pygame.draw.circle(surface, (142, 142, 142), self.map.mtopx_tuple((wp.x,wp.y)), 7)
        if self.vehicle.mission.target_waypoint():
            target = self.vehicle.mission.target_waypoint()
            pygame.draw.circle(surface, (113, 244, 66), self.map.mtopx_tuple((target.x, target.y)), 7)

        # draw current state
        if self.vehicle.mission.exists():
            label = self.font.render("Limit: " + str(self.vehicle.mission.target_waypoint().speed_limit), 1, (0, 0, 0))
            surface.blit(label, (180, constants.HEIGHT - self.car.id * 36))

        states = self.current_state[-2 if self.current_state else 0:]
        label = self.font.render("State: " + ".".join(states), 1, (0, 0, 0))
        surface.blit(label, (300, constants.HEIGHT - self.car.id * 36))

        # draw detected lanes
        self.perception.render(surface)

        # draw tracker
        self.tracker.render(surface)

    def buildVehicle(self,perception):
        self.vehicle.set_position(Waypoint(self.car.center[0], self.car.center[1]))
        self.vehicle.set_heading(self.car.pose)
        self.vehicle.set_speed(self.car.speed)
        lanes = perception["lane_center"]

        if len(lanes[0])>1:
            # self.vehicle.set_lanes(lanes[0][0], lanes[0][1], lanes[1][0], lanes[1][1])
            # lanes[r/l][n][x/y]
            # self.vehicle.set_lanes(lanes[1][0][0], lanes[1][0][1], lanes[1][1][0], lanes[1][1][1])
            right = Point(lanes[0][0][0], lanes[0][0][1])
            right_ahead = Point(lanes[0][1][0], lanes[0][1][1])
        else:
            right = Point(0,0)
            right_ahead = Point(0,0)

        if len(lanes[1]) > 1:
            left = Point(lanes[1][0][0], lanes[1][0][1])
            left_ahead = Point(lanes[1][1][0], lanes[1][1][1])
        else:
            left = Point(0,0)
            left_ahead = Point(0,0)

        self.vehicle.set_lanes(Lanes(right, right_ahead, left, left_ahead))

        self.vehicle.set_obstacles([])
        return self.vehicle

    def setVehicleMdf(self,mission):
        vehicle_mission = Mission()

        for wayp in mission.waypoints:
            y = self.map.transform_y_back(wayp.y)
            vehicle_mission.add_waypoint(wayp.x, y, wayp.type, speed_limit=wayp.speed_limit)

        # the car starts from the first waypoint
        if len(vehicle_mission.waypoints) > 1:
            self.car.pos = (vehicle_mission.waypoints[0].x, vehicle_mission.waypoints[0].y)
            pose = atan2(vehicle_mission.waypoints[1].y - vehicle_mission.waypoints[0].y,
                         vehicle_mission.waypoints[1].x - vehicle_mission.waypoints[0].x)
            self.car.pose = atan2(sin(pose), cos(pose))

        self.vehicle.set_mission(vehicle_mission)

    def setVehicleMission(self):
        vehicle_mission = Mission()
        reset = self.mission.setRandomGoal()

        # the car starts from the first waypoint
        if reset and len(self.mission.waypoints) > 1:
            self.car.pos = self.mission.waypoints[0].pos
            pose = atan2(self.mission.waypoints[1].pos[1] - self.mission.waypoints[0].pos[1],
                         self.mission.waypoints[1].pos[0] - self.mission.waypoints[0].pos[0])
            self.car.pose = atan2(sin(pose), cos(pose))
            self.mission.next_waypoint()

        for wayp in self.mission.waypoints:
            if node.CHECKPOINT in wayp.props:
                vehicle_mission.add_checkpoint(wayp.pos[0], wayp.pos[1], speed_limit=wayp.speed_limit)
            elif node.STOP in wayp.props:
                vehicle_mission.add_stop(wayp.pos[0], wayp.pos[1], speed_limit=wayp.speed_limit)
            elif node.ENTRY in wayp.props:
                vehicle_mission.add_entry(wayp.pos[0], wayp.pos[1], speed_limit=wayp.speed_limit)
            elif node.ROAD in wayp.props:
                vehicle_mission.add_waypoint(wayp.pos[0], wayp.pos[1], speed_limit=wayp.speed_limit)
        self.vehicle.set_mission(vehicle_mission)

    def has_changed(self,target):
        return self.frame and (target.start.x != self.frame.ox or
                               target.start.y != -self.frame.oy)

