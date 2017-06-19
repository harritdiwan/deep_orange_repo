#!/usr/bin/env python

# Author: Jasprit S Gill
# Date: October 20, 2016
# 

import sys
import rospy
import cv2
import os
import threading
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from do8autonomouscar.msg import objectMap
from do8autonomouscar.msg import laneInfo
#from trajectory import trajectory_planner as tp
from controller.planner.trajectory import *
from controller.planner.frame import Frame
import math
from simulator.msg import TargetMsg
from simulator.msg import LanesMsg
from simulator.msg import PoseMsg
from do8autonomouscar.msg import do8Trajectory
from do8autonomouscar.msg import vehicleState
from objects.target import Target
import constants


class MotionPlanningManager:

    def __init__(self):

        # topic names for listening
        listeningTopicForOdometryTopic = "pose"
        listeningTopicForMissionOccupancyTopic = 'MissionPlanner/OccupancyGrid'
        listeningTopicsForobjectMapTopic = 'perception'
        #TODO: Figure out if GPS is needed, and how to use this
        listeningTopicForBehaviorOccupancy = 'behaviorPlanner/occupancyGrid'
        listeningTopicForTrajectory = 'target'
        listeningTopicForLanes = 'lanes'

        #Topic names for publishing
        publishTrajectoryInfo = 'motionPlanner/trajectory'

        # inputs
        self.odometry = PoseMsg()
        self.missionOccupancyGrid = nav_msgs.msg.OccupancyGrid()
        self.behavioralOccupancyGrid = nav_msgs.msg.OccupancyGrid()
        self.objectMap = objectMap()
        self.behavioralTrajectory = TargetMsg()
        self.lane_information = LanesMsg()
        self.trajectory = Trajectory_Planner_Blend()

        # outputs
        self.outputTrajectory = do8Trajectory()

        #TODO: populate the gridmap from an RNDF file and MDF file
        self.map = None
        self.frame = None
        self.semaphore = threading.Semaphore()    # Semaphore for mutual exclusion of the images

        # publishers
        self.pubTrajectory = rospy.Publisher(publishTrajectoryInfo, do8Trajectory, queue_size=10)

        # subscribers
        rospy.Subscriber(listeningTopicForBehaviorOccupancy, nav_msgs.msg.OccupancyGrid, self.behaviorOccupancyGridCallback)
        rospy.Subscriber(listeningTopicForOdometryTopic, PoseMsg, self.odometryCallback)
        rospy.Subscriber(listeningTopicsForobjectMapTopic, objectMap, self.objectMapCallback)
        rospy.Subscriber(listeningTopicForMissionOccupancyTopic, nav_msgs.msg.OccupancyGrid, self.missionOccupancyGridCallback)
        rospy.Subscriber(listeningTopicForTrajectory, TargetMsg, self.behavioralTrajectoryCallback)
        rospy.Subscriber(listeningTopicForLanes, LanesMsg, self.laneInfoCallback)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        # TODO: Release/close any resources (semaphores, locks, file descripters, etc)
        print (type, value, traceback)
        return self

    # Set data functions
    def setOdometryData(self, data):
        with self.semaphore:
            self.odometry = data

    def setBehavioralOccupancyGridMap(self, data):
        with self.semaphore:
            self.behavioralOccupancyGrid = data

    def setMissionOccupancyGridMap(self, data):
        with self.semaphore:
            self.missionOccupancyGrid = data

    def setObjectMap(self, data):
        with self.semaphore:
            self.objectMap = data

    def setLaneInfo(self, data):
        with self.semaphore:
            self.lane_information = data

    def setBehavioralTrajectory(self, data):
        with self.semaphore:
            self.behavioralTrajectory = data

    def getBehavioralTrajectory(self):
        behavior_planner_data = None
        if self.behavioralTrajectory:
            behavior_planner_data = self.behavioralTrajectory
        else:
            print("no behaviorplanner data received")
        return behavior_planner_data

    def getOdometryData(self):
        odometry_data = None
        if self.odometry:
            odometry_data = self.odometry
        else:
            print("no localization data received")
        return odometry_data

    # analysis and calculation functions
    def calculateTrajectory(self):
        self.outputTrajectory.trajectory = []
        with self.semaphore:
            # TODO: Use all inputs to identify the best trajectory to be followed
            rospy.logdebug("calculating the trajectory")
            behavior_data = self.getBehavioralTrajectory()
            localization_data = self.getOdometryData()
            # start_wp_x = behavior_data.start.x
            # start_wp_y = behavior_data.start.y
            # start_wp_v = behavior_data.start.speed
            # start_wp_heading = behavior_data.start.heading
            # end_wp_x = behavior_data.end.x
            # end_wp_y = behavior_data.end.y
            # end_wp_v = behavior_data.end.speed
            # end_wp_heading = behavior_data.end.heading
            #t3 = 2.0 * (localization_data.pose.pose.orientation.w * localization_data.pose.pose.orientation.z + localization_data.pose.pose.orientation.x * localization_data.pose.pose.orientation.y)
            #ysqr = localization_data.pose.pose.orientation.y * localization_data.pose.pose.orientation.y
            #t4 = 1.0 - 2.0 * (ysqr + localization_data.pose.pose.orientation.z() * localization_data.pose.pose.orientation.z())
            #inst_heading = math.atan2(t3,t4) - start_wp_heading
            # inst_heading = localization_data.heading
            #inst_v_x = localization_data.twist.twist.linear.x
            #inst_v_y = localization_data.twist.twist.linear.y
            #inst_v = math.sqrt(inst_v_x*inst_v_x + inst_v_y*inst_v_y)
            # inst_v = localization_data.speed
            # current_x = localization_data.x
            # current_y = localization_data.y

            target = Target()
            target.speed_limit = behavior_data.speed_limit
            target.start.x = behavior_data.start.x
            target.start.y = self.transform_y_back(behavior_data.start.y)
            target.start.heading = behavior_data.start.heading
            target.start.speed = behavior_data.start.speed
            target.end.x = behavior_data.end.x
            target.end.y = self.transform_y_back(behavior_data.end.y)
            target.end.heading = behavior_data.end.heading
            target.end.speed = behavior_data.end.speed

            print "loc1",localization_data
            localization_data.y = self.transform_y_back(localization_data.y)
            localization_data.heading = localization_data.heading+math.pi/2
            print "loc2",localization_data

            wp_traversed = self.has_changed(target)

            if not self.frame or wp_traversed:
                # Harrit has y as car longitudinal axis
                car_heading = math.atan2(math.sin(localization_data.heading + math.pi / 2), math.cos(localization_data.heading + math.pi / 2))
                self.frame = Frame(target.start.x, target.start.y, car_heading)
                print "new frame:", target.start.x, target.start.y, target.end.x, target.end.y, localization_data.x, localization_data.y, car_heading*180/math.pi
                wp_traversed = True

            # convert to local frame
            current_wp_x, current_wp_y = self.frame.global2local(target.start.x, target.start.y)
            next_wp_x, next_wp_y = self.frame.global2local(target.end.x, target.end.y)
            car_x, car_y = self.frame.global2local(localization_data.x, localization_data.y)

            if (current_wp_x, current_wp_y) != (next_wp_x, next_wp_y) and target.start.x and target.start.y:
                target_heading = - target.end.heading - self.frame.angle  # Compensate for clockwise angle
                target_heading = math.atan2(math.sin(target_heading), math.cos(target_heading))  # Between [-pi, pi]
                target_heading = target_heading if target_heading>0 else 2*math.pi-target_heading  # Between [0, 2*pi]
                inst_heading = - localization_data.heading - self.frame.angle # Compensate for clockwise angle
                inst_heading = math.atan2(math.sin(inst_heading), math.cos(inst_heading))  # Between [-pi, pi]
                inst_heading = inst_heading if inst_heading>0 else 2*math.pi-inst_heading  # Between [0, 2*pi]

                print "cwx: ", current_wp_x, "cwy:", current_wp_y, "nwx:", next_wp_x, "nwy:", next_wp_y, 5, 5, \
                        "cx:", car_x, "cy:", car_y, "s:", localization_data.speed, "th: {0:.2f}".format(
                        target_heading*180/math.pi), "ih: {0:.2f}".format(
                        inst_heading*180/math.pi), "trv:", wp_traversed

                C_alpha = 0.3
                A1=1
                A2=0.1
                A3=0.1
                steering_kappa=0.5
                la_dist = 1
                dt = 0.1
                [next_x_des, next_y_des, next_v_x_des, next_v_y_des, accn_x_des, accn_y_des, heading_des, time_d] = self.trajectory.trajectory_planner(current_wp_x=current_wp_x, current_wp_y=current_wp_y,
                                                                 next_wp_x=next_wp_x, next_wp_y=next_wp_y,
                                                                 current_wp_v=target.start.speed, next_wp_v=target.end.speed,
                                                                 # current_wp_v=5, next_wp_v=5,
                                                                 current_x=car_x, current_y=car_y, inst_v=localization_data.speed,
                                                                 heading=target_heading, inst_heading=inst_heading,
                                                                 wp_traversed=wp_traversed)

                global_traj = [[0 for x in range(len(next_x_des))] for y in range(2)]
                for i in range(0, len(next_x_des)):
                    (tx, ty) = self.frame.local2global(next_x_des[i], next_y_des[i])
                    global_traj[0][i] = tx
                    global_traj[1][i] = ty

                print "t",global_traj

                for i in range(len(next_x_des)):
                    self.vehiclestate = vehicleState()
                    self.vehiclestate.seq = i+1
                    self.vehiclestate.secs = time_d
                    self.vehiclestate.nsecs = time_d
                    self.vehiclestate.x = global_traj[0][i]
                    self.vehiclestate.y = global_traj[1][i]
                    self.vehiclestate.vx = next_v_x_des[i]
                    self.vehiclestate.vy = next_v_y_des[i]
                    self.vehiclestate.ax = accn_x_des[i]
                    self.vehiclestate.ay = accn_y_des[i]
                    self.vehiclestate.th = heading_des[i]
                    self.vehiclestate.k = 0
                    self.outputTrajectory.trajectory.append(self.vehiclestate)
        return

    def has_changed(self, target):
        return self.frame and (target.start.x != self.frame.ox or
                               target.start.y != -self.frame.oy)

    def transform_y_back(self,y):
        return (constants.MH + constants.TR) - y

    # Call back functions for the listener topics
    def odometryCallback(self, data):
        rospy.logdebug('Odometry data received')
        self.setOdometryData(data)

    def behaviorOccupancyGridCallback(self, data):
        rospy.logdebug('Behavior occupancy grid received')
        self.setBehavioralOccupancyGridMap(data)

    def missionOccupancyGridCallback(self, data):
        rospy.logdebug('Mission occupancy grid received')
        self.setMissionOccupancyGridMap(data)

    def objectMapCallback(self, data):
        rospy.logdebug('object map received')
        self.setObjectMap(data)

    def behavioralTrajectoryCallback(self, data):
        rospy.logdebug('behavioral trajectory received')
        self.setBehavioralTrajectory(data)

    def laneInfoCallback(self, data):
        rospy.logdebug('lane information received')
        self.setLaneInfo(data)

    # Publish functions
    def publishTrajectory(self):
        self.outputTrajectory.header = std_msgs.msg.Header()
        self.outputTrajectory.header.stamp = rospy.Time.now()
        self.pubTrajectory.publish(self.outputTrajectory)

    def motionPlannerModule(self):
        rospy.init_node('motionPlanner', log_level= rospy.DEBUG)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.calculateTrajectory()
            self.publishTrajectory()
            rate.sleep()

def main(args):
    with MotionPlanningManager() as mp:

        # dc = dataCapture()
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        mp.motionPlannerModule()

    print('Destroying all the windows...')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
