#!/usr/bin/env python

# Author: Jasprit S Gill
# Date: October 19, 2016
# 

import sys
import rospy
import cv2
import os
import threading
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg

class MissionPlanner:

    def __init__(self):

        # topic names for listening
        odometryTopic = "localization/odometry"
        #TODO: Figure out if GPS is needed, and how to use this
        gpsTopic = "vehicle/gps/fix"

        self.odometry = None
        self.occupancyGrid = None

        #TODO: populate the gridmap from an RNDF file and MDF file
        self.map = None

        self.calculateGlobalRoute()

        self.semaphore = threading.Semaphore()    # Semaphore for mutual exclusion of the images

        # publishers
        self.pub_route = rospy.Publisher('MissionPlanner/OccupancyGrid', nav_msgs.msg.OccupancyGrid, queue_size=10)

        # subscribers
        rospy.Subscriber(gpsTopic, sensor_msgs.msg.NavSatFix, self.gpsCallback)
        rospy.Subscriber(odometryTopic, nav_msgs.msg.Odometry, self.odometryCallback)

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

    def setGPSData(self, data):
        with self.semaphore:
            self.gpsData = data

    def calculateGlobalRoute(self):
        # TODO: Use gridmap to calculate the route using A-star algorithm and populate the occupancy grid

        self.occupancyGrid = nav_msgs.msg.OccupancyGrid()
        return

    def updatePositionOnMap(self):
        #TODO: Use odometry to update the current location of the ego-car in the occupancy grid
        return

    # Call back functions for the sensor data
    def odometryCallback(self, data):
        rospy.logdebug('Odometry data received')
        self.setOdometryData(data)
        self.updatePositionOnMap()

    def gpsCallback(self, data):
        rospy.logdebug('GPS data received')
        self.setGPSData(data)

    def publishRouteInfo(self):
        self.occupancyGrid.header = std_msgs.msg.Header()
        self.occupancyGrid.header.stamp = rospy.Time.now()
        self.pub_route.publish(self.occupancyGrid)

    def missionPlannerModule(self):
        rospy.init_node('missionPlanner', log_level= rospy.DEBUG)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            self.publishRouteInfo()

            rate.sleep()

def main(args):
    with MissionPlanner() as mp:

        # dc = dataCapture()
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        mp.missionPlannerModule()

    print('Destroying all the windows...')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
