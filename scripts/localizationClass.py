#!/usr/bin/env python

# Author: Jasprit S Gill
# Date: October 17, 2016
# 

import sys
import rospy
import cv2
import numpy
import os
#import thread
import threading
#from std_msgs.msg import String
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from lanedetection import lanedetector as ln
#from dbw_mkz_msgs.msg import SteeringReport
#from do8autonomouscar.msg import objectMap
#from do8autonomouscar.msg import obstacleInfo
from do8autonomouscar.msg import laneInfo
from do8autonomouscar.msg import lane

class LocalizationManager:

    def __init__(self):
        self.centerImage = None
        self.imuData = None
        self.gpsData = None
        self.librarystateData = None
        self.odometry = nav_msgs.msg.Odometry()
        self.laneinfo = laneInfo()

        self.semaphore = threading.Semaphore()    # Semaphore for mutual exclusion of the images

        self.bridge = CvBridge()
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # topic names for the node
        self.centerCameraTopic = "camera/image_color"
        self.imuTopic = "vehicle/imu/data_raw"
        self.gpsTopic = "vehicle/gps/fix"
        self.librarystateData = "ssf_core/state_out/data"

        # publishers
        self.pub_lane = rospy.Publisher('localization/laneInfo', laneInfo, queue_size=10)
        self.pub_odometry = rospy.Publisher('localization/odometry', nav_msgs.msg.Odometry, queue_size=10)

        # subscribers
        rospy.Subscriber(self.gpsTopic, sensor_msgs.msg.NavSatFix, self.gpsCallback)
        rospy.Subscriber(self.centerCameraTopic, sensor_msgs.msg.Image, self.imageCallback)
        rospy.Subscriber(self.imuTopic, sensor_msgs.msg.Imu, self.imuCallback)
        rospy.Subscriber(self.librarystateData, std_msgs.msg.Float64, self.librarystateCallback)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        # TODO: Release/close any resources (semaphores, locks, file descripters, etc)
        print (type, value, traceback)
        return self

    # Set data functions
    def setlibrarystateData(self, data):
        with self.semaphore:
            self.librarystateData = data

    def setImuData(self, data):
        with self.semaphore:
            self.imuData = data

    def setCenterImageData(self, data):
        with self.semaphore:
            self.centerImage = data

    def setGPSData(self, data):
        with self.semaphore:
            self.gpsData = data

    def getCenterImageData(self):
        cv_image = None
        if self.centerImage:
            with self.semaphore:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(self.centerImage, 'bgr8')
                except CvBridgeError as e:
                    print(e)

        return cv_image

    def getlibrarystateData(self):
        state_data = None
        if self.librarystateData:
            state_data = self.librarystateData
        else:
            print("no library data received.")
        return state_data

    def laneDetection(self):
        image = self.getCenterImageData()
        self.laneinfo = laneInfo()
        self.laneinfo.lanes = lane()

        if not image == None:
            cv2.imshow("Image window", image)
            cv2.waitKey(2)

            height = image.shape[0]
            width = image.shape[1]

            # TODO: Code for lane detection and classification comes over here

            leftpointlist = []
            rightpointlist = []
            [leftPoints, rightPoints] = ln(image, 0, height, 0, width)

            for point_array in leftPoints:
                pt = geometry_msgs.msg.Point()
                pt.x = point_array[0]
                pt.y = point_array[1]
                pt.z = 0
                leftpointlist.append(pt)

            for point_array in rightPoints:
                pt = geometry_msgs.msg.Point()
                pt.x = point_array[0]
                pt.y = point_array[1]
                pt.z = 0
                rightpointlist.append(pt)

            leftLanePoints = lane(leftpointlist)
            rightLanePoints = lane(rightpointlist)
            lanesArray = [leftLanePoints, rightLanePoints]
            self.laneinfo.lanes = lanesArray
            # TODO: Populate the lane information in the laneinfo object
        else:
            print("No image detected")

        return

    def poseEstimation(self):
        with self.semaphore:
            state_data = self.getlibrarystateData()
            position_x = state_data[0]
            position_y = state_data[1]
            position_z = state_data[2]
            velocity_x = state_data[3]
            velocity_y = state_data[4]
            velocity_z = state_data[5]
            accn_bias_x = state_data[13]
            accn_bias_y = state_data[14]
            accn_bias_z = state_data[15]
            accn_scale = state_data[16]
            orientation_w = state_data[6]
            orientation_x = state_data[7]
            orientation_y = state_data[8]
            orientation_z = state_data[9]
            gyro_bias_x = state_data[10]
            gyro_bias_y = state_data[11]
            gyro_bias_z = state_data[12]
            #print("PoseEstimation: %f, %f and IMU: %f, %f", accel.x, accel.y, lat, longt)
            # TODO: Code for pose estimation comes over here below (make sure its in the with block)

        self.odometry = nav_msgs.msg.Odometry()
        # TODO: populate odometry message after implementing the pose estimation, populate velocity in it

    # Call back functions for the sensor data
    def imuCallback(self, data):
        rospy.logdebug('IMU data received')
        self.setImuData(data)
        self.poseEstimation()

    def gpsCallback(self, data):
        rospy.logdebug('GPS data received')
        self.setGPSData(data)

    def imageCallback(self, data):
        rospy.logdebug('Image received')
        self.setCenterImageData(data)

    def publishLaneInfo(self):
        self.laneinfo.header = std_msgs.msg.Header()
        self.laneinfo.header.stamp = rospy.Time.now()
        self.pub_lane.publish(self.laneinfo)

    def publishOdometry(self):
        self.odometry.header = std_msgs.msg.Header()
        self.odometry.header.stamp = rospy.Time.now()
        self.pub_odometry.publish(self.odometry)

    def localizationModule(self):
        rospy.init_node('localizationNode', log_level= rospy.INFO)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # Lane detection call
            self.laneDetection()

            #TODO: Figure out when to publish odometry, as the IMU data comes or the latest pose?
            #Get velocity from pose estimation
            self.publishOdometry()

            self.publishLaneInfo()

            rate.sleep()

def main(args):
    #with LocalizationManager() as pm:
    pm = LocalizationManager()
    # dc = dataCapture()
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pm.localizationModule()

    print('Destroying all the windows...')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
