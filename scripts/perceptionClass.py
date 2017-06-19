#!/usr/bin/env python

# Author: Jasprit S Gill
# Date: October 15, 2016
# 

import sys
import rospy
import cv2
import os
import thread
import threading
#from std_msgs.msg import String
import std_msgs.msg
import sensor_msgs.msg
import nav_msgs
from cv_bridge import CvBridge, CvBridgeError
from dbw_mkz_msgs.msg import SteeringReport
from do8autonomouscar.msg import objectMap
from do8autonomouscar.msg import obstacleInfo

class PerceptionManager:

    def __init__(self):
        self.centerImage = None
        self.imuData = None
        self.radarData = None
        self.stereoImageData = None

        self.semaphore = threading.Semaphore()    # Semaphore for mutual exclusion of the images

        self.bridge = CvBridge()
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # topic names for the node
        self.centerCameraTopic = "center_camera/image_color"
        self.stereoCameraTopic = "stereo_camera/image"
        self.radarTopic = "radar_topic"
        self.imuTopic = "vehicle/IMU"

        # publishers
        self.pub = rospy.Publisher('perception', objectMap, queue_size=10)

        # subscribers
        rospy.Subscriber(self.stereoCameraTopic, sensor_msgs.msg.Image, self.stereoCallback)
        rospy.Subscriber(self.centerCameraTopic, sensor_msgs.msg.Image, self.imageCallback)
        rospy.Subscriber(self.radarTopic, sensor_msgs.msg.PointCloud, self.radarCallback)
        rospy.Subscriber(self.imuTopic, sensor_msgs.msg.Imu, self.imuCallback)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        # TODO: Release/close any resources (semaphores, locks, file descripters, etc)
        return self

    # Set data functions
    def setImuData(self, data):
        with self.semaphore:
            self.imuData = data

    def setCenterImageData(self, data):
        with self.semaphore:
            self.centerImage = data

    def setRadarData(self, data):
        with self.semaphore:
            self.radarData = data

    def setStereoImageData(self, data):
        with self.semaphore:
            self.stereoImageData = data

    def getCenterImageData(self):
        cv_image = None
        if self.centerImage:
            with self.semaphore:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(self.centerImage, "bgr8")
                except CvBridgeError as e:
                    print(e)

        return cv_image

    def imageDetection(self):
        image = self.getCenterImageData()

        if not image == None:
            cv2.imshow("Image window", image)
            cv2.waitKey(2)

            # Deep learning code for detection and classification comes over here and updates the classification and depth
        return

    def sensorFusion(self):
        with self.semaphore:
            rospy.logdebug("Performing sensor fusion...")
            # Code for sensor fusion comes over here below (make sure its in the with block)

    # Call back functions for the sensor data
    def imuCallback(self, data):
        self.setImuData(data)

    def stereoCallback(self, data):
        rospy.logdebug('Stereo camera data received')
        self.setStereoImageData(data)

    def radarCallback(self, data):
        rospy.logdebug('Radar data received')
        self.setRadarData(data)

    def imageCallback(self, data):
        rospy.logdebug('Image received')
        self.setCenterImageData(data)

    def publishObjectMap(self, obstacleInfo, occupancygrid):
        obstacles = objectMap()
        obstacles.objects = obstacleInfo
        obstacles.header = std_msgs.msg.Header()
        obstacles.header.stamp = rospy.Time.now()
        obstacles.occupancyGrid = occupancygrid
        self.pub.publish(obstacles)

    def perceptionModule(self):
        rospy.init_node('perceptionNode', log_level= rospy.DEBUG)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # detection using deep learning
            self.imageDetection()

            #sensor fusion
            self.sensorFusion()

            # update the following from the sensor fusion
            obs_info = [obstacleInfo()]
            grid = nav_msgs.msg.OccupancyGrid()

            # publish the outcome of the sensor fusion
            self.publishObjectMap(obs_info, grid)
            hello_str = "Time %s" % rospy.get_time()
            rospy.loginfo(hello_str)

            rate.sleep()

def main(args):
    with PerceptionManager() as pm:

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        pm.perceptionModule()

    print('Destroying all the windows...')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
