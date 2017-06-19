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
import geometry_msgs.msg
import nav_msgs.msg
import diagnostic_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from dbw_mkz_msgs.msg import SteeringReport
from do8autonomouscar.msg import objectMap
from do8autonomouscar.msg import obstacleInfo
from do8autonomouscar.msg import DeepOrangeVehicleMessage
from do8autonomouscar.msg import ObstacleCharacteristics

class RemoteSync:

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
        self.pub = rospy.Publisher('remoteSync/vehicleMessage', DeepOrangeVehicleMessage, queue_size=10)

        # subscribers
#        rospy.Subscriber(self.imuTopic, sensor_msgs.msg.Imu, self.imuCallback)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        # TODO: Release/close any resources (semaphores, locks, file descripters, etc)
        return self

    # Set data functions
    def setImuData(self, data):
        with self.semaphore:
            self.imuData = data


    # Call back functions for the sensor data
    def imuCallback(self, data):
        self.setImuData(data)

    def publishVehicleMessage(self, vehMessage):
        self.pub.publish(vehMessage)

    def populateVehicleMesage(self, point_x, point_y, point_z, orient_x, orient_y, orient_z, orient_w):
        odo = nav_msgs.msg.Odometry()
        odo.header = std_msgs.msg.Header()
        odo.header.stamp = rospy.Time.now()
        poseWithCov = geometry_msgs.msg.PoseWithCovariance()
        poseWithCov.pose.position.x = point_x
        poseWithCov.pose.position.y = point_y
        poseWithCov.pose.position.z = point_z
        poseWithCov.pose.orientation.x = orient_x
        poseWithCov.pose.orientation.y = orient_y
        poseWithCov.pose.orientation.z = orient_z
        poseWithCov.pose.orientation.w = orient_w
        odo.pose = poseWithCov

        twistWithCov = geometry_msgs.msg.TwistWithCovariance()
        twistWithCov.twist.linear.x = 0
        twistWithCov.twist.linear.y = 0
        twistWithCov.twist.linear.z = 0
        twistWithCov.twist.angular.x = 0
        twistWithCov.twist.angular.y = 0
        twistWithCov.twist.angular.z = 0
        odo.twist = twistWithCov

        diag = diagnostic_msgs.msg.DiagnosticStatus()
        diag.name = 'Name'
        diag.message = 'Message'
        diag.hardware_id = 'Hardware ID'

        keyVal = diagnostic_msgs.msg.KeyValue()
        keyVal.key = 'WheelSpeed'
        keyVal.value = '25, 24, 23, 26'
        diagnosticValues = []
        diagnosticValues.append(keyVal)

        keyVal2 = diagnostic_msgs.msg.KeyValue()
        keyVal2.key = 'SteeringAngle'
        keyVal2.value = '0.017'
        diagnosticValues.append(keyVal2)

        diag.values = diagnosticValues
        diagArray = []
        diagArray.append(diag)
        vehicMon = diagnostic_msgs.msg.DiagnosticArray()
        vehicMon.header = std_msgs.msg.Header()
        vehicMon.header.stamp = rospy.Time.now()
        vehicMon.status = diagArray

        obstChar = ObstacleCharacteristics()
        obstChar.description = 'Stop Sign'
        obstOdometry = nav_msgs.msg.Odometry()
        obstOdometry.header = std_msgs.msg.Header()
        obstOdometry.header.stamp = rospy.Time.now()

        obstaclePosition = geometry_msgs.msg.PoseWithCovariance()
        obstaclePosition.pose.position.x = 4
        obstaclePosition.pose.position.y = 2
        obstOdometry.pose = obstaclePosition
        obstChar.pose_velocity = obstOdometry

        obstacleBoundingBox = geometry_msgs.msg.Polygon()

        point1 = geometry_msgs.msg.Point()
        point1.x = 4.25
        point1.y = 1.75

        point2 = geometry_msgs.msg.Point()
        point2.x = 4.25
        point2.y = 2.25

        point3 = geometry_msgs.msg.Point()
        point3.x = 3.75
        point3.y = 2.25

        point4 = geometry_msgs.msg.Point()
        point4.x = 3.75
        point4.y = 1.75

        obstacleBoundingBox.points = [point1, point2, point3, point4]

        obstChar.shape = obstacleBoundingBox

        vehMessage = DeepOrangeVehicleMessage()
        vehMessage.header = std_msgs.msg.Header()
        vehMessage.header.stamp = rospy.Time.now()
        vehMessage.vehicle_id = 'DO8_car1'

        gpsLocation = sensor_msgs.msg.NavSatFix()
        gpsLocation.header = std_msgs.msg.Header()
        gpsLocation.header.stamp = rospy.Time.now()
        gpsLocation.latitude = 34.754233
        gpsLocation.longitude = 82.366872

        acc = geometry_msgs.msg.Accel()
        acc.linear.x = 0
        acc.linear.y = 0
        acc.linear.z = 0
        acc.angular.x = 0
        acc.angular.y = 0
        acc.angular.z = 0

        vehMessage.acceleration = acc
        vehMessage.RTK_GPS_Fixed_location = odo
        vehMessage.GPS_location = gpsLocation
        vehMessage.velocity = twistWithCov
        vehMessage.vehicleMonitor = vehicMon
        vehMessage.heading_direction = poseWithCov
        vehMessage.obstacles = obstChar
        vehMessage.battery_status = sensor_msgs.msg.BatteryState()
        vehMessage.behavior = 'Lane following'
        return vehMessage

    def remoteIntefaceModule(self):
        rospy.init_node('perceptionNode', log_level= rospy.DEBUG)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            # update the following from the sensor fusion
            vm = self.populateVehicleMesage(110, 61.75, 0, 0, 0, 0, 1)
            self.publishVehicleMessage(vm)

            hello_str = "Time %s: vehicle 1 transmitted" % rospy.get_time()
            rospy.loginfo(hello_str)

            rospy.sleep(5.)


            vm = self.populateVehicleMesage(134, 5.25, 0, 0, 0, 1, 0)
            self.publishVehicleMessage(vm)

            hello_str = "Time %s: vehicle 2 transmitted" % rospy.get_time()
            rospy.loginfo(hello_str)

            rospy.sleep(5.)

            vm = self.populateVehicleMesage(6.75, 35., 0, 0, 0, 0.7071, 0.7071)
            self.publishVehicleMessage(vm)


            # publish the outcome of the sensor fusion
            hello_str = "Time %s: vehicle 3 transmitted, retransmitting..." % rospy.get_time()
            rospy.loginfo(hello_str)

            rospy.sleep(5.)

def main(args):
#    with RemoteSync() as rs:
    rs = RemoteSync()
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
    rs.remoteIntefaceModule()

    print('Destroying all the windows...')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
