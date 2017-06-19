#!/usr/bin/env python

# Author: Jasprit S Gill
# Date: May 9, 2017
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
from simulator.msg import PoseMsg
from do8autonomouscar.msg import objectMap
from do8autonomouscar.msg import obstacleInfo
from do8autonomouscar.msg import DeepOrangeVehicleMessage
from do8autonomouscar.msg import ObstacleCharacteristics
from do8autonomouscar.msg import obstacles
from do8autonomouscar.msg import DOVehicleInfoReceiveMessage
from do8autonomouscar.msg import vehicleState
from do8autonomouscar.msg import do8Trajectory
import do8autonomouscar
from rospy_message_converter import json_message_converter
from transforms3d import euler
import json
import socket
import math


class SimulatorInterface:
    def readFile(self, filename):
        file = open(filename, 'r')
        fileText = file.read()
        file.close()
        return fileText

    def writeFile(self, filename, text):
        file = open(filename, 'w')
        file.write(text)
        return

    def UDPServerSetup(self, ip_addr = "127.0.0.1", port = 25000):
        # Set up a UDP server
        self.UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Listen on port 21567
        # (to all IP addresses on this system)
        listen_addr = (ip_addr, port)
        self.UDPSock.bind(listen_addr)
        return

    def sendToRemote(self,text):
#        UDP_IP = "198.21.204.12"# 130.127.116.220
        UDP_IP = "130.127.116.220"# 130.127.116.220
        UDP_PORT = 25000
        MESSAGE = "Hello, World!"

        print "UDP target IP:", UDP_IP
        print "UDP target port:", UDP_PORT

        sock = socket.socket(socket.AF_INET,  # Internet
                             socket.SOCK_DGRAM)  # UDP
        sock.sendto(text, (UDP_IP, UDP_PORT))

    def jsonconverter(self,vmessage):
        v_m= json_message_converter.convert_ros_message_to_json(vmessage)
        return v_m

    def __init__(self):
        self.localIPAddress = "198.21.196.77"
        self.localPort = 26000
        self.centerImage = None
        self.imuData = None
        self.radarData = None
        self.stereoImageData = None

        self.semaphore = threading.Semaphore()    # Semaphore for mutual exclusion of the images

        # publishers
        self.odoPub = rospy.Publisher('pose', PoseMsg, queue_size=10)
        self.lanePub = rospy.Publisher('simulatorInterface/LaneInfo', do8autonomouscar.msg.laneInfo, queue_size=10)
        rospy.Subscriber('/motionPlanner/trajectory', do8Trajectory, self.trajectoryCallback)
#        self.obstaclePub = rospy.Publisher('simulatorInterface/obstacles', do8autonomouscar.msg.obstacleInfo, queue_size=10)
        self.UDPServerSetup(self.localIPAddress, self.localPort)

        # subscribers
#        rospy.Subscriber(self.imuTopic, sensor_msgs.msg.Imu, self.imuCallback)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        # TODO: Release/close any resources (semaphores, locks, file descripters, etc)
        self.UDPSock.close()
        return self

    def trajectoryCallback(self, trajectoryMessage):
        messagecopy = do8Trajectory()
        messagecopy.header = trajectoryMessage.header
        messagecopy.trajectory = trajectoryMessage.trajectory[:30]
        print('message is...')
#        print(messagecopy)
        jsonText = json_message_converter.convert_ros_message_to_json(messagecopy)
#        print (jsonText)
        self.sendToRemote(jsonText)
        return

    def publishOdoMessage(self, odo):
        odo_mine = PoseMsg()
        odo_mine.x = odo.pose.pose.position.x
        odo_mine.y = odo.pose.pose.position.y-0.8

        quat = ([odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z, odo.pose.pose.orientation.w])
        odo_mine.heading = euler.quat2euler(quat)[0]

        inst_v_x = odo.twist.twist.linear.x
        inst_v_y = odo.twist.twist.linear.y
        odo_mine.speed = math.sqrt(inst_v_x*inst_v_x + inst_v_y*inst_v_y)

        self.odoPub.publish(odo_mine)

    def publishLaneMessage(self, laneMessage):
        self.lanePub.publish(laneMessage)

    def populateOdometryMessage(self, unityMessage):
 #       unityMessage = DOVehicleInfoReceiveMessage()
        odo = nav_msgs.msg.Odometry()
        odo.header = unityMessage.header
        odo.pose.pose = unityMessage.pose
        odo.twist.twist.linear = unityMessage.velocity
        return  odo

    def populateLaneMessage(self, unityMessage):
#        unityMessage = DOVehicleInfoReceiveMessage()
        lanes = do8autonomouscar.msg.laneInfo()
        lanes.header = unityMessage.header
#        lanes.lanes.append(unityMessage.leftLaneMarkers)
        lanes.lanes = [unityMessage.leftLaneMarkers, unityMessage.rightLaneMarkers]
        return lanes

    def remoteInterfaceModule(self):
        rospy.init_node('simulatorInterface', log_level= rospy.DEBUG)
#        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            # Report on all data packets received and
            # where they came from in each case (as this is
            # UDP, each may be from a different source and it's
            # up to the server to sort this out!)
            data, addr = self.UDPSock.recvfrom(self.localPort)
            receivedJSONMessage = data.strip()
            print(receivedJSONMessage, addr)

#            receivedJSONMessage = self.readFile("Json_corrected.txt")
#            print("Printing unity message....:")
#            print(receivedJSONMessage)
            unityROSMessage = json_message_converter.convert_json_to_ros_message("do8autonomouscar/DOVehicleInfoReceiveMessage", receivedJSONMessage)
#            print(unityROSMessage)
            laneMessage = self.populateLaneMessage(unityROSMessage)
            odo = self.populateOdometryMessage(unityROSMessage)
            self.publishLaneMessage(laneMessage)
            self.publishOdoMessage(odo)

#            print ("Lane message...")
#            print(laneMessage )
#            print("Odometry mmessage...")
#            print (odo)

 #           rospy.sleep(5.)
        return

def main(args):
#    with RemoteSync() as rs:
    si = SimulatorInterface()
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
    si.remoteInterfaceModule()

    print('Destroying all the windows...')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
