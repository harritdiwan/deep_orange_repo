#!/usr/bin/env python

# Author: Juan Gutierrez Aguilar
# Date: April 28, 2017

import rospy
from simulator.msg import MissionMsg
from simulator.msg import PoseMsg
from mission.RNDF import s, v
from objects.vehicle import Vehicle
from objects.point import Point
from objects.waypoint import Waypoint


class MissionPlanner:

    def __init__(self):
        self.vehicle = Vehicle()

        self.mission_top = 'mission/mdf'
        self.request_top = 'mission/request'
        self.pose_top = 'pose'

        self.mission_pub = rospy.Publisher(self.mission_top, MissionMsg, queue_size=10)
        self.pose_sub = rospy.Subscriber(self.pose_top, PoseMsg, self.set_pose)
        self.request_sub = rospy.Subscriber(self.request_top, PoseMsg, self.path_to_goal)

    def set_pose(self, pose):
        self.vehicle.pose = pose

    def path_to_goal(self, start, goal):
        # Find best path
        start_wp = self.vehicle.mission.find_closest_waypoint(start.x, start.y)
        goal_wp = self.vehicle.mission.find_closest_waypoint(goal.x, goal.y)

        shortest = self.vehicle.mission.find_shortest_path(start_wp, goal_wp)

        # Flatten for indexing
        flatten = []
        for i in s:
            for j in i:
                flatten.append(j)

        for point in shortest:
            waypoint = Waypoint(point[0], point[1])

            m = flatten.index((waypoint.x, waypoint.y))

            waypoint.type = 'road'
            waypoint.type = 'stop' if v[m][1] == 'stop_exit' else waypoint.type
            waypoint.type = 'road' if v[m][1] == 'checkpoint' else waypoint.type
            waypoint.speed_limit = v[m][0]

            self.vehicle.mission.add_waypoint(waypoint.x, waypoint.y, waypoint.type, waypoint.speed_limit)

        self.vehicle.mission.waypoints[-1].type = 'checkpoint'

    # ROS Publisher
    def run(self):
        rospy.init_node('mission_planner')
        rate = rospy.Rate(10)  # 10hz

        mission_msg = MissionMsg()

        start = Point(19.25, 32.50)
        goal = Point(94.500, 61.750)
        self.path_to_goal(start, goal)

        while not rospy.is_shutdown():
            self.vehicle.mission.update(self.vehicle)

            mission_msg.waypoints = self.vehicle.mission.waypoints
            mission_msg.current = self.vehicle.mission.current

            self.mission_pub.publish(mission_msg)

            rate.sleep()

# Main program
if __name__ == '__main__':

    print("Mission Planner Started.")

    # # Starting location (x,y)
    # x = 19.25
    # y = 32.50
    # # Destination
    # goal = s[6][4]  # PickUp s[0][6] // DropOff s[4][6] // Charger s[6][4]

    MissionPlanner().run()
