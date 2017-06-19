#!/usr/bin/env python

import rospy
from behavior.machine.master_manager import MasterManager
from objects.vehicle import Vehicle
from simulator.msg import PoseMsg
from simulator.msg import LanesMsg
from simulator.msg import MissionMsg
from simulator.msg import TargetMsg
from simulator.msg import StatusMsg
from simulator.msg import MissionStatusMsg


class BehaviorPlanner:
    def __init__(self):
        self.machine = MasterManager()
        self.vehicle = Vehicle()

        # subscribe to
        self.pose_topic = 'pose'
        self.lanes_topic = 'lanes'
        self.mission_mdf_topic = 'mission/mdf'
        # publish to
        self.target_topic = 'target'
        self.status_topic = 'status'
        self.mission_status_topic = 'mission/status'

        self.sub_pose = rospy.Subscriber(self.pose_topic, PoseMsg, self.set_pose)
        self.sub_lanes = rospy.Subscriber(self.lanes_topic, LanesMsg, self.set_lanes)
        self.sub_mission_mdf = rospy.Subscriber(self.mission_mdf_topic, MissionMsg, self.set_mission)
        self.pub_target = rospy.Publisher(self.target_topic, TargetMsg, queue_size=10)
        self.pub_status = rospy.Publisher(self.status_topic, StatusMsg, queue_size=10)
        self.pub_mission_status = rospy.Publisher(self.mission_status_topic, MissionStatusMsg, queue_size=10)

    def run(self):
        rospy.init_node('behavior_planner')

        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():

            # Run Machine
            self.machine.run(self.vehicle)

            # Send Target Trajectory
            target = TargetMsg()
            target.start = self.vehicle.target.start
            target.end = self.vehicle.target.end
            target.speed_limit = self.vehicle.target.speed_limit

            self.pub_target.publish(target)

            # Send Machine Status
            status = StatusMsg()
            status.states = self.machine.get_state_path()
            status.target = self.vehicle.mission.current

            self.pub_status.publish(status)

            # Send Mission Status
            mission = MissionStatusMsg()
            mission.done = self.vehicle.mission_accomplished

            self.pub_mission_status.publish(mission)

            rate.sleep()

    def set_pose(self, pose_msg):
        self.vehicle.set_pose(pose_msg.x, pose_msg.y, pose_msg.speed, pose_msg.heading)

    def set_lanes(self, lanes_msg):
        self.vehicle.set_lanes(lanes_msg)

    def set_mission(self, mission_msg):
        self.vehicle.mission.set_mission(mission_msg)

if __name__ == "__main__":
    print("Mission Planner Started.")
    BehaviorPlanner().run()
