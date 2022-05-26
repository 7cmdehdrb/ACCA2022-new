#!/usr/bin/env python

from loadPose import LoadPose
from state import State
from stanley import Stanley
import sys
import os
import rospy
import tf
import math as m
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from erp42_msgs.msg import SerialFeedBack


ACCA_FOLDER = rospy.get_param("/acca_folder", "/home/enbang/catkin_ws/src")
ODOMETRY_TOPIC = rospy.get_param("/odometry_topic", "/odom")

max_steer = rospy.get_param("/max_steer", 30.0)  # DEG
initial_idx = int(rospy.get_param("/initial_idx", 0))

global_path_file = rospy.get_param("/global_path_file", "static_path.csv")


"""
Export module for global path and stanley control
"""


class PathFinder(object):
    def __init__(self, load):
        super(PathFinder, self).__init__()

        self.path = Path()

        self.load = load

        self.cx = load.cx
        self.cy = load.cy
        self.cyaw = load.cyaw


class GlobalStanley(object):
    def __init__(self, state, cmd_msg, cmd_publisher, main_path_file=global_path_file):
        super(GlobalStanley, self).__init__()

        self.state = state
        self.stanley = Stanley()
        self.load = LoadPose(file_name=main_path_file)
        self.path = PathFinder(load=self.load)

        self.desired_speed = rospy.get_param("/desired_speed", 5.0)

        self.pubFlag = True

        self.cmd_msg = cmd_msg
        self.cmd_pub = cmd_publisher
        self.doPublishingMsg = True

        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = "map"

        self.path_pub = rospy.Publisher(
            "/cublic_global_path", Path, queue_size=1)
        self.goal_pub = rospy.Publisher(
            "/global_goal", PoseStamped, queue_size=5)

        self.last_idx = len(self.path.cx) - 1
        self.target_idx, _ = self.stanley.calc_target_index(
            self.state, self.path.cx[:500], self.path.cy[:500])

        if initial_idx != 0:
            self.target_idx = initial_idx

    def setDesiredSpeed(self, value):
        self.desired_speed = value
        return self.desired_speed

    def main(self):
        target_idx = self.target_idx

        di, target_idx = self.stanley.stanley_control(
            self.state, self.path.cx[:target_idx+100], self.path.cy[:target_idx+100], self.path.cyaw[:target_idx+100], target_idx)

        self.target_idx = target_idx
        di = np.clip(di, -m.radians(max_steer), m.radians(max_steer))

        speed, brake = self.checkGoal(
            last_idx=self.last_idx, current_idx=self.target_idx)

        if self.doPublishingMsg is True:

            self.cmd_msg.speed = speed
            self.cmd_msg.steer = -di
            self.cmd_msg.brake = brake

            self.cmd_pub.publish(self.cmd_msg)

        if self.pubFlag is True:
            # self.path.load.pathPublish(pub=self.path_pub)
            print("PUBLISH PATH")
            self.load.pathPublish(pub=self.path_pub)
            self.pubFlag = False

        self.goal_msg.header.stamp = rospy.Time.now()

        self.goal_msg.pose.position.x = self.path.cx[self.target_idx]
        self.goal_msg.pose.position.y = self.path.cy[self.target_idx]

        quat = tf.transformations.quaternion_from_euler(
            0.0, 0.0, self.path.cyaw[self.target_idx])

        self.goal_msg.pose.orientation.x = quat[0]
        self.goal_msg.pose.orientation.y = quat[1]
        self.goal_msg.pose.orientation.z = quat[2]
        self.goal_msg.pose.orientation.w = quat[3]

        self.goal_pub.publish(self.goal_msg)

    def checkGoal(self, last_idx, current_idx):
        temp_speed = 0.0
        temp_brake = 1

        if abs(last_idx - current_idx) < 10:
            temp_speed = 0.0
            temp_brake = 80

        else:
            temp_speed = self.desired_speed
            temp_brake = 1

        return temp_speed, temp_brake


if __name__ == "__main__":
    rospy.init_node("global_stanley")

    state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)
    cmd_pub = rospy.Publisher("/Control_msg", SerialFeedBack, queue_size=1)

    cmd_msg = SerialFeedBack()

    rospy.Subscriber(ODOMETRY_TOPIC, Odometry, state.odometryCallback)

    global_stanley = GlobalStanley(
        state=state, cmd_msg=cmd_msg, cmd_publisher=cmd_pub)

    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        global_stanley.main()
        r.sleep()
