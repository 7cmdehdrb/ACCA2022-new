#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import numpy as np
import math as m
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from cubic_spline_planner import calc_spline_course

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import OdomState, State
except Exception as ex:
    rospy.logfatal(ex)


class OdometryPath(object):
    def __init__(self):

        self.pub = rospy.Publisher(
            "/create_global_path", PoseArray, queue_size=1)

        rospy.Subscriber(
            "/reset_path", Empty, self.resetCallback)

        self.xs = []
        self.ys = []

    def resetCallback(self, msg):
        self.xs = []
        self.ys = []

    def posePublish(self, cx, cy, cyaw):
        path = PoseArray()

        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        for i in range(0, len(cx)):
            pose = Pose()

            quat = quaternion_from_euler(0, 0, cyaw[i])

            pose.position.x = cx[i]
            pose.position.y = cy[i]
            pose.position.z = 0.0

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            path.poses.append(pose)

        self.pub.publish(path)

    def appendPath(self, x, y):
        if len(self.xs) == 0 and len(self.ys) == 0:
            if x != 0:
                self.xs.append(x)
                self.ys.append(y)

        else:
            if self.calculateDistance(x, self.xs[-1], y, self.ys[-1]) > 1.0:
                self.xs.append(x)
                self.ys.append(y)

                return calc_spline_course(self.xs, self.ys)

        return None, None, None, None, None

    def calculateDistance(self, x1, x2, y1, y2):
        return m.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


if __name__ == "__main__":
    rospy.init_node("odometry_path")

    odom_topic = rospy.get_param(
        "/odometry_path/odom_topic", "/odometry/kalman")
    odom_path = OdometryPath()
    state = OdomState(odometry_topic=odom_topic)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        cx, cy, cyaw, _, _ = odom_path.appendPath(state.x, state.y)
        if cx is not None:
            odom_path.posePublish(cx, cy, cyaw)
        r.sleep()
