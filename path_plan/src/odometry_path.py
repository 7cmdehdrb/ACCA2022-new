#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import numpy as np
import math as m
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry
from cubic_spline_planner import calc_spline_course

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
except Exception as ex:
    rospy.logfatal(ex)


def posePublish(cx, cy, cyaw):
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

    Pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("odom_path")

    Pub = rospy.Publisher("/create_global_path", PoseArray, queue_size=1)

    st = State(odometry_topic="/odometry/kalman")

    xs = []
    ys = []
    yaws = []

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        xs.append(st.x)
        ys.append(st.y)
        # yaws.append(state.yaw)

        new_xs = []
        new_ys = []
        # new_yaws = []

        for v in xs:
            if v not in new_xs:
                new_xs.append(v)

        for i in ys:
            if i not in new_ys:
                new_ys.append(i)

        # print(new_xs, new_ys)

        if len(new_xs) < 5:
            continue

        cx, cy, cyaw, _, _ = calc_spline_course(new_xs, new_ys)

        posePublish(cx, cy, cyaw)

        r.sleep()
