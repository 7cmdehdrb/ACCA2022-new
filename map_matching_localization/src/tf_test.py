#!/usr/bin/env python


import rospy
import rospkg
import math as m
import numpy as np
import time as t
import threading
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from point_cloud import array_to_xyz_pointcloud2f


odom_msg = None


def odomCallback(msg):
    global odom_msg
    odom_msg = msg


if __name__ == "__main__":
    rospy.init_node("tf_test")

    odom_sub = rospy.Subscriber(
        "odometry/global", Odometry, callback=odomCallback)

    odom_broad = tf.TransformBroadcaster()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        if odom_msg is None:
            continue

        trans = (odom_msg.pose.pose.position.x,
                 odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
        rot = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
               odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]

        odom_broad.sendTransform(
            trans, rot, rospy.Time.now(), "base_link", "odom"
        )

        rospy.loginfo("Publishing...")

        r.sleep()
