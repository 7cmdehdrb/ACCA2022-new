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


def odomCallback(msg):
    trans = (msg.pose.pose.position.x,
             msg.pose.pose.position.y, msg.pose.pose.position.z)
    rot = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    odom_broad.sendTransform(
        trans, rot, rospy.Time.now(), "base_link", "odom"
    )

    rospy.loginfo("Publishing...")


if __name__ == "__main__":
    rospy.init_node("odom_tf")

    odom_sub = rospy.Subscriber(
        "odometry/global", Odometry, callback=odomCallback)

    odom_broad = tf.TransformBroadcaster()

    rospy.spin()
