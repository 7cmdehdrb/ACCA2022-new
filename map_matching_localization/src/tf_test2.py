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


def pclCallback(msg):
    msg.header.frame_id = "base_link"
    pcl_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pcl_header")

    pcl_sub = rospy.Subscriber(
        "velodyne_points", PointCloud2, callback=pclCallback)
    pcl_pub = rospy.Publisher("velodyne_points2", PointCloud2, queue_size=5)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.spin()