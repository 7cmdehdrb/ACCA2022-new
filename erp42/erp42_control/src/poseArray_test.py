import os
import sys
import rospy
import rospkg
import numpy as np
import math as m
from geometry_msgs.msg import *
from path_plan.msg import PathResponse
from erp42_control.msg import ControlMessage
from abc import *
from enum import Enum
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
import csv
from nav_msgs.msg import Odometry
from time import sleep


def pose_Callback(msg):
    a = msg.poses


if __name__ == "__main__":
    rospy.init_node("posearray_test")
    a = 'start'
    sub = rospy.Subscriber('/adaptive_clustering/poses',
                           PoseArray, callback=pose_Callback)
    sleep(1.0)

    r = rospy.Rate(1.)

    while not rospy.is_shutdown():
        print(a)

        r.sleep
