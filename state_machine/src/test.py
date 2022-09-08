#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import tf
import math as m
import numpy as np
from enum import Enum
from time import sleep
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from lidar_camera_calibration.msg import Signmsg
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import PoseStamped


cx = np.arange(0, 10, 0.1)
cy = np.arange(10, 0, -0.1)

target = np.array([3.2, 3.1])

dists = []

for i in range(len(cx)):
    dist = dists.append(np.hypot(cx[i] - target[0], cy[i] - target[1]))

dists = np.array(dists)
target_idx = np.argmin(dists)


for i in range(len(cx)):
    print(cx[i], cy[i], np.hypot(cx[i] - target[0], cy[i] - target[1]))

print(cx[target_idx], cy[target_idx])
