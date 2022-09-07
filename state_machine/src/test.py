#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import tf
import math as m
import numpy as np
from enum import Enum
from erp42_control.msg import ControlMessage
from path_plan.msg import PathRequest, PathResponse
from lidar_camera_calibration.msg import Signmsg
from std_msgs.msg import Float32, Int16
from time import sleep
from geometry_msgs.msg import PoseStamped
from mission.msg import obTF



def path_callback(msg):
    path = msg
    
if __name__ == "__main__":
    path_response = rospy.Subscriber(
            "/path_response", PathResponse, callback=path_callback)    
    
    