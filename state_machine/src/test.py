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
from std_msgs.msg import Empty, UInt8



def path_callback(msg):
    path = msg
    
if __name__ == "__main__":
  
    
    while not rospy.is_shutdown():
        print('a')
        b = rospy.wait_for_message("/overwrite_ans", UInt8)
        print(b.data)