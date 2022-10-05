import os
import sys
from time import sleep
import rospy
import rospkg
import numpy as np
import math as m
import genpy
import copy
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped, PoseArray, Pose
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from erp42_control.msg import ControlMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque


class ConeMapping():
    def __init__(self):
        rospy.Subscriber("/odometry/kalman", Odometry, callback=self.OdomCallback)
        rospy.Subscriber("/obejct", PoseArray, callback=self.MappingCallback)
        self.Odom = Odometry()
        
        self.object = []
        
    def OdomCallback(self, msg):
        self.Odom = msg
        
        for i in msg.poses:
            x = self.Odom.pose.pose.orientation.x
            y = self.Odom.pose.pose.orientation.y
            z = self.Odom.pose.pose.orientation.z
            w = self.Odom.pose.pose.orientation.w

            _, _, self.yaw = euler_from_quaternion([x,y,z,w])
        
    def MappingCallback(self, msg):
        self.object = []
        for i in msg.poses:
            self.object.append[i[0], i[1]]
            
    def SelectNearestCone(self):
        arr = []
        for i in self.object:
            dis = m.sqrt((self.Odom.pose.pose.position.x - i[0]) ** 2 + (self.Odom.pose.pose.position.y - i[1]) ** 2)
            x_vec = i[0] - self.Odom.pose.pose.position.x
            y_vec = i[1] - self.Odom.pose.pose.position.y
            angle = m.atan2(y_vec, x_vec)
            theta = angle - self.yaw
        
if __name__ == "__main__":

    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        r.sleep()
