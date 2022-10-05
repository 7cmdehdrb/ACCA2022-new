
import os
import sys
from time import sleep
import rospy
import rospkg
import numpy as np
import math as m
import genpy
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped, PoseArray, Pose
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from erp42_control.msg import ControlMessage

class ClusterFilter():
    def __init__(self):
        rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.PosesCallback)
        self.pub = rospy.Publisher("cluster_filter", PoseArray, queue_size=1)
        self.r = 12.
        self.theta = 0.523
        self.poses = []
        
    def PosesCallback(self, msg):
        self.poses = []
        for i in msg.poses:
            if m.sqrt((i.position.x) **2 + (i.position.y) ** 2 ) < self.r and (2.0 >= m.atan2(i.position.y, i.position.x) and m.atan2(i.position.y, i.position.x) > -2.0):
                self.poses.append([i.position.x, i.position.y])

    def PubCluster(self):
        msgarray = PoseArray()
        for i in self.poses:
            msg = Pose()
            msg.position.x = i[0]
            msg.position.y = i[1]
            msgarray.poses.append(msg)
            
        self.pub.publish(msgarray)
        
if __name__ == "__main__":
    rospy.init_node("ClusterFilter")
    cf = ClusterFilter()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        cf.PubCluster()
        r.sleep()
    