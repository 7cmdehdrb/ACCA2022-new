#!/usr/bin/env python



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
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from erp42_control.msg import ControlMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque


try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State, OdomState
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)




class ConeMapping():
    def __init__(self, state):
        rospy.Subscriber("/cluster_filter", PoseArray, callback=self.ClusterCallback)
        self.marker_pub = rospy.Publisher("/marker/obejct", MarkerArray, queue_size=1)
        self.object_pub = rospy.Publisher("/cone_simulator/detected_cones", PoseArray, queue_size=1)
        
        self.list1 = []
        self.list2 = deque(maxlen=200)
        self.list3 = deque(maxlen=200)
        self.list4 = []
        
        self.state = state
        # parameter
        self.r = 1.5
        self.count = 10
        self.threshold = 20

        
    def ClusterCallback(self, msg):
        
        self.list1 = []
        for i in msg.poses:
            
            map_x = self.state.x + (i.position.x + 0.5) * m.cos(self.state.yaw) - i.position.y * m.sin(self.state.yaw)
            map_y = self.state.y + (i.position.x + 0.5) * m.sin(self.state.yaw) + i.position.y * m.cos(self.state.yaw)
            
            self.list1.append([map_x, map_y])
            self.list2.appendleft([map_x, map_y])
            
        
    def CalDistance(self):
        for i in self.list1:
            n = 0
            
            for j in copy.deepcopy(self.list2):
                dis = m.sqrt((i[0] - j[0]) ** 2 + (i[1] - j[1]) ** 2)
                if dis <= self.r:
                    n += 1
                    
            if n >= self.count:
                
                if len(self.list3) == 0:
                    self.list3.appendleft([i[0], i[1], n])
                    
                else:
                    dis_arr = []
                    for k in self.list3:
                        dis = m.sqrt((i[0] - k[0]) ** 2 + (i[1] - k[1]) ** 2)
                        dis_arr.append(dis)
                        min_idx = dis_arr.index(min(dis_arr))
                        
                    if min(dis_arr) <= self.r:
                        self.list3[min_idx][0] = (self.list3[min_idx][0] + i[0]) / 2
                        self.list3[min_idx][1] = (self.list3[min_idx][1] + i[1]) / 2
                        self.list3[min_idx][2] += 1
                        
                    else :
                        self.list3.appendleft([i[0], i[1], n])
                    
    def ConeMapping(self):
        
        self.list4 = []
        for i in self.list3:
            state_vec = np.array(
                    [m.cos(self.state.yaw), m.sin(self.state.yaw)])
            point_vec = np.array([
                i[0] - self.state.x, i[1] - self.state.y])
            dot = np.dot(state_vec, point_vec)
            theta = abs(m.acos(
                (dot) / (np.hypot(state_vec[0], state_vec[1]) * np.hypot(point_vec[0], point_vec[1]))))


            if i[2] >= self.threshold and 3.15 >= theta:
                self.list4.append([i[0], i[1]])
        print(len(self.list4))
                    
    def PubObject(self):
        msg_arr = PoseArray()
        for i in self.list4:
            msg = Pose()
            msg.position.x = i[0]
            msg.position.y = i[1]
            msg_arr.poses.append(msg)
            
        self.object_pub.publish(msg_arr)
                
    def MarkObject(self):
    
        marker_arr = MarkerArray()

        for i in range(len(self.list4)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()

            marker.ns = '1'
            marker.id = i

            marker.type = 3
            marker.action = 0

            marker.pose.position = Point(self.list4[i][0], self.list4[i][1], 0.)
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(1.,0.,0.,1.)

            marker.lifetime = genpy.Duration(secs=0.1)
            marker_arr.markers.append(marker)
            
        self.marker_pub.publish(marker_arr)
if __name__ == "__main__":
    rospy.init_node("mapping")
    state = State(odometry_topic="/odometry/kalman", hz=30, test=True)

    mapping = ConeMapping(state)

    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        mapping.CalDistance()
        mapping.ConeMapping()
        mapping.MarkObject()
        mapping.PubObject()

        r.sleep()
