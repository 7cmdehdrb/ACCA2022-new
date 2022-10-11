#!/usr/bin/env python
# -*- coding:utf-8 -*-

from ast import Pass
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time
import tf
import sys
import numpy as np
import math as m
import genpy
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from path_plan.msg import PathResponse
from collections import deque


class Delivery():
    def __init__(self):
        self.ax = deque(maxlen=100)
        self.ay = deque(maxlen=100)
        self.bx = deque(maxlen=100)
        self.by = deque(maxlen=100)

        # check id_number
        self.panel = [[4, 7], [5, 8], [6, 9]]  # [[A1,B1],[A2,B2],[A3,B3]]
        self.panel_id = [0, 0, 0]
        self.tf_sub = tf.TransformListener()
        self.markers = MarkerArray()
        self.delivery = False
        self.is_delivery_path = False
        
        self.panel_A = None
        self.panel_B = Point()
        
        rospy.Subscriber("/nearmarkers", MarkerArray, self.Callback)

    def Callback(self, msg):
        self.markers = msg

    def delivery_A(self):            
        for marker in self.markers.markers:
            for i in range(3):
                if (marker.id)//10000 in self.panel[i]:
                    m_to_p = PoseStamped()

                    m_to_p.header.frame_id = "velodyne"
                    m_to_p.header.stamp = rospy.Time(0)
                    m_to_p.pose.position.x = marker.pose.position.x
                    m_to_p.pose.position.y = marker.pose.position.y
                    m_to_p.pose.position.z = marker.pose.position.z
                    m_to_p.pose.orientation.x = marker.pose.orientation.x
                    m_to_p.pose.orientation.y = marker.pose.orientation.y
                    m_to_p.pose.orientation.z = marker.pose.orientation.z
                    m_to_p.pose.orientation.w = marker.pose.orientation.w

                    m_to_p = self.tf_sub.transformPose("map", m_to_p)

                    self.ax.append(m_to_p.pose.position.x)
                    self.ay.append(m_to_p.pose.position.y)

                    self.panel_id[i] += 1
        
        self.panel_A = Point()
        self.panel_A.x = np.mean(self.ax)
        self.panel_A.y = np.mean(self.ay)
        
    
    def delivery_B(self, panel_id):
        for marker in self.markers.markers:
            if (marker.id)//10000 == self.panel[panel_id.index(max(panel_id))][1]:
                                
                m_to_p = PoseStamped()

                m_to_p.header.frame_id = "velodyne"
                m_to_p.header.stamp = rospy.Time(0)
                m_to_p.pose.position.x = marker.pose.position.x
                m_to_p.pose.position.y = marker.pose.position.y
                m_to_p.pose.position.z = marker.pose.position.z
                m_to_p.pose.orientation.x = marker.pose.orientation.x
                m_to_p.pose.orientation.y = marker.pose.orientation.y
                m_to_p.pose.orientation.z = marker.pose.orientation.z
                m_to_p.pose.orientation.w = marker.pose.orientation.w

                m_to_p = self.tf_sub.transformPose("map", m_to_p)

                self.bx.append(m_to_p.pose.position.x)
                self.by.append(m_to_p.pose.position.y)

                # if d <= self.dis :
                #     self.pointpublish()

        self.panel_B.x = np.mean(self.bx)
        self.panel_B.y = np.mean(self.by)