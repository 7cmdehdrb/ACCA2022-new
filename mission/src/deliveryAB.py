#!/usr/bin/env python
# -*- coding:utf-8 -*-

from ast import Pass
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import time
import tf
import sys
import numpy as np
import math as m
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from path_plan.msg import PathResponse
from collections import deque


class Delivery(object):
    def __init__(self):
        self.ax = deque(maxlen=200)
        self.ay = deque(maxlen=200)
        self.bx = deque(maxlen=200)
        self.by = deque(maxlen=200)

        # check id_number
        self.panel = [[4, 7], [5, 8], [6, 9]]  # [[A1,Bs1],[A2,B2],[A3,B3]]
        self.panel_id = [0, 0, 0]
        self.deli_A = False
        self.deli_B = False
        self.tf_sub = tf.TransformListener()
        self.markers = MarkerArray()
        self.target_idx = float("inf")
        self.delivery = False
        self.is_delivery_path = False
        rospy.Subscriber("/nearmarkers", MarkerArray, self.Callback)

    def Callback(self, msg):
        if self.delivery == True:
            self.markers = msg
            self.delivery_AB()

    def delivery_AB(self):
        if self.deli_A == False and self.deli_B == False:
            for marker in self.markers:

                # d = m.sqrt(marker.pose.position.x**2 + marker.pose.position.y**2)

                for i in range(3):
                    if (marker.id)//10000 in self.panel[i]:
                        m_to_p = PoseStamped()

                        m_to_p.header.frame_id = "velodyne"
                        m_to_p.header.stamp = rospy.Time(0)
                        m_to_p.pose.position.x = marker.pose.position.x
                        m_to_p.pose.position.y = marker.pose.position.y
                        m_to_p.pose.position.z = 0
                        m_to_p.pose.orientation.x = 0
                        m_to_p.pose.orientation.y = 0
                        m_to_p.pose.orientation.z = 0
                        m_to_p.pose.orientation.w = 1

                        m_to_p = self.tf_sub.transformPose("map", m_to_p)

                        self.ax.append(m_to_p.pose.position.x)
                        self.ay.append(m_to_p.pose.position.y)

                        self.panel_id[i] += 1

            self.panel_x = sum(self.ax)/len(self.ax)
            self.panel_y = sum(self.ay)/len(self.ay)

        elif self.deli_A == True and self.deli_B == False:

            for marker in self.markers:

                # d = m.sqrt(marker.pose.position.x**2 + marker.pose.position.y**2)

                if (marker.id)//10000 == self.panel[self.panel_id.index(max(self.panel_id))][1]:
                    m_to_p = PoseStamped()

                    m_to_p.header.frame_id = "velodyne"
                    m_to_p.header.stamp = rospy.Time(0)
                    m_to_p.pose.position.x = marker.pose.position.x
                    m_to_p.pose.position.y = marker.pose.position.y
                    m_to_p.pose.position.z = 0
                    m_to_p.pose.orientation.x = 0
                    m_to_p.pose.orientation.y = 0
                    m_to_p.pose.orientation.z = 0
                    m_to_p.pose.orientation.w = 1

                    m_to_p = self.tf_sub.transformPose("map", m_to_p)

                    self.x.append(m_to_p.pose.position.x)
                    self.y.append(m_to_p.pose.position.y)

                    # if d <= self.dis :
                    #     self.pointpublish()

            self.panel_x = sum(self.bx)/len(self.bx)
            self.panel_y = sum(self.by)/len(self.by)

    def calc_path_point(self, x, y, path):
        path.cx = np.array(path.cx)
        path.cy = np.arrya(path.cy)

        dx = path.cx - x
        dy = path.cy - y

        d = np.hypot(dx, dy)

        target_idx = np.argmin(d)

        return path.cx[target_idx], path.cy[target_idx]

    def pointpublish(self):

        panel_x = sum(self.x)/len(self.x)
        panel_y = sum(self.y)/len(self.y)

        d2 = []
        for i in range(len(self.pathdb[0])):
            d2.append(((self.pathdb[0][i]-panel_x) **
                      2+(self.pathdb[1][i]-panel_y)**2))
        p_index = d2.index(min(d2))

        # publish
        posestamp = PoseStamped()
        posestamp.header.frame_id = "map"
        posestamp.header.stamp = rospy.Time(0)
        posestamp.pose.position.x = self.pathdb[0][p_index]
        posestamp.pose.position.y = self.pathdb[1][p_index]
        posestamp.pose.position.z = 0

        quat = quaternion_from_euler(0., 0., self.pathdb[2][p_index])

        posestamp.pose.orientation.x = quat[0]
        posestamp.pose.orientation.y = quat[1]
        posestamp.pose.orientation.z = quat[2]
        posestamp.pose.orientation.w = quat[3]

        pose_pub.publish(posestamp)

        print("publish end.")
        if self.deli_A == True:
            self.deli_B = True
        self.pause = True
        self.deli_A = True
        self.x = []
        self.y = []


if __name__ == '__main__':

    rospy.init_node('delivery_AB', anonymous=True)

    deli = Delivery()
    deli.tf_sub = tf.TransformListener()

    pose_pub = rospy.Publisher("delivery_AB", PoseStamped, queue_size=5)

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        '''for test'''
        deli.testcallback()
        ''' for test end'''
        r.sleep()
