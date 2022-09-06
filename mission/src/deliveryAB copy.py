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


class Delivery(object):
    def __init__(self):
        self.x = []
        self.y = []

        # check id_number 
        self.panel = [[4,7],[5,8],[6,9]]  #[[A1,B1],[A2,B2],[A3,B3]]
        self.panel_id = [0,0,0]
        self.dis = 3 # distance
        self.deli_A = False
        self.deli_B = False
        self.pause = False
        self.count = 0
        self.tf_sub = tf.TransformListener()


    def Callback(self, msg):
        if self.deli_A == False and self.deli_B == False: # state machine
            for marker in msg.markers:

                d = m.sqrt(marker.pose.position.x**2 + marker.pose.position.y**2)

                for i in range(3):
                    if (marker.id)//10000 in self.panel[i] :  
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
                        
                        self.panel_id[i] += 1
                        
                        # if (d<=self.dis):
                        #     self.pointpublish()
                        #     # self.pathdb = pathdbB


        elif self.deli_A == True and self.deli_B == False:

            for marker in msg.markers:
                
                d = m.sqrt(marker.pose.position.x**2 + marker.pose.position.y**2)
                
                if (marker.id)//10000 == self.panel[self.panel_id.index(max(self.panel_id))][1] :
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

                    if d <= self.dis :
                        self.pointpublish()


                else:
                    pass
    
        else: 
            pass

    def pointpublish(self):

        panel_x = sum(self.x)/len(self.x)
        panel_y = sum(self.y)/len(self.y)
        
        d2 = []    
        for i in range(len(self.pathdb[0])):
            d2.append(((self.pathdb[0][i]-panel_x)**2+(self.pathdb[1][i]-panel_y)**2))
        p_index = d2.index(min(d2))

        #publish
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

    # def testcallback(self):
    #     db = DB("/school_test.db")
    #     if self.pathid == "" :
    #         self.pathdb = db.bring_pathinfo("E1A1")
    #         self.pathid = "E1A1"
    #     elif self.pathid == "E1A1" and self.deli_A == True:
    #         self.pathdb = db.bring_pathinfo("E1A1")
    #         self.pathid = "E1A1"
    #     else: pass

if __name__ == '__main__':
   
    rospy.init_node('delivery_AB',anonymous=True)
    
    
    # db = DB("/school_test.db")
    # pathdbA = db.bring_pathinfo("E1A1")
    # pathdbB = db.bring_pathinfo("E1A1")
    
    
    deli = Delivery()
    deli.tf_sub = tf.TransformListener()
    
    rospy.Subscriber("/nearmarkers",MarkerArray,deli.Callback)
    pose_pub = rospy.Publisher("delivery_AB",PoseStamped,queue_size = 5)
    
    
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        '''for test'''
        deli.testcallback()
        ''' for test end'''
        r.sleep()
        