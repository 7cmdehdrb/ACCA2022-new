#!/usr/bin/env python

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


class ConeTracking(object):
    
    def __init__(self):
        

        rospy.Subscriber("/cone_position", PoseArray, callback=self.ConeCallback)

        self.polypub = rospy.Publisher("poly", PoseArray, queue_size=1)
        self.waypointpub = rospy.Publisher("target", Marker, queue_size=10)
        self.Cmdpub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)

        self.ConeMsg = PoseArray()
        self.cone_class_num = 10
        
        #parameter
        self.offset = 1.5
        self.L = 1.04
        self.speed = 5
        
        
    def ConeCallback(self, msg):
        self.ConeMsg = msg


    def SetClass(self):
        
        self.cone_class = [[],[],[],[],[],[],[],[],[],[]]

        for i in self.ConeMsg.poses:
            
            x = i.position.x
            y = i.position.y
            angle = m.atan2(y,x)
            dis = abs(m.sqrt((x**2 + y**2)))
            
            if dis <= 10 and x > 0.0 and y < 1:
                self.cone_class[int(m.floor(dis))].append([x,y,angle])
            
            
    def SelectRight(self):
        
        self.cone_right = []

        for i in range(self.cone_class_num):
            if len(self.cone_class[i]) != 0:
                self.cone_class[i].sort(key= lambda x: x[2])
                self.cone_right.append([self.cone_class[i][0][0], self.cone_class[i][0][1]])
        print(self.cone_right)
        print("###")
    def Polyfit(self):
        
        self.cone_select = []
        self.points = []
        self.SetClass()
        self.SelectRight()
        # minimum polifit cone number : 3
        if len(self.cone_right) >= 3:
            x1 = []
            y1 = []
            for i in range(len(self.cone_right)):
                x1.append(self.cone_right[i][0])
                y1.append(self.cone_right[i][1])
                
            x2 = []
            y2 = []
            for j in self.cone_class:
                for k in range(len(j)):
                    x2.append(j[k][0])
                    y2.append(j[k][1])
        
            f = np.polyfit(x1, y1, 2)
            self.PublishPolyfit(f)
            a, b, c = f[0], f[1], f[2]
            
            coor = []
            for p, t in zip(x2, y2):
                
                eq_y = a*p**2 + b*p + c     
                dis = t - eq_y


                if dis <= 1.5:
                    coor.append([p, t])
                    
            if len(coor) >= 3:
                coor.sort()
                self.points = [coor[0], coor[1]]
            
            elif len(coor) == 2:
                self.points = coor
            elif len(coor) == 1:
                self.points = [coor[0]]
            else :
                self.points = []
                
        elif len(self.cone_right) == 2:
            
            self.points = [[self.cone_right[0][0], self.cone_right[0][1]], [self.cone_right[1][0], self.cone_right[1][1]]] 
        elif len(self.cone_right) == 1:
            
            self.points = [[self.cone_right[0][0], self.cone_right[0][1]]]
        else:
            self.points = []

    def GetTarget(self):
        if len(self.points) == 2:
            x_vec = self.points[0][0] - self.points[1][0]
            y_vec = self.points[0][1] - self.points[1][1]

            a = [x_vec, y_vec, 0.0]
            b = [0.0, 0.0, -1.0]
            
            vector = np.cross(a, b)

            norm = np.linalg.norm(vector)
            unit = vector/norm

            data1 = [self.points[0][0] - self.offset * unit[0],
                        self.points[0][1] - self.offset * unit[1]]
            data2 = [self.points[1][0] - self.offset * unit[0],
                        self.points[1][1] - self.offset * unit[1]]

            data1_len = m.sqrt(data1[0]**2 + data1[1] **2)

            if data1_len <= 1.5:
                self.target = data2
                
            else :
                self.target = data1
                
        elif len(self.points) == 1:
            
            self.target = [self.points[0][0], self.points[0][1] + self.offset]
            
        else:
            self.target = [1.5, 0.0001]

    def GetSteer(self):
        if self.target[1] >= 0.:
            alpha = -m.atan2(self.target[0], self.target[1])
        else :
            alpha = m.atan2(self.target[0], self.target[1])

        Ld_point = [-1.04, 0]
        Ld = m.sqrt((Ld_point[0] - self.target[0]) ** 2 + (Ld_point[1] - self.target[1]) ** 2)
        self.delta_ref = m.atan(2 * self.L * m.sin(alpha) / Ld)   
        self.di = np.clip(self.delta_ref / np.pi * 180, -30, 30) * 0.4
        
    def PublishCmdMsg(self):
        msg = ControlMessage()
        msg.Speed = self.speed
        msg.Steer = self.di
        msg.Gear = 2
        msg.brake = 0

        self.Cmdpub.publish(msg)

    def PublishPolyfit(self, f):
        
        obstacle = PoseArray()
        x = np.linspace(0, 10)
        y = x**2*f[0] + x*f[1] + f[2]
        obstacle.header.frame_id = "laser"
        obstacle.header.stamp = rospy.Time.now()
        
        for i in range(len(x)):

            pose = Pose()
            pose.position.x = x[i]
            pose.position.y = y[i]
            pose.position.z = 0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            obstacle.poses.append(pose)

        self.polypub.publish(obstacle)
        
    def PublishPolyfit2(self, f):
        
        obstacle = PoseArray()
        x = np.linspace(0, 10)
        y = x*f[0] + f[1]
        obstacle.header.frame_id = "laser"
        obstacle.header.stamp = rospy.Time.now()

        for i in range(len(x)):

            pose = Pose()
            pose.position.x = x[i]
            pose.position.y = y[i]
            pose.position.z = 0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            obstacle.poses.append(pose)

        self.polypub.publish(obstacle)
        
        
    def publishPoint(self, point):

        marker = Marker()

        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time.now()

        marker.ns = '1'
        marker.id = 1

        marker.type = 3
        marker.action = 0

        marker.pose.position = Point(point[0], point[1], 0.)
        marker.pose.orientation = Quaternion(0., 0., 0., 1)
        marker.scale = Vector3(0.2, 0.2, 0.5)
        marker.color = ColorRGBA(1.,0.,0.,1.)

        marker.lifetime = genpy.Duration(secs=0.2)

        self.waypointpub.publish(marker)
        
if __name__ == "__main__":
    
    rospy.init_node("conetrack")

    cone = ConeTracking()

    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)     
    CmdMsg = ControlMessage()    
    r = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        cone.SetClass()
        cone.SelectRight()
        cone.Polyfit()
        cone.GetTarget()
        cone.GetSteer()
        cone.PublishCmdMsg()
        cone.publishPoint(cone.target)
        r.sleep()