#!/usr/bin/env python

import os
import sys
from time import sleep
# from setuptools import sic
import rospy
import rospkg
import numpy as np
import math as m
import genpy
from tf.transformations import quaternion_from_euler
from enum import Enum
from random import randint, random
from matplotlib import pyplot as plt
from scipy.spatial import Delaunay
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from erp42_control.msg import ControlMessage
from cubic_spline_planner import calc_spline_course
# from dynamic_window_approach import dwa_control, motion, calc_target_index, Config
from tf.transformations import euler_from_quaternion


try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


class Color(Enum):
    NONE = 0
    YELLOW = 1
    BLUE = 2
    CENTER = 3


class Cone(np.ndarray):
    def __new__(cls, shape, *args, **kwargs):
        if "color" in kwargs.keys():
            cls.color = kwargs["color"]
        else:
            cls.color = Color.NONE

        obj = np.asarray(shape).view(cls)
        return obj


class Cones(object):
    
    def __init__(self):
        self.centers = []
        self.blue = [[0, 1, 2]]
        self.yellow = [[0.,-1., 2]]
        
        self.cone_sub1 = rospy.Subscriber(
            "/after_clustering1/markers", MarkerArray, callback=self.CarCallback1)  
        self.cone_sub2 = rospy.Subscriber(
            "/after_clustering2/markers", MarkerArray, callback=self.CarCallback2) 


        self.cones_pub = rospy.Publisher("cones", MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher("cone_path", Path, queue_size=10)
        
        self.ConeMsg1 = MarkerArray()
        self.ConeMsg2 = MarkerArray()

        
        self.xs = []
        self.ys = []

        self.dis_cone2cone = 0.5
        self.dis_cone2car = 10.
        self.count_cone = 10

    def CarCallback1(self, msg):
        self.ConeMsg1 = msg

    def CarCallback2(self, msg):
        self.ConeMsg2 = msg
        
    def ConePosition(self, state):
        
        Y_cone = []
        B_cone = []
        
        for i in self.ConeMsg1.markers:
            
            pose = (i.pose.position.x, i.pose.position.y)
            size_x = i.scale.x
            size_y = i.scale.y
            size_z = i.scale.z
            volume = size_x * size_y * size_z
            # Map frame cone position
            pose_x = state.x + pose[0] * m.cos(state.yaw) - pose[1] * m.sin(state.yaw)
            pose_y = state.y + pose[0] * m.sin(state.yaw) + pose[1] * m.cos(state.yaw)
            if  i.id >= 30000 and volume <= 0.01:
                Y_cone.append([pose_x, pose_y])
            elif 29999 >= i.id >= 20000 and 0.01:
                B_cone.append([pose_x, pose_y])
        
        for j in self.ConeMsg2.markers:
            
            pose = (j.pose.position.x, j.pose.position.y)
            size_x = j.scale.x
            size_y = j.scale.y
            size_z = j.scale.z
            volume = size_x * size_y * size_z
            # Map frame cone position
            pose_x = state.x + pose[0] * m.cos(state.yaw) - pose[1] * m.sin(state.yaw)
            pose_y = state.y + pose[0] * m.sin(state.yaw) + pose[1] * m.cos(state.yaw)
            if  j.id >= 30000 and volume <= 0.01:
                Y_cone.append([pose_x, pose_y])
            elif 29999 >= j.id >= 20000 and 0.01:
                B_cone.append([pose_x, pose_y])
    
 
        return Y_cone, B_cone
    
    
    def CalDistance(self, Y_cone, B_cone, state):
        
        if len(Y_cone) != 0:
            
            for i in Y_cone:
                
                dis_array = []
                local_dis_array = []
                
                for j in self.yellow[:]:
                    
                    dis = m.sqrt((j[0] - i[0]) ** 2 + (j[1] - i[1]) ** 2)
                    local_dis = m.sqrt((state.x - i[0]) ** 2 + (state.y - i[1]) ** 2)

                    dis_array.append(dis)
                    local_dis_array.append(local_dis)
                    
                min_index = dis_array.index(min(dis_array))
                    
                    
                if dis_array[min_index] >= self.dis_cone2cone and local_dis_array[min_index] <= self.dis_cone2car:
                    
                    self.yellow.append([i[0], i[1], 1])
                    
                elif dis_array[min_index] <= self.dis_cone2cone and local_dis_array[min_index] <= self.dis_cone2car:
                    
                    mean_x = (i[0] + self.yellow[min_index][0]) / 2
                    mean_y = (i[1] + self.yellow[min_index][1]) / 2
                    mean = [mean_x, mean_y]                    
                    self.yellow[min_index] = [mean[0], mean[1], self.yellow[min_index][2] + 1]
                    
                else :
                    pass
                
                    
        if len(B_cone) != 0:
            
            for i in B_cone:
                
                dis_array = []
                local_dis_array = []
                
                for j in self.blue[:]:
                    
                    dis = m.sqrt((j[0] - i[0]) ** 2 + (j[1] - i[1]) ** 2)
                    local_dis = m.sqrt((state.x - i[0]) ** 2 + (state.y - i[1]) ** 2)

                    dis_array.append(dis)
                    local_dis_array.append(local_dis)
                    
                min_index = dis_array.index(min(dis_array))
                    
                    
                if dis_array[min_index] >= self.dis_cone2cone and local_dis_array[min_index] <= self.dis_cone2car:
                    self.blue.append([i[0], i[1], 1])

                elif dis_array[min_index] <= self.dis_cone2cone and local_dis_array[min_index] <= self.dis_cone2car:
                    
                    mean_x = (i[0] + self.blue[min_index][0]) / 2
                    mean_y = (i[1] + self.blue[min_index][1]) / 2
                    mean = [mean_x, mean_y]                    
                    self.blue[min_index] = [mean[0], mean[1], self.blue[min_index][2] + 1]

                else :
                    pass
    

        
    def set_cones(self, state):
        
        self.cones = []
        self.y_cones_num = 0
        self.b_cones_num = 0

        for i in self.yellow:
            if i[2] >= self.count_cone:
                yellow = Cone([i[0], i[1]])
                yellow.color = Color.YELLOW
                self.cones.append(yellow)
                self.y_cones_num += 1
            
            
        for j in self.blue:
            if j[2] >= self.count_cone:
                blue = Cone([j[0], j[1]])
                blue.color = Color.BLUE
                self.cones.append(blue)  
                self.b_cones_num += 1  
            
        for i in range(2):
            pose_x = state.x + float(i + 3) * m.cos(state.yaw) - -1. * m.sin(state.yaw)
            pose_y = state.y + float(i + 3) * m.sin(state.yaw) + -1. * m.cos(state.yaw)    
            b_pose = Cone([pose_x, pose_y])
            b_pose.color = Color.YELLOW
            self.cones.append(b_pose)
            
        for i in range(2): 
            pose_x = state.x + float(i + 3) * m.cos(state.yaw) - 1. * m.sin(state.yaw)
            pose_y = state.y + float(i + 3) * m.sin(state.yaw) + 1. * m.cos(state.yaw)    
            y_pose = Cone([pose_x, pose_y])
            y_pose.color = Color.BLUE
            self.cones.append(y_pose)
        
        return self.cones, self.y_cones_num, self.b_cones_num
        
        
    def createVirtualCones(self):
        j = 0

        for i in range(0, 10):
            yellow = Cone(
                [i + randint(-10, 10) * 0.01, j-(2 + randint(0, 10) * 0.05)]
            )

            yellow.color = Color.YELLOW
            self.cones.append(yellow)

            blue = Cone(
                [i + randint(-10, 10) * 0.01, j+(2 + randint(0, 10) * 0.05)]
            )

            blue.color = Color.BLUE
            self.cones.append(blue)

            j = 16 * (m.sin(i * 0.1)) ** 3
            # j = m.sin(i * 0.1) * 10
        return self.cones

    def getDelaunayCenters(self, cones, ids):
        res = []

        p0 = cones[ids[0]]
        p1 = cones[ids[1]]
        p2 = cones[ids[2]]

        if p0.color != p1.color:
            x = (p0[0] + p1[0]) / 2.
            y = (p0[1] + p1[1]) / 2.

            res.append(Cone([x, y], color=Color.CENTER))

        if p1.color != p2.color:
            x = (p1[0] + p2[0]) / 2.
            y = (p1[1] + p2[1]) / 2.

            res.append(Cone([x, y], color=Color.CENTER))

        if p0.color != p2.color:
            x = (p0[0] + p2[0]) / 2.
            y = (p0[1] + p2[1]) / 2.

            res.append(Cone([x, y], color=Color.CENTER))

        return res

    def getAllCenters(self, cones, state):
        centers = []
        tri = Delaunay(cones)

        for ids in tri.simplices:
            center = self.getDelaunayCenters(cones, ids)
            centers += center

        centers = self.get_ordered_list(centers, state)

        return centers

    def getPathFromCenters(self, centers):
        centers = np.array(centers)

        xs = centers[:, 0]
        ys = centers[:, 1]

        cx, cy, cyaw, _, _ = calc_spline_course(
            xs[:], ys[:], ds=0.05)

        return cx, cy, cyaw

    def getDistance(self, p1, p2):
        dist = m.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        return dist

    def removeDuplicate(self, centers):
        n = np.unique(centers, axis=0)
        return n

    def get_ordered_list(self, centers, state):
        centers = self.removeDuplicate(centers)
        sorted_point = sorted(
            centers, key=lambda e: self.getDistance(e, np.array([state.x, state.y])))

        return sorted_point

    def publishPath(self, cx, cy, cyaw):
        path = Path()

        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        for i in range(len(cx)):
            pose = PoseStamped()

            pose.header.frame_id = "odom"
            pose.header.stamp = rospy.Time.now()

            quat = quaternion_from_euler(0., 0., cyaw[i])

            pose.pose.position = Point(cx[i], cy[i], 0.)
            pose.pose.orientation = Quaternion(
                quat[0], quat[1], quat[2], quat[3])

            path.poses.append(pose)

        self.path_pub.publish(path)

    def publishCones(self, cones):
        msg = MarkerArray()

        for i, cone in enumerate(cones):
            marker = Marker()

            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()

            marker.ns = str(i)
            marker.id = i

            marker.type = 3
            marker.action = 0

            marker.pose.position = Point(cone[0], cone[1], 0.)  # cone.point
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(
                1. if (cone.color == Color.YELLOW) else 0.,
                1. if (cone.color == Color.YELLOW) else 0.,
                1. if (cone.color == Color.BLUE) else 0.,
                1.
            )

            marker.lifetime = genpy.Duration(secs=0.2)

            msg.markers.append(marker)

        self.cones_pub.publish(msg)

    def detectCones(self, state, r=10.):
        res = []

        car = np.array([state.x, state.y])

        for cone in self.cones:
            dist = self.getDistance(car, cone)

            if dist < r:
                cone_vec = np.array([cone[0] - car[0], cone[1] - car[1]])
                car_vec = np.array([m.cos(state.yaw), m.sin(state.yaw)])

                if np.dot(car_vec, cone_vec) > 0:
                    res.append(cone)
        return res


if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    target_idx = 0
    last_idx = 0
    length = 0

    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)

    state = State()
    stanley = Stanley()

    cones = Cones()

    r = rospy.Rate(10.)
    while not rospy.is_shutdown():
        y, b = cones.ConePosition(state)
        cones.CalDistance(y, b, state)
        cones.set_cones(state)
        print("aaaa")
        if cones.y_cones_num >= 0 and cones.b_cones_num >= 0:
            try:
                detected_cone = cones.detectCones(state, 10.)
                centers = cones.getAllCenters(detected_cone, state)
                cx, cy, cyaw = cones.getPathFromCenters(centers)

                l = len(cx)
                if l != length:
                    length = l
                    target_idx = 1

                if target_idx == l:
                    continue

                di, target_idx = stanley.stanley_control(
                    state, cx, cy, cyaw, target_idx)

                # print(target_idx)

                msg = ControlMessage()
                msg.Speed = 3.
                msg.Steer = -m.degrees(di)

                cmd_pub.publish(msg)
                cones.publishPath(cx, cy, cyaw)
                # cones.publishCones(detected_cone)
                cones.publishCones(cones.cones)

                print("run!!")
                print(len(cones.cones))
            except ValueError:
                print("error!!")
        r.sleep()