#!/usr/bin/env python

from collections import deque
import os
from posixpath import join
import sys
from time import sleep
import copy
from chardet import detect
import rospy
import rospkg
import pandas as pd
import numpy as np
import math as m
import genpy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped, PoseArray
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from erp42_control.msg import ControlMessage
from tf.transformations import euler_from_quaternion
from cubic_spline_planner import calc_spline_course


try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from stanley import Stanley
    from pid_tuner import PID
except Exception as ex:
    rospy.logfatal(ex)


class Obstacle(object):
    
    def __init__(self, state):
        
        self.obs_mapping = []

        path_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/mission/data/ys/ys_static_path.csv")
        center_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/mission/data/ys/ys_static_path.csv")
        left_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/mission/data/ys/static_left.csv")

        self.pid = PID()
        
        # path line
        self.path_cx, self.path_cy, self.path_cyaw = path_data.cx.tolist(), path_data.cy.tolist(), path_data.cyaw.tolist()
        self.path = []
        for i in range(len(self.path_cx)):
            self.path.append([self.path_cx[i], self.path_cy[i]])

        # left line
        self.left_cx, self.left_cy = left_data.cx.tolist(), left_data.cy.tolist()
        self.left = []
        for j in range(len(self.left_cx)):
            self.left.append([self.left_cx[j], self.left_cy[j]])

        # center line      
        self.center_cx, self.center_cy, self.center_cyaw = center_data.cx.tolist(), center_data.cy.tolist(), center_data.cyaw.tolist()
        self.center = []
        for j in range(len(self.center_cx)):
            self.center.append([self.center_cx[j], self.center_cy[j]])
      

        rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.ObstacleCallback)  

        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)
        self.obs_pub_obs = rospy.Publisher("/obstacle/position", MarkerArray, queue_size=10) 
        self.obs_pub_way = rospy.Publisher("/obstacle/waypoints", MarkerArray, queue_size=10)        
       
        self.ObsMsg = PoseArray()
        self.PathMsg = PathResponse()
        self.msg = ControlMessage()

        self.state = state
        self.stanley = Stanley()
        self.obs_state = False

        self.target_idx = 0
        self.length = 0

        # parameter
        self.center_distance = 1.5
        self.detect_obs_angle = 0.8
        self.prox_dis = 1.        
        self.r = 1.6
        self.det_iter= 5

        
        #obstacle mapping
        self.list1 = []
        self.list2 = deque(maxlen=300)
        self.list3 = deque(maxlen=300)
        self.list4 = []
        
        self.count = 3
        self.threshold =5
        self.same_r = 1.5
        self.theta = 1.57
        
    def ObstacleCallback(self, msg):

        self.list1 = []
        for i in msg.poses:
            if m.sqrt((i.position.x) **2 + (i.position.y) ** 2 ) < 20. and (1.57 >= m.atan2(i.position.y, i.position.x)) and (m.atan2(i.position.y, i.position.x) > -1.57):
                map_x = self.state.x + (i.position.x + 0.5) * m.cos(self.state.yaw) - i.position.y * m.sin(self.state.yaw)
                map_y = self.state.y + (i.position.x + 0.5) * m.sin(self.state.yaw) + i.position.y * m.cos(self.state.yaw)
                dis = self.GetDistance2(self.center, [map_x, map_y])
                if dis <= self.center_distance:
                    self.list1.append([map_x, map_y])
                    self.list2.appendleft([map_x, map_y])

        
    def CountObstacle(self):  
        for i in self.list1:
            n = 0
            for j in copy.deepcopy(self.list2):
                dis = m.sqrt((i[0] - j[0]) ** 2 + (i[1] - j[1]) ** 2)
                if dis <= self.same_r:
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
                        
                    if min(dis_arr) <= self.same_r:
                        self.list3[min_idx][0] = (self.list3[min_idx][0] + i[0]) / 2
                        self.list3[min_idx][1] = (self.list3[min_idx][1] + i[1]) / 2
                        self.list3[min_idx][2] += 1
                        
                    else :
                        self.list3.appendleft([i[0], i[1], n])

    def Mapping(self):
                
        self.list4 = []
        for i in self.list3:
            state_vec = np.array(
                    [m.cos(self.state.yaw), m.sin(self.state.yaw)])
            point_vec = np.array([
                i[0] - self.state.x, i[1] - self.state.y])
            dot = np.dot(state_vec, point_vec)
            theta = abs(m.acos(
                (dot) / (np.hypot(state_vec[0], state_vec[1]) * np.hypot(point_vec[0], point_vec[1]))))
            dis = m.sqrt((self.state.x - i[0])**2 + (self.state.x - i[0])**2)
            if i[2] >= self.threshold and self.theta > theta and dis <= 20:
                self.list4.append([i[0], i[1]])
                     
    def CreateWaypoint(self):
        
        self.waypoint_arr = []
        if len(self.list4) != 0:
    
            #obs position -> [x,y] list
            for obs in self.list4:
            
                a, b = obs[0], obs[1]
                center_target_idx = self.calc_target_index(self.center_cx, self.center_cy, [a, b]) #nearest target IDX ... center path
                left_target_idx = self.calc_target_index(self.left_cx, self.left_cy, [a, b]) #nearest target IDX ... left path
                
                center_point = [self.center_cx[center_target_idx], self.center_cy[center_target_idx]]
                p = (b - center_point[1]) / (a- center_point[0])
                c = b - p * a
                
                t1 = (2*a + 2*p*b - 2*p*c + m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - self.r**2)))/(2 * (1 + p**2))
                t2 = (2*a + 2*p*b - 2*p*c - m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - self.r**2)))/(2 * (1 + p**2))

                waypoint1 = [t1, p * t1 + c]
                waypoint2 = [t2, p * t2 + c]
                
                obs2left = self.GetDistance([a, b], [self.left[left_target_idx][0], self.left[left_target_idx][1]])
                center2left = self.GetDistance([self.left[left_target_idx][0], self.left[left_target_idx][1]], [self.center[center_target_idx][0], self.center[center_target_idx][1]])

                way1_2left = self.GetDistance([waypoint1[0], waypoint1[1]], [self.left[left_target_idx][0], self.left[left_target_idx][1]])
                way2_2left = self.GetDistance([waypoint2[0], waypoint2[1]], [self.left[left_target_idx][0], self.left[left_target_idx][1]])
        
                if obs2left - center2left < 0 : # left obstacle --> right waypoint
                    if way1_2left > way2_2left: # left waypoint : 2
                        waypoint = waypoint1 # waypoint 1 : right position
                    else : #left waypoint : 1
                        waypoint = waypoint2 # waypoint 2 : right position
                        
                else : # right obstacle --> left waypoint
                    if way1_2left > way2_2left: # left waypoint : 2
                        waypoint = waypoint2 # waypoint 2 : left position
                    else :
                        waypoint = waypoint1 #waypoint 1 : left position
        
                self.waypoint_arr.append(waypoint)

            
            
    def CreatPath(self):

        state_tar_idx = self.calc_target_index(self.path_cx, self.path_cy, [self.state.x, self.state.y])

        if len(self.waypoint_arr) != 0:
            
            
            
            max_tar_idx = -1
            self.waypoint_arr.sort(key = lambda x : self.calc_target_index(self.path_cx, self.path_cy, x))
        
            if len(self.waypoint_arr) >= 2:
                    
                min_tar_idx = self.calc_target_index(self.path_cx, self.path_cy, self.waypoint_arr[0])
                max_tar_idx = self.calc_target_index(self.path_cx, self.path_cy, self.waypoint_arr[-1])
                xs = [self.path[min_tar_idx - 40][0]]
                ys = [self.path[min_tar_idx - 40][1]]
        
                for i in self.waypoint_arr:
                    xs.append(i[0])
                    ys.append(i[1])
                    
                try:
                    xs.append(self.path[max_tar_idx + 40][0])
                    ys.append(self.path[max_tar_idx + 40][1])
                    
                except IndexError:
                    xs.append(self.path[len(self.path_cx) - 1][0])
                    ys.append(self.path[len(self.path_cy) - 1][1])   

                self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1) 


            else : # waypoint array len : 1
                max_tar_idx = self.calc_target_index(self.path_cx, self.path_cy, self.waypoint_arr[0])
                xs = [self.path[max_tar_idx - 40][0], self.waypoint_arr[0][0], self.path[max_tar_idx + 40][0]]
                ys = [self.path[max_tar_idx - 40][1], self.waypoint_arr[0][1], self.path[max_tar_idx + 40][1]]

                self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
       
        else :
            self.cx, self.cy, self.cyaw = self.path_cx, self.path_cy, self.path_cyaw
        
            
    def GetDistance(self, point1, point2):
        distance = m.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        return distance
    
    
    def GetDistance2(self, path, point):   
        dis_r_array = []
        for i in range(len(path)):
            dis = m.sqrt((point[0] - path[i][0]) ** 2 + (point[1] - path[i][1]) ** 2)
            dis_r_array.append(dis)
        min_dis = min(dis_r_array)
        return min_dis


    def calc_target_index(self, cx, cy, point):
        fx = point[0]
        fy = point[1]
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        return target_idx
                
    def publishPath(self, cx, cy, cyaw):
        
        path = Path()

        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()

        for i in range(len(cx)):
            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            quat = quaternion_from_euler(0., 0., cyaw[i])

            pose.pose.position = Point(cx[i], cy[i], 0.)
            pose.pose.orientation = Quaternion(
                quat[0], quat[1], quat[2], quat[3])

            path.poses.append(pose)

        self.path_pub.publish(path)
            
            
    def publishObstacle(self, position, color=ColorRGBA, scale=Vector3):
        msg = MarkerArray()

        for i in range(len(position)):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            marker.ns = str(i)
            marker.id = 1

            marker.type = 3
            marker.action = 0

            marker.pose.position = Point(position[i][0], position[i][1], 0.)  # cone.point
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = scale
            marker.color = color

            marker.lifetime = genpy.Duration(secs=0.2)

            msg.markers.append(marker)

        self.obs_pub_obs.publish(msg)
        
    def publishWaypoint(self, position, color=ColorRGBA, scale=Vector3):
        msg = MarkerArray()

        for i in range(len(position)):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            marker.ns = str(i)
            marker.id = 1

            marker.type = 3
            marker.action = 0

            marker.pose.position = Point(position[i][0], position[i][1], 0.)  # cone.point
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = scale
            marker.color = color
            marker.lifetime = genpy.Duration(secs=0.2)
            msg.markers.append(marker)

        self.obs_pub_way.publish(msg)            
            
    def GetDistance(self, point1, point2):
        distance = m.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        return distance
    
    
    def GetDistance2(self, path, point):   
        dis_r_array = []
        for i in range(len(path)):
            dis = m.sqrt((point[0] - path[i][0]) ** 2 + (point[1] - path[i][1]) ** 2)
            dis_r_array.append(dis)
        min_dis = min(dis_r_array)
        return min_dis


    def calc_target_index(self, cx, cy, point):
        fx = point[0]
        fy = point[1]
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        return target_idx


    def main(self):

        self.CountObstacle()
        self.Mapping()
        self.CreateWaypoint()
        self.CreatPath()


        self.publishPath(self.cx, self.cy, self.cyaw)
        self.publishObstacle(self.list4, ColorRGBA(1., 0., 0., 1.), Vector3(0.5, 0.5, 0.5))
        self.publishWaypoint(self.waypoint_arr, ColorRGBA(1., 1., 0., 1.), Vector3(0.2, 0.2, 0.2))
        
        l = len(self.cx)
        
        if l != self.length:
            self.length = l
            self.target_idx = 1

        if self.target_idx == l:
            pass
        
        
        di, self.target_idx = self.stanley.stanley_control(
            self.state, self.cx, self.cy, self.cyaw, self.target_idx)
        self.msg.Steer = -m.degrees(di)
        self.msg.Gear = 2
        
        if len(self.list4) != 0:
            self.obs_state = True
            desire_speed = int(5)
            self.msg.Speed = int(self.pid.PIDControl(desire_speed))
        else:
            self.obs_state = False
            desire_speed = int(7)
            self.msg.Speed = int(self.pid.PIDControl(desire_speed))        

