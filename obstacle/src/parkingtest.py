#!/usr/bin/env python

import os
from posixpath import join
import sys
from time import sleep
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
    from state import State
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)


class Parking(object):
    
    def __init__(self):
        
        path_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/obstacle/data/center1.csv")
        
        self.path = []
        self.path_cx = []
        self.path_cy = []
        self.path_cyaw = []

        for i, j, k in zip(path_data.cx, path_data.cy, path_data.cyaw):
            self.path.append([i, j])
            self.path_cx.append(i)
            self.path_cy.append(j)
            self.path_cyaw.append(k)

        # [x, y, yaw]
        self.area0, self.area1, self.area2, self.area3, self.area4, self.area5 = [], [], [], [], [], []
        self.area = [[5, 3], [6,3], [7,3], [8,3], [9,3], [10,3]]
        self.area_num = [0, 0, 0, 0, 0, 0]
        self.in_detect_range = [False, False, False, False, False, False]

        self.parking_check = [False, False, False, False, False, False]
        self.count = 30
        self.create_path_FS = 2 # meter
        self.parking_state = 'detect area'
        
        
        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.ObstacleCallback)
                
        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)
        self.obs_pub_obs = rospy.Publisher("obstacle_position", MarkerArray, queue_size=10) 
        self.obs_pub_way = rospy.Publisher("waypoint_position", MarkerArray, queue_size=10)        
       
        
        self.ObsMsg = PoseArray()
        self.PathMsg = Path()
        
        self.det_start = 50
        self.obs_range = 0.5    
        self.start_signal = False
        
            
        # Detect Obstacle
        self.volume = 0.001
        self.angle = 0.8
        self.dis_path = 3.
        self.detection_range_min = 0.1
        self.detection_range_max = 5.
        self.r = 1.6
        self.prox_dis = 1.        
        self.det_iter= 10
        
    def ObstacleCallback(self, msg):
        self.ObsMsg = msg


    
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
    
    def calc_target_index_state(self, cx, cy, state):

        # Calc front axle position
        fx = state.x + 1.040 * np.cos(state.yaw) / 2.0
        fy = state.y + 1.040 * np.sin(state.yaw) / 2.0

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]

        d = np.hypot(dx, dy)
        
        target_idx = np.argmin(d)
            
        return target_idx
    
    
    def calc_target_index(self, cx, cy, point):

        fx = point[0]
        fy = point[1]
        # print(fx)
        # print(fy)
        # Search nearest point index
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
            
            
    def publishPoint(self, position, color=ColorRGBA, scale=Vector3):
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
  
    def GetPoint(self, area):
        
        per_idx = self.calc_target_index(self.path_cx, self.path_cy, [area[0], area[1]])
        
        point1 = [area[0], area[1]]
        point2 = self.path[per_idx]
        point3 = self.path[per_idx - self.det_start]
        
        meetpoint_x = (((point3[1] - point2[1]) / (point3[0] - point2[0]) * point2[0]) -point2[1] - (area[2] * point1[0]) + point1[1]) / (((point3[1] - point2[1]) / (point3[0] - point2[0])) - area[2])
        meetpoint_y = area[2] * (meetpoint_x - point1[0]) + point1[1]
        
        return point1, [meetpoint_x, meetpoint_y], point3
    
    
    def InDetectRange(self, state, meetpoint, point3):
        
        
        for i in range(6):
            
            _, meetpoint, point3 = self.GetPoint(self.area[i])
            
            curren_idx = self.calc_target_index(self.path_cx, self.path_cy, [state.x, state.y])
            meet_idx = self.calc_target_index(self.path_cx, self.path_cy, [meetpoint[0], meetpoint[1]])
            point3_idx = self.calc_target_index(self.path_cx, self.path_cy, [point3[0], point3[1]])

            if meet_idx < curren_idx < point3_idx:
                
                self.in_detect_range[i] = True
                
            else:
                self.in_detect_range[i] = False
    
    
    def CarMapping(self, state):
                
        for i in self.ObsMsg.poses:
            
            for j in range(6):
                
                if self.in_detect_range[j] == True:
                    
                    velodyne_x = i.position.x
                    velodtne_y = i.position.y
                    angle = abs(m.atan2(velodtne_y, velodyne_x)) #radian
                    map_x = state.x + velodyne_x * m.cos(state.yaw) - velodtne_y * m.sin(state.yaw)
                    map_y = state.y + velodyne_x * m.sin(state.yaw) + velodtne_y * m.cos(state.yaw)
                
                    dis = self.GetDistance([self.area[j][0], self.area[j][1]], [map_x, map_y])
                    
                    if dis <= self.obs_range:
                        self.area_num[j] += 1
    
    
    def StartAlgorithmSignal(self):
        
        for i in range(len(self.area_num)):
            
            if self.area_num[i] >= self.count:
                self.parking_state = 'in'
                self.target_area = i
    
    
    def CreatPath(self):
        
        if self.parking_state == 'in':
            point1, meetpoint, point3 = self.GetPoint(self.area[self.target_area])
            
            a, b = point1[0], point1[1]
            p = self.area[self.target_area][2]
            c = b - p * a
            
            xs = [state.x]
            ys = [state.y]
        
            for i in range(10):
                
                r = self.create_path_FS * 0.1 * i
                t1 = (2*a + 2*p*b - 2*p*c + m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - r**2)))/(2 * (1 + p**2))
                t2 = (2*a + 2*p*b - 2*p*c - m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - r**2)))/(2 * (1 + p**2))

                waypoint1 = [t1, p * t1 + c]
                waypoint2 = [t2, p * t2 + c]
                
                dis1 = self.GetDistance2(self.path, waypoint1)
                dis2 = self.GetDistance2(self.path, waypoint2)
                
                if dis1 > dis2:
                    xs.insert(-1, waypoint2)
                    ys.insert(-1, waypoint2)
                    
                else :
                    xs.insert(-1, waypoint1)
                    ys.insert(-1, waypoint1)
                    
                
            self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
                

    def parking(self, state, stanley):
        target_idx = 0
        length = 0   
        
        msg = ControlMessage()
          
        while self.parking_state == 'in':
               
            l = len(self.cx)

            if l != length:
                length = l
                target_idx = 1

            if target_idx == l:
                continue
            
            di, target_idx = stanley.stanley_control(
            state, self.cx, self.cy, self.cyaw, target_idx)

            msg.Speed = 3.
            msg.Steer = -m.degrees(di)
            msg.Gear = 2
            
            state_idx = self.calc_target_index_state(self.cx, self.cy, state)
            cmd_pub.publish(msg)

            if state_idx - len(self.cx) < 3:
                self.parking_state = 'out'
                break
            
        while self.parking_state == 'out':
        
            msg.Speed = 3.
            msg.Steer = -m.degrees(di)
            msg.Gear = 2
            cmd_pub.publish(msg)

            dis = self.GetDistance2(self.path, [state.x, state.y])
        
            if dis < 0.2:
                self.parking_state = 'complete'
                break
                

        

        
if __name__ == "__main__":
    
    rospy.init_node("parking")

    parking = Parking()
    state = State()
    stanley = Stanley()
    
    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)
    
    r = rospy.Rate(10.)
    
    while not rospy.is_shutdown():
        
        parking.CarMapping(state)
        parking.StartAlgorithmSignal()
        parking.CreatPath()
        parking.parking(state, stanley)
        
        if parking.parking_state == 'complete':
            break