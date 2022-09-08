#!/usr/bin/env python

import os
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
from enum import Enum
from cubic_spline_planner import calc_spline_course

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import OdomState
    from stanley import Stanley
except Exception as ex:
    rospy.logfatal(ex)



class DiagonalParking(Enum):
    detect_area = 0
    in_area = 1
    out_area = 2
    End = 3


class Parking(object):
    
    def __init__(self):


        path_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/mission/data/center.csv")

        self.path = []
        self.path_cx = []
        self.path_cy = []
        self.path_cyaw = []

        for i, j, k in zip(path_data.cx, path_data.cy, path_data.cyaw):
            self.path.append([i, j])
            self.path_cx.append(i)
            self.path_cy.append(j)
            self.path_cyaw.append(k)


        # parameter
        self.area = [[1.0, 6.5], [3.0, 7.0], [5.0, 7.5], [7.0, 8.5], [9.0, 9.5], [11.0, 10.5]] #school
        self.p = -m.atan2(0.1, 8.) # school
        
        
        self.create_path_length = 2 # meter
        self.point3_inx = 30
        self.detect_start_idx = 20 # point3 - parameter
        self.obs_range = 1.
        self.target_area = 10
        self.obs_count = 20
        self.width = 2.
        self.length_p = 3.
           
        self.parking_state = 'detect area'
        self.start_signal = False
        self.area_num = [0, 0, 0, 0, 0, 0]

        rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.ObstacleCallback)
        rospy.Subscriber("/path_response", PathResponse, callback=self.path_callback)    
        self.obs_pub_parking = rospy.Publisher("parking_position", MarkerArray, queue_size=10)        
        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)



        self.target_idx = 0
        self.length = 0

        self.local_target_idx = 0
        self.local_length = 0  

        self.detect_target_idx = 0
        self.detect_length = 0

        self.ObsMsg = PoseArray()

        self.PathMsg = PathResponse()
        self.msg = ControlMessage()

        self.r = rospy.Rate(30)

        self.state = OdomState()
        self.stanley = Stanley()

    def ObstacleCallback(self, msg):
        self.ObsMsg = msg
        
    def path_callback(self, msg):
        self.PathMsg = msg

    def GetPoint(self, area):
        
        per_idx = self.calc_target_index(self.path_cx, self.path_cy, [area[0], area[1]])
        
        point1 = [area[0], area[1]]
        point2 = [self.path_cx[per_idx], self.path_cy[per_idx]]
        point3 = [self.path_cx[per_idx - self.point3_inx], self.path_cy[per_idx - self.point3_inx]]
        
        p = self.p
        
        meetpoint_x = (((point3[1] - point2[1]) / (point3[0] - point2[0]) * point2[0]) -point2[1] - (p * point1[0]) + point1[1]) / (((point3[1] - point2[1]) / (point3[0] - point2[0])) - p)
        meetpoint_y = p * (meetpoint_x - point1[0]) + point1[1]
        
        return point1, [meetpoint_x, meetpoint_y], point3
    

    def InDetectRange(self, area):
        
        _, meetpoint, point3 = self.GetPoint(area)
        
        current_idx = self.calc_target_index(self.path_cx, self.path_cy, [self.state.x, self.state.y])
        meet_idx = self.calc_target_index(self.path_cx, self.path_cy, [meetpoint[0], meetpoint[1]])
        point3_idx = self.calc_target_index(self.path_cx, self.path_cy, [point3[0], point3[1]])

        
        if point3_idx - self.detect_start_idx < current_idx < point3_idx:
            in_detect_range = 'detect range'
            
        elif current_idx >= point3_idx:
            in_detect_range = 'complete detect range'
            
        else:
            in_detect_range = 'wait detect range'
            
        return in_detect_range
    
    
    def CarMapping(self, target, target_idx):
                
        for i in self.ObsMsg.poses:
        
            velodyne_x = i.position.x
            velodtne_y = i.position.y
            map_x = self.state.x + velodyne_x * m.cos(self.state.yaw) - velodtne_y * m.sin(self.state.yaw)
            map_y = self.state.y + velodyne_x * m.sin(self.state.yaw) + velodtne_y * m.cos(self.state.yaw)
        
            dis = self.GetDistance([target[0], target[1]], [map_x, map_y])
            
            if dis <= self.obs_range:
                self.area_num[target_idx] += 1
    
    
    def SelectArea(self):
        
        self.msg = ControlMessage()
        print("detect!!!")
        self.publishArea(self.area)

        l = len(self.path_cx)

        if l != self.detect_length:
            self.detect_length = l
            self.detect_target_idx = 1

        if self.detect_target_idx == l:
            pass
                
        di, self.detect_target_idx = self.stanley.stanley_control(
        self.state, self.path_cx, self.path_cy, self.path_cyaw, self.detect_target_idx)
        
        self.msg.Speed = 7.
        self.msg.Steer = -m.degrees(di)
        self.msg.Gear = 2


        for i in range(6):
            
            inrange = 'none'
            inrange = self.InDetectRange(self.area[i])
            
            if inrange == 'detect range':
                self.CarMapping(self.area[i], i)
                
            else :
                pass

            if inrange == 'complete detect range' and self.area_num[i] < self.obs_count:
                self.target_area = i
                self.CreatPath()
                self.parking_state = 'in'  

    def CreatPath(self):

        point1, meetpoint, point3 = self.GetPoint(self.area[self.target_area])
        
        a, b = point1[0], point1[1]
        p = -1 / self.p
        c = b - p * a
        
        xs = [self.state.x]
        ys = [self.state.y]
    
        for i in range(10):
            
            r = 0.01 + self.create_path_length * 0.1 * i
            t1 = (2*a + 2*p*b - 2*p*c + m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - r**2)))/(2 * (1 + p**2))
            t2 = (2*a + 2*p*b - 2*p*c - m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - r**2)))/(2 * (1 + p**2))

            waypoint1 = [t1, p * t1 + c]
            waypoint2 = [t2, p * t2 + c]
            
            dis1 = self.GetDistance2(self.path, waypoint1)
            dis2 = self.GetDistance2(self.path, waypoint2)
            
            if dis1 > dis2:
                xs.insert(1, waypoint2[0])
                ys.insert(1, waypoint2[1])
                
            else :
                xs.insert(1, waypoint1[0])
                ys.insert(1, waypoint1[1])
                
        self.local_cx, self.local_cy, self.local_cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
            


    def parking(self):
                
        if self.parking_state == 'in':
            self.publishArea(self.area)
            print("in!!!")

            self.publishPath(self.local_cx, self.local_cy, self.local_cyaw)
            
            l = len(self.local_cx)

            if l != self.local_length:
                self.local_length = l
                self.local_target_idx = 1

            if self.local_target_idx == l:
                pass
            
            di, self.local_target_idx = self.stanley.stanley_control(
            self.state, self.local_cx, self.local_cy, self.local_cyaw, self.local_target_idx)

            self.msg.Speed = 5.
            self.msg.Steer = -m.degrees(di)
            self.msg.Gear = 2

            state_idx = self.calc_target_index(self.local_cx, self.local_cy, [self.state.x, self.state.y])

                        
            if abs(state_idx - len(self.local_cx)) < 3:
                self.parking_state = 'stop'


        elif self.parking_state == 'stop':
                self.msg.Speed = 0.
                self.msg.Steer = 0.
                self.msg.brake = 70.

                sleep(6)

                self.parking_state = 'out'
            

        elif self.parking_state == 'out':
            self.publishArea(self.area)
            print("out!!!")

            self.msg.Speed = 5.
            self.msg.Steer = 0.
            self.msg.brake = 0.
            self.msg.Gear = 0

            dis = self.GetDistance2(self.path, [self.state.x, self.state.y])
            
            if dis < 0.2:

                self.msg.Speed = 0.
                self.msg.Steer = 0.
                self.msg.brake = 70.
                self.msg.Gear = 2

                sleep(2)

                self.parking_state = 'complete'
    
            
        # elif self.parking_state == 'complete':

        #     self.publishArea(self.area)

        #     l = len(self.path_cx)

        #     if l != length:
        #         length = l
        #         target_idx = 1

        #     if target_idx == l:
        #         pass
            
        #     di, target_idx = self.stanley.stanley_control(
        #     self.state, self.path_cx, self.path_cy, self.path_cyaw, target_idx)
            
        #     self.msg.Speed = 7.
        #     self.msg.Steer = -m.degrees(di)
        #     self.msg.brake = 0.

        #     self.msg.Gear = 2
        #     # self.cmd_pub.publish(self.msg)
        #     self.r.sleep()
        
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
            
            
    def publishArea(self, parking_area):
        msg = MarkerArray()
        for i in range(len(parking_area)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = str(i)
            marker.id = 1
            marker.type = 1
            marker.action = 0
            marker.pose.position = Point(parking_area[i][0], parking_area[i][1], 0.)
            marker.pose.orientation = Quaternion(0., 0., self.p, 1)
            marker.scale = Vector3(self.width, self.length_p, 0.1)
            marker.color = ColorRGBA(1., 0., 0., 0.7)
            marker.lifetime = genpy.Duration(secs=0.2)
            msg.markers.append(marker)
        self.obs_pub_parking.publish(msg)
  
  
    
    def main(self):
        
        if self.parking_state == 'detect area':
            self.SelectArea()
        else:
            self.parking()
