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
from enum import Enum
from cubic_spline_planner import calc_spline_course

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State
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


        # parameter
        self.area = [[1.0, 6.5], [3.0, 7.0], [5.0, 7.5], [7.0, 8.5], [9.0, 9.5], [11.0, 10.5]]
        self.p = -m.atan2(0.1, 8.)
        self.create_path_length = 2 # meter
        self.point3_inx = 30
        self.detect_start_idx = 20 # point3 - parameter
        self.obs_range = 1.
        self.target_area = 10
        self.obs_count = 20
        self.width = 2.
        self.length = 3.
           
        self.parking_state = 'detect area'
        self.start_signal = False
        self.area_num = [0, 0, 0, 0, 0, 0]

        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.ObstacleCallback)
        self.path = rospy.Subscriber("/path_response", PathResponse, callback=self.path_callback)    
        self.obs_pub_parking = rospy.Publisher("parking_position", MarkerArray, queue_size=10)        
        self.cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=10)
        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)


        self.ObsMsg = PoseArray()
        self.PathMsg = PathResponse()
        self.msg = ControlMessage()

    def ObstacleCallback(self, msg):
        self.ObsMsg = msg
        
    def path_callback(self, msg):
        self.PathMsg = msg
        

    def GetPoint(self, area):
        
        per_idx = self.calc_target_index(self.PathMsg.cx, self.PathMsg.cy, [area[0], area[1]])
        
        point1 = [area[0], area[1]]
        point2 = [self.PathMsg.cx[per_idx], self.PathMsg.cy[per_idx]]
        point3 = [self.PathMsg.cx[per_idx - self.point3_inx], self.PathMsg.cy[per_idx - self.point3_inx]]
        
        p = self.p
        
        meetpoint_x = (((point3[1] - point2[1]) / (point3[0] - point2[0]) * point2[0]) -point2[1] - (p * point1[0]) + point1[1]) / (((point3[1] - point2[1]) / (point3[0] - point2[0])) - p)
        meetpoint_y = p * (meetpoint_x - point1[0]) + point1[1]
        
        return point1, [meetpoint_x, meetpoint_y], point3
    

    def InDetectRange(self, area, state):
        
        _, meetpoint, point3 = self.GetPoint(area)
        
        current_idx = self.calc_target_index(self.PathMsg.cx, self.PathMsg.cy, [state.x, state.y])
        meet_idx = self.calc_target_index(self.PathMsg.cx, self.PathMsg.cy, [meetpoint[0], meetpoint[1]])
        point3_idx = self.calc_target_index(self.PathMsg.cx, self.PathMsg.cy, [point3[0], point3[1]])

        
        if point3_idx - self.detect_start_idx < current_idx < point3_idx:
            in_detect_range = 'detect range'
            
        elif current_idx >= point3_idx:
            in_detect_range = 'complete detect range'
            
        else:
            in_detect_range = 'wait detect range'
            
        return in_detect_range
    
    
    def CarMapping(self, state, target, target_idx):
                
 
        for i in self.ObsMsg.poses:
        
            velodyne_x = i.position.x
            velodtne_y = i.position.y
            map_x = state.x + velodyne_x * m.cos(state.yaw) - velodtne_y * m.sin(state.yaw)
            map_y = state.y + velodyne_x * m.sin(state.yaw) + velodtne_y * m.cos(state.yaw)
        
            dis = self.GetDistance([target[0], target[1]], [map_x, map_y])
            
            if dis <= self.obs_range:
                self.area_num[target_idx] += 1
    
    
    def SelectArea(self, state):
        target_idx = 0
        length = 0
        msg = ControlMessage()
        while self.parking_state == 'detect area' and not rospy.is_shutdown():
            self.publishArea(parking.area)
            # self.publishPath(parking.path_cx, parking.path_cy, parking.path_cyaw)
            l = len(self.PathMsg.cx)

            if l != length:
                length = l
                target_idx = 1

            if target_idx == l:
                continue
            
            di, target_idx = stanley.stanley_control(
            state, self.PathMsg.cx, self.PathMsg.cy, self.PathMsg.cyaw, target_idx)
            
            msg.Speed = 5.
            msg.Steer = -m.degrees(di)
            msg.Gear = 2
            self.cmd_pub.publish(msg)

            for i in range(6):
                
                inrange = 'none'
                inrange = self.InDetectRange(self.area[i], state)
                
                if inrange == 'detect range':
                    self.CarMapping(state, self.area[i], i)
                    
                else :
                    pass

                if inrange == 'complete detect range' and self.area_num[i] < self.obs_count:
                    self.target_area = i
                    self.parking_state = 'in'  
                r.sleep()


    def CreatPath(self, state):

        point1, meetpoint, point3 = self.GetPoint(self.area[self.target_area])
        
        a, b = point1[0], point1[1]
        p = -1 / self.p
        c = b - p * a
        
        xs = [state.x]
        ys = [state.y]
    
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
            

    def parking(self, state, stanley):
        local_target_idx = 0
        local_length = 0   
        
        target_idx = 0
        length = 0
        
        self.CreatPath(state)
        msg = ControlMessage()
        while self.parking_state == 'in' and not rospy.is_shutdown():
            self.publishArea(parking.area)

            print("in!!!")
            self.publishPath(self.local_cx, self.local_cy, self.local_cyaw)
            
            l = len(self.local_cx)

            if l != local_length:
                local_length = l
                local_target_idx = 1

            if local_target_idx == l:
                continue
            
            di, local_target_idx = stanley.stanley_control(
            state, self.local_cx, self.local_cy, self.local_cyaw, target_idx)

            msg.Speed = 3.
            msg.Steer = -m.degrees(di)
            msg.Gear = 2
            
            state_idx = self.calc_target_index(self.local_cx, self.local_cy, [state.x, state.y])
            self.cmd_pub.publish(msg)

            r.sleep()
                        
            if abs(state_idx - len(self.local_cx)) < 3:

                msg.Speed = 0.
                msg.Steer = 0.
                msg.brake = 70.
                self.cmd_pub.publish(msg)

                sleep(5)
                
                self.parking_state = 'out'
                break

        while self.parking_state == 'out' and not rospy.is_shutdown():
            self.publishArea(parking.area)

            msg.Speed = 5.
            msg.Steer = 0.
            msg.brake = 0.
            msg.Gear = 0
            self.cmd_pub.publish(msg)
            dis = self.GetDistance2(self.path, [state.x, state.y])
            
            if dis < 0.2:
                self.parking_state = 'complete'

                break            
            r.sleep()

        while parking.parking_state == 'complete' and not rospy.is_shutdown():
            sleep(1.)
            break
            # self.publishArea(parking.area)

            # l = len(self.PathMsg.cx)

            # if l != length:
            #     length = l
            #     target_idx = 1

            # if target_idx == l:
            #     continue
            
            # di, target_idx = stanley.stanley_control(
            # state, self.PathMsg.cx, self.path_cy, self.PathMsg.cyaw, target_idx)
            
            # msg.Speed = 7.
            # msg.Steer = -m.degrees(di)
            # msg.Gear = 2
            # self.cmd_pub.publish(msg)
        
        
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
            marker.scale = Vector3(self.width, self.length, 0.1)
            marker.color = ColorRGBA(1., 0., 0., 0.7)
            marker.lifetime = genpy.Duration(secs=0.2)
            msg.markers.append(marker)
        self.obs_pub_parking.publish(msg)
  
if __name__ == "__main__":
    
    rospy.init_node("parking")

    parking = Parking()
    state = State()
    stanley = Stanley()

    parking_state = DiagonalParking.detect_area
    r = rospy.Rate(5)

    if parking.PathMsg.path_id == "adfdf" :
        if parking.parking_state == 'detect area':
            parking.SelectArea(state)
        
        else:
            parking.parking(state, stanley)
    
    else:
        pass

