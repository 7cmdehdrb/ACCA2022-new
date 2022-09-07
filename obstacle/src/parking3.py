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



        area_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/obstacle/data/area_school.csv")
        path_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/obstacle/data/left.csv")
        
        self.area = []
        self.center = []
        for a, b, c, d, e, f in zip(area_data.area1, area_data.area2, area_data.area3, area_data.area4, area_data.area5, area_data.area6):
            self.area[0].append(a)
            self.area[1].append(b)
            self.area[2].append(c)
            self.area[3].append(d)
            self.area[4].append(e)
            self.area[5].append(f)



        self.path = []
        self.path_cx = []
        self.path_cy = []
        self.path_cyaw = []

        for i, j, k in zip(self.path_cx, self.path_cy, self.path_cyaw):
            self.path.append([i, j])
            self.path_cx.append(i)
            self.path_cy.append(j)
            self.path_cyaw.append(k)


        self.area = [[4.5, 2.1], [6.7, 2.9], [8.7, 3.7], [11., 4.5], [13., 5.3], [15.5, 6.2]]
        self.area_num = [10000, 0, 0, 0, 0, 0]
        self.in_detect_range = [False, False, False, False, False, False]

        self.create_path_FS = 2 # meter
        self.parking_state = 'detect area'
        
        
        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.ObstacleCallback)
                
        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)
        self.obs_pub_obs = rospy.Publisher("obstacle_position", MarkerArray, queue_size=10) 
        self.obs_pub_way = rospy.Publisher("waypoint_position", MarkerArray, queue_size=10)        
        self.obs_pub_parking = rospy.Publisher("parking_position", MarkerArray, queue_size=10)        

        self.p = -m.atan2(1., 1.0)
        self.ObsMsg = PoseArray()
        self.PathMsg = Path()
        
        self.det_start = 100
        self.obs_range = 1.
        self.start_signal = False
        
            
        # Detect Obstacle
        self.target_area = 10
        self.dis_path = 3.
        
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

            marker.pose.position = Point(position[i][0], position[i][1], 0.)
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
  
    def publishparkingpoint(self, position, color=ColorRGBA, scale=Vector3):
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

        self.obs_pub_parking.publish(msg)
  
  
  ##################code##################
  
    def GetPoint(self, area):
        
        per_idx = self.calc_target_index(self.path_cx, self.path_cy, [area[0], area[1]])
        
        point1 = [area[0], area[1]]
        point2 = self.path[per_idx]
        point3 = self.path[per_idx - self.det_start]
        
        p = self.p
        
        meetpoint_x = (((point3[1] - point2[1]) / (point3[0] - point2[0]) * point2[0]) -point2[1] - (p * point1[0]) + point1[1]) / (((point3[1] - point2[1]) / (point3[0] - point2[0])) - p)
        meetpoint_y = p * (meetpoint_x - point1[0]) + point1[1]
        
        return point1, [meetpoint_x, meetpoint_y], point3
    

    def InDetectRange(self, area, state):
        

        _, meetpoint, point3 = self.GetPoint(area)
        
        current_idx = self.calc_target_index(self.path_cx, self.path_cy, [state.x, state.y])
        meet_idx = self.calc_target_index(self.path_cx, self.path_cy, [meetpoint[0], meetpoint[1]])
        point3_idx = self.calc_target_index(self.path_cx, self.path_cy, [point3[0], point3[1]])

        
        # print("area_num : ", self.area.index(area), "current : ", current_idx, ", meet : ", meet_idx, ", point3 : ",point3_idx)
        if point3_idx < current_idx < meet_idx - 10:
            
            in_detect_range = 'in'
            
        elif current_idx > meet_idx - 10:
            
            in_detect_range = 'out'
            
        else:
            in_detect_range = 'wait'
            
            
        return in_detect_range
    
    
    
    def CarMapping(self, state, target, target_idx):
                
 
        for i in self.ObsMsg.poses:
        
            velodyne_x = i.position.x
            velodtne_y = i.position.y
            angle = abs(m.atan2(velodtne_y, velodyne_x)) #radian
            map_x = state.x + velodyne_x * m.cos(state.yaw) - velodtne_y * m.sin(state.yaw)
            map_y = state.y + velodyne_x * m.sin(state.yaw) + velodtne_y * m.cos(state.yaw)
        
            dis = self.GetDistance([target[0], target[1]], [map_x, map_y])
            
            if dis <= self.obs_range:
                self.area_num[target_idx] += 1
    
    
    def SelectArea(self, state):
        target_idx = 0
        length = 0
        
        
        msg = ControlMessage()

        while self.parking_state == 'detect area':

                        
            l = len(self.path_cx)

            if l != length:
                length = l
                target_idx = 1

            if target_idx == l:
                continue
            
            di, target_idx = stanley.stanley_control(
            state, self.path_cx, self.path_cy, self.path_cyaw, target_idx)
            
            msg.Speed = 5.
            msg.Steer = -m.degrees(di)
            msg.Gear = 2
            cmd_pub.publish(msg)

            print(self.area_num)
            for i in range(6):
                
                inrange = self.InDetectRange(self.area[i], state)
                
                if inrange == 'in':
                    self.CarMapping(state, self.area[i], i)
                    
                else :
                    pass

                if inrange == 'out' and self.area_num[i] < 20:
                    self.target_area = i
                    self.parking_state = 'in'  






    def CreatPath(self, state):

        # self.target_area = 5
        print(self.target_area)
        point1, meetpoint, point3 = self.GetPoint(self.area[self.target_area])
        
        a, b = point1[0], point1[1]
        p = self.p
        c = b - p * a
        
        xs = [state.x]
        ys = [state.y]
    
        for i in range(10):
            
            r = 0.2 + 0.2 * i
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
        
        
        msg = ControlMessage()
        self.CreatPath(state)

        while self.parking_state == 'in':
            
            self.publishPoint(parking.area, ColorRGBA(1., 0, 0, 1.), Vector3(0.5, 0.5, 0.5))
            self.publishWaypoint([[self.local_cx[-1], self.local_cy[-1]]], ColorRGBA(1., 1., 0.,1.), Vector3(0.2, 0.2, 0.2))            
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
            
            state_idx = self.calc_target_index_state(self.local_cx, self.local_cy, state)
            cmd_pub.publish(msg)
            
            # print('in!!!')
            
            if abs(state_idx - len(self.local_cx)) < 3:

                msg.Speed = 0.
                msg.Steer = 0.
                msg.brake = 70.
                cmd_pub.publish(msg)

                sleep(5)
                
                self.parking_state = 'out'
                print("out!!!!")
                break
            
        while self.parking_state == 'out':
        
            msg.Speed = 5.
            msg.Steer = 0.
            msg.brake = 0.

            msg.Gear = 0
            cmd_pub.publish(msg)

            dis = self.GetDistance2(self.path, [state.x, state.y])
            # print("out!!!")
            if dis < 0.2:
                self.parking_state = 'complete'

                break            

        while parking.parking_state == 'complete':
            
            l = len(self.path_cx)

            if l != length:
                length = l
                target_idx = 1

            if target_idx == l:
                continue
            
            di, target_idx = stanley.stanley_control(
            state, self.path_cx, self.path_cy, self.path_cyaw, target_idx)
            
            msg.Speed = 7.
            msg.Steer = -m.degrees(di)
            msg.Gear = 2
            cmd_pub.publish(msg)
        
if __name__ == "__main__":
    
    rospy.init_node("parking")

    parking = Parking()
    state = State()
    stanley = Stanley()
    
    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)
    
    r = rospy.Rate(5.)
    
    while not rospy.is_shutdown():
        parking.publishparkingpoint(parking.area, ColorRGBA(1.,1.,1.,1.), Vector3(0.5 ,0.5, 0.5))
        if parking.parking_state == 'detect area':
            parking.SelectArea(state)
        
        else:
            parking.parking(state, stanley)
            
        r.sleep()

            
