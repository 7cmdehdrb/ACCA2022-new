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


class obstacle(object):
    
    def __init__(self):
        
        left_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/obstacle/data/left1.csv")
        right_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/obstacle/data/right1.csv")
        path_data = pd.read_csv("/home/acca/catkin_ws/src/ACCA2022-new/obstacle/data/center1.csv")
        
        self.left = []
        self.right = []
        self.path = []
        self.path_cx = []
        self.path_cy = []
        self.path_cx_new = []
        self.path_cy_new = []
        self.path_cyaw = []
        self.obs_mapping = []

        for i, j in zip(left_data.cx, left_data.cy):
            self.left.append([i, j])
        for i, j in zip(right_data.cx, right_data.cy):
            self.right.append([i, j])

        for i, j, k in zip(path_data.cx, path_data.cy, path_data.cyaw):
            self.path.append([i, j])
            self.path_cx.append(i)
            self.path_cy.append(j)
            self.path_cx_new.append(i)
            self.path_cy_new.append(j)
            self.path_cyaw.append(k)

        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.ObstacleCallback)  
        self.path_response = rospy.Subscriber("/path_response", PathResponse, callback=self.path_callback)    

        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)
        self.obs_pub_obs = rospy.Publisher("obstacle_position", MarkerArray, queue_size=10) 
        self.obs_pub_way = rospy.Publisher("waypoint_position", MarkerArray, queue_size=10)        
       
        self.ObsMsg = PoseArray()
        self.PathMsg = Path()
        self.msg = ControlMessage()

        # parameter
        self.detect_obs_angle = 0.8
        self.detect_obs_range = 3.
        self.prox_dis = 1.        
        self.r = 1.6
        self.det_iter= 10
        
    def ObstacleCallback(self, msg):
        self.ObsMsg = msg
   
    def path_callback(self, msg):
        self.path = msg
        
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
        
        
    def DetectObstacle(self, state):

        self.obstacle = []
        
        for i in self.ObsMsg.poses:
            
            velodyne_x = i.position.x
            velodtne_y = i.position.y
            angle = abs(m.atan2(velodtne_y, velodyne_x))
            map_x = state.x + velodyne_x * m.cos(state.yaw) - velodtne_y * m.sin(state.yaw)
            map_y = state.y + velodyne_x * m.sin(state.yaw) + velodtne_y * m.cos(state.yaw)
            dis_path = self.GetDistance2(self.path, [map_x, map_y])
                        
            if angle <= self.detect_obs_angle and dis_path <= self.detect_obs_range:

                if len(self.obs_mapping) != 0:

                    dis_arr = []
                    for j in self.obs_mapping:
                        
                        dis = self.GetDistance([map_x, map_y], j)
                        dis_arr.append(dis)
                        
                    prox_dis = min(dis_arr)
                    prox_idx = dis_arr.index(prox_dis)

                    if prox_dis <= self.prox_dis:
                        map_x, map_y, new_count = (map_x + self.obs_mapping[prox_idx][0]) / 2, (map_y + self.obs_mapping[prox_idx][1]) / 2,  self.obs_mapping[prox_idx][2] + 1
                        self.obs_mapping[prox_idx] = [map_x, map_y, new_count]
                    else:
                        self.obs_mapping.append([map_x, map_y, 1])       
                else:
                    self.obs_mapping.append([map_x, map_y, 1])
            else:
                pass
        
        for k in self.obs_mapping:    
            if k[2] >= self.det_iter:   
                self.obstacle.append(k)
            else:
                pass
            
                    
    def CreateWaypoint(self):
        
        self.waypoint_arr = []
        
        if len(self.obstacle) != 0:
            
            for obs in self.obstacle:
            
                a, b = obs[0], obs[1]
                target_idx = self.calc_target_index(self.path_cx_new, self.path_cy_new, [a, b])
                p = -1. / self.path_cyaw[target_idx]
                c = b - p * a
                
                t1 = (2*a + 2*p*b - 2*p*c + m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - self.r**2)))/(2 * (1 + p**2))
                t2 = (2*a + 2*p*b - 2*p*c - m.sqrt((-2*a -2*p*b + 2*p*c)**2 - 4 * (1+p**2) * (a**2 + b**2 + c**2 -2*b*c - self.r**2)))/(2 * (1 + p**2))

                waypoint1 = [t1, p * t1 + c]
                waypoint2 = [t2, p * t2 + c]
                
                dis_obs_left = self.GetDistance([a, b], [self.left[target_idx][0], self.left[target_idx][1]])
                dis_left_path = self.GetDistance([self.left[target_idx][0], self.left[target_idx][1]], [self.path[target_idx][0], self.path[target_idx][1]])

                dis_way1_left = self.GetDistance([waypoint1[0], waypoint1[1]], [self.left[target_idx][0], self.left[target_idx][1]])
                # dis_way1_right = self.GetDistance([waypoint1[0], waypoint1[1]], [self.right[target_idx][0], self.right[target_idx][1]])
                dis_way2_left = self.GetDistance([waypoint2[0], waypoint2[1]], [self.left[target_idx][0], self.left[target_idx][1]])
                # dis_way2_right = self.GetDistance([waypoint2[0], waypoint2[1]], [self.right[target_idx][0], self.right[target_idx][1]])   
        
                if dis_left_path - dis_obs_left < 0 : # obstacle position : right
                    if dis_way2_left > dis_way1_left: # waypoint1 position : left 
                        waypoint = waypoint1
                    else : 
                        waypoint = waypoint2
                        
                else : # obstacle position : left
                    if dis_way2_left > dis_way1_left: # waypoint position : right 
                        waypoint = waypoint2    
                    else :
                        waypoint = waypoint1
        
                self.waypoint_arr.append(waypoint)
                
        else :
            pass
            
            
    def CreatPath(self, state):
        
        state_tar_idx = self.calc_target_index(self.path_cx_new, self.path_cy_new, [state.x, state.y])

        if len(self.waypoint_arr) != 0:
            max_tar_idx = -1
            self.waypoint_arr.sort(key = lambda x : self.calc_target_index(self.path_cx_new, self.path_cy_new, x))
        
            if len(self.waypoint_arr) >= 2:
                    
                min_tar_idx = self.calc_target_index(self.path_cx_new, self.path_cy_new, self.waypoint_arr[0])
                max_tar_idx = self.calc_target_index(self.path_cx_new, self.path_cy_new, self.waypoint_arr[-1])
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
                    ys.append(self.path[len(self.path_cx) - 1][1])   

                if max_tar_idx + 40 >= state_tar_idx:
                    self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)    
                else :
                    self.cx, self.cy, self.cyaw = self.path_cx, self.path_cy, self.path_cyaw
                    
            else : # waypoint array len : 1
                max_tar_idx = self.calc_target_index(self.path_cx_new, self.path_cy_new, self.waypoint_arr[0])
                xs = [self.path[max_tar_idx - 40][0], self.waypoint_arr[0][0], self.path[max_tar_idx + 40][0]]
                ys = [self.path[max_tar_idx - 40][1], self.waypoint_arr[0][1], self.path[max_tar_idx + 40][1]]
        
                if max_tar_idx >= state_tar_idx:
                    self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
                else :
                    self.cx, self.cy, self.cyaw = self.path_cx, self.path_cy, self.path_cyaw           
        else :
            self.cx, self.cy, self.cyaw = self.path_cx, self.path_cy, self.path_cyaw
        
                
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

        
if __name__ == "__main__":
    
    rospy.init_node("obstacle")

    obs = obstacle()
    state = State()
    stanley = Stanley()
    
    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)
    target_idx = 0
    last_idx = 0
    length = 0
    
    r = rospy.Rate(10.)
    
    while not rospy.is_shutdown():
        if obs.PathMsg.path_id == 'M1M2':
            obs.DetectObstacle(state)
            obs.CreateWaypoint()
            obs.CreatPath(state)
            obs.publishPath(obs.cx, obs.cy, obs.cyaw)
            
            if len(obs.obstacle) != 0:
                obs.publishPoint(obs.obstacle, ColorRGBA(1., 0., 0., 1.), Vector3(0.5, 0.5, 0.5))
                obs.publishWaypoint(obs.waypoint_arr, ColorRGBA(1., 1., 0., 1.), Vector3(0.2, 0.2, 0.2))
                
            l = len(obs.cx)
            if l != length:
                length = l
                target_idx = 1

            if target_idx == l:
                continue
            
            di, target_idx = stanley.stanley_control(
                state, obs.cx, obs.cy, obs.cyaw, target_idx)

            obs.msg.Speed = 5.
            obs.msg.Steer = -m.degrees(di)
            obs.msg.Gear = 2
            cmd_pub.publish(obs.msg)
        else:
            pass
        
        r.sleep()