#!/usr/bin/env python

import os
import sys
from time import sleep
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

        self.obstacle_sub = rospy.Subscriber("/after_clustering/markers", MarkerArray, callback=self.ObstacleCallback)
        
        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)
        self.obs_pub = rospy.Publisher("Obs_position", MarkerArray, queue_size=10)
        self.point_pub = rospy.Publisher("r_point", Point, queue_size = 10 )
        
        
        #SelectInrageObstacle
        self.detection_range_obstacle_distance_min = 1.
        self.detection_range_obstacle_distance_max = 7.

        #DetectPath
        self.detect_path_dis = 7.
        self.target_point = [25., 0]
        self.left_lane = [[0,2.], [50.3515,2.]]
        self.right_lane = [[0.,-2], [60.51,-2.]]
        self.detect_distance = 5.
        self.ObsMsg = MarkerArray()
        
        #test
        self.obs_candidates = [[13., 0], [20., 0]]
        
    def ObstacleCallback(self, msg):
        self.ObsMsg = msg
        
        
    def GetLaneInformation(self, point):
        
        a_left = (self.left_lane[0][1] - self.left_lane[-1][1]) / (self.left_lane[0][0] - self.left_lane[-1][0])
        c_left = -1 * a_left * self.left_lane[0][0] + self.left_lane[0][1]
        
        a_right = (self.right_lane[0][1] - self.right_lane[-1][1]) / (self.right_lane[0][0] - self.right_lane[-1][0])
        c_right = -1 * a_right * self.right_lane[0][0] + self.right_lane[0][1]
        
        cross_lane_left = a_left * point[0] + point[1] + c_left
        cross_lane_right = a_right * point[0] + point[1] + c_right

        if cross_lane_left * cross_lane_right <= 0 : 
            self.OutofLane = True
            
        else :
            self.OutofLane = False
            
        return self.OutofLane
    
    
    
    def GetDistancetoObstacle(self, obstacle, state, point):
        a_left = (state.y - point[1]) / (state.x - point[1])
        c_left = -1 * a_left * state.x + state.y
        
        dis_path2obs = abs(a_left * obstacle[0] + obstacle[1] + c_left) / m.sqrt(a_left ** 2 + 1)
        
        return dis_path2obs
    
    
    def SelectInrangeObstacle(self, state):
        
        #detect inrange obstacle candidates
        self.detection_obstacle = []
        
        for i in self.obs_candidates:
    
            position_x = i[0]
            position_y = i[1]
            volume = 1.
            angle = abs(m.atan2(position_y, position_x) - state.yaw) #radian
            print(angle)
            distance = m.sqrt((position_x - state.x) ** 2 + (position_y - state.y) ** 2)
            
            # if volume >= 0.001 and angle <= 0.5 and self.detection_range_obstacle_distance_min < distance < self.detection_range_obstacle_distance_max:
            if volume >= 0.001 and state.x <= position_x and self.detection_range_obstacle_distance_min < distance < self.detection_range_obstacle_distance_max:
    
                position_map_x = position_x
                position_map_y = position_y

                self.detection_obstacle.append([[position_map_x, position_map_y], distance, angle])

        return self.detection_obstacle 
         
    def SelectObstacle(self):
        
        #select nearest obstacle
        
        if len(self.detection_obstacle) != 0:
            dis_array = []
            for i in self.detection_obstacle:
                dis_array.append(i[1])
            min_dis_index = dis_array.index(min(dis_array))
            
            self.select_obstacle = self.detection_obstacle[min_dis_index] 
        
        else :
            pass
        
        
    def DetectPath(self, state):

        obs_dis_array = []
        
        for i in range(7):
            
            # detect angel range : -30 ~ 30 (degree)
            theta = state.yaw + 0.523599 - i * 0.174533
            
            dis_max_point_x = state.x + self.detect_path_dis * m.cos(theta)
            dis_max_point_y = state.y + self.detect_path_dis * m.sin(theta)
            
            # get distance : obstacle to path
            dis_obstacle = self.GetDistancetoObstacle(self.select_obstacle[0], state, [dis_max_point_x, dis_max_point_y])
            
            # search out of lane
            OutLane = self.GetLaneInformation([dis_max_point_x, dis_max_point_y])
            
            if dis_obstacle >= 0.5 and OutLane == True:
                
                obs_dis_array.append(dis_obstacle)
                
        if len(obs_dis_array) != 0:  
            select_index = obs_dis_array.index(min(obs_dis_array))
        
            xs = [state.x, state.x + self.detect_path_dis * m.cos(state.yaw + 0.523599 - select_index * 0.174533)]
            ys = [state.x, state.y + self.detect_path_dis * m.sin(state.yaw + 0.523599 - select_index * 0.174533)]
            
            self.cx, self.cy, cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
            
            
    def CreatPath(self, state):
        
        # to search for the nearest point
        dis_r_array = []
        for i in range(len(self.cx)):
            dis = m.sqrt((self.select_obstacle[0][0] - self.cx[i]) ** 2 + (self.select_obstacle[0][1] - self.cy[i]) ** 2)
            dis_r_array.append(dis)
            
        r_index = dis_r_array.index(min(dis_r_array))
        
        xs = [state.x, self.cx[r_index], self.target_point[0]]
        ys = [state.y, self.cy[r_index], self.target_point[1]]

        self.final_cx, self.final_cy, self.final_cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds = 0.5)
        
        self.r_x = self.cx[r_index]
        self.r_y = self.cy[r_index]
        
    def NotDetectPath(self, state):
        xs = [state.x, self.target_point[0]]
        ys = [state.y, self.target_point[1]]
        
        self.final_cx, self.final_cy, self.final_cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds = 0.5)
               
    def publishPath(self):
        
        path = Path()

        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()


        for i in range(len(self.final_cx)):
            pose = PoseStamped()

            pose.header.frame_id = "odom"
            pose.header.stamp = rospy.Time.now()

            quat = quaternion_from_euler(0., 0., self.final_cyaw[i])

            pose.pose.position = Point(self.final_cx[i], self.final_cy[i], 0.)
            pose.pose.orientation = Quaternion(
                quat[0], quat[1], quat[2], quat[3])

            path.poses.append(pose)

        self.path_pub.publish(path)
            
            
    def publishPoint(self, position):
        msg = MarkerArray()

        for i in range(len(position)):
            marker = Marker()

            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()

            marker.ns = str(i)
            marker.id = 1

            marker.type = 3
            marker.action = 0

            marker.pose.position = Point(position[i][0], position[i][1], 0.)  # cone.point
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = Vector3(0.5, 0.5, 0.5)
            marker.color = ColorRGBA(1.,0.,0.,1.)

            marker.lifetime = genpy.Duration(secs=0.2)

            msg.markers.append(marker)

        self.obs_pub.publish(msg)
        
    # def publishPoint(self, x, y):
    #     msg = Point(x, y, 0.)
    #     self.point_pub.publish(msg)
        
        
if __name__ == "__main__":
    rospy.init_node("cone_tracker")

    obs = obstacle()
    state = State()
    r = rospy.Rate(10.)
    count = 0
    while not rospy.is_shutdown():

        obs.SelectInrangeObstacle(state)
        if len(obs.detection_obstacle) != 0:
            obs.SelectObstacle()
            if len(obs.select_obstacle) != 0:
                obs.DetectPath(state)
                obs.CreatPath(state)
                obs.publishPoint([[obs.r_x, obs.r_y], [obs.select_obstacle[0][0], obs.select_obstacle[0][1]]])
                print("Detect!!")
        else :
            obs.NotDetectPath(state)
            print("not detect!")
            # obs.publishPoint([[obs.select_obstacle[0][0], obs.select_obstacle[0][1]]])
        obs.publishPath()
        r.sleep()