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
        
        left = pd.read_csv("/home/acca/catkin_ws/src/obstacle/data/left1.csv")
        right = pd.read_csv("/home/acca/catkin_ws/src/obstacle/data/right1.csv")
        center = pd.read_csv("/home/acca/catkin_ws/src/obstacle/data/center1.csv")
        path = pd.read_csv("/home/acca/catkin_ws/src/obstacle/data/center1.csv")
        
        self.left = []
        self.right = []
        self.center = []
        self.path = []
        self.path_cx = []
        self.path_cy = []
        self.path_cyaw = []
        
        
        for i, j in zip(left.cx, left.cy):
            self.left.append([i, j])
        for i, j in zip(right.cx, right.cy):
            self.right.append([i, j])
        for i, j in zip(center.cx, center.cy):
            self.center.append([i, j])
        for i, j, k in zip(path.cx, path.cy, path.cyaw):
            self.path.append([i, j])
            self.path_cx.append(i)
            self.path_cy.append(j)
            self.path_cyaw.append(k)

            

        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray, callback=self.ObstacleCallback)
                
        self.path_pub = rospy.Publisher("Obs_path", Path, queue_size=10)
        self.obs_pub = rospy.Publisher("Obs_position", MarkerArray, queue_size=10) 
        self.obs_pub2 = rospy.Publisher("waypoint", MarkerArray, queue_size=10)        
       
        
        self.ObsMsg = PoseArray()
        self.PathMsg = Path()
        
        # Detect Obstacle
        self.volume = 0.001
        self.angle = 0.8
        self.dis_path = 2.
        self.detection_range_min = 0.1
        self.detection_range_max = 5.
        self.r = 1.

        

    def ObstacleCallback(self, msg):
        self.ObsMsg = msg

        
    def GetLaneInformation(self, point):
        
        a_left = (self.left[0][1] - self.left[-1][1]) / (self.left[0][0] - self.left[-1][0])
        c_left = -1 * a_left * self.left[0][0] + self.left[0][1]
        
        a_right = (self.right[0][1] - self.right[-1][1]) / (self.right[0][0] - self.right[-1][0])
        c_right = -1 * a_right * self.right[0][0] + self.right[0][1]
        
        cross_lane_left = a_left * point[0] - point[1] + c_left
        cross_lane_right = a_right * point[0] - point[1] + c_right

        if cross_lane_left * cross_lane_right <= 0 : 
            OutofLane = True
            
        else :
            OutofLane = False
            
        return OutofLane
    
    
    def GetDistancetoObstacle(self, obstacle, state, point):
        a_left = (state.y - point[1]) / (state.x - point[1])
        c_left = -1 * a_left * state.x + state.y
        
        dis_path2obs = abs(a_left * obstacle[0] + obstacle[1] + c_left) / m.sqrt(a_left ** 2 + 1)
        
        return dis_path2obs
    
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
    
    def calc_target_index(self, state):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + 1.040 * np.cos(state.yaw) / 2.0
        fy = state.y + 1.040 * np.sin(state.yaw) / 2.0

        # Search nearest point index
        dx = [fx - icx for icx in self.path_cx]
        dy = [fy - icy for icy in self.path_cy]

        d = np.hypot(dx, dy)
        
        target_idx_ = np.argmin(d)

        if target_idx_ + 120 <= len(self.path_cx):
            target_idx = target_idx_ + 120
            
        else :
            target_idx = -1
            
        return target_idx
        
        
    def InrangeObstacle(self, state):
        last_obs_map = []
        last_obs_velodyne = []
        #detect inrange obstacle 
        inrange_obs_map = []
        inrange_obs_velodyne = []
        self.obs_map = []
        self.obs_velodyne = []
        
        for i in self.ObsMsg.poses:
            velodyne_x = i.position.x
            velodtne_y = i.position.y
            # print(velodyne_x)
            angle = abs(m.atan2(velodtne_y, velodyne_x)) #radian
            # print(state.x, state.y)
            map_x = state.x + velodyne_x * m.cos(state.yaw) - velodtne_y * m.sin(state.yaw)
            map_y = state.y + velodyne_x * m.sin(state.yaw) + velodtne_y * m.cos(state.yaw)
            # print(map_x, map_y)
            # volume = i.scale_x * i.scale_y * i.scale_z
            
            distance = self.GetDistance([0, 0], [velodyne_x, velodtne_y]) # ERP42 - Obstacle
            outlane = self.GetLaneInformation([map_x, map_y])
            dis_path = self.GetDistance2(self.path, [map_x, map_y]) # Path - Obstacle
            
            if len(last_obs_map) != 0:
                for i, j in zip(last_obs_map, last_obs_velodyne):
                    dis = self.GetDistance([map_x, map_y], [i[0], i[1]])
                    if dis < 1.:
                        map_x, map_y = (map_x + i[0]) / 2, (map_y + i[1]) / 2
                        inrange_obs_map.remove(i)
                        inrange_obs_velodyne.remove(j)
                        
                        print("true")
                    else :
                        pass
            # print(dis_path)
            # if angle <= self.angle and self.detection_range_min < distance < self.detection_range_max and outlane == True and dis_path < self.dis_path:
            if angle <= self.angle and dis_path < self.dis_path and self.detection_range_min < distance < self.detection_range_max:
    
            # if dis_path < self.dis_path:
                # print(dis_path)
                inrange_obs_map.append([map_x, map_y, distance])
                inrange_obs_velodyne.append([velodyne_x, velodtne_y, distance])
                last_obs_map = inrange_obs_map
                last_obs_velodyne = inrange_obs_velodyne
                # print("read")


        if len(inrange_obs_map) != 0:
            sort_inrange_obs_map = sorted(inrange_obs_map, key=lambda x:x[2])
            sort_inrange_obs_velodyne = sorted(inrange_obs_velodyne, key=lambda x:x[2])
            for i in range(len(sort_inrange_obs_velodyne)):
                self.obs_map.append([sort_inrange_obs_map[i][0], sort_inrange_obs_map[i][1]])
                self.obs_velodyne.append([sort_inrange_obs_velodyne[i][0], sort_inrange_obs_velodyne[i][1]])
         
            
    def create_path(self, state):
        
        target_idx = self.calc_target_index(state)
        
        if len(self.obs_map) == 0:
            self.cx, self.cy, self.cyaw = self.path_cx, self.path_cy, self.path_cyaw
            
            xs = []
            ys = []
            # print("0")
        elif len(self.obs_map) == 1:
            # print("1")
            l = self.GetDistance([0, 0], self.obs_velodyne[0])
            # print(l)
            # print(self.r)
            th = m.asin(self.r/l)
            
            contact1_x_velodyne= self.obs_velodyne[0][0] * m.cos(th) - self.obs_velodyne[0][1] * m.sin(th)
            contact1_y_velodyne = self.obs_velodyne[0][0] * m.sin(th) + self.obs_velodyne[0][1] * m.cos(th)
            contact2_x_velodyne = self.obs_velodyne[0][0] * m.cos(-th) - self.obs_velodyne[0][1] * m.sin(-th)
            contact2_y_velodyne = self.obs_velodyne[0][0] * m.sin(-th) + self.obs_velodyne[0][1] * m.cos(-th)

            contact1_x_map = state.x + contact1_x_velodyne * m.cos(state.yaw) - contact1_y_velodyne * m.sin(state.yaw)
            contact1_y_map = state.y + contact1_x_velodyne * m.sin(state.yaw) + contact1_y_velodyne * m.cos(state.yaw)
            contact2_x_map = state.x + contact2_x_velodyne * m.cos(state.yaw) - contact2_y_velodyne * m.sin(state.yaw)
            contact2_y_map = state.y + contact2_x_velodyne * m.sin(state.yaw) + contact2_y_velodyne * m.cos(state.yaw)
            
            
            dis_center_1 = self.GetDistance2(self.center, [contact1_x_map, contact1_y_map])
            dis_center_2 = self.GetDistance2(self.center, [contact2_x_map, contact2_y_map])
            
            if dis_center_1 > dis_center_2:
                
                xs = [state.x, contact2_x_map, self.path_cx[target_idx]]
                ys = [state.y, contact2_y_map, self.path_cy[target_idx]]
                
            else : 
                xs = [state.x, contact1_x_map, self.path_cx[target_idx]]
                ys = [state.y, contact1_y_map, self.path_cy[target_idx]]
                
            self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
            
        elif len(self.obs_map) >= 2:
            # print("2")
            xs = [state.x]
            ys = [state.y]
            
            for i in range(len(self.obs_map)):
                l = self.GetDistance([0, 0], self.obs_velodyne[i])

                th = m.asin(self.r/l)
                
                contact1_x_velodyne= self.obs_velodyne[i][0] * m.cos(th) - self.obs_velodyne[i][1] * m.sin(th)
                contact1_y_velodyne = self.obs_velodyne[i][0] * m.sin(th) + self.obs_velodyne[i][1] * m.cos(th)
                contact2_x_velodyne = self.obs_velodyne[i][0] * m.cos(-th) - self.obs_velodyne[i][1] * m.sin(-th)
                contact2_y_velodyne = self.obs_velodyne[i][0] * m.sin(-th) + self.obs_velodyne[i][1] * m.cos(-th)

                contact1_x_map = state.x + contact1_x_velodyne * m.cos(state.yaw) - contact1_y_velodyne * m.sin(state.yaw)
                contact1_y_map = state.y + contact1_x_velodyne * m.sin(state.yaw) + contact1_y_velodyne * m.cos(state.yaw)
                contact2_x_map = state.x + contact2_x_velodyne * m.cos(state.yaw) - contact2_y_velodyne * m.sin(state.yaw)
                contact2_y_map = state.y + contact2_x_velodyne * m.sin(state.yaw) + contact2_y_velodyne * m.cos(state.yaw)
                    
                dis_center_1 = self.GetDistance2(self.center, [contact1_x_map, contact1_y_map])
                dis_center_2 = self.GetDistance2(self.center, [contact2_x_map, contact2_y_map])
                
                if dis_center_1 > dis_center_2:
                    
                    xs.append(contact2_x_map)
                    ys.append(contact2_y_map)
                    
                else : 
                    xs.append(contact1_x_map)
                    ys.append(contact1_y_map)
                
            xs.append(self.path_cx[target_idx])
            ys.append(self.path_cy[target_idx])

            self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
                
        return xs, ys
                
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
            
            
    def publishPoint(self, position):
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
            marker.scale = Vector3(0.5, 0.5, 0.5)
            marker.color = ColorRGBA(1.,0.,0.,1.)

            marker.lifetime = genpy.Duration(secs=0.2)

            msg.markers.append(marker)

        self.obs_pub.publish(msg)
        
    def publishPoint2(self, xx, yy):
        msg = MarkerArray()

        for i in range(len(xx)):
            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()

            marker.ns = str(i)
            marker.id = 2

            marker.type = 3
            marker.action = 0

            marker.pose.position = Point(xx[i], yy[i], 0.)  # cone.point
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = Vector3(0.2, 0.2, 0.2)
            marker.color = ColorRGBA(1.,1.,0.,1.)

            marker.lifetime = genpy.Duration(secs=0.2)

            msg.markers.append(marker)

        self.obs_pub2.publish(msg)

        
if __name__ == "__main__":
    
    rospy.init_node("obstacle")

    obs = obstacle()
    state = State()
    stanley = Stanley()
    cmd_pub = rospy.Publisher("/cmd_msg", ControlMessage, queue_size=1)
    target_idx = 0
    last_idx = 0
    length = 0
    
    r = rospy.Rate(1.)
    
    while not rospy.is_shutdown():
        obs.InrangeObstacle(state)
        xx, yy = obs.create_path(state)
        obs.publishPath(obs.cx, obs.cy, obs.cyaw)
        
        if len(obs.obs_map) != 0:
            obs.publishPoint(obs.obs_map)
            obs.publishPoint2(xx, yy)
            
        
        l = len(obs.cx)
        if l != length:
            length = l
            target_idx = 1

        if target_idx == l:
            continue
        di, target_idx = stanley.stanley_control(
            state, obs.cx, obs.cy, obs.cyaw, target_idx)

        msg = ControlMessage()
        msg.Speed = 3.
        msg.Steer = -m.degrees(di)

        cmd_pub.publish(msg)
                
        r.sleep()