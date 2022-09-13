#!/usr/bin/env python

import rospy
import sys
import rospkg
import numpy as np
import pandas as pd
import math as m
import genpy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseStamped, PoseArray
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from cubic_spline_planner import calc_spline_course

from mission.msg import obTF

"""
Subscribe 'scan_filtered' and
Publish 'ob_TF'
"""


class Lidar(object):
    def __init__(self):
        super(Lidar, self).__init__()

        xs = [0, 20]
        ys = [0, 0]
        
        cx, cy, _, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
        self.path = []
        for i in range(len(cx)):
            self.path.append([cx[i], cy[i]])
            
        self.thr_dis = 1.
        self.thr_path_dis = 1.0
        
        rospy.Subscriber("/scan_filtered", LaserScan, callback=self.laserCallback)
        rospy.Subscriber("/odometry/kalman", Odometry, callback=self.odomcallback)
        self.obs_pub = rospy.Publisher("/obs_pub", MarkerArray, queue_size=10)        
        self.part_pub = rospy.Publisher("ob_TF", obTF, queue_size=5)

        self.ranges = []

    def laserCallback(self, msg):

        self.ranges = msg.ranges

    def odomcallback(self, msg):
        self.state_x = msg.pose.pose.position.x
        self.state_y = msg.pose.pose.position.y
        self.state_yaw = msg.pose.pose.orientation.z
        
        
    def GetDistance(self, path, point):           
        dis_r_array = []  
        for i in range(len(path)):
            dis = m.sqrt((point[0] - path[i][0]) ** 2 + (point[1] - path[i][1]) ** 2)
            dis_r_array.append(dis)     
        min_dis = min(dis_r_array)
        return min_dis
    
    
    def publishPoint(self, point):
        msg = MarkerArray()
        for i in range(len(point)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = str(i)
            marker.id = 1
            marker.type = 3
            marker.action = 0
            marker.pose.position = Point(point[i][0], point[i][1], 0.)
            marker.pose.orientation = Quaternion(0., 0., 0., 1)
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA((i+1)/41., 0., 0., 1.)
            marker.lifetime = genpy.Duration(secs=0.2)
            msg.markers.append(marker)
        self.obs_pub.publish(msg)
        
        
    def detect_range(self):
        
        self.mapping = []
        self.obstacle = []

        if len(self.ranges) == 0:
            return 0

        for i in range(1, 41):
            
            theta = (100 + (i / 2)) * np.pi / 180
            lidar_x = self.ranges[158+i] * m.cos(theta)
            lidar_y = self.ranges[158+i] * m.sin(theta)
            map_x = self.state_x + lidar_x * m.cos(self.state_yaw) - lidar_y * m.sin(self.state_yaw)
            map_y = self.state_y + lidar_x * m.sin(self.state_yaw) + lidar_y * m.cos(self.state_yaw)
            
            dis_path = self.GetDistance(self.path, [map_x, map_y])
            
            self.obstacle.append([self.ranges[158+i], dis_path])
            self.mapping.append([map_x, map_y])
            
            
    def Result(self):
        
        obs_num = 0
        partTF = obTF()
        print(self.obstacle)
        for i in self.obstacle:
            
            if i[0] < self.thr_dis and i[1] < self.thr_path_dis and i[0] != 0.0:
                obs_num += 1
            
        print(obs_num)
        if obs_num >= 5:
            partTF.front_right = 1
            partTF.front_left = 1
            partTF.side_right = 0
            partTF.side_left = 0
            rospy.logfatal("stop!!!!!")
            
        else:
            partTF.front_right = 0
            partTF.front_left = 0
            partTF.side_right = 0
            partTF.side_left = 0
            rospy.logwarn("keep going!!!!!")

    def main(self):
        self.detect_range()
        self.Result()
        self.publishPoint(self.mapping)




if __name__ == "__main__":
    
    rospy.init_node("dynamic")
    dy = Lidar()

    r = rospy.Rate(5.)
    
    while not rospy.is_shutdown():
        dy.main()
        r.sleep()
