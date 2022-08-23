#!/usr/bin/env python

import rospy
import numpy as np
import math as m
from geometry_msgs.msg import PoseArray, Pose
from path_plan.msg import PathResponse
from state import State

class LocalPath():
    def __init__(self):
        self.pose = PoseArray()
        self.global_path = PathResponse()
        self.__L = 1.040 
    
    def poseCallback(self, msg):
        self.pose = msg        

    def pathCallback(self, msg):
        self.global_path = msg      

    def calc_target_index(self, state):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + self.__L * np.cos(state.yaw) / 2.0 + 2.0 * m.cos(state.yaw)
        fy = state.y + self.__L * np.sin(state.yaw) / 2.0 + 2.0 * m.sin(state.yaw)

        # Search nearest point index
        dx = [fx - icx for icx in self.global_path.cx]
        dy = [fy - icy for icy in self.global_path.cy]

        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        self.goal_x = dx[target_idx]
        self.goal_y = dx[target_idx]
    
    def cal_range(self, a, b, d):
        try:
            x1 = (-(a*b - self.goal_x - self.goal_y*a) + m.root((a*b - self.goal_x - self.goal_y*a)**2 - (a+1)*(self.goal_x**2 + b**2 - 2*self.goal_y*b + self.goal_y**2 -d**2)))/(a+1)
            x2 = (-(a*b - self.goal_x - self.goal_y*a) - m.root((a*b - self.goal_x - self.goal_y*a)**2 - (a+1)*(self.goal_x**2 + b**2 - 2*self.goal_y*b + self.goal_y**2 -d**2)))/(a+1)

        
        except :
            print(x)
    def generate_trajectory(self, state):
        self.calc_target_index(state)

        current_goal_l = np.polyfit([self.goal_x, state.x], [self.goal_y, state.y], 1)
        vertical_a = (-1)/current_goal_l[0]
        vertical_b = ((self.goal_y + state.y)/2)-vertical_a*((self.goal_x +state.x)/2) 



if __name__ == "__main__":
    r = rospy.Rate(10)

    odom_topic = rospy.get_param(
        "/odometry_path/odom_topic", "/odometry/kalman")
    state = State(odometry_topic=odom_topic)
    path = LocalPath()

    rospy.Subscriber("/adaptive_clustering/poses", PoseArray, path.poseCallback)
    rospy.Subscriber("/path_response", PathResponse, path.pathCallback)

    while not rospy.is_shutdown():
        path.generate_trajectory(state)
        r.sleep()