#!/usr/bin/env python

import rospy
import numpy as np
import math as m
import tf
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose
from path_plan.msg import PathResponse
from state import State

class LocalPath():
    def __init__(self):
        self.pose = PoseArray()
        self.global_path = PathResponse()
        self.goal_point = PointStamped()
        self.__L = 1.040 
    
    def poseCallback(self, msg):
        self.pose = msg        

    def pathCallback(self, msg):
        self.global_path = msg      

    def calc_target_point(self, state):
        point_stamped = PointStamped()
        point = Point()
        tf_sub = tf.TransformListener()
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
        
        point.x = dx[target_idx]
        point.y = dy[target_idx]
        point.z = 0
        
        point_stamped.header.stamp = rospy.Time(0)
        point_stamped.header.frame_id = "map"
        point_stamped.point = point
        
        if tf_sub.canTransform("base_link", "map", rospy.Time(0)):
            self.goal_point = tf.TransformListener.transformPoint(ps=point_stamped, target_frame="base_link")
        else:
            rospy.logwarn("Cannot lookup transform between map and base_link : local_path.py")
            
    def generate_trajectory(self, state, d_max, step):
        self.calc_target_point(state)
        
        current_goal_l = np.polyfit([self.goal_point.x, state.x], [self.goal_point.y, state.y], 1)
        
        vertical_a = (-1)/current_goal_l[0]
        vertical_b = ((self.goal_point.y+ state.y)/2)-vertical_a*((self.goal_point.x +state.x)/2) 
        
        vertical_y = np.arange((-1)*d_max, d_max, step)
        vertical_x = (vertical_y - vertical_b)/vertical_a
        
        traject_x = np.arrange(state.x, self.goal_point.x, step)
        
        traject_y = []
        for i in range(len(vertical_x)):
            factor = np.polyfit([state.x, self.goal_point.x, vertical_x[i]], [state.y, self.goal_point.y, vertical_y[i]], 2)
            traject = factor[0]*traject_x**2 + factor[1]*traject_x + factor[2]
            traject_y.append(traject)

        return traject_x, traject_y

    def calc_cost(self, traject_x, traject_y):
        
        pass
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
        tf.TransformListener.transformPoint("base_link", "map", )
        r.sleep()