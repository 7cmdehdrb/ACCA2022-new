#!/usr/bin/env python

import rospy
import rospkg
import sys
import numpy as np
import math as m
import tf
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker
from path_plan.msg import PathResponse
from cubic_spline_planner import *

try:
    sys.path.append(rospkg.RosPack().get_path("erp42_control") + "/src")
    from state import State, OdomState
except Exception as ex:
    rospy.logfatal("Import Error : Vertical Parking")
    rospy.logfatal(ex)

class LocalPath():
    def __init__(self, state):
        self.markers = MarkerArray()
        self.global_path = PathResponse()
        self.goal_point = PointStamped()
        self.__L = 1.040 
        self.state = state
        
    def poseCallback(self, msg):
        self.markers = msg
        
    def pathCallback(self, msg):
        self.global_path = msg      

    def calc_target_point(self):
        point_stamped = PointStamped()
        point = Point()
        tf_sub = tf.TransformListener()

        # Calc front axle position
        fx = self.state.x + self.__L * np.cos(self.state.yaw) / 2.0 + 2.0 * m.cos(self.state.yaw)
        fy = self.state.y + self.__L * np.sin(self.state.yaw) / 2.0 + 2.0 * m.sin(self.state.yaw)

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
        tf_sub.canTransform("")
        if tf_sub.canTransform("base_link", "map", rospy.Time(0)):
            self.goal_point = tf.TransformListener.transformPoint(ps=point_stamped, target_frame="base_link")
        else:
            rospy.logwarn("Cannot lookup transform between map and base_link : local_path.py")
    
    
    def generate_trajectory(self, d_max, step):
        self.traject_ys = []
        self.factors = []
        self.calc_target_point()
        
        current_goal_l = np.polyfit([self.goal_point.point.x, self.state.x], [self.goal_point.point.y, self.state.y], 1)
        
        vertical_a = (-1)/current_goal_l[0]
        vertical_b = ((self.goal_point.point.y+ self.state.y)/2)-vertical_a*((self.goal_point.point.x +self.state.x)/2) 
        
        vertical_y = np.arange((-1)*d_max, d_max, step)
        vertical_x = (vertical_y - vertical_b)/vertical_a
        
        self.traject_xs = np.arrange(self.state.x, self.goal_point.point.x, 0.2)
        
        for i in range(len(vertical_x)):
            factor = np.polyfit([self.state.x, self.goal_point.point.x, vertical_x[i]], [self.state.y, self.goal_point.point.y, vertical_y[i]], 2)
            self.factors.append(self.factors)
            traject = factor[0]*self.traject_xs**2 + factor[1]*self.traject_xs + factor[2]
            self.traject_ys.append(traject)

        rospy.loginfo("generate trajectory")
    def calc_cost1(self, obstacle):
        total_cost = []
        # obstacle position
        ox = obstacle.pose.position.x 
        oy = obstacle.pose.position.y
        
        # Search nearest point index
        dx = [ox - icx for icx in self.traject_xs]
        

        for i in range(len(self.factors)):
            cost_c = np.abs(self.factors[i][0])
            
            dy = [oy - icy for icy in self.traject_ys[i]]
            d = np.hypot(dx, dy)
            cost_o = np.argmin(1/d)

            if self.factors[i][0] < 0:
                cost_m = 0.5
            else:
                cost_m = 0  
            cost = cost_c + cost_m + cost_o
            total_cost.append(cost)
        
        self.traject_idx = total_cost.index(min(total_cost))
        
        rospy.loginfo("calc_cost1")
        
    def calc_cost2(self, obstacle):
        total_cost = [] 
        # obstacle position
        ox = obstacle.pose.position.x 
        oy = obstacle.pose.position.y
        
     
        d_limit = (obstacle.scale.y + self.__L)/2

        for factor in self.factors:
            cost_c = np.abs(factor[0])
            
            d = oy - factor[0]*(ox**2) + factor[1]*ox + factor[2]
            if d > d_limit:
                cost_o = 1/d
            else:
                cost_o = 10 ##

            if factor[0] < 0:
                cost_m = 0.5
            else:
                cost_m = 0
            cost = cost_c + cost_m + cost_o
            total_cost.append(cost)

        self.traject_idx = total_cost.index(min(total_cost))
        

    def pathPublish(self):
        msg = PoseArray()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        temp_poses = []

        # use planner, create cx, cy, cyaw

        cx = []
        cy = []
        cyaw = []

        try:
            cx, cy, cyaw, ck, s = calc_spline_course(
                self.traject_xs, self.traject_ys[self.traject_idx], ds=0.1)
        except Exception as ex:
            pass

        for i in range(0, len(cx)):
            pose = Pose()

            quat = tf.transformations.quaternion_from_euler(0, 0, cyaw[i])

            pose.position.x = cx[i]
            pose.position.y = cy[i]
            pose.position.z = 0.0

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            temp_poses.append(pose)

        msg.poses = temp_poses

        path_pub.publish(msg)

        rospy.loginfo("pub path")

if __name__ == "__main__":
    rospy.init_node("LocalPathPlan")
    r = rospy.Rate(10)

    odom_topic = rospy.get_param(
        "/odometry_path/odom_topic", "/odometry/kalman")
    state = OdomState(odometry_topic=odom_topic)
    path = LocalPath(state)

    path_pub = rospy.Publisher("/local_path", PathResponse, queue_size=1)
    
    rospy.Subscriber("/adaptive_clustering/markers", MarkerArray, path.poseCallback)
    rospy.Subscriber("/path_response", PathResponse, path.pathCallback)


    while not rospy.is_shutdown():
        rospy.wait_for_message("/path_response", PathResponse)
        path.generate_trajectory(2, 0.25)
        for obstacle in path.obstacles:
            path.calc_cost1(obstacle)
            # path.calc_cost2(obstacle)
        path.pathPublish()
        r.sleep()