#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import *
from visualization_msgs.msg import*
from std_msgs.msg import UInt8
from parking_area import ParkingArea
import sys
import rospkg
from load_parking_area import loadCSV

try:
    erp42_control_pkg_path = rospkg.RosPack().get_path("erp42_control") + "/src"
    sys.path.append(erp42_control_pkg_path)
    from state import State, OdomState
    from stanley import Stanley
    
except Exception as ex:
    rospy.logfatal(ex)

class ParkingAreaSelector():
    def __init__(self, state, parking_areas=[]):
        self.obstacles = []
        self.obstacle_zone = []

        self.flag = False
        self.target_idx = 0
        self.state = state
        self.parking_areas = parking_areas
        self.parking_possibilities = [
            0 for _ in range(len(self.parking_areas))]

        self.tf_sub = tf.TransformListener()
        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray,
                                             callback=self.obstacleCallback)
        rospy.Subscriber("/mission_state", UInt8, callback=self.missionCallback)
        
        self.pub = rospy.Publisher("/detect_p", PoseStamped, queue_size=1)
        
        self.mission_state = 0
        
    def missionCallback(self, msg):
        self.mission_state = msg.data
        
    def obstacleCallback(self, msg):
        if self.mission_state == 6:
            self.obstacle_zone = []

            if self.tf_sub.canTransform("map", "velodyne", rospy.Time(0)):
                for p in msg.poses:
                    pose = PoseStamped()
                    pose.header.frame_id = "velodyne"
                    pose.header.stamp = rospy.Time(0)

                    pose.pose = p
                    
                    temp = self.tf_sub.transformPose(ps=pose, target_frame="map")
                    self.obstacles.append(
                        [temp.pose.position.x, temp.pose.position.y])
            else:
                rospy.loginfo("tf error")
                
            self.loop()

    def isPossibleParkingArea(self, obstacle, box):
        _, _, box_yaw = euler_from_quaternion(
            [box.orientation.x, box.orientation.y, box.orientation.z, box.orientation.w])

        # if dist <= box.scale.x / 2.0:
        #     return True
        detect_x, detect_y = self.parkingTomap(box, box_yaw)

        dist = np.hypot(detect_x - obstacle[0],
                        detect_y - obstacle[1])
        
        area_VEC = np.array([
            m.cos(box_yaw - m.radians(90.0)), m.sin(box_yaw - m.radians(90.0))
        ])

        ob_VEC = np.array([
            obstacle[0] - detect_x, obstacle[1] - detect_y
        ])

        theta = m.acos(np.dot(area_VEC, ob_VEC) /np.hypot(ob_VEC[0], ob_VEC[1]))

        x_dist = abs(dist * m.cos(theta))
        y_dist = abs(dist * m.sin(theta))
        
        if (x_dist <= (box.scale.x - 0.8) / 2.0) and (y_dist <= (box.scale.y - 1.2) / 2.0):
            return True

        return False

    def loop(self):
        p_orientation = self.parking_areas[self.target_idx].orientation
        _, _, pyaw = euler_from_quaternion(
            [p_orientation.x, p_orientation.y, p_orientation.z, p_orientation.w])

        print(len(self.parking_areas), self.target_idx)
        
        pyaw_VEC = [m.cos(pyaw), m.sin(pyaw)]
        car_VEC = [self.parking_areas[self.target_idx].position.x - self.state.x,
                   self.parking_areas[self.target_idx].position.y - self.state.y]

        inner_product = np.dot(pyaw_VEC, car_VEC)
        print("inner",inner_product)
        
        if inner_product < 0.:
            if self.parking_possibilities[self.target_idx] < 30:
                # Parking : Possible
                self.flag = True
                self.obstacle_sub.unregister()
                return 0

            else:
                self.target_idx += 1
                if len(self.parking_areas) - 1 < self.target_idx:
                    self.target_idx = len(self.parking_areas) - 1
                    rospy.logfatal(
                        "Cannot try parking into every parking areas!!!!")

                    self.flag = True
                    return 0

        else:
            for obstacle in self.obstacles:
                if self.isPossibleParkingArea(
                        obstacle, self.parking_areas[self.target_idx]) is True:
                    # Cannot park.
                    self.parking_possibilities[self.target_idx] += 1
                    
                    break

                    
        print(self.parking_possibilities)
        self.obstacles = []
    
    def parkingTomap(self, box, box_yaw):
        t = np.array([[m.cos(box_yaw), -m.sin(box_yaw), 0, box.position.x],
                      [m.sin(box_yaw), m.cos(box_yaw), 0, box.position.y],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        p = np.array([[0],[1.35],[0],[1]])
        
        detect_p = np.dot(t, p)
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time(0)
        
        pose.pose.position.x = detect_p[0]
        pose.pose.position.y = detect_p[1]
        pose.pose.position.z = 0
        
        self.pub.publish(pose)
    
        return detect_p[0], detect_p[1]
    
if __name__ == "__main__":

    rospy.init_node("parking_area_selector")

    state = State(odometry_topic="/ndt_matching/ndt_pose", hz=30, test=False)
    
    parkingAreas = loadCSV("/home/acca/catkin_ws/src/ACCA2022-new/parking/parking/festi_parking.csv")
    
    selector = ParkingAreaSelector(state=state, parking_areas=parkingAreas)
    
    rospy.spin()
