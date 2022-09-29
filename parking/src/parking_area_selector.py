#!/usr/bin/env python


import rospy
import numpy as np
import math as m
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import *
from parking_area import ParkingArea


class ParkingAreaSelector():
    def __init__(self, state, parking_areas=[]):
        self.obstacles = []
        self.obstacle_zone = []

        self.flag = False
        self.target_idx = 0
        self.state = state
        self.parking_areas = parking_areas
        self.parking_possibilities = [
            None for _ in range(len(self.parking_areas))]

        self.tf_sub = tf.TransformListener()
        self.obstacle_sub = rospy.Subscriber("/adaptive_clustering/poses", PoseArray,
                                             callback=self.obstacleCallback)

    def obstacleCallback(self, msg):
        self.obstacle_zone = []

        if self.tf_sub.canTransform("map", "velodyne", rospy.Time(0)):
            for p in msg.poses:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time(0)

                pose.pose = p

                temp = self.tf_sub.transformPose(ps=pose, target_frame="map")
                self.obstacles.append(
                    [temp.pose.position.x, temp.pose.position.y])

        self.loop()

    def isPossibleParkingArea(self, obstacle, box):
        dist = np.hypot(box.position.x - obstacle[0],
                        box.position.y - obstacle[1])
        box_yaw = euler_from_quaternion(
            [box.orientation.x, box.orientation.y, box.orientation.z, box.orientation.w])

        if dist <= box.scale.x / 2.0:
            return True

        area_VEC = np.array([
            m.cos(box_yaw - m.radians(90.0)), m.sin(box_yaw - m.radians(90.0))
        ])

        ob_VEC = np.array([
            obstacle.x - box.x, obstacle.y - box.y
        ])

        theta = m.acos(np.dot(area_VEC, ob_VEC) / dist)

        x_dist = abs(dist * m.sin(theta))
        y_dist = abs(dist * m.cos(theta))

        if x_dist <= box.scale.x / 2.0 and y_dist <= box.scale.y / 2.0:
            return True

        return False

    def loop(self):
        p_orientation = self.parking_areas[self.target_idx].orientation
        pyaw = euler_from_quaternion(
            [p_orientation.x, p_orientation.y, p_orientation.z, p_orientation.w])

        pyaw_VEC = [m.cos(pyaw), m.sin(pyaw)]
        car_VEC = [self.state.x - self.parking_areas[self.target_idx].x,
                   self.state.y - self.parking_areas[self.target_idx].y]

        inner_product = np.dot(pyaw_VEC, car_VEC)
        if inner_product < 0.:
            if self.parking_areas[self.target_idx] is True:
                # Parking : Possible
                self.flag = True
                self.obstacle_sub.unregister()
                return 0

            else:
                self.target_idx += 1
                if len(self.parking_areas) < self.target_idx:
                    self.target_idx = len(self.parking_areas) - 1
                    rospy.logfatal(
                        "Cannot try parking into every parking areas!!!!")

        else:
            for obstacle in self.obstacles:
                if self.isPossibleParkingArea(
                        obstacle, self.parking_areas[self.target_idx]) is True:
                    # Cannot park.

                    self.parking_possibilities[self.target_idx] = False
                    self.target_idx += 1
                    break

            self.parking_possibilities[self.target_idx] = True

        self.obstacles = []
