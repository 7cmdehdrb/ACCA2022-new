#!/usr/bin/env python

import rospy
import math as m
import tf
import pandas as pd
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import MarkerArray


class ParkingDetector(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            "/odometry/global", Odometry, callback=self.OdomCallback)
        self.parked_sub = rospy.Subscriber(
            "/after_clustering/markers", MarkerArray, callback=self.CarCallback)

        self.OdomMsg = Odometry()
        self.CarMsg = MarkerArray()


        self.area1 = (1,1)
        self.area2 = (3,3)
        self.area3 = (5,5)
        self.area4 = (9,9)

    def OdomCallback(self, msg):
        self.OdomMsg = msg

    def CarCallback(self, msg):
        self.CarMsg = msg

    def parkposition(self):

        position_x = self.OdomMsg.pose.pose.position.x
        position_y = self.OdomMsg.pose.pose.position.y
        x = self.OdomMsg.pose.pose.orientation.x
        y = self.OdomMsg.pose.pose.orientation.y
        z = self.OdomMsg.pose.pose.orientation.z
        w = self.OdomMsg.pose.pose.orientation.w
        _, _, Yaw = euler_from_quaternion([x,y,z,w])
        pose_array = []
        for i in self.CarMsg.markers:
            pose = (i.pose.position.x, i.pose.position.y)
            pose_x = position_x + pose[0] * m.cos(Yaw) - pose[1] * m.sin(Yaw)
            pose_y = position_y + pose[0] * m.sin(Yaw) + pose[1] * m.cos(Yaw) # Map frame position

            dis1 = (self.area1[0] - pose_x) ** 2 + (self.area1[1] - pose_y)
            dis2 = (self.area2[0] - pose_x) ** 2 + (self.area2[1] - pose_y)
            dis3 = (self.area3[0] - pose_x) ** 2 + (self.area3[1] - pose_y)
            dis4 = (self.area4[0] - pose_x) ** 2 + (self.area4[1] - pose_y)
            dis_array = [dis1, dis2, dis3, dis4]

            number = dis_array.index(min(dis_array)) + 1
            pose_array.append(number)
        print(pose_array)
        return pose_array

if __name__ == "__main__":
    rospy.init_node("parking")
    parking = ParkingDetector()
    r = rospy.Rate(100.0)
    parking_area = [[1],[2],[3],[4]]
    count = 0
    while True:
        position = parking.parkposition()
        parking_area.append(position)
        r.sleep()
        count += 1
        if count == 500:
            break
    parking_area_ = sum(parking_area, [])
    num = min(parking_area_, key=parking_area_.count)
    print(num)