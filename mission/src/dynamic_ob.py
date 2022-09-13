#!/usr/bin/env python

import rospy
import sys
import rospkg
import numpy as np
from geometry_msgs.msg import PoseArray,Pose
from mission.msg  import obTF
import math


"""
Subscribe 'scan_filtered' and
Publish 'ob_TF'
"""


class Lidar(object):
    def __init__(self):
        super(Lidar, self).__init__()
        self.p_arr = np.empty((0,3), float)
        self.threshold_range = rospy.get_param("threshold_range", 5.0)
        self.part_fin = []
        self.part_index = []
        self.fin = []
        rospy.Subscriber("/adaptive_clustering/poses", PoseArray, self.lidarCallback)

        self.part_pub = rospy.Publisher("ob_TF", obTF, queue_size=1)

    def lidarCallback(self, msg):
        p_arr = np.empty((0,3), float)
        for pose in msg.poses:
            d = math.sqrt((pose.position.x)**2+(pose.position.y)**2)
            if (d <= self.threshold_range) and (pose.position.x >= 0.1) and (pose.position.z >= -1.2):
                p_arr = np.append(p_arr, np.array([[pose.position.x,pose.position.y,pose.position.z]]), axis=0)
        self.p_arr = p_arr

    def partYN(self):
        self.part_fin=[0,0,0,0]
        DEG = 15
        RAD = math.radians(DEG)
        for i in range(len(self.p_arr)):
            temp_rad = math.atan(self.p_arr[i][1]/self.p_arr[i][0])
            
            if temp_rad >= RAD :
                self.part_fin[0] = 1
            elif temp_rad >= 0 and temp_rad < RAD :
                self.part_fin[1] = 1
            elif temp_rad <0 and temp_rad >= (-1)*RAD :
                self.part_fin[2] = 1
            elif temp_rad < (-1)*RAD :
                self.part_fin[3] = 1

    def xylistMake(self):
        if len(self.part_index) < 5:
            self.part_index.append(self.part_fin)

        else:
            print("error")

    def thresholding(self):
        self.fin = []
        A = 0
        B = 0
        C = 0
        D = 0
        a = 0
        b = 0
        c = 0
        d = 0
        for i in range(len(self.part_index)):
            A = A + self.part_index[i][0]
            B = B + self.part_index[i][1]
            C = C + self.part_index[i][2]
            D = D + self.part_index[i][3]
        if A > 2:
            a = 1
        if B > 2:
            b = 1
        if C > 2:
            c = 1
        if D > 2:
            d = 1

        self.fin = [a, b, c, d]

    def pubResults(self):
        self.partTF = obTF()

        try:
            self.partTF.side_left = self.fin[0]
            self.partTF.front_left = self.fin[1]
            self.partTF.front_right = self.fin[2]
            self.partTF.side_right = self.fin[3]

            self.part_pub.publish(self.partTF)

        except Exception as ex:
            print(ex)

    def main(self):
        self.partYN()
        self.xylistMake()
        if len(self.part_index) == 5:
            self.thresholding()
            self.pubResults()
            # print(self.fin)
            del self.part_index[0]


if __name__ == "__main__":
    rospy.init_node("dynamic_ob")
    
    lidar = Lidar()

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        lidar.main()
        r.sleep()
        
# code from : https://github.com/7cmdehdrb/ACCA/blob/master/cone_tracker/src/check_obstacles.py
# after : check https://github.com/7cmdehdrb/ACCA/blob/master/cone_tracker/src/estopTF.py
# after2 : check https://github.com/7cmdehdrb/ACCA/blob/master/path_planner/msg/obTF.msg