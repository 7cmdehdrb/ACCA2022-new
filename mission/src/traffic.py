#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
from mission.msg import BoundingBox, BoundingBoxes
from lidar_camera_calibration.msg import Signmsg
from collections import deque
    
class Traffic():

    def __init__(self):
        self.light = deque([0 for i in range(10)], maxlen=10)
        self.light_class = ["1301","1302","1401","1402","1404","1303","1305","1403","1300","1400","1406","1405"]
        self.stop_0 = ["1301","1302","1401","1402","1404"]
        self.left_1 = ["1303","1305","1403"]
        self.straight_2 = ["1300","1400","1406"]
        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes,self.callback)

    def callback(self, msg): # select near sign
        temp_class = []
        temp_size = []
        temp_where = []
        self.boxclass = []
        self.boxwhere = []
        
        for box in msg.bounding_boxes :
            if box.Class in self.light_class:
                temp_size.append(box.ymax-box.ymin)
                temp_where.append(box.xmin)
                temp_class.append(box.Class)
                
        if len(temp_size) > 0 :
            near_sign = max(temp_size)
            for i in range(len(temp_size)):
                if temp_size[i] >= near_sign - 10 : # number is not perfect
                    self.boxclass.append(temp_class[i])
                    self.boxwhere.append(temp_where[i])


    def choosenum(self,num): #num type = string
        if num in self.stop_0:
            self.light.append = 0
        elif num in self.left_1:
            self.light.append = 1
        elif num in self.straight_2:
            self.light.append = 2
        else:
            self.light.append = 3


    def calculate(self,class_list,where_list):

        class_set = set(class_list)

        if len(class_set) == 0:
            self.light.append(self.light[-1])
        
        elif len(class_set) == 1 :
            self.choosenum(class_set[0])
        
        elif len(class_set) == 2:
            for i in range(len(class_list)):
                if class_list[i] != class_list[0]:
                    # if class_list[i]%100 == class_list[0]%100:
                    #     self.choosenum(1400+class_list[i]%100)
                    if where_list[0] > where_list[i]: 
                        self.choosenum(class_list[0])
                    else:
                        self.choosenum(class_list[i])
        
        else:
            self.light.append(self.light[-1])
    
    def main(self):
        #custom msg
        self.calculate(self.boxclass,self.boxwhere)
        
        self.msg = Signmsg()
        
        result = max(self.light, key=self.light.count)
        
        print(self.light)
        
        if result == 0 :
            self.msg.left = 0
            self.msg.straight = 0
        elif result == 1 :
            self.msg.left = 1
            self.msg.straight = 0
        elif result == 2 :
            self.msg.left =0
            self.msg.straight = 1
        else:
            self.msg.left = 1
            self.msg.straight = 1
