#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
import Queue
from lidar_camera_calibration.msg import Signmsg

class SignSearch(object):
    def __init__(self):
        self.sign =[0 for i in range(9)]
        self.final_index = None
        self.stop_0 = ["1301","1302","1401","1402","1404"]
        self.left_1 = ["1303","1305","1403"]
        self.straight_2 = ["1300","1400","1406"]
        self.both_3 = ["1405"]


    def callback(self, msg):
        self.sign.append(0)

        temp_class = []
        temp_size = []
        temp_where = []
        boxclass = []
        boxwhere = []
        for box in msg.bounding_boxes :
            if box.Class in ["0000","1301","1302","1401","1402","1404","1303","1305","1403","1300","1400","1406","1405"] :
                temp_size.append(box.ymax-box.ymin)
                temp_where.append(box.xmin)
                temp_class.append(box.Class)
        if len(temp_size) > 0 :
            near_sign = max(temp_size)
            for i in range(len(temp_size)):
                if temp_size[i] >= near_sign -10 : # number is not perfect
                    boxclass.append(temp_class[i])
                    boxwhere.append(temp_where[i])

        # print(len(temp_size))

        self.calculate(boxclass,boxwhere)



    def choosenum(self,num): #num type = string
        if num in self.stop_0:
            self.sign[-1] = 0
        elif num in self.left_1:
            self.sign[-1] = 1
        elif num in self.straight_2:
            self.sign[-1] = 2
        else:
            self.sign[-1] = 3




    def calculate(self,class_list,where_list):

        class_set = set(class_list)

        if len(class_set) == 0:
            self.sign[-1] = self.sign[-2]
        elif len(class_set) == 1 :
            if "0000" in class_set:
                self.sign[-1] = self.sign[-2]
            else:
                for a in class_set:
                    self.choosenum(a)

    
        elif len(class_set) == 2:
            if "0000" in class_set:
                for a in class_set:
                    if a != "0000":
                        self.choosenum(a)
            else:
                for i in range(len(class_list)):
                    if class_list[i] != class_list[0]:
                        if class_list[i]%100 == class_list[0]%100:
                            self.choosenum(1400+class_list[i]%100)
                        else:
                            if where_list[0] > where_list[i]:
                                self.choosenum(class_list[0])
                            else:
                                self.choosenum(class_list[i])
                    else:
                        pass
        else:
            if "0000" in class_set :
                temp_max = 0
                for i in range(len(class_list)):
                    if where_list[i]> temp_max:
                        temp_max = where_list[i]
                for i in range(len(where_list)):
                    if where_list[i] == temp_max:
                        self.choosenum(class_list[i])
            else:
                self.sign[-1] == self.sign[-2]
            
        self.publishresult()
    
    def publishresult(self):
        
        
        sign = Signmsg()
        
        #custom msg
        
        
        result = max(self.sign, key=self.sign.count)
        
        print(self.sign)
        
        if result == 0 :
            sign.left = 0
            sign.straight = 0
        elif result == 1 :
            sign.left = 1
            sign.straight = 0
        elif result == 2 :
            sign.left =0
            sign.straight = 1
        else:
            sign.left = 1
            sign.straight = 1
        
        
        
        sign_pub.publish(sign)

        del self.sign[0]
        


if __name__ == '__main__':
    
    rospy.init_node('sign_search',anonymous=True)
    
    sign = SignSearch()
    
    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes,sign.callback)
    
    sign_pub = rospy.Publisher("sign_publish",Signmsg,queue_size = 5)
    
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()