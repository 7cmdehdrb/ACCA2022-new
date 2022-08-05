#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
import Queue
from lidar_camera_calibration.msg import Signmsg

class SignSearch(object):
    def __init__(self):
        self.sign =[0 for i in range(10)]
        self.count = 0
        self.final_index = None
        
    def callback(self, msg):
        sign_id = [4,5,6,7,8,9,10,11] # need to check
        
        for i in range(10):
            self.sign[i-1] = self.sign[i]
            
        temp = []
        for box in msg.bounding_boxes :
            if box.id in sign_id :
                temp.append(box.id)
        temp_set = set(temp)
        
        
        
        if len(temp_set) == 0:
            self.sign[-1] = 0
        
        elif len(temp_set) == 1:
            if 11 in temp_set : # flicker number = 11
                self.sign[-1] = self.sign[-2]
            else: 
                for a in temp_set :
                    self.sign[-1] = a
            
        elif len(temp_set) == 2:
            if 11 in temp_set:
                for a in temp_set:
                    if a == 11 :
                        pass
                    else: 
                        self.sign[-1] = a   

            else: 
                result = []
                max_num = max(temp)
                for idx, val in enumerate(temp):
                    if val == max_score_val:
                        result.append(idx + 1)
                if len(result) > 1 :
                    self.sign[-1] = self.sign[-2]
                else:
                    self.sign[-1] = result[0]
        else:
            self.sign[-1] = self.sign[-2]
            
            
        self.publishresult()
    
    def publishresult(self):
        
        
        sign = Signmsg()
        
        #custom msg
        
        
        result = max(data, key=self.sign.count)
        
        
        if result in [4,5,6] : # left
            sign.left = 1
            sign.straight = 0
        elif result in [7] : # straight
            sign.left = 0
            sign.straight = 1
        elif result in [8,9] :
            sign.left =1
            sign.straight = 1
        else:
            sign.left =0
            sign.straight = 0
        
        
        sign_pub.publish(sign)
        
            


if __name__ == '__main__':
    
    rospy.init_node('sign_search',anonymous=True)
    
    sign = SignSearch()
    
    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes,sign.callback)
    
    sign_pub = rospy.Publisher("sign_publish",Signmsg,queue_size = 5)
    
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        r.sleep()



