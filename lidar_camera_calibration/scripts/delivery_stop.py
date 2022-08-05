#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from visualization_msgs.msg import Marker,MarkerArray
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
import Queue

class Delivery(object):
    def __init__(self) :
        self.Class = None
        self.panel = {"A1":4,"A2":5,"A3":6}
        self.line = [0,0] #[a,b] : y=ax+b
        self.x = 0
        self.y = 0
        self.num = 0
    def tfcallback(self,msg):
        if self.Class :
            for marker in msg.markers:
                if (marker.id)//10000 == self.panel[self.Class] :
                    self.x += marker.pose.position.x
                    self.y += marker.pose.position.y
                    self.num += 1
                if self.num == 20:
                    self.paneltopath()
    
    def paneltopath(self):
        panel_x = self.x/20
        panel_y = self.y/20
        

    def yolocallback(self,msg):
        if msg:
            self.class_num = msg.bounding_boxes.Class
            
    def calculate(self):
        
if __name__ == '__main__':
    
    rospy.init_node('delivery_stop',anonymous=True)
    
    deli = Delivery()
    
    # calculation : use np.polyfit > result : [a,b] ; y = ax + b
    
    rospy.Subscriber("after_clustering/tf_markers",MarkerArray,deli.callback)
    rospy.Subscriber("idontknow",BoundingBoxes,deli.yolocallback)
    
