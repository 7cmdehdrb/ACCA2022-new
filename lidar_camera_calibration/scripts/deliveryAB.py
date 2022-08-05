#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import Queue
from visualization_msgs.msg import Marker, MarkerArray
import time

class Delivery(object):
    def __init__(self):
        self.x = Queue.Queue()
        self.y = Queue.Queue()
        self.tfx = Queue.Queue()
        self.tfy = Queue.Queue()
        self.panel_id = [0,0,0]
        self.total = 0
        self.num = 0
        self.deli_A = False
        self.deli_B = False
        self.result_x = 0
        self.result_y = 0
        self.pause = False
        self.count = 0


    def callback(self, msg):
        
        if self.pause == True and self.deli_B == False:
            self.count += 1
            # print("...")
            if self.count >= 50:
                self.pause = False
            
        elif self.deli_A == False and self.pause == False:
            for marker in msg.markers:
            # if...A1 = 5 / A2 = 6 / A3 = 7
                if (marker.id)//10000 in [5,6,7,8,9,10] :
                    # use IDWM (Inverse Distance Weighting method)
                    self.x.put(marker.pose.position.x)
                    self.y.put(marker.pose.position.y)
                    self.total += 1/(marker.pose.position.x**2+marker.pose.position.y**2)
                    if (marker.id)%100 == 0:
                        self.panel_id[marker.id%3-2] += 1
                    # print(max(self.panel_id))
                else:
                    pass

                
                
        elif self.deli_A == True and self.deli_B == False:
            for marker in msg.markers:
                # if...B1 = 8 / B2 = 9 / B3 = 10
                if (marker.id)//10000 == (self.panel_id.index(max(self.panel_id))+5 or self.panel_id.index(max(self.panel_id))+8) :
                    # use IDWM (Inverse Distance Weighting method)
                    self.x.put(marker.pose.position.x)
                    self.y.put(marker.pose.position.y)
                    self.total += 1/(marker.pose.position.x**2+marker.pose.position.y**2)
                    if (marker.id)%100 == 0:
                        self.num += 1
                        # print(self.num)
                else:
                    pass
            
        else: 
            pass
        
    def tfcallback(self, msg):
        if self.pause == True and self.deli_B == False:
             if self.count >= 50:
                self.pause = False
        
        elif self.deli_A == False:
            for tf_marker in msg.markers:
                if (tf_marker.id)//10000 in [5,6,7,8,9,10] :
                    # use IDWM (Inverse Distance Weighting method)
                    self.tfx.put(tf_marker.pose.position.x)
                    self.tfy.put(tf_marker.pose.position.y)
                
                        
            if max(self.panel_id)>=20:
                self.find_distance()    
                
            
        elif self.deli_A == True and self.deli_B == False:

            for tf_marker in msg.markers:
                if (tf_marker.id)//10000 == (self.panel_id.index(max(self.panel_id))+5 or self.panel_id.index(max(self.panel_id))+8):
                    # use IDWM (Inverse Distance Weighting method)
                    self.tfx.put(tf_marker.pose.position.x)
                    self.tfy.put(tf_marker.pose.position.y)
                    
            if self.num>= 20:
                self.find_distance()
                
        else: 
            pass
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    def find_distance(self):
        while self.x.empty() == False:
            x = self.x.get()
            y = self.y.get()
            tfx = self.tfx.get()
            tfy = self.tfy.get()
            self.result_x += tfx*(1/(x**2+y**2))/self.total
            self.result_y += tfy*(1/(x**2+y**2))/self.total
        
        self.panelpublish()
        
        self.x = Queue.Queue()
        self.y = Queue.Queue()
        self.result_x = 0
        self.result_y = 0
        self.total = 0
        
        if self.deli_A == True:
            self.deli_B = True
        
        self.deli_A = True
        self.tfx = Queue.Queue()
        self.tfy = Queue.Queue()

    def panelpublish(self):
        marker = Marker()
        marker.ns = "deliveryAB"
        
        if self.deli_A == False:
            marker.id = 1+self.panel_id.index(max(self.panel_id))
        else :
            marker.id = 4+self.panel_id.index(max(self.panel_id))

        marker.type = 1
        marker.pose.position.x = self.result_x
        marker.pose.position.y = self.result_y
        marker.pose.position.z = 0.0
        
        marker.pose.orientation.w = 1
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5
        
        marker_pub.publish(marker)
        
        print("publish end.")
        self.pause = True


if __name__ == '__main__':
   
    rospy.init_node('delivery_AB',anonymous=True)
    
    deli = Delivery()

    rospy.Subscriber("after_clustering/markers", MarkerArray,deli.callback)
    rospy.Subscriber("after_clustering/tf_markers", MarkerArray,deli.tfcallback)

    marker_pub = rospy.Publisher("delivery_AB",Marker,queue_size = 5)
    
    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        r.sleep()
        