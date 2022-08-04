#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray 

def callback(data):
    d2array = []
    temp = []
    for marker in data.markers:
        d2 = (marker.pose.position.x)*(marker.pose.position.x)+(marker.pose.position.y)*(marker.pose.position.y)
        if (marker.id)%100 == 0 and (marker.id)%10000 >= 10:
            d2array.append(temp)
            temp = []
            temp.append(d2)
        else:
            temp.append(d2)
    d2array.append(temp)
    markerarr = MarkerArray()
    d2s = sum(d2array, [])
    
    for i in range(len(d2array)):
        minl = min(d2array[i])
        num = d2s.index(minl)
        mark = Marker()
        mark = data.markers[num]
        markerarr.markers.append(mark)

    marker_pub.publish(markerarr)

if __name__ == '__main__':
   
    rospy.init_node('nearmarker',anonymous=True)
    rospy.Subscriber("after_clustering/markers",MarkerArray,callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker_pub = rospy.Publisher("nearmarkers",MarkerArray,queue_size = 5)
        rate.sleep()
