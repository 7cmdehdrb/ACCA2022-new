#!/usr/bin/env python2.7
# Python 2/3 compatibility
from __future__ import print_function
PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import pcl
import numpy as np
from Kdtree import KdTree_class

def callback(data):
    points = []
    for mark in data.markers:
        for j in range(len(mark.points)):
            temp = [mark.points[j].x,mark.points[j].y,mark.points[j].z,mark.id]
            points.append(temp)
    
    tree = KdTree_class()
    root_node = tree.insert_points(points, display_output=False)
    print(tree.search_elements(root_node, (3.12998489, -4.83957614, -6.24208885), 0.5))


if __name__ == '__main__':
    rospy.init_node('adaptive_py', anonymous=True)
    array_sub = rospy.Subscriber("before_adaptive", MarkerArray, callback)
    rospy.spin()

