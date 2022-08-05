#!/usr/bin/env python2.7

# Python 2/3 compatibility
from __future__ import print_function

PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from pcl import *
import numpy as np
import ros_numpy
from Kdtree import KdTree_class

def callback(data):
    points = []
    for mark in data.markers:
        for j in range(len(mark.points)):
            temp = [mark.points[j].x,mark.points[j].y,mark.points[j].z,mark.id]
            points.append(temp)
    dividecircle(points)

def dividecircle(array):
    cluster_size_min = 3 #need to make param
    region_max_ = 10
    regions_ = [None]*100
    regions_[0] = 2; regions_[1] = 3; regions_[2] = 3; regions_[3] = 3; regions_[4] = 3;
    regions_[5] = 3; regions_[6] = 3; regions_[7] = 2; regions_[8] = 3; regions_[9] = 3;
    regions_[10]= 3; regions_[11]= 3; regions_[12]= 3; regions_[13]= 3;
    regions_array = [None]* region_max_

    for i in range(len(array)):
        range = 0.0
        for j in range(region_max_):
            d2 = array[i][0]**2+array[i][1]**2 + array[i][2]**2
            if d2>range**2 and d2 <=(range+regions_[j])**2 :
                regions_array[j].append(array[i])
                break
            range += regions_[j]

    do_euclidean_clustering(regions_array)


def do_euclidean_clustering(white_cloud):
    tolerence = 0.0
    for i in range(10):
        tolerance += 0.1
        if len(white_cloud[i])>3:
            tree = white_cloud[i].make_kdtree()

            # Create Cluster-Mask Point Cloud to visualize each cluster separately
            ec = white_cloud[i].make_EuclideanClusterExtraction()
            ec.set_ClusterTolerance(tolerence)
            ec.set_MinClusterSize(3)
            ec.set_MaxClusterSize(100000) # cluster_size
            ec.set_SearchMethod(tree)
            cluster_indices = ec.Extract()
            print(cluster_indices)


# Euclidean Clustering





if __name__ == '__main__':
    rospy.init_node('adaptive_py', anonymous=True)
    
    array_sub = rospy.Subscriber("before_adaptive", MarkerArray, callback)
    rospy.spin()

