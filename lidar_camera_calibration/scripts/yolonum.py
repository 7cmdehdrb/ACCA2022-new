#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-

# Python 2/3 compatibility
from __future__ import print_function

# Built-in modules
import os
import sys
import time
import threading


# External modules
import cv2
import numpy as np
import matplotlib.cm
import random

# ROS modules
PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf2_ros
import ros_numpy
import image_geometry
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_matrix
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from lidar_camera_calibration.msg import BoxregionArray, Boxregion

# Global variables
OUSTER_LIDAR = False
PAUSE = False
FIRST_TIME = True
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()


def project_point_cloud(velodyne, img_msg, box_data, image_pub, points_pub, regions_pub):
    # Read image using CV bridge
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
    except CvBridgeError as e: 
        rospy.logerr(e)
        return

    # Transform the point cloud
    try:
        transform = TF_BUFFER.lookup_transform('world', 'velodyne', rospy.Time())
        velodyne = do_transform_cloud(velodyne, transform)
        
    except tf2_ros.LookupException:
        pass

    # Extract points from message
    points3D = ros_numpy.point_cloud2.pointcloud2_to_array(velodyne)
    points3D = np.asarray(points3D.tolist())
    # Filter points in front of camera
    inrange = np.where((points3D[:, 2] > 0) &
                       (points3D[:, 2] < 20) &
                       (points3D[:, 0] < 10) & (points3D[:,0]>-10)&
                       (points3D[:, 1] < 20) & (points3D[:,1]>-3))
    max_intensity = np.max(points3D[:, -1])
    points3D = points3D[inrange[0]]
    # Color map for the points
    cmap = matplotlib.cm.get_cmap('jet')
    colors = cmap(points3D[:, -1] / max_intensity) * 255

    # Project to 2D and filter points within image boundaries
    points2D = [ CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3] ]
    points2D = np.asarray(points2D)
    inrange = np.where((points2D[:, 0] >= 0) &
                       (points2D[:, 1] >= 0) &
                       (points2D[:, 0] < img.shape[1]) &
                       (points2D[:, 1] < img.shape[0]))
    points2D = points2D[inrange[0]].round().astype('int')


    for i in range(len(points2D)):
        cv2.circle(img, tuple(points2D[i]), 2, tuple(colors[i]), -1)

    # Publish the projected points image
    try:
        image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e: 
        rospy.logerr(e)
    
    
    
    
    
    
    points3D = points3D[inrange[0]]
    points_inbox,boxregionarray= select_points_in_box(points2D, box_data)
    
    regions_pub.publish(boxregionarray)
    
    box_3D = np.empty((0,4),float)
    # image_dots = np.empty((0,4),float)
    
    for i in range(len(points_inbox)):
        # image_dots = np.append(image_dots,points2D[points_inbox[i]],axis = 0)
        box_3D = np.append(box_3D,points3D[points_inbox[i]],axis = 0)


    box_3D_F = np.zeros(len(box_3D),dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity','f4')])

    box_3D_F['x'] = box_3D[:, 0]

    box_3D_F['y'] = box_3D[:, 1]
    box_3D_F['z'] = box_3D[:, 2]
    box_3D_F['intensity'] = box_3D[:, 3]

    box_3D_F = ros_numpy.point_cloud2.array_to_pointcloud2(box_3D_F, frame_id = 'world',stamp = rospy.Time())
    points_pub.publish(box_3D_F)
    
    # box_regions
    boxregionarray.header.stamp = rospy.Time()
    boxregionarray.header.frame_id = 'world'
    
    

'''
Callback function to publish project image and run calibration

Inputs:
    image - [sensor_msgs/Image] - ROS sensor image message
    camera_info - [sensor_msgs/CameraInfo] - ROS sensor camera info message
    velodyne - [sensor_msgs/PointCloud2] - ROS velodyne PCL2 message
    image_pub - [sensor_msgs/Image] - ROS image publisher

Outputs: None
'''




def select_points_in_box(xy_arr, box_info):

    box_arr = []
    boxregionarray = BoxregionArray()
    for i in range(len(box_info)):
        temp_arr = np.where((xy_arr[:, 0] >= box_info[i][0]) &
                       (xy_arr[:, 1] >= box_info[i][1]) &
                       (xy_arr[:, 0] < box_info[i][2]) &
                       (xy_arr[:, 1] < box_info[i][3]))
        temp_arr = temp_arr[0].tolist()
        box_arr.append(temp_arr)
        
        if box_info[i][4] == 'cone_B':
            whatinbox = 2
        elif box_info[i][4] == 'cone_Y':
            whatinbox = 3
        else:
            whatinbox = 1
    
        # publish box_regions
        box_region = Boxregion()
        box_region.num = whatinbox
        box_region.magnitude = len(temp_arr)
        boxregionarray.regions.append(box_region)
        
    return box_arr, boxregionarray




# def cluster_callback(data):
#     global cluster_range
#     cluster_range = []
#     for mark in data.markers:
#         # max : [0] / min : [6]
#         cluster_range.append([mark.points[6].x, mark.points[6].y, mark.points[6].z, \
#             mark.points[0].x,mark.points[0].y,mark.points[0].z])

def callback(image, camera_info, velodyne, yolo, image_pub=None, points_pub = None, regions_pub = None):
    global CAMERA_MODEL, FIRST_TIME, PAUSE, TF_BUFFER, TF_LISTENER

    # Setup the pinhole camera model
    if FIRST_TIME:
        FIRST_TIME = False

        # Setup camera model
        rospy.loginfo('Setting up camera model')
        CAMERA_MODEL.fromCameraInfo(camera_info)

        # TF listener
        rospy.loginfo('Setting up static transform listener')
        TF_BUFFER = tf2_ros.Buffer()
        TF_LISTENER = tf2_ros.TransformListener(TF_BUFFER)

    box_data = []
    for box in yolo.bounding_boxes :
        if box.probability >= 0.2 :
            temp = [box.xmin,box.ymin,box.xmax,box.ymax,box.Class]
            box_data.append(temp)
    # print(box_data)
    project_point_cloud(velodyne, image, box_data, image_pub, points_pub, regions_pub)


    
'''
The main ROS node which handles the topics

Inputs:
    camera_info - [str] - ROS sensor camera info topic
    image_color - [str] - ROS sensor image topic
    velodyne - [str] - ROS velodyne PCL2 topic
    camera_lidar - [str] - ROS projected points image topic

Outputs: None
'''
def listener(camera_info, image_color, velodyne_points,yolo_box,camera_lidar=None):
    # Start node
    rospy.init_node('calibrate_camera_lidar', anonymous=True)

    # Subscribe to topics
    info_sub = message_filters.Subscriber(camera_info, CameraInfo)
    image_sub = message_filters.Subscriber(image_color, Image)
    velodyne_sub = message_filters.Subscriber(velodyne_points, PointCloud2)
    yolo_sub = message_filters.Subscriber(yolo_box, BoundingBoxes)
    
    # Publish output topic
    image_pub = None
    points_pub = None
    regions_pub = None
    if camera_lidar: 
        image_pub = rospy.Publisher(camera_lidar, Image, queue_size=5)
        points_pub = rospy.Publisher('before_adaptive',PointCloud2,queue_size = 5)
        regions_pub = rospy.Publisher('box_regions', BoxregionArray, queue_size = 5)
    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub, info_sub, velodyne_sub,yolo_sub], queue_size=5, slop= 1.7e9)
    ats.registerCallback(callback, image_pub,points_pub,regions_pub)


    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    
    camera_info = rospy.get_param('camera_info_topic')
    image_color = rospy.get_param('image_color_topic')
    velodyne_points = rospy.get_param('velodyne_points_topic')
    camera_lidar = rospy.get_param('camera_lidar_topic')
    yolo_box = rospy.get_param('yolo_box_topic')
    
    # Start subscriber
    listener(camera_info, image_color, velodyne_points, yolo_box, camera_lidar)
