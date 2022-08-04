#!/usr/bin/env python2.7


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

# Global variables
OUSTER_LIDAR = False
PAUSE = False
FIRST_TIME = True
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()


def project_point_cloud(velodyne, img_msg, box_data, image_pub, points_pub):
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
    # Group all beams together and pick the first 4 columns for X, Y, Z, intensity.
    if OUSTER_LIDAR: points3D = points3D.reshape(-1, 9)[:, :4]
    
    # Filter points in front of camera
    inrange = np.where((points3D[:, 2] > 0) &
                       (points3D[:, 2] < 6) &
                       (np.abs(points3D[:, 0]) < 6) &
                       (np.abs(points3D[:, 1]) < 6))
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
    points3D = points3D[inrange[0]]
    points_inbox = select_points_in_box(points2D, box_data)
    
    box_3D = np.empty((0,4),float)
    for i in range(len(points_inbox)):
        box_3D = np.append(box_3D,points3D[points_inbox[i]],axis = 0)
    

    box_3D_F = np.zeros(len(box_3D),dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity','f4')])

    box_3D_F['x'] = box_3D[:, 0]

    box_3D_F['y'] = box_3D[:, 1]
    box_3D_F['z'] = box_3D[:, 2]
    box_3D_F['intensity'] = box_3D[:, 3]

    box_3D_F = ros_numpy.point_cloud2.array_to_pointcloud2(box_3D_F, frame_id = 'world')
    points_pub.publish(box_3D_F)
    
    
    # final_array = ClusterArray()
    # for i in range(len(points_inbox)):
    #     tempbox = points3D[points_inbox[i]]
    #     tempbox = ros_numpy.array_to_pointcloud2(tempbox, frame_id = "velodyne")
    #     final_array.clusters.append(tempbox)
    
    # points_pub.publish(final_array)
        
    
    
    # beforeset = []
    # for i in range(len(box_data)):
    #     temp_box = np.where((points2D[:, 0] >= box_data[i][0]) &
    #                    (points2D[:, 1] >= box_data[i][1]) &
    #                    (points2D[:, 0] < box_data[i][2]) &
    #                    (points2D[:, 1] < box_data[i][3]))
    #     temp_3D = points3D(temp_box[0])
    #     for j in range(len(box_data)):
    #         box_inrange = np.where((temp_3D[:, 0] >= box_data[j][0]) &
    #                    (temp_3D[:, 1] >= box_data[j][1]) &
    #                    (temp_3D[:, 2] >= box_data[j][2]) &
    #                    (temp_3D[:, 0] < box_data[j][3]) &
    #                    (temp_3D[:, 1] < box_data[j][4]) &
    #                    (temp_3D[:, 2] < box_data[j][5]))
            
    # before adaptive_clustering

    # print(xyzrgb_array_to_pointcloud2(points_inbox,points3D))

    


    # markerarray = MarkerArray()
    # for i in range(len(points_inbox)):

    #     rviz_points = Marker()
    #     rviz_points.header.frame_id = "velodyne"
    #     rviz_points.ns = "points"
    #     rviz_points.id = i+1

    #     rviz_points.type = 8
    #     rviz_points.action = 0

    #     rviz_points.color = ColorRGBA(i/(len(points_inbox)+1),1,1,1)
    #     rviz_points.scale.x = 0.01
    #     rviz_points.scale.y = 0.01
    #     rviz_points.scale.z = 0
    #     for j in range(len(points3D[points_inbox[i]])):
    #         temp = points3D[points_inbox[i]]
    #         rviz_points.points.append(Point(temp[j][0],temp[j][1],temp[j][2]))
        
    #     markerarray.markers.append(rviz_points)
    # points_pub.publish(markerarray)

    # Roi points -> clustering

    

    # Draw the projected 2D points
    for i in range(len(points2D[points_inbox[0]])):
        cv2.circle(img, tuple(points2D[points_inbox[0]][i]), 2, tuple(colors[i]), -1)

    # Publish the projected points image
    try:
        image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e: 
        rospy.logerr(e)
    

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
    for i in range(len(box_info)):
        temp_arr = np.where((xy_arr[:, 0] >= box_info[i][0]) &
                       (xy_arr[:, 1] >= box_info[i][1]) &
                       (xy_arr[:, 0] < box_info[i][2]) &
                       (xy_arr[:, 1] < box_info[i][3]))
        temp_arr = temp_arr[0].tolist()
        box_arr.append(temp_arr)
    return box_arr




# def cluster_callback(data):
#     global cluster_range
#     cluster_range = []
#     for mark in data.markers:
#         # max : [0] / min : [6]
#         cluster_range.append([mark.points[6].x, mark.points[6].y, mark.points[6].z, \
#             mark.points[0].x,mark.points[0].y,mark.points[0].z])

def callback(image, camera_info, velodyne, yolo, image_pub=None, points_pub = None):
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
            temp = [box.xmin,box.ymin,box.xmax,box.ymax]
            box_data.append(temp)
    project_point_cloud(velodyne, image, box_data, image_pub, points_pub)


    
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
    if camera_lidar: 
        image_pub = rospy.Publisher(camera_lidar, Image, queue_size=5)
        points_pub = rospy.Publisher('before_adaptive',PointCloud2,queue_size = 5)
    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub, info_sub, velodyne_sub,yolo_sub], queue_size=5, slop= 1.7e9)
    ats.registerCallback(callback, image_pub,points_pub)


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
