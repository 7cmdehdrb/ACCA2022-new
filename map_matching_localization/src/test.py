#!/usr/bin/env python


import rospy
import rospkg
import threading
from header import *


if __name__ == "__main__":
    rospy.init_node("map_matching_localization")

    global_map_topic = rospy.get_param("global_map_topic", "cloud_pcd")
    velodyne_topic = rospy.get_param("velodyne_topic", "velodyne_points")
    odom_topic = rospy.get_param("odom_topic", "odometry/global")

    point_cloud_publisher = rospy.Publisher(
        "align_test", PointCloud2, queue_size=5)

    point_cloud_publisher2 = rospy.Publisher(
        "sliced_map", PointCloud2, queue_size=5)

    global_map = PointCloud(pcd_topic=global_map_topic)
    velodyne = PointCloud(pcd_topic=velodyne_topic)
    odom = MapMatchingOdometry(odom_topic)
    map_odom_tf = MapOdomTF()

    velodyne.setSlicingPoint(PoseStamped())

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            slicing_point = odom.getOdomOnMap()

            global_map.setSlicingPoint(slicing_point)
            global_map.slicePointCloud()
            velodyne.slicePointCloud()

            # base_link > map 4x4
            tf_matrix = doMapMatching(
                target=global_map.data, source=velodyne.data)

            transformed_velodyne = doTransform(
                source=velodyne.data, tf_matrix=tf_matrix)

            print(tf_matrix)
            # new_tf = map_odom_tf.createNewTF(tf_matrix)

            data = array_to_xyz_pointcloud2f(
                cloud_arr=transformed_velodyne, stamp=rospy.Time.now(), frame_id="map")

            # data2 = array_to_xyz_pointcloud2f(
            #     cloud_arr=global_map.sliced_data, stamp=rospy.Time.now(), frame_id="map")

            point_cloud_publisher.publish(data)
            # point_cloud_publisher2.publish(data2)

        except Exception as ex:
            rospy.logwarn(ex)
            print("MAIN EXCEPTION")

        finally:
            # rate.sleep()
            pass
