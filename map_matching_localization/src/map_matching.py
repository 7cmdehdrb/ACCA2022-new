#!/usr/bin/env python


import rospy
import rospkg
import threading
from header import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


# initial_pose = None


def initialPoseCallback(msg):
    # global initial_pose
    # initial_pose = msg

    global_map.setSlicingPoint(changePWCS2PS(data=msg))
    global_map.slicePointCloud()


def changePWCS2PS(data):
    new_data = PoseStamped()
    new_data.header.frame_id = data.header.frame_id
    new_data.header.stamp = data.header.stamp

    new_data.pose = data.pose.pose

    return new_data


if __name__ == "__main__":
    rospy.init_node("map_matching_localization")

    global_map_topic = rospy.get_param("global_map_topic", "cloud_pcd")
    velodyne_topic = rospy.get_param("velodyne_topic", "velodyne_points")
    odom_topic = rospy.get_param("odom_topic", "odometry/global")

    point_cloud_publisher = rospy.Publisher(
        "align_test", PointCloud2, queue_size=5)

    point_cloud_publisher2 = rospy.Publisher(
        "sliced_map", PointCloud2, queue_size=5)

    pose_publisher = rospy.Publisher(
        "matching_pose", Odometry, queue_size=1
    )

    rospy.Subscriber("initialpose", PoseWithCovarianceStamped,
                     callback=initialPoseCallback)

    global_map = PointCloud(pcd_topic=global_map_topic)
    velodyne = PointCloud(pcd_topic=velodyne_topic)
    odom = MapMatchingOdometry(odom_topic)

    # map_odom_tf = MapOdomTF()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:

            # slicing_point = odom.getOdomOnMap()

            # if initial_pose is not None:
            #     global_map.setSlicingPoint(changePWCS2PS(data=initial_pose))
            #     global_map.slicePointCloud()
            # velodyne.slicePointCloud()

            # base_link > map 4x4
            tf_matrix = doMapMatching(
                target=global_map.sliced_data, source=velodyne.data)

            transformed_velodyne = doTransform(
                source=velodyne.data, tf_matrix=tf_matrix)

            p1 = np.array([0, 0, 0, 1])

            p1p = np.dot(tf_matrix, p1)

            print(p1p)

            pose = Odometry()

            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()

            pose.pose.pose.position.x = p1p[0]
            pose.pose.pose.position.y = p1p[1]
            pose.pose.pose.position.z = p1p[2]

            # print(tf_matrix)

            data = array_to_xyz_pointcloud2f(
                cloud_arr=transformed_velodyne, stamp=rospy.Time.now(), frame_id="map")
            data2 = array_to_xyz_pointcloud2f(
                cloud_arr=global_map.sliced_data, stamp=rospy.Time.now(), frame_id="map")

            point_cloud_publisher.publish(data)
            point_cloud_publisher2.publish(data2)

            pose_publisher.publish(pose)

        except Exception as ex:
            rospy.logwarn(ex)
            print("MAIN EXCEPTION")

        finally:
            # rate.sleep()
            pass
