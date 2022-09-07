#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from time import sleep

def adaptive_obstacle(msg):
    global markers

    temp = []

    for pose in msg.poses:
        new_pose = PoseStamped(
            Header(
                None, rospy.Time(0), msg.header.frame_id
            ),
            pose
        )

        temp.append(new_pose)

    markers = temp


if __name__ == "__main__":
    rospy.init_node('parking_tf')

    tf_listener = tf.TransformListener()

    obstcle_sub = rospy.Subscriber(
        '/adaptive_clustering/poses', PoseArray, callback=adaptive_obstacle)
    obstcle_pub = rospy.Publisher(
        "/obstacle_around_parking_areas", PoseArray, queue_size=1
    )

    sleep(1.)

    """
        Variables
    """

    markers = []

    r = rospy.Rate(hz=10)
    while not rospy.is_shutdown():

        if tf_listener.canTransform(target_frame='map', source_frame='velodyne', time=rospy.Time(0)):

            transformed_poses = PoseArray()
            transformed_poses.header = Header(None, rospy.Time.now(), "map")

            for ps in markers:
                transformed_pose = tf_listener.transformPose(
                    ps=ps, target_frame='map')

                transformed_poses.poses.append(
                    transformed_pose.pose
                )

            obstcle_pub.publish(transformed_poses)

        else:
            rospy.logwarn("Cannot lookup transform between map and velodyne")

        r.sleep()
