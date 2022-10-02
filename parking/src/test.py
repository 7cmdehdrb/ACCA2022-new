from turtle import position
import numpy as np
import rospy
from geometry_msgs.msg import *


if __name__ == "__main__":
    rospy.init_node("test")
    pub = rospy.Publisher("/test", PoseArray, queue_size=1)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = PoseArray()
        msg.header.frame_id = 'map'
        pose = Pose()
        pose.position.x = 22.0
        pose.position.y = 38.0
        msg.poses.append(pose)
        pub.publish(msg)
        r.sleep()