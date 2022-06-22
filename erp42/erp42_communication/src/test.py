#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
import numpy as np

angle_quat = [0,0,0,0]
def callback(data):
    angle_quat[0] = data.orientation.x
    angle_quat[1] = data.orientation.y
    angle_quat[2] = data.orientation.z
    angle_quat[3] = data.orientation.w

    _,_,yaw = tf.transformations.euler_from_quaternion(angle_quat)
    print(np.rad2deg(yaw))


if __name__ == "__main__":
    rospy.init_node('test')
    imu = rospy.Subscriber("/imu/data",Imu,callback,queue_size=1)
    rospy.spin()