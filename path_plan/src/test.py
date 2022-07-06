#!/usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node("test")

    r = rospy.Rate(1.)
    while not rospy.is_shutdown():
        r.sleep()
