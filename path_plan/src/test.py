#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import UInt8
from DB import *


if __name__ == "__main__":
    rospy.init_node("test")

    db = DB()

    while not rospy.is_shutdown():
        path_id = 'A1B1'
        db.deletePath(path_id)
        break
