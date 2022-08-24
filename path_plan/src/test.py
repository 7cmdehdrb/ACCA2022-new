#!/usr/bin/env python

import rospy
import time
import csv
from time import sleep
from DB import *
from path_plan.msg import PathRequest, PathResponse
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

db_name = rospy.get_param("/LoadPath/db_name", "/path.db")

if __name__ == "__main__":
    rospy.init_node("Test")
    db = DB(db_name)
    path = []
    path_info1 = db.bring_pathinfo('B1C1')
    path_info2 = db.bring_pathinfo('C1D1')
    for info in path_info1:
        path.append(info)
    for info in path_info2:
        path.append(info)

    print(len(path_info2))

    print(type(path))
    print(len(path))
