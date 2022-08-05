#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf_conversions
import numpy as np
import pandas as pd
ref = pd.DataFrame(data=None, columns={"x","y"})

def callback(data):
    x=data.poses.pose.position.x
    y=data.poses.pose.position.y
    ref.loc[len(ref)]=[x,y]
    ref.to_csv("/home/taehokim/csv_file/gnss_state.csv", index=False)
    
if __name__ == "__main__":
    rospy.init_node("test")
    
    sub_imu = rospy.Subscriber("/path_gnss",Path,callback,queue_size=1)
    
    rospy.spin()