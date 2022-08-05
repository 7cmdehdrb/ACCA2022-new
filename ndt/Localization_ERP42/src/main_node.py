#! /usr/bin/env python

from os import path
import numpy as np
from pyparsing import col
import local_cartesian
import math as m
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from erp42_msgs.msg import SerialFeedBack
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from sensor_node import *
from init_publisher import *
from EKF_CTRV import *
from view import Visualization
from nav_msgs.msg import Path
import pandas as pd
import local_cartesian
from autoware_msgs.msg import NDTStat

# ori: degree
# x, y, z : m


if __name__ == '__main__':
    rospy.init_node("main_node")
    rate = rospy.Rate(10)
    variance_x = rospy.get_param('~variance_x',0.1)
    variance_y = rospy.get_param('~variance_y',0.1)
    variance_v = rospy.get_param('~variance_v',0.5)
    variance_yaw = rospy.get_param('~variance_yaw',1.0)
    variance_gyr = rospy.get_param('~variance_gyr',1.0)
    
    gnss = GNSS()
    imu = IMU()
    ndt = NDT()
    ekf = EKF(variance_x=variance_x, variance_y=variance_y, variance_vel=variance_v, variance_yaw=variance_yaw, variance_gyr=variance_gyr) 
    view = Visualization()
    
    sub_gnss = rospy.Subscriber("/ublox_gps/fix",NavSatFix, gnss.callback, queue_size=1)
    sub_imu = rospy.Subscriber("/imu/data",Imu,imu.callback,queue_size=1)
    sub_velocity = rospy.Subscriber("/erp42_feedback", SerialFeedBack, ekf.callback,queue_size=1)
    sub_ndt = rospy.Subscriber("/ndt_stat", NDTStat, ndt.ndt_stat_callback,queue_size=1)
    sub_ndt = rospy.Subscriber("/ndt_pose_local", PoseStamped, ndt.ndt_pose_local_callback,queue_size=1)

    map_generation = True
    if map_generation:
        gps_df = pd.DataFrame(data=None,columns=['latitude','longitude'])
        EKF_df = pd.DataFrame(data=None,columns=['latitude','longitude'])
        ndt_df = pd.DataFrame(data = None, columns=["latitude", 'longitude'])
        ref = pd.DataFrame(data=None, columns={"x","y"})
    
    while imu.ori.all() == 0 or gnss.lla.all() == 0 or ndt.ndt_pose.all()==0:
        print("There is no obtained data from GNSS or IMU")

    gnss.initialization()
    imu.initialization()
    Rot = imu.rotation_matrix()
  
    
    while not rospy.is_shutdown():
        
        ekf.ekf_prediction(imu)
        R = gnss.get_cov()
        ndt_cov = ndt.get_cov()
      
        local_data = gnss.local_Cartesian()
        transformed_local_data = np.matmul(Rot,local_data)
        
        time_thres = 3 #sec
        update = False
        if check_old_ndt(ndt.old_time, time_thres):
            update = True
            ndt_cov = np.eye(2)*[20, 20]
            state = ekf.update(np.array([[transformed_local_data[0]], [transformed_local_data[1]]]),[[ndt.ndt_pose[0]], [ndt.ndt_pose[1]]],R, ndt_cov, update)
        else:          
            state = ekf.update(np.array([[transformed_local_data[0]], [transformed_local_data[1]]]),[[ndt.ndt_pose[0]], [ndt.ndt_pose[1]]],R, ndt_cov, update)
        
        view.Est_publish(state=state, update=False)
        view.GNSS_publish(state = transformed_local_data, update=update, orientation=imu.get_data())

        if ndt.ndt_pose.all() != 0:
            view.NDT_publish(state = ndt.ndt)
        
    
        if map_generation:
            # gps_global = gnss.l2g_Cartesian(local_gnss)
            # ekf_global = gnss.l2g_Cartesian([state[0],state[1]])
            # ndt_global = gnss.l2g_Cartesian([local_ndt_pose[0],local_ndt_pose[1]])        
            # gps_df.loc[len(gps_df)]=[gps_global[0],gps_global[1]]
            # EKF_df.loc[len(EKF_df)]=[ekf_global[0],ekf_global[1]]
            # ndt_df.loc[len(ndt_df)]=[ndt_global[0],ndt_global[1]]
            gps_df.loc[len(gps_df)]=[transformed_local_data[0],transformed_local_data[1]]
            EKF_df.loc[len(EKF_df)]=[state[0,0],state[1,0]]
            ndt_df.loc[len(ndt_df)]=[ndt.ndt_pose[0], ndt.ndt_pose[1]]
            gps_df.to_csv("/home/taehokim/csv_file/gps_local3.csv", index=False)
            EKF_df.to_csv("/home/taehokim/csv_file/ekf_local3.csv", index=False)
            ndt_df.to_csv("/home/taehokim/csv_file/ndt_local3.csv", index=False)
        rate.sleep()
