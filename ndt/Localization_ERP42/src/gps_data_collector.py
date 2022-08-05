#! /usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
import pandas as pd


gps = pd.DataFrame(data=None,columns=['latitude','longitude'])

def callback(data):
    global gps
    lat = data.latitude
    lon = data.longitude
    gps.loc[len(gps)]=[lat,lon]
    gps.to_csv("/home/taehokim/csv_file/raw_gps.csv", index=False)


if __name__ == "__main__":
    rospy.init_node("gps_data_collector")
    sub_gnss = rospy.Subscriber("/ublox_gps/fix",NavSatFix, callback)
    rospy.spin()