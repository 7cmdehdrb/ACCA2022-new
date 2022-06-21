#!/bin/bash
# Apache License 2.0
# Copyright (c) 2017, ROBOTIS CO., LTD.

echo "[Start waypoints.launch]"
roslaunch erp42_control waypoints.launch

echo "[Start Monitor Server]"
cd ~/catkin_ws/src/ACCA2022-new/monitor_server/
npm start

exit 0