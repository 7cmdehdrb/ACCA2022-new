# ACCA 2022

Autonomous driving project made by ACCA in 2022

## Project Dependency

- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)

- [darknet](https://github.com/leggedrobotics/darknet)

- [erp42_vehicle, erp42_velodyne, erp42_msgs](https://github.com/jdj2261/ERP42-ROS)

### Project Env

Ubuntu 18.04.5

ROS Melodic

Python2

### Installing

```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
