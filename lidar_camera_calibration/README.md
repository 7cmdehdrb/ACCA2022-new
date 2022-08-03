# ROS Camera LIDAR Calibration Package

## Setup

Install dependencies.

```
sudo apt install ros-DISTRO-camera-calibration
```

Run the following to clone the `lidar_camera_calibration` package in `ros_workspace/src` directory.

```
cd ~/ros_workspace/src
git clone https://github.com/heethesh/lidar_camera_calibration

cd ~/ros_workspace/
catkin_make
source devel/setup.bash
```

Make sure you have the ROS bag file in `lidar_camera_calibration/bagfiles` folder. Then you can use the following launch files. This package assumes that the bag file has atleast the following topic names and message types by default, these can be modified in the launch scripts.

```
/sensors/velodyne_points    (sensor_msgs/PointCloud2)
/sensors/camera/image_color (sensor_msgs/Image)
/sensors/camera/camera_info (sensor_msgs/CameraInfo) (optionally generated by camera_calibration, see below)
```

## Play ROS Bag File

This launch file will only play the rosbag record file.

```
roslaunch lidar_camera_calibration play_rosbag.launch bagfile:=/path/to/file.bag
```

## Run Camera Calibration

This launch file will play the rosbag record and runs the `camera_calibration` package from ROS. The results are stored by default at `~/.ros/camera_info`.

```
roslaunch lidar_camera_calibration camera_calibration.launch
```

23 images were automatically selected by the calibrator and the sample parameters obtained are stored [here](calibration_data/camera_calibration/ost.yaml). The following results were obtained:

#### Camera Matrix

```
484.130454    0.000000  457.177461
  0.000000  484.452449  364.861413
  0.000000    0.000000    1.000000
```

#### Distortion Coefficients

```
-0.199619  0.068964  0.003371  0.000296  0.000000
```

## Update the ROS Bag File

This script will update the camera matrices and the distortion coefficients in the `/sensors/camera/camera_info` topic and creates a new bag file in the same location. Note, if you did not have any camera calibration information before, ROS would automatically pick the camera info from `~/.ros/camera_info` when playing the bag file and you can skip this step after verification (`rostopic echo /<CAMERA_NAME>/camera_info`).

```
rosrun lidar_camera_calibration update_camera_info.py <original_file.bag> <calibration_file.yaml>
```

## Display Camera Calibration

This launch file will play the updated rosbag record, run `image_proc` for camera image rectification and displays the rectified and unrectified images.

```
roslaunch lidar_camera_calibration display_camera_calibration.launch
```

### [YouTube Link for Camera Calibration Demo](https://youtu.be/8FHSmFBTL3U)

[<img src="https://github.com/heethesh/lidar_camera_calibration/blob/master/images/camera_calibration.png?raw=true">](https://youtu.be/8FHSmFBTL3U)

## Calibrate Camera-LiDAR Point Correspondences

This script will perform calibration using the matplotlib GUI to pick correspondences in the camera and the LiDAR frames. You first need to play the rosbag record in another terminal.

```
roslaunch lidar_camera_calibration play_rosbag.launch bagfile:=/path/to/file.bag
rosrun lidar_camera_calibration calibrate_camera_lidar.py --calibrate
```

Press [ENTER] to launch the GUIs and pick the corresponding points by selecting the four corner points of the checkerboard in both the camera and the LiDAR frames. You may update the point cloud field-of-view to display [here](https://github.com/heethesh/lidar_camera_calibration/blob/master/scripts/calibrate_camera_lidar.py#L232)  

OpenCV's PnP RANSAC + refinement using LM is used to find the rotation and translation transforms between the camera and the LiDAR. Since OpenCV's function rectifies the images internally, the 2D points are picked from the unrectified image. Additional, the `rectify` flag can be set to `True` while creating the GUI process to pick points from a rectified image.

**NOTE: If you are using Ouster LiDAR, set `OUSTER_LIDAR = True` [here](https://github.com/heethesh/lidar_camera_calibration/blob/master/scripts/calibrate_camera_lidar.py#L75). (see issue #26)**

**NOTE: The point files are appended and the extrinsics estimates are calculated and refined continuously using a RANSAC approach.**

**NOTE: To use `solvePnPRefineLM`, you need OpenCV >= 4.1.1, otherwise the LM pose refinement step will be skipped.**

### [YouTube Link for Camera-LiDAR Calibration GUI Demo](https://youtu.be/FgP8jZ_siJI)

[<img src="https://github.com/heethesh/lidar_camera_calibration/blob/master/images/gui_demo.png?raw=true">](https://youtu.be/FgP8jZ_siJI)

The point correspondences are saved as following:
- Image Points: `lidar_camera_calibration/calibration_data/lidar_camera_calibration/img_corners.npy`
- LiDAR Points: `lidar_camera_calibration/calibration_data/lidar_camera_calibration/pcl_corners.npy`

The calibrated extrinsics are saved as following:
- `lidar_camera_calibration/calibration_data/lidar_camera_calibration/extrinsics.npz`
    - 'euler' : Euler Angles (RPY rad)
    - 'R'     : Rotation Matrix
    - 'T'     : Translation Offsets (XYZ m)

The following calibrated extrinsics were obtained:

#### Rotation Matrix
```
-9.16347982e-02  -9.95792677e-01  -8.74577923e-05
 1.88123595e-01  -1.72252569e-02  -9.81994299e-01
 9.77861226e-01  -9.00013023e-02   1.88910532e-01
```

#### Euler Angles (RPY rad)

```
-0.44460865  -1.35998386   2.0240699
```

#### Translation Offsets (XYZ m)

```
-0.14614803  -0.49683771  -0.27546327
```

## Display Camera-LiDAR Projection

This launch file will play the updated rosbag record, run `calibrate_camera_lidar.py` in projection mode and displays the LiDAR point cloud projected on to the image. A static transform is set up between the `world` and the `velodyne` frame which needs to be updates with the values above in the format `X Y Z Y P R` within the launch file. You may update the point cloud field-of-view to display [here](https://github.com/heethesh/lidar_camera_calibration/blob/master/scripts/calibrate_camera_lidar.py#L383).

**NOTE: If you are using Ouster LiDAR, set `OUSTER_LIDAR = True` [here](https://github.com/heethesh/lidar_camera_calibration/blob/master/scripts/calibrate_camera_lidar.py#L75). (see issue #26)**

```
roslaunch lidar_camera_calibration display_camera_lidar_calibration.launch
```

### [YouTube Link for Camera-LiDAR Projection Demo](https://youtu.be/lu2HwMWESj8)

[<img src="https://github.com/heethesh/lidar_camera_calibration/blob/master/images/camera_lidar_calibrated.png?raw=true">](https://youtu.be/lu2HwMWESj8)

## TODO
- [ ] Shift to Rviz point-picker for point cloud. Matplotlib GUI picker is not very convinient to use.