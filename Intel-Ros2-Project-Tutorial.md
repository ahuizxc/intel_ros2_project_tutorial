## Overview

Intel Ros2 Project contains several ROS2 packages in object classification, detection, localization and tracking and SLAM.

## Package Lists

* [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)
* [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs)
* [ros2_object_analytics](https://github.com/intel/ros2_object_analytics)
* [ros2_message_filters](https://github.com/intel/ros2_message_filters)
* [ros2_object_msgs](https://github.com/intel/ros2_object_msgs)
* [ros2_object_map](https://github.com/intel/ros2_object_map)
* [ros2_moving_object](https://github.com/intel/ros2_moving_object)

## Dependencies

### Hardware

* An x86_64 computer running Ubuntu 16.04. OS X and Windows are not supported yet
* [Intel® RealSense™ Devices](https://realsense.intel.com/)  
* [™ Neural Compute Stick](https://developer.movidius.com/)

### ROS1 and ROS2

* Install ROS1 Kinetic ([guide](wiki.ros.org/kinetic/Installation/Ubuntu))
* Install ROS2 Ardent ([guide](https://github.com/ros2/ros2/wiki/Linux-Install-Debians))

### Installation Instructions

* **1.Install the Intel® RealSense™ SDK 2.0**\\
Install tag v2.9.1 [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/tree/v2.9.1) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/v2.9.1/doc/installation.md).\\
**Note:** Use `git checkout v2.9.1` to switch to the v2.9.1 branch.
* **2.Install ROS1 Kinetic**\\
[Ubuntu install of ROS Kinetic(**ros-kinetic-desktop-full**)](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* **3.Install ROS2 Bouncy**\\
[Ubuntu install of ROS Bouncy](https://github.com/ros2/ros2/wiki/Linux-Development-Setup)\\
Source the environment
```bash
$ cd ~/ros2_ws
$ source install/local_setup.bash
```
* **4.Install [ros2 cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2)**
```bash
# Creating a new ROS2 workspace is recommended
$ mkdir -p ~/myros2_ws/src
$ cd ~/myros2_ws/src
$ git clone https://github.com/ros-perception/vision_opencv.git
$ git checkout ros2
$ cd ~/myros2_ws
$ colcon build --symlink-install
```

* **5.Install Intel® RealSense™ ROS2 from Sources**
```bash
# Goto the new ROS workspace step 4 created before
$ cd ~/myros2_ws/src
# Clone the latest Intel® RealSense™ ROS2 repository
$ git clone https://github.com/intel/ros2_intel_realsense.git
$ cd ~/myros2_ws
$ colcon build --symlink-install --packages-select realsense_camera_msgs realsense_ros2_camera
$ source ./install/local_setup.bash
```

