## Overview

Intel Ros2 Project contains several ROS2 packages in object classification, detection, localization and tracking and SLAM.

## Package Lists

* [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)
* [ros2_message_filters](https://github.com/intel/ros2_message_filters)
* [ros2_object_msgs](https://github.com/intel/ros2_object_msgs)
* [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs)
* [ros2_object_analytics](https://github.com/intel/ros2_object_analytics)
* [ros2_object_map](https://github.com/intel/ros2_object_map)
* [ros2_moving_object](https://github.com/intel/ros2_moving_object)

## Dependencies

### Hardware

* An x86_64 computer running Ubuntu 16.04. OS X and Windows are not supported yet
* [Intel® RealSense™ Devices](https://realsense.intel.com/)  
* [™ Neural Compute Stick](https://developer.movidius.com/)

### ROS1,ROS2 and OpenCV 3.X

* Install ROS1 Kinetic ([guide](wiki.ros.org/kinetic/Installation/Ubuntu))
* Install ROS2 Bouncy  ([guide](https://github.com/ros2/ros2/wiki/Linux-Install-Debians))
* Install OpenCV 3.X([guid](https://docs.opencv.org/3.3.0/d7/d9f/tutorial_linux_install.html))

### Installation Instructions

* **1. Install the Intel® RealSense™ SDK 2.0**\\
Install tag v2.9.1 [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/tree/v2.9.1) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/v2.9.1/doc/installation.md).\\
**Note:** Use `git checkout v2.9.1` to switch to the v2.9.1 branch.
* **2. Install ROS1 Kinetic**\\
[Ubuntu install of ROS Kinetic(**ros-kinetic-desktop-full**)](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* **3. Install ROS2 Bouncy**\\
[Ubuntu install of ROS Bouncy](https://github.com/ros2/ros2/wiki/Linux-Development-Setup)\\
Source the environment
```bash
$ cd ~/ros2_ws
$ source install/local_setup.bash
```
* **4. Install [ros2 cv_bridge](https://github.com/ros-perception/vision_opencv/tree/ros2)**
```bash
# Creating a new ROS2 workspace 'ros2_overlay_ws' instead of using 'ros2_ws' is recommended
$ mkdir -p ~/ros2_overlay_ws/src
$ cd ~/ros2_overlay_ws/src
$ git clone https://github.com/ros-perception/vision_opencv.git
$ git checkout ros2
$ cd ~/ros2_overlay_ws
$ source ~/ros2_ws/install/local_setup.bash
$ colcon build --symlink-install
$ source ~/ros2_overlay_ws/install/local_setup.bash
```

* **5. Install [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense)**
```bash

# Goto the new ROS workspace step 4 created before
$ cd ~/ros2_overlay_ws/src
# Clone the latest Intel® RealSense™ ROS2 repository and build use colcon 
$ git clone https://github.com/intel/ros2_intel_realsense.git
$ cd ~/ros2_overlay_ws
$ source ~/ros2_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-select realsense_camera_msgs realsense_ros2_camera
$ source ~/ros2_overlay_ws/install/local_setup.bash
# Create a symbol link from libusb.a to libusb-1.0.a, otherwise "libusb.a" is probably not to be found by librealsense
$ sudo ln -s /usr/lib/x86_64-linux-gnu/libusb-1.0.a /usr/lib/libusb.a

```
* **6. Install [ros2_object_msgs](https://github.com/intel/ros2_object_msgs)**
```bash
$ cd ~/ros2_overlay_ws/src
# Clone the ros2_object_msgs repository and build use colcon
$ git clone https://github.com/intel/ros2_object_msgs.git
$ cd ~/ros2_overlay_ws
$ source ~/ros2_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-select object_msgs
$ source ~/ros2_overlay_ws/install/local_setup.bash
```
* **7. Install [ros2_message_filters](https://github.com/intel/ros2_message_filters)**
```bash
$ cd /usr/lib/x86_64-linux-gnu
# Create a symbol link from libboost_python-py35.so to libboost_python3.so
$ sudo ln -s libboost_python-py35.so libboost_python3.so
$ cd ~/ros2_overlay_ws/src
$ git clone https://github.com/intel/ros2_message_filters.git
$ cd ~/ros2_overlay_ws
$ source ~/ros2_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-select message_filters
$ source ~/ros2_overlay_ws/install/local_setup.bash
```

* **8. Install [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs)**
```bash
# Install [NCSDK 1.X](https://github.com/movidius/ncsdk) and [NCAPPZOO](https://github.com/movidius/ncappzoo) at first
# create  workspace to install libraries ros2_intel_moidius_ncs relies on
$ mkdir -p ~/workspace/libraries
$ cd ~/workspace/libraries
# install ncsdk and ncappzoo
$ git clone https://github.com/movidius/ncsdk.git
$ git clone https://github.com/movidius/ncappzoo.git
$ cd ~/workspace/libraries/ncsdk
$ sudo make install
$ export PYTHONPATH="${PYTHONPATH}:/opt/movidius/caffe/python"
# Download and compile the object detection model
$ cd ~/workspace/libraries/ncappzoo/caffe/
$ sudo make
$ cd ~/workspace/libraries/ncappzoo/tensorflow/
$ sudo make
# NCSDK should be installed in /opt/movidius by default. Create a symbol link in /opt/movidius to NCAPPZOO
$ sudo ln -s ~/workspace/libraries/ncappzoo /opt/movidius/ncappzoo


# Install ros2_intel_movidius_ncs
$ cd ~/ros2_overlay_ws/src
$ git clone https://github.com/intel/ros2_intel_movidius_ncs.git
$ cd ~/ros2_overlay_ws
$ source ~/ros2_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-select movidius_ncs_example  movidius_ncs_image  movidius_ncs_launch  movidius_ncs_lib  movidius_ncs_stream
$ source ~/ros2_overlay_ws/install/local_setup.bash
```
* **9. Install [ros2_object_analytics](https://github.com/intel/ros2_object_analytics)**

```bash
# Install pcl_conversions package at first
$ cd ~/ros2_ws/src
$ git clone https://github.com/ros2/pcl_conversions.git
$ cd pcl_conversions
$ git checkout bouncy
$ cd ~/ros2_ws
$ source ~/ros2_ws/install/local_setup.bash
$ colcon build --symlink-install --packages-select pcl_conversions

# Install ros2_object_analytics
$ cd ~/ros2_overlay_ws/src
$ git clone https://github.com/intel/ros2_object_analytics.git
$ cd ..
$ colcon build --symlink-install --packages-select object_analytics_launch  object_analytics_node object_analytics_msgs object_analytics_rviz

 
