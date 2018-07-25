# ROS2 Wrapper for Intel&reg; RealSense&trade; Devices

## 1 Instroduction 

These are packages for using Intel RealSense cameras (D400 series) with ROS2.


## 2 Running the demo

### 2.1 Start the camera node
To start the camera node in ROS2, plug in the camera, then type the following command:

```bash
# To launch with "ros2 run"
$ ros2 run realsense_ros2_camera realsense_ros2_camera
# OR, to invoke the executable directly
$ realsense_ros2_camera
```

This will stream all camera sensors and publish on the appropriate ROS2 topics. PointCloud2 is enabled by default, till we provide ROS2 python launch options.

### 2.2 View camera data

To start the camera node in ROS2 and view the depth pointcloud in rviz via [ros1_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md):
```bash
# firstly self-build ros1_bridge, than refer to section "Example 1b: ROS 2 talker and ROS 1 listener"

# in console #1 launch roscore
$ source /opt/ros/kinetic/setup.bash
$ roscore

# in console #2 launch ros1_bridge
$ source /opt/ros/kinetic/setup.bash
$ cd ~/ros2_ws
$ source ./install/local_setup.bash
$ export ROS_MASTER_URI=http://localhost:11311
$ ros2 run ros1_bridge dynamic_bridge

# in console #3 launch rviz
$ source /opt/ros/kinetic/setup.bash
$ rosrun rviz rviz -d ~/ros2_ws/src/ros2_intel_realsense/realsense_ros2_camera/rviz/ros2.rviz

# in console #4 launch realsense_ros2_camera
$ source ~/ros2_ws/install/local_setup.bash
$ realsense_ros2_camera
```

This will launch [RViz](http://wiki.ros.org/rviz) and display the five streams: color, depth, infra1, infra2, pointcloud.

NOTE: in case PointCloud2 stream is not observed, try stop the "realsense_ros2_camera" and re-launch this node from console #4. This's a known issue and workaround is made (right fixing in ros1_bridge, details discussed in [ROS discourse](https://discourse.ros.org/t/ros1-bridge-failed-to-pass-tf-static-message-when-subscribed-from-rviz/3863)).

NOTE: visulization in ROS2 pending on [rviz2](https://github.com/ros2/rviz).

![realsense_ros2_camera visualization results](https://github.com/intel/ros2_intel_realsense/raw/master/realsense_ros2_camera/rviz/ros2_rviz.png "realsense_ros2_camera visualization results")

## 3 Interfaces

[/camera/depth/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/color/image_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/infra1/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/infra2/image_rect_raw](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg)

[/camera/depth/color/points](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)

## 4 Known Issues
* This ROS2 node does not currently provide any dynamic reconfigure support for camera properties/presets.
* We support Ubuntu Linux Xenial Xerus 16.04 on 64-bit, but not support Mac OS X 10.12 (Sierra) and Windows 10 yet.

## 5 TODO
A few features to be ported from the latest realsense_ros_camera v2.0.2
* RGB-D point cloud (depth_registered)
* Preset/Controls

###### *Any security issue should be reported using process at https://01.org/security*