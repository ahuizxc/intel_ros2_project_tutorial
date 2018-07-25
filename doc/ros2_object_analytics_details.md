# ros2_object_analytics

## 1 Introduction
Object Analytics (OA) is ROS2 wrapper for realtime object detection, localization and tracking.
These packages aim to provide real-time object analyses over RGB-D camera inputs, enabling ROS developer to easily create amazing robotics advanced features, like intelligent collision avoidance and semantic SLAM. It consumes [sensor_msgs::PointClould2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) data delivered by RGB-D camera, publishing topics on [object detection](https://github.com/intel/ros2_object_msgs), [object tracking](https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs), and [object localization](https://github.com/intel/ros2_object_analytics/object_analytics_msgs) in 3D camera coordination system.

OA keeps integrating with various "state-of-the-art" algorithms.
* Object detection offload to VPU, Intel Movidius NCS, with MobileNet SSD model and Caffe framework.

## 2 Running the demo

### Step 1. *[In terminal 1]* Launch Realsense camera node
  
  ```bash
  # Terminal 1:
  . <install-space-with-realsense-ros2-camera>/local_setup.bash
  realsense_ros2_camera
  ```

### Step 2. *[In terminal 1]* Launch NCS and OA node

  ```bash
  # Terminal 2
  . <install-space-with-object-analytics-launch>/local_setup.bash
  echo -e "param_file: mobilenetssd.yaml\ninput_topic: /object_analytics/rgb" > `ros2 pkg prefix movidius_ncs_launch`/share/movidius_ncs_launch/config/default.yaml
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
  ```

### Step 3. *[In terminal 1]* Launch OA Rviz
  
  ```bash
  # Terminal 3
  . <install-space-with-object-analytics-launch>/local_setup.bash
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/object_rviz.py
  ```

## 3 Interfaces

### 3.1 Subscribed topics
  /movidius_ncs_stream/detected_objects ([object_msgs::msg::ObjectsInBoxes](https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg))

### 3.2 Published topics
  /object_analytics/rgb ([sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg))

  /object_analytics/pointcloud ([sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg))

  /object_analytics/localization ([object_analytics_msgs::msg::ObjectsInBoxes3D](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg))

  /object_analytics/tracking ([object_analytics_msgs::msg::TrackedObjects](https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg))


## 3.3 Customize launch
  By default, object analytics will launch both tracking and localization features, but either tracking or localization or both can be dropped. Detailed please refer comments embedded in launch file.

## 4 Known issues

--

## 5 TODO

--

###### *Any security issue should be reported using process at https://01.org/security*
