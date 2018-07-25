# ros2_object_map

## 1 Introduction
ros2_object_map is ROS2 package which designes to mark tag of objects on map when SLAM. It uses [ros2_object_analytics](https://github.com/intel/ros2_object_analytics) for object detection.

![Architecture of Object Map](https://github.intel.com/otc-rse/ros2_object_map/blob/dev/object_map/object_map/ObjectMap.PNG "architecture of object map")

## 2 Running the demo

###  Step 1. *[In terminal 1]* Launch Realsense Camera node

```bash
# terminal 1 
source ~/ros2_ws/install/local_setup.bash
realsense_ros2_camera
```

### Step 2. *[In terminal 2]* Launch object_analytics node

```bash
# terminal 2
source ~/ros2_ws/install/local_setup.bash
launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
```

### Step 3. *[In terminal 3]* Launch ros2_object_map node

```bash
# terminal 3
source source ~/ros2_ws/install/local_setup.bash
ros2 run object_map object_map_node
```

### Step 4. *[In terminal 4]* Launch ROS1 roscore

```bash
# terminal 4
source /opt/ros/kinetic/setup.bash
roscore
```

### Step 5. *[In terminal 5]* Launch ROS1 bridge

```bash
# terminal 5
source /opt/ros/kinetic/setup.bash
source ros2_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge

```

### Step 6. *[In terminal 6]* Launch ROS1 Rviz

``` bash
# terminal 6
source /opt/ros/kinetic/setup.bash
roslaunch turtlebot_rviz_launchers view_robot.launch

within rviz gui, click "Add", and select "MarkerArray", then input "/object_map/Markers" into "Marker Topic"
```

## 3 Interfaces

### 3.1 Topic

  * ```/object_map/Markers``` : Publish MarkerArray on RVIZ
  * ```/object_map/map_save``` : Subscribe map_save topic to save object maps
  * ```/movidius_ncs_stream/detected_objects```: Subscribe ObjectsInBoxes from object_analytics
  * ```/object_analytics/tracking```: Subscribe TrackedObjects from object_analytics
  * ```/object_analytics/localization```: Subscribe ObjectsInBoxes3D from object_analytics

### 3.2 Save object map

```bash
ros2 topic pub --once /object_map/map_save std_msgs/Int32 -1

```

## 4 Known Issues

### 4.1 Map tag cannot be correctly displayed in Rviz while robot is moving

reason: tf2 python api is not supported in ROS2 currrently

next step: will implement it while tf2-python api is ready in ROS2  

### 4.2 Configure File is not supported 

reason: yaml configure file and dynamic configure file are not supported in ROS2 currently

next step: will implement it while it is ready in next release of ROS2

## 5 TODO

---

###### *Any security issue should be reported using process at https://01.org/security*
