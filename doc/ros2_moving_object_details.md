# Vision-Based Moving Object

## 1. Introduction
Moving Object component is addressing moving objects based on messages generated by
Object Analytics [ros2_object_analytics](https://github.com/intel/ros2_object_analytics).
ros2_moving_object delivers further analysis for the localized and tracked objects from Object Analytics by adding **motion information**, i.e., the **velocity** information about tracked objects. Such information can extend robot's ability of motion planing and collision avoidance.

Thanks to [ros2_object_analytics](https://github.com/intel/ros2_object_analytics) and [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs) to provide an AI solution for object detection, tracking and localization. See [the umbrella wiki page](http://wiki.ros.org/intelrosproject) to learn the hierarchical data flow and overview description for the related components.

This component involves 2 ROS2 packages:
- **moving_object**: the main package covering logic of moving object analysis and information generation.
- **moving_object_msgs**: the message package storing the motion information of moving objects and published into ROS2 system.

## 2. Running the demo

  ### Step 1. *[In terminal 1]* Launch realsense camera node.

  ```bash
  source </ros2/install/dir>/local_setup.bash
  source </my/overlay_ws/dir>/install/local_setup.bash
  realsense_ros2_camera
  ```
  ### Step 2. *[In terminal 2]* Launch object analysis node.
  ```bash
  source </ros2/install/dir>/local_setup.bash
  source </my/overlay_ws/dir>/install/local_setup.bash
  echo -e "param_file: alexnet.yaml\ninput_topic: /object_analytics/rgb > src/ros2_intel_movidius_ncs/movidius_ncs_launch/config/default.yaml"
  launch `ros2 pkg prefix object_analytics_launch`/share/object_analytics_launch/launch/analytics_movidius_ncs.py
  ```
  ### Step 3. *[In terminal 3]* Launch moving object node.
  ```bash
  source </ros2/install/dir>/local_setup.bash
  source </my/overlay_ws/dir>/install/local_setup.bash
  ros2 run moving_object moving_object
  ```

## 3. Interfaces

ros2_moving_object package publishes some messages to indicate different status/data.
 - **/moving\_object/moving\_objects** merges info from the 3 input messages into one message and calculating (on demand) the velocity info of the tracked moving objects.

## 4. Known issues

---

## 5. TODO

---

##### *Any security issue should be reported using process at https://01.org/security
