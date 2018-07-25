# ros2_intel_movidius_ncs

## 1 Introduction
The Movidius™ Neural Compute Stick ([NCS](https://developer.movidius.com/)) is a tiny fanless deep learning device that you can use to learn AI programming at the edge. NCS is powered by the same low power high performance Movidius™ Vision Processing Unit ([VPU](https://www.movidius.com/solutions/vision-processing-unit)) that can be found in millions of smart security cameras, gesture controlled drones, industrial machine vision equipment, and more.  

This project is a ROS2 wrapper for NC API of [NCSDK](https://movidius.github.io/ncsdk/), providing the following features:
* A ROS2 service for object classification and detection of a static image file
* A ROS2 publisher for object classification and detection of a video stream from a RGB camera
* Demo applications to show the capabilities of ROS2 service and publisher
* Support multiple CNN models of Caffe and Tensorflow
  

## 2 Running the Demo
### 2.1 Classification
#### 2.1.1 Supported CNN Models
###### *Table1*
|CNN Model|Framework|Usage|
|:-|:-|:-|
|AlexNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#alexnet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#alexnet)|
|GoogLeNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#googlenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#googlenet)|
|SqueezeNet|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#squeezenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#squeezenet)|
|Inception_v1|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v1)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v1)|
|Inception_v2|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v2)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v2)|
|Inception_v3|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v3)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v3)|
|Inception_v4|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#inception_v4)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#inception_v4)|
|MobileNet|Tensorflow|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md#mobilenet)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md#mobilenet)|
#### 2.1.2 Classification Result with GoogLeNet
![classification with googlenet](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/data/results/googlenet_dog.png "classification with googlenet")
#### 2.1.3 Running the Demo
* [Static Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_classification.md)
* [Video Streaming](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_classification.md)

### 2.2 Detection
#### 2.2.1 Supported CNN Models
|CNN Model|Framework|Usage|
|:-|:-|:-|
|MobileNetSSD(Recommended)|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md#mobilenet_ssd)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md#mobilenet_ssd)|
|TinyYolo_v1|Caffe|[Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md#tinyyolo_v1)/[Video](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md#tinyyolo_v1)|
#### 2.2.2 Detection Result with MobileNetSSD
![detection with mobilenetssd](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/data/results/mobilenetssd_car_bicycle.png "detection with mobilenetssd")
#### 2.2.3 Running the Demo
* [Static Image](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/image_detection.md)
* [Video Streaming](https://github.com/intel/ros2_intel_movidius_ncs/blob/master/doc/video_detection.md)

## 3 Interfaces
### 3.1 Topic
Classification: ```/movidius_ncs_nodelet/classified_objects```  
Detection: ```/movidius_ncs_nodelet/detected_objects```
### 3.2 Service
Classification: ```/movidius_ncs_image/classify_object```  
Detection: ```/movidius_ncs_image/detect_object```

## 4 Known Issues
* Only absolute path of image file supported in image inference demo
* Only test on RealSense D400 series camera

## 5 TODO
* Keep synchronized with [ROS NCS Package](https://github.com/intel/ros_intel_movidius_ncs/tree/master)

###### *Any security issue should be reported using process at https://01.org/security*
