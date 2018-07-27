#!/bin/bash
echo "Begin installing....."
sudo apt-get update
sudo apt-get install -y git cmake
sudo apt-get install -y libusb-1.0.0-dev pkg-config libgtk-3-dev libglfw3-dev libudev-dev
cd /usr/local/lib


if [ -e "librealsense2.so.2.9.1"]
then
    echo "librealsense 2.9.1 has been installed"
    sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
elif [ -e "librealsense2.so"]
then
    echo "librealsense's version doesn't match"
    sudo rm -rf librealsense*
    echo "librealsense 2.9.1 will be installed in ~/workspace/intel_ros_libraries ..."
    sudo mkdir -p ~/workspace/intel_ros_libraries
    cd ~/workspace/intel_ros_libraries
    sudo git clone https://github.com/IntelRealSense/librealsense && cd librealsense
    sudo git checkout v2.9.1
    sudo mkdir build && cd build
    sudo cmake ..
    sudo make clean
    sudo make
    sudo make install
    sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    echo "Done..."
else 
    echo "librealsense has not been installed"
    echo "librealsense 2.9.1 will be installed in ~/workspace/intel_ros_libraries ..."
    sudo mkdir -p ~/workspace/intel_ros_libraries
    cd ~/workspace/intel_ros_libraries
    sudo git clone https://github.com/IntelRealSense/librealsense && cd librealsense
    sudo git checkout v2.9.1
    sudo mkdir build && cd build
    sudo cmake ..
    sudo make clean
    sudo make
    sudo make install
    echo "Done..."

fi


cd ~
if [ -x "/usr/local/lib/mvnc"]
then
    echo "NCSDK has been installed"
else 
    echo "NCSDK has not been installed"
    echo "NCSDK and NCAPPZOO will be installed in ~/workspace/intel_ros_libraries"
    sudo mkdir -p ~/workspace/intel_ros_libraries
    cd ~/workspace/intel_ros_libraries
    sudo git clone https://github.com/movidius/ncsdk.git
    sudo git clone https://github.com/movidius/ncappzoo.git
    cd ncsdk
    "Installing NCSDK..."
    sudo make install
    echo "Done..."
    cd ..
    "Installing NCAPPZOO..."
    cd ncappzoo/caffe/
    export PYTHONPATH="${PYTHONPATH}:/opt/movidius/caffe/python"
    sudo make
    cd ../tensorflow
    sudo make
    echo "create symbol link in /opt/movidius/ to NCAPPZOO"
    sudo ln -s ~/workspace/libraries/ncappzoo /opt/movidius/ncappzoo    
    echo "Done..."
fi


echo "checking OpenCV"
check_opencv= pkg-config --modversion opencv

if [${check_opencv:0:1}="3"]
then 
    echo "OpenCV 3.x already exist"
else
    echo "OpenCV 3.x has not been installed, installing..."
    cd ~/workspace/intel_ros_libraries
    sudo git clone https://github.com/opencv/opencv.git
    sudo git clone https://github.com/opencv/opencv_contrib.git
    cd ~/Desktop
    wget https://raw.githubusercontent.com/opencv/opencv_3rdparty/a62e20676a60ee0ad6581e217fe7e4bada3b95db/ippicv/ippicv_2017u2_lnx_intel64_20170418.tgz
    sudo mkdir ~/workspace/intel_ros_libraries/opencv/.cache/ippicv/ -p
    sudo mv ippicv_2017u2_lnx_intel64_20170418.tgz ~/workspace/intel_ros_libraries/opencv/.cache/ippicv/87cbdeb627415d8e4bc811156289fa3a-ippicv_2017u2_lnx_intel64_20170418.tgz
    cd ~/workspace/intel_ros_libraries/opencv
    git checkout 3.3.0
    cd ~/workspace/intel_ros_libraries/opencv_contrib
    git checkout 3.3.0
    cd ~/workspace/intel_ros_libraries/opencv
    mkdir build && cd build
    cmake -DOPENCV_EXTRA_MODULES_PATH=~/workspace/intel_ros_libraries/opencv_contrib/modules -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_cnn_3dobj=OFF ..
    make -j4
    sudo make install
    sudo ldconfig
    echo "Done..."
fi


check_ros= apt-cache search ros-kinetic-desktop | grep "ros-kinetic-desktop"
# echo $check_ros
echo "checking ros-kinetic-desktop"
if [ ! -n $check_ros ]
then
    echo "ros-kinetic-desktop not installed"
    sudo apt-get install ros-kinetic-desktop-full
fi

echo "ros2 bouncy will be installed in ~/ros2_ws_bk"

sudo locale-gen en_US en_US.UTF-8
sudo -S update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo -S sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -S apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo -S apt-get update
sudo -S apt-get install -y git wget
sudo -S apt-get install -y build-essential cppcheck cmake libopencv-dev python-empy python3-catkin-pkg-modules python3-dev python3-empy python3-nose python3-pip python3-pyparsing python3-setuptools python3-vcstool python3-yaml libtinyxml-dev libeigen3-dev libassimp-dev libpoco-dev
sudo -S apt-get install -y python3-colcon-common-extensions

# dependencies for testing
sudo -S apt-get install -y clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify

# Install argcomplete for command-line tab completion from the ROS2 tools.
# Install from pip rather than from apt because of a bug in the Ubuntu 16.04 version of argcomplete:
sudo -S -H python3 -m pip install argcomplete

# additional testing dependencies from pip (because not available on ubuntu 16.04)
sudo -S -H python3 -m pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest pytest-cov pytest-runner

# additional pytest plugins unavailable from Debian
sudo -S -H python3 -m pip install pytest-repeat pytest-rerunfailures

# dependencies for FastRTPS
sudo -S apt-get install -y libasio-dev libtinyxml2-dev
 
# dependencies for RViz
sudo -S apt-get install -y libcurl4-openssl-dev libqt5core5a libqt5gui5 libqt5opengl5 libqt5widgets5 libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qtbase5-dev


cd /usr/lib/x86_64-linux-gnu

if [ ! -f "libboost_python3.so" ]
then
  sudo -S ln -s libboost_python-py35.so libboost_python3.so
else
  echo "soft link libboost_python3 already exists, skip..."
fi

echo "Installing ros2 bouncy..."
mkdir -p ~/ros2_ws_bk/src
cd ~/ros2_ws_bk
wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
vcs-import src < ros2.repos
cd ~/ros2_ws_bk/src
git clone https://github.com/ros2/pcl_conversions.git
cd pcl_conversions
git checkout bouncy
cd ..
git clone https://github.com/ros-perception/vision_opencv.git
cd ~/ros2_ws_bk/src/vision_opencv
git checkout ros2
cd ~/ros2_ws_bk/
colcon build --symlink-install
source install/local_setup.bash

echo "Done..."
echo "Intel ros packages will be installed in ~/ros2_overlay_ws"

sudo mkdir -p ~/ros2_overlay_ws/src
cd ~/ros2_overlay_ws/src


echo "Installing ros2_intel_realsense..."
cd ~/ros2_overlay_ws/src
# Clone the latest Intel® RealSense™ ROS2 repository and build use colcon 
git clone https://github.com/intel/ros2_intel_realsense.git
cd ~/ros2_overlay_ws
source ~/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select realsense_camera_msgs realsense_ros2_camera
source ~/ros2_overlay_ws/install/local_setup.bash
# Create a symbol link from libusb.a to libusb-1.0.a, otherwise "libusb.a" is probably not to be found by librealsense
sudo ln -s /usr/lib/x86_64-linux-gnu/libusb-1.0.a /usr/lib/libusb.a
echo "Done..."

echo "Installing ros2_object_msgs..."
cd ~/ros2_overlay_ws/src
# Clone the ros2_object_msgs repository and build use colcon
git clone https://github.com/intel/ros2_object_msgs.git
cd ~/ros2_overlay_ws
source ~/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select object_msgs
source ~/ros2_overlay_ws/install/local_setup.bash
echo "Done..."

echo "Installing ros2_message_filters..."
cd ~/ros2_overlay_ws/src
git clone https://github.com/intel/ros2_message_filters.git
cd ~/ros2_overlay_ws
source ~/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select message_filters
source ~/ros2_overlay_ws/install/local_setup.bash
echo "Done..."

echo "Installing ros2_intel_movdius_ncs..."
cd ~/ros2_overlay_ws/src
git clone https://github.com/intel/ros2_intel_movidius_ncs.git
cd ~/ros2_overlay_ws
source ~/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select movidius_ncs_example  movidius_ncs_image  movidius_ncs_launch  movidius_ncs_lib  movidius_ncs_stream
source ~/ros2_overlay_ws/install/local_setup.bash
echo "Done..."

echo "Installing ros2_object_analytics..."
cd ~/ros2_overlay_ws/src
git clone https://github.com/intel/ros2_object_analytics.git
cd ..
source ~/ros2_ws/install/local_setup.bash 
colcon build --symlink-install --packages-select object_analytics_launch  object_analytics_node object_analytics_msgs object_analytics_rviz
source ~/ros2_overlay_ws/install/local.setup.bash
echo "Done..."

echo "Installing ros2_object_map..."
cd ~/ros2_overlay_ws/src
git clone https://github.com/intel/ros2_object_map.git
cd ..
source ~/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select object_map object_map_msgs
source ~/ros2_overlay_ws/install/local.setup.bash
echo "Done..."

echo "Installing ros2_moving_object..."
cd ~/ros2_overlay_ws/src
git clone https://github.com/intel/ros2_moving_object.git
cd ..
source ~/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select moving_object moving_object_msgs
source ~/ros2_overlay_ws/install/local.setup.bash
echo "Done..."

"Installation done!"

"Goto https://github.com/ahuizxc/intel_ros2_project_tutorial/blob/master/Intel-Ros2-Project-Tutorial.md for more informations."
