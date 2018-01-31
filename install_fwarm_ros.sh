#!/bin/bash

clear

echo "Starting install of FWARM ROS software"

echo "Update sources."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  

echo "Set up keys"
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

echo "Update ubtuntu"
sudo apt-get update

echo "Install ROS"
sudo apt-get install ros-kinetic-desktop-full

echo "Initialize rosdep"
sudo rosdep init
rosdep update

echo "environment setup"
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Configure workspace"
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

echo "Download libfreenect2 from source"
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

echo "Install build tools"
sudo apt-get install build-essential cmake pkg-config

echo "Install libusb"
sudo apt-get install libusb-1.0-0-dev

echo "Install TurboJPG"
sudo apt-get install libturbojpeg libjpeg-turbo8-dev

echo "Install OpenGL"
sudo apt-get install libglfw3-dev

echo "Install intel OpenCL drivers, if other GPU, please refer to lib freenect2 for more info"
sudo apt-get install beignet-dev

echo "Install VAAPI, for intel"
sudo apt-get install libva-dev libjpeg-dev

echo "Install OpeNI2"
sudo apt-get install libopenni2-dev

echo "Make build directory"
mkdir build && cd build

echo "Change to libfreenect2 build dir"
cd ~/libcreenect2/build

echo "Make using iai_kinect2 build command"
cmake .. -DENABLE_CXX11=ON
make
sudo make install 

echo "Clone and install iai_kinect2"
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2

echo "Modify CMakeLists.txt in kinect2_registration"
cd kinect2_registration
match='set(EXPORTED_DEPENDENCIES OpenCL)'
insert='add_definitions( -fexceptions )'
file='CMakeLists.txt'
sed -i "s/$match/$match\n$insert/" $file

echo "Build all"
cd
rosdep install -r --from-paths --rosdistro kinetic .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="kinetic"

echo "Seting up correct USB permissions"
sudo cp ~/catkin_ws/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/


