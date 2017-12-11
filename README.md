# D0020E-FWARM-ROS
Repository for the course D0020E, project in computer science. 

## Table of contents
- [Install](#install)

## Install
Steps to get a lab environment up and running.

#### Install ROS
This is the mandatory commands, for references see http://wiki.ros.org/kinetic/Installation/Ubuntu .
* Update sources.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
```
* Set up keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
* Update Ubuntu 
```
sudo apt-get update
```
* Install ROS
```
sudo apt-get install ros-kinetic-desktop-full
```
* Initialize rosdep
``` 
sudo rosdep init
rosdep update 
```
* Environment setup
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
* Configure workspace
```
source /opt/ros/<distro>/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
#### Install libfreenect2
If unclear, please refer to https://github.com/OpenKinect/libfreenect2
* Download libfreenect2 from source
```
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
```
* Install build tools
```
sudo apt-get install build-essential cmake pkg-config
```
* Install libusb. The version must be >= 1.0.20. 
```
sudo apt-get install libusb-1.0-0-dev
```
* Install TurboJPG
```
sudo apt-get install libturbojpeg libjpeg-turbo8-dev
```
* Install OpenGL 
```
sudo apt-get install libglfw3-dev
```
* Install OpenCL 
If you use other gpu than Intel, please refer to https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux
```
sudo apt-get install beignet-dev
```
* Install VAAPI
```
sudo apt-get install libva-dev libjpeg-dev
```
* Install OpenNI2
```
sudo apt-get install libopenni2-dev
```
* Make build directory
```
mkdir build && cd build
```

* Change to libfreenect2 build dir
```
cd ~/libcreenect2/build
```
* Make using iai_kinect2 build command
```
cmake .. -DENABLE_CXX11=ON
make
sudo make install 
```
* Clone and install
```
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
```
* Edit CMakeLists.txt in kinect2_registration
```
cd kinect2_registration
vim +56 CMakeLists.txt
```
* Paste the following line
```
add_definitions( -fexceptions )
```
Dont forget to :wq.
```
cd ..
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="kinetic"
```
