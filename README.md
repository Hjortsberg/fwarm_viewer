# D0020E-FWARM-ROS
Repository for the course D0020E, project in computer science. 

## Table of content  
- [Introduction](#introduction)  
- [Install](#install)
  * [Install with script](#installation-script)  
- [Usage](#usage)  
- [Future improvements](#future-improvements)
## Introduction  
This viewer is a modified fork of Wiedemeyers kinect2_viewer from the https://github.com/code-iai/iai_kinect2 package.

The main goal of this project was to be explore how you could use a 3D-LiDAR to improve the navigation of heavy veihcles.
To be able to test with a 3D-LiDAR we obtained a kinect one(2).
The iai_kinect2 package contians a bridge between the libfreenect kinectdriver and the ROS universe. This bridge publishes pointclouds, IR-images and color images. Our conclusion was that the pointcloud representation of LiDAR data is to bandwitdh intensive to be sent over a cellular network and thus was not needed in our implementation and thereby we stripped the viewer of it's capabilites to recieve and visualize pointclouds. 

The initial design goals was ability to:
* subscribe to a arbitrary ros imagefeed (videofeed) in order to be compatible with general cameras.  
* be able to merge a desparity image (color coded depth image) with the normal picture stream from a arbitrary video feed.  

The project then matured and the goals shifted a bit to more experimantal features. 
* The viewer is now able to show videos on multiple monitors. This was added since the current remote controller stations already had multiple monitors and need to show the different camera feeds at different monitors.
* The viewer does not just blindly overlay the disparity image anymore. After the inital implementation we quickly understood that it was completly unusable and would certanly not improove driving capabilities. Now, the dispariy image does only get overlaid at areas where a human is detected to improve awareness and to alert the driver. 
* The viewer now takes speed in consideration for highlighting humans. Since you might not want to high light all humans the computer detects in order to not clutter the screen. This is fixed with the "adaptive safe distance", a function that increases the range for detection the faster you drive due to longer brakeing distance and shorter reaction times to avoid collisions.  
* Full screen without borders, in a real world scenario, borders around your videostreams would make no sense.
* Ability to resize and change aspectratio for the incoming video feeds. In order to make the overlaying simpler and to be able to subscribe to arbitrary video feeds. The program can't merge if it considers the LiDAR feed and camera feed different in resolution. Kind of like trying to fit a square into a round hole.
* The ability to change namespace via commandline. The namespace at which the ROS topics are published to. 

## Installation with script
* Download the script through your terminal with wget in your home directory
```
wget https://raw.githubusercontent.com/Hjortsberg/fwarm_viewer/master/install_fwarm_ros.sh
```
* Make the script executable, do __NOT__ run as sudo. Then execute with the ./ command.
```
chmod +x install_fwarm_ros.sh
./install_fwarm_ros.sh
```
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
* Install VAAPI driver
```
sudo apt-get install i965-va-driver
```
* install wrapper for VAAPI
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
* Paste the following line, you do this in vim by pressing i, [ctrl + shift + v] then esc.
```
add_definitions( -fexceptions )
```
Dont forget to :wq (vim command for write and quit).
```
cd ..
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="kinetic"
```  
install a usbcam package (optional)
To get supported ```usb_cam``` resolutions and framerates: 
```
v4l2-ctl --list-formats-ext
```

## Usage
You now have a complete development environment.
For notes on how to test your kinect installation see https://github.com/OpenKinect/libfreenect2/.
Remember to connect it to a USB3.0 controller with nothing else plugged in to it, libfreenect lists compatible controllers.

Start a shell
1. `roscore` you must always have a roscore running when wanting to interact via ROS.

Start a new shell  
2. `cd ~/catkin_ws` Navigate to the workspace  
3. `source devel/setup.bash` source the shell, if your using zsh, swap .bash to .zsh  
4. `rosrun kinect2_bridge kinect2_bridge`Start the kinect bridge.  

Start a third shell  
5. repeat step 2 and 3 in the new shell.  
6. TODO: insert commands to start the velocity simulator node.  

Start a fourth shell  
7. repeat step 2 and 3 in the new shell.  
8. `rosrun fwarm_viewer fwarm_viewer approx` approx is currently needed to sync the videostreams with the simulated velocity.
9. You should now be presented with a full screen viewer that detects humans with a disparity iamge overlay with simulated velocity! To exit either press q or ctrl + c

TODO: add tutorial on how to start with startup script  

## Future improvements  
The viewer is still far from optimal though. Here are som things that could be improved.  
* Building the viewer for a more real life scenario by using CUDA accelerated opencv functions. We suspect that this will probably be one of the greatest performanceboosts one can make.
* Implementing auto detection of number of screens, their orientation and resolution for autoconfiguring the multimonitor part.
* Implementing the ability for the viewer to subscribe to all the neccesary feeds. right now it can handle 2 feeds, one for color camera and one for lidar disparity image. What's missing is probably one left, one right and one back camera, so 3 addiditonal image feed subscribers.
* Tune the HOG paramters for better human detection.
* Better selection masking when highlighting humans, rightnow it's a box. There is things like canny edge detectors that can find the edges which could leave you with a nice outer glow around the human instead.
* Carefully looking at the opencv methods, using as few copyto functions as possible inorder to reduce the image proccessing time. 
