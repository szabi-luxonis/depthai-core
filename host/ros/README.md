# depthai ROS driver

## Content description

- **depthai** - contains the code for the ros node
- **lib** - C++ library (static library for now, currently there are some issues with `.so` shared library version)   

## Tested platforms

- Ubuntu 18.04, ROS Melodic;


## Setup

- Install development environement dependencies for depthai from here:

      https://github.com/luxonis/depthai-api

- Install ROS Melodic following the instructions from http://wiki.ros.org/melodic/Installation/Ubuntu
 including 17.Dependencies for building packages

 - Logout and login


## Build and run

- **library**
- From ros dir:
      `cd lib`
      `mkdir build`
      `cd build`
      `cmake ..`
      `make`
      `sudo make install`

  Alternatively you can test the library instalation bulding and running the binary from test directory:
      `cd test`
      `mkdir build`
      `cd build`
      `cmake ..`
      `./libdepthaitest`

- **ros driver/node**  
- From ros dir:
      `mkdir -p ~/catkin_ws/src`
      `cp -r depthai ~/catkin_ws/src`
      `cd ~/catkin_ws/`
      `catkin_make`
      `catkin_make install`
      `source devel/setup.bash`
- From another terminal start roscore with `roscore` command.
- Now from build terminal launch the node with `roslaunch depthai_node depthai.launch node_name:=depthai_node_01`
- Execute `rviz` from the 3rd terminal to visualize the image. Use "Add" button or "Ctrl+N" select "By topic" and select the Image(not Camera).
- Device canfiguration can be changed in `depth_ai.launch` file.


