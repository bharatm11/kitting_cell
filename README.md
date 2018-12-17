# Kuka_IIWA_Kitting_Cell

[![Build Status](https://travis-ci.org/bharatm11/kitting_cell.svg?branch=master)](https://travis-ci.org/bharatm11/kitting_cell)
[![Coverage Status](https://coveralls.io/repos/github/bharatm11/kitting_cell/badge.svg?branch=master)](https://coveralls.io/github/bharatm11/kitting_cell?branch=master)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)
---

## Overview
Today’s fulfillment centers and warehouse management systems have been eroding at product
delivery time by maximizing on their utilization of agile and autonomous systems. In this project
we propose a scenario where ACME needs to design an automated order fulfilment station for
mechanical kit parts. The station is equipped with a seven D.O.F arm for pick and place and
an RGB camera for object identification and pose extraction. Parts are cylindrical in shape and have multiple colors to resemble different types of products. They are laid out on a table
for the arm. Based on the order received, the arm  picks the proper parts and places them at the desired locations. 

The developed system is capable of:
* Identifying parts based on color segmentation
* Pick and place in appropriate kits using a mnaipulator with a vacuum gripper
* Sort objects based on color and shape

 The code was checked for proper documentation,and google style guide was folloed. Throughout development cppcheck, cpplint, Travis ci build test, and coverages checks for tests were utilized as part of the toolchain to ensure code is properly written, tested, and is bug-free. Thecode was developed in C++11/14 and Atom was used as our IDE for development.
 
 A Solo Iterative Process (SIP) approach was followed in the development of this project over four sprints of seven days each.
 
 
 <p align="center">
<img src=https://github.com/bharatm11/kitting_cell/blob/updateReadme/results/cell.png>
</p>

## About the Developers

 * Bharat Mathur

 Bharat is a roboticist based in Maryland with a background in Controls & Embedded Systems for Medical Robots and Unmanned Aerial Systems. He is currently pursuing his master's degree in Robotics at the University of Maryland, College Park and is a graduate researcher in the Medical Robotics & Equipment Lab (MREL) at the University of Maryland under Dr. Axel Krieger. His research is focused on Human-Robot Interaction and haptics while integrating methodologies from robotics, control theory, artificial intelligence, machine vision, and human psychology to develop intelligent robots for deployment in medical and surgical applications. 
He received his bachelor's in Mechatronics Engineering from SRM University, Katlankulathur where his research was focused around civilian and industrial applications of Unmanned Aerial Systems (UAS). He was a founding member of the university's UAS research team, SRM-UAV during his freshman year. He worked as a Controls & Embedded Systems engineer and also led the team during his junior and senior years. While at SRM-UAV, he worked on a variety of aerial platforms such as rotary-wings, fixed-wings, tilt-rotors, and hybrids for applications like wind turbine inspection, mapping, surveillance, package delivery etc. More details about his work can be found at: https://www.mathurrobotics.com

 * Royneal Rayess

 Royneal is a recent robotics graduate from the University of Maryland, he earned his bachelors of science in electrical engineering in 2010 at the Grove School of Engineering at the City College of New York. As an undergrad, he worked on developing obstacle avoidance algorithms and participated in multiple Intelligent Ground Vehicle competitions. He also, in an independent study, investigated quadruped locomotion on a 28 DOF robot. His professional experience spans seven years of test engineering and test automation in renewable smart grid and iris biometrics industries. Currently, he is studying visual servoing techniques and implementation for six DOF manipulators using ROS. More details about his work can be found at: https://www.roybotics.com

## Solo Iterative Process (SIP)
A Solo Iterative Process (SIP) approach was followed in the development of this project over four sprints of seven days each.
The progress was ytracked using the following files:

Link to SIP Planning:
https://docs.google.com/spreadsheets/d/1LIiShhtAPbntBMWH1PGLz5oI0wvwpbRwA-c3s_XedA4/edit#gid=0

Link to Sprint Planning Notes: https://docs.google.com/document/d/1iX81NFR4FATnFmHgEvva8LaHFodJwQEUFGGf4ZuHZtY/edit?usp=sharing

## Video Presentation

Video presentations about the project can be found at:
* Project Overview: https://www.youtube.com/watch?v=sZZJj4xKO0Q&t=2s
* Installing and running instructions: https://www.youtube.com/watch?v=JFjWG8gV_GU&t=5s

## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu 16.04
* ROS Kinetic Kame
* Gazebo 7.x (part of ros-kinetic-desktop-full package)
* openCV

#### ROS Kinetic

* To install ROS Kinetic in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu). Make sure to install the recommended version (Desktop-Full).

* To install catkin, follow the installation steps for ROS kinetic in this [link](http://wiki.ros.org/catkin).

#### ROS Control Dependencies (REQUIRED)

Please install these **required** dependencies by running the following commands:
```
sudo apt-get install ros-kinetic-velocity-controllers ros-kinetic-ros-control
sudo apt-get install ros-kinetic-position-controllers ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-joint-trajectory-controller ros-kinetic-moveit
sudo apt install ros-kinetic-gazebo-ros-control
sudo apt install  ros-kinetic-kdl-conversions  ros-kinetic-kdl-parser-py ros-kinetic-kdl-parser ros-kinetic-kdl-typekit ros-kinetic-eigen-conversions 
```
## Installation of additional packages (IIWA_STACK)

This section install the IIWA_STACK package which is used to spawn the manipulator, gripper, world, and action trajectory server for the simulation.

**Note: This package spawns the robot, the action trajectory server, and the world in Gazebo. This is a mandatory package**

If you do not have a catkin workspace, then to make a new one, run the following:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
Following this, install the following package in the catkin workspace directory:

```
cd ~/catkin_ws/src/
git clone --recursive https://github.com/bharatm11/iiwa_stack_kitting_cell
cd ..
catkin_make
```
**If there are issues in building the package or spawning the robot in Gazebo, please refer to this wiki:** https://github.com/IFL-CAMP/iiwa_stack/wiki


## Build Instructions

To build this code in the previously built catkin workspace, run the following commands:
```
cd ~/catkin_ws/src/
git clone https://github.com/bharatm11/kitting_cell
cd ..
catkin_make
source devel/setup.bash
```

## Running the Demo using Launch File

This launch file loads the Gazebo environment and runs the kuka node to detect the objects on a table and place them at the desired locations. 
After bulding, the demo can be launched as:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch kitting_cell demo.launch 
```
__The color of the object to be placed in the kit is given as an argument while launching the demo. By default, the robot picks and places the red object. To change the color of the object, the demo can be launched in the following ways:__
For blue,
```
roslaunch kitting_cell demo.launch colorInput:=blue
```
For green,
```
roslaunch kitting_cell demo.launch colorInput:=green
```
For red,
```
roslaunch kitting_cell demo.launch colorInput:=red
```
## Record ROSBag File

A ros bag file records all the topic and messages being published in the terminal. After following the build instructions, to record the bag file, run the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch kitting_cell demo.launch record:=true
```
#### Playing the bag File Generated

To play the bag file, you need ROS master to be running. So, in a new terminal, run the following command:
```
roscore
```
Then, in a new terminal, run the following commands:
```
cd ~/catkin_ws/src/kitting_cell/
rosbag play <name_of_bag_file>
```
## Services

The vacuum gripper used has two services that are used to switch it ON and switch OFF.

To switch ON the gripper, call the service by running the following commands:
```
source ~/catkin_ws/devel/setup.bash
rosservice call /robot/left_vacuum_gripper/on
```
Similarly, to switch OFF the gripper, call the service by running the following command:
```
source ~/catkin_ws/devel/setup.bash
rosservice call /robot/left_vacuum_gripper/off
```
Note that, both these services are part of the IIWA_STACK package and hence, it is required to build the package using the above instructions and starting the simulation using the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_gazebo iiwa_gazebo.launch
```
## Testing

The kuka and Grip classes are tested using level 1 tests whereas the Perception class is tested by means of integration testing. 

The level 1 tests generate their own data and verify their their computation and functioning. They do not require any independent ROS nodes. 

The integration tests are conducted by running a headless simulator for rostest.

### 1) Run the Tests while Compiling the Code

You can run the tests while building the code by running the following command:
```
cd ~/catkin_ws/
catkin_make run_tests
```

### 2) Run the Tests using Launch File

After compiling using the above instructions, to run the tests independently using the launch file, use the following commands:
```
source ~/catkin_ws/devel/setup.bash
rostest kitting_cell nodetest.launch 
```
## Code Coverage
Line Coverage: ![Code coverage](https://img.shields.io/badge/coverage-85.7%25-green.svg) Function Coverage: ![Code coverage](https://img.shields.io/badge/coverage-100%25-green.svg)

(NOTE: The above tags are self-generated using locally obtained code coverage results. These are not to be confused with the tags generated from coveralls)


As integration tests are performed in this package which are directly dependent on a third-party package, IIWA_STACK, Coveralls has issues picking up the build from Travis-ci for code coverage testing. Hence, code coverage was tested locally using lcov. 


We recived a 100% function coverage and a 85.7% line coverage for the code. The results for the same are shown below:

<p align="center">
<img src=https://github.com/bharatm11/kitting_cell/blob/updateReadme/results/coverage.png>
</p>

```
cd ~/catkin_ws/build/
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*test_*' '*_test*' --output-file coverage.info
lcov --list coverage.info
```

## Plug-ins

##### CppCheck

If cppcheck package is not installed, run the following command:
```
sudo apt-get install cppcheck
```
To run cppcheck in Terminal, run the following commands:
```
cd ~/catkin_ws/src/kitting_cell/ 
cppcheck --enable=all --std=c++11 -I include/kitting_cell/ -I ../../../../../../../opt/ros/kinetic/include/ros/ -I ../../../../../../usr/include/opencv2/core/ --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

##### Google C++ Style

If cpplint package is not installed, run the following command:
```
sudo apt-get install python-pip
pip install cpplint
```
To check Google C++ Style formatting in Terminal, run the following commands:
```
cd ~/catkin_ws/src/kitting_cell/
cpplint $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./docs/" -e "^./launch/" -e "^./results/" -e "^./UML/" -e "./world/")
````
## Known Issues/Bugs

The current version of the code has no known issue/bugs that might affect the functionality. As far as testing is concerned, the package is unable to generate testing coverage using Coveralls.

## Generating Doxygen Documentation

To install doxygen run the following command: 
```
sudo apt install doxygen
```
To generate the documentation:
```
cd ~/catkin_ws/src/kitting_cell/
doxygen doxyConfig 
```
Doxygen files will be generated to /docs folder. To view them in a browser, run the following commands:
```
cd docs/html
google-chrome index.html
```
## License 

* OpenCV:  Copyright (C) 2015-2016, OpenCV Foundation, all rights reserved.
* Doxygen license: Copyright © 1997-2016 by Dimitri van Heesch.
* Googletest license: Copyright 2008, Google Inc.
* iiwa_stack license: Copyright (c) 2016-2017, Salvatore Virga - salvo.virga@tum.de
* kdl_ros license: Copyright (c) 2016-2017, Ruben Smits - ruben.smits@intermodalics.eu

## Disclaimer

This software is released under the GNU Lesser General Public License v3.0. Please read the following License Agreement carefully before using this program. The license can be found at the following link: https://www.gnu.org/licenses/lgpl-3.0
