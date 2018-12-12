# Kuka_IIWA_Kitting_Cell

[![Build Status](https://travis-ci.org/bharatm11/kitting_cell.svg?branch=master)](https://travis-ci.org/bharatm11/kitting_cell)
[![Coverage Status](https://coveralls.io/repos/github/bharatm11/kitting_cell/badge.svg?branch=master)](https://coveralls.io/github/bharatm11/kitting_cell?branch=master)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)
---

## Overview
Today’s fulfillment centers and warehouse management systems have been eroding at product
delivery time by maximizing on their utilization of agile and autonomous systems. In this project
we propose a scenario where ACME needs to design an automated order fulfilment station for
mechanical kit parts. The station will be equipped with a seven D.O.F arm for pick and place and
an RGB camera for object identification and pose extraction. Parts will be cylindrical in shape and
will have multiple colors to resemble different types of products. Parts will be laid out on the table
for the arm. Based on the order received, the arm will pick the proper parts and place them in a
box.

## Solo Iterative Process (SIP)

Link to SIP Planning:
https://docs.google.com/spreadsheets/d/1LIiShhtAPbntBMWH1PGLz5oI0wvwpbRwA-c3s_XedA4/edit#gid=0

Link to Sprint Planning Notes: https://docs.google.com/document/d/1iX81NFR4FATnFmHgEvva8LaHFodJwQEUFGGf4ZuHZtY/edit?usp=sharing

## Video Presentation

The link to the video presentation and demo of the project will be posted here on project completion.

## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu 16.04
* ROS Kinetic Kame
* Gazebo 7.x (part of ros-kinetic-desktop-full package)
* openCV

#### ROS Kinetic
* To install ROS Kinetic in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

* To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

#### openCV

Install OpenCV 3.3.0 using the following commands:

Install OpenCV Dependencies
```
sudo apt-get install build-essential checkinstall cmake pkg-config yasm gfortran git
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
## If you are using Ubuntu 14.04
sudo apt-get install libtiff4-dev
## If you are using Ubuntu 16.04
sudo apt-get install libtiff5-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install libqt4-dev libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
```
Download and Compile OpenCV
```
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.0 
cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.0
cd ..
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
## find out number of CPU cores in your machine
nproc
## substitute 4 by output of nproc
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```
#### Other Dependencies
Make sure you have these packages installed in the environment:
* ros-kinetic-velocity-controllers
* ros-kinetic-ros-control
* ros-kinetic-position-controllers
* ros-kinetic-joint-state-controller
* ros-kinetic-joint-trajectory-controller

If not, type:
```
sudo apt-get install ros-kinetic-velocity-controllers ros-kinetic-ros-control ros-kinetic-position-controllers ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller
```
## Build Instructions

To build this code in a catkin workspace:
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/bharatm11/Kuka_IIWA_Kitting_Cell
cd ..
catkin_make
```
Note, that if you do not have a catkin workspace, then to build this code use the following commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/bharatm11/Kuka_IIWA_Kitting_Cell
cd ..
catkin_make
```
## Installation of additional packages

In your catkin workspace directory,
```
git clone --recursive https://github.com/bharatm11/iiwa_stack_kitting_cell
```

## Running the Demo using Launch File

This launch file loads the Gazebo environment and runs the node to detect the objects on a table and place them in bins based on the order.

**Note: This is an ongoing project and the following instructions may not be functional.** 


After following the build instructions, to run the demo, launch the code using the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch
```

##### CppCheck

To run cppcheck in Terminal
```
cd <path to directory>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./launch/" -e "^./results/" -e "./world/")
```
##### Google C++ Sytle (Cpplint)

To check Google C++ Style formatting in Terminal
```
cd <path to directory>
cpplint $(find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./launch/" -e "^./world/" -e "^./results")
```
## Known Issues/Bugs

This is an ongoing project and has no known issues or bugs.

## Generating Doxygen Documentation

To install doxygen run the following command: 

```
sudo apt install doxygen
cd <path to repository>
mkdir docs
doxygen -g config
```
Open the Doxygen configuration file "config" and update the following parameters:

PROJECT_NAME           = "Kuka_IIWA_Kitting_Cell"

INPUT                  = ./src ./include/ ./test

OUTPUT_DIRECTORY       = docs

Rename the config file as doxconfig

Now, to generate doxygen documentation, run the following commands:
```
doxygen doxconfig
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
* Delaunay triangulation S-hull license: Copyright 2016 Dr David Sinclair
* iiwa_stack license: Copyright (c) 2016-2017, Salvatore Virga - salvo.virga@tum.de
* kdl_ros license: Copyright (c) 2016-2017, Ruben Smits - ruben.smits@intermodalics.eu

## Disclaimer

This software is released under the GNU Lesser General Public License v3.0.
```
                   GNU LESSER GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.


  This version of the GNU Lesser General Public License incorporates
the terms and conditions of version 3 of the GNU General Public
License, supplemented by the additional permissions listed below.

  0. Additional Definitions.

  As used herein, "this License" refers to version 3 of the GNU Lesser
General Public License, and the "GNU GPL" refers to version 3 of the GNU
General Public License.

  "The Library" refers to a covered work governed by this License,
other than an Application or a Combined Work as defined below.

  An "Application" is any work that makes use of an interface provided
by the Library, but which is not otherwise based on the Library.
Defining a subclass of a class defined by the Library is deemed a mode
of using an interface provided by the Library.

  A "Combined Work" is a work produced by combining or linking an
Application with the Library.  The particular version of the Library
with which the Combined Work was made is also called the "Linked
Version".

  The "Minimal Corresponding Source" for a Combined Work means the
Corresponding Source for the Combined Work, excluding any source code
for portions of the Combined Work that, considered in isolation, are
based on the Application, and not on the Linked Version.

  The "Corresponding Application Code" for a Combined Work means the
object code and/or source code for the Application, including any data
and utility programs needed for reproducing the Combined Work from the
Application, but excluding the System Libraries of the Combined Work.

  1. Exception to Section 3 of the GNU GPL.

  You may convey a covered work under sections 3 and 4 of this License
without being bound by section 3 of the GNU GPL.

  2. Conveying Modified Versions.

  If you modify a copy of the Library, and, in your modifications, a
facility refers to a function or data to be supplied by an Application
that uses the facility (other than as an argument passed when the
facility is invoked), then you may convey a copy of the modified
version:

   a) under this License, provided that you make a good faith effort to
   ensure that, in the event an Application does not supply the
   function or data, the facility still operates, and performs
   whatever part of its purpose remains meaningful, or

   b) under the GNU GPL, with none of the additional permissions of
   this License applicable to that copy.

  3. Object Code Incorporating Material from Library Header Files.

  The object code form of an Application may incorporate material from
a header file that is part of the Library.  You may convey such object
code under terms of your choice, provided that, if the incorporated
material is not limited to numerical parameters, data structure
layouts and accessors, or small macros, inline functions and templates
(ten or fewer lines in length), you do both of the following:

   a) Give prominent notice with each copy of the object code that the
   Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the object code with a copy of the GNU GPL and this license
   document.

  4. Combined Works.

  You may convey a Combined Work under terms of your choice that,
taken together, effectively do not restrict modification of the
portions of the Library contained in the Combined Work and reverse
engineering for debugging such modifications, if you also do each of
the following:

   a) Give prominent notice with each copy of the Combined Work that
   the Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the Combined Work with a copy of the GNU GPL and this license
   document.

   c) For a Combined Work that displays copyright notices during
   execution, include the copyright notice for the Library among
   these notices, as well as a reference directing the user to the
   copies of the GNU GPL and this license document.

   d) Do one of the following:

       0) Convey the Minimal Corresponding Source under the terms of this
       License, and the Corresponding Application Code in a form
       suitable for, and under terms that permit, the user to
       recombine or relink the Application with a modified version of
       the Linked Version to produce a modified Combined Work, in the
       manner specified by section 6 of the GNU GPL for conveying
       Corresponding Source.

       1) Use a suitable shared library mechanism for linking with the
       Library.  A suitable mechanism is one that (a) uses at run time
       a copy of the Library already present on the user's computer
       system, and (b) will operate properly with a modified version
       of the Library that is interface-compatible with the Linked
       Version.

   e) Provide Installation Information, but only if you would otherwise
   be required to provide such information under section 6 of the
   GNU GPL, and only to the extent that such information is
   necessary to install and execute a modified version of the
   Combined Work produced by recombining or relinking the
   Application with a modified version of the Linked Version. (If
   you use option 4d0, the Installation Information must accompany
   the Minimal Corresponding Source and Corresponding Application
   Code. If you use option 4d1, you must provide the Installation
   Information in the manner specified by section 6 of the GNU GPL
   for conveying Corresponding Source.)

  5. Combined Libraries.

  You may place library facilities that are a work based on the
Library side by side in a single library together with other library
facilities that are not Applications and are not covered by this
License, and convey such a combined library under terms of your
choice, if you do both of the following:

   a) Accompany the combined library with a copy of the same work based
   on the Library, uncombined with any other library facilities,
   conveyed under the terms of this License.

   b) Give prominent notice with the combined library that part of it
   is a work based on the Library, and explaining where to find the
   accompanying uncombined form of the same work.

  6. Revised Versions of the GNU Lesser General Public License.

  The Free Software Foundation may publish revised and/or new versions
of the GNU Lesser General Public License from time to time. Such new
versions will be similar in spirit to the present version, but may
differ in detail to address new problems or concerns.

  Each version is given a distinguishing version number. If the
Library as you received it specifies that a certain numbered version
of the GNU Lesser General Public License "or any later version"
applies to it, you have the option of following the terms and
conditions either of that published version or of any later version
published by the Free Software Foundation. If the Library as you
received it does not specify a version number of the GNU Lesser
General Public License, you may choose any version of the GNU Lesser
General Public License ever published by the Free Software Foundation.

  If the Library as you received it specifies that a proxy can decide
whether future versions of the GNU Lesser General Public License shall
apply, that proxy's public statement of acceptance of any version is
permanent authorization for you to choose that version for the
Library.
```














