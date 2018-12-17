/*
*                     GNU LESSER GENERAL PUBLIC LICENSE
*                         Version 3, 29 June 2007
*
*   Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
*   Everyone is permitted to copy and distribute verbatim copies
*   of this license document, but changing it is not allowed.
*
*   This version of the GNU Lesser General Public License incorporates
* the terms and conditions of version 3 of the GNU General Public
* License, supplemented by the additional permissions listed below.
*
*   0. Additional Definitions.
*
*   As used herein, "this License" refers to version 3 of the GNU Lesser
* General Public License, and the "GNU GPL" refers to version 3 of the GNU
* General Public License.
*
*   "The Library" refers to a covered work governed by this License,
* other than an Application or a Combined Work as defined below.
*
*   An "Application" is any work that makes use of an interface provided
* by the Library, but which is not otherwise based on the Library.
* Defining a subclass of a class defined by the Library is deemed a mode
* of using an interface provided by the Library.
*
*   A "Combined Work" is a work produced by combining or linking an
* Application with the Library.  The particular version of the Library
* with which the Combined Work was made is also called the "Linked
* Version".
*
*   The "Minimal Corresponding Source" for a Combined Work means the
* Corresponding Source for the Combined Work, excluding any source code
* for portions of the Combined Work that, considered in isolation, are
* based on the Application, and not on the Linked Version.
*
*   The "Corresponding Application Code" for a Combined Work means the
* object code and/or source code for the Application, including any data
* and utility programs needed for reproducing the Combined Work from the
* Application, but excluding the System Libraries of the Combined Work.
*
*   1. Exception to Section 3 of the GNU GPL.
*
*   You may convey a covered work under sections 3 and 4 of this License
* without being bound by section 3 of the GNU GPL.
*
*   2. Conveying Modified Versions.
*
*   If you modify a copy of the Library, and, in your modifications, a
* facility refers to a function or data to be supplied by an Application
* that uses the facility (other than as an argument passed when the
* facility is invoked), then you may convey a copy of the modified
* version:
*
*    a) under this License, provided that you make a good faith effort to
*    ensure that, in the event an Application does not supply the
*    function or data, the facility still operates, and performs
*    whatever part of its purpose remains meaningful, or
*
*    b) under the GNU GPL, with none of the additional permissions of
*    this License applicable to that copy.
*
*   3. Object Code Incorporating Material from Library Header Files.
*
*   The object code form of an Application may incorporate material from
* a header file that is part of the Library.  You may convey such object
* code under terms of your choice, provided that, if the incorporated
* material is not limited to numerical parameters, data structure
* layouts and accessors, or small macros, inline functions and templates
* (ten or fewer lines in length), you do both of the following:
*
*    a) Give prominent notice with each copy of the object code that the
*    Library is used in it and that the Library and its use are
*    covered by this License.
*
*    b) Accompany the object code with a copy of the GNU GPL and this license
*    document.
*
*   4. Combined Works.
*
*   You may convey a Combined Work under terms of your choice that,
* taken together, effectively do not restrict modification of the
* portions of the Library contained in the Combined Work and reverse
* engineering for debugging such modifications, if you also do each of
* the following:
*
*    a) Give prominent notice with each copy of the Combined Work that
*    the Library is used in it and that the Library and its use are
*    covered by this License.
*
*    b) Accompany the Combined Work with a copy of the GNU GPL and this license
*    document.
*
*    c) For a Combined Work that displays copyright notices during
*    execution, include the copyright notice for the Library among
*    these notices, as well as a reference directing the user to the
*    copies of the GNU GPL and this license document.
*
*    d) Do one of the following:
*
*        0) Convey the Minimal Corresponding Source under the terms of this
*        License, and the Corresponding Application Code in a form
*        suitable for, and under terms that permit, the user to
*        recombine or relink the Application with a modified version of
*        the Linked Version to produce a modified Combined Work, in the
*        manner specified by section 6 of the GNU GPL for conveying
*        Corresponding Source.
*
*        1) Use a suitable shared library mechanism for linking with the
*        Library.  A suitable mechanism is one that (a) uses at run time
*        a copy of the Library already present on the user's computer
*        system, and (b) will operate properly with a modified version
*        of the Library that is interface-compatible with the Linked
*        Version.
*
*    e) Provide Installation Information, but only if you would otherwise
*    be required to provide such information under section 6 of the
*    GNU GPL, and only to the extent that such information is
*    necessary to install and execute a modified version of the
*    Combined Work produced by recombining or relinking the
*    Application with a modified version of the Linked Version. (If
*    you use option 4d0, the Installation Information must accompany
*    the Minimal Corresponding Source and Corresponding Application
*    Code. If you use option 4d1, you must provide the Installation
*    Information in the manner specified by section 6 of the GNU GPL
*    for conveying Corresponding Source.)
*
*   5. Combined Libraries.
*
*   You may place library facilities that are a work based on the
* Library side by side in a single library together with other library
* facilities that are not Applications and are not covered by this
* License, and convey such a combined library under terms of your
* choice, if you do both of the following:
*
*    a) Accompany the combined library with a copy of the same work based
*    on the Library, uncombined with any other library facilities,
*    conveyed under the terms of this License.
*
*    b) Give prominent notice with the combined library that part of it
*    is a work based on the Library, and explaining where to find the
*    accompanying uncombined form of the same work.
*
*   6. Revised Versions of the GNU Lesser General Public License.
*
*   The Free Software Foundation may publish revised and/or new versions
* of the GNU Lesser General Public License from time to time. Such new
* versions will be similar in spirit to the present version, but may
* differ in detail to address new problems or concerns.
*
*   Each version is given a distinguishing version number. If the
* Library as you received it specifies that a certain numbered version
* of the GNU Lesser General Public License "or any later version"
* applies to it, you have the option of following the terms and
* conditions either of that published version or of any later version
* published by the Free Software Foundation. If the Library as you
* received it does not specify a version number of the GNU Lesser
* General Public License, you may choose any version of the GNU Lesser
* General Public License ever published by the Free Software Foundation.
*
*   If the Library as you received it specifies that a proxy can decide
* whether future versions of the GNU Lesser General Public License shall
* apply, that proxy's public statement of acceptance of any version is
* permanent authorization for you to choose that version for the
* Library.
*/
/**
* @file main.cpp
* @author Bharat Mathur [bharatm11]
* @author Royneal Rayess [royneal]
* @date 15 Dec 2018
* @copyright 2018 Bharat Mathur, Royneal Rayess
* @brief This is the main file of the kitting_cell package to perform color
*        based kitting of items with a KUKA IIWA manipulator equipped with a vacuum gripper.
*/

#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include<iostream>
#include<kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Perception.hpp"
#include "Grip.hpp"
#include "kuka.hpp"


int main(int argc, char **argv) {
  ros::init(argc, argv, "kuka");
  ros::Time::init();
  int loop_freq = 100;
  float dt = static_cast<float>(1/loop_freq);
  ros::Rate loop_rate(loop_freq);
  ROS_INFO_STREAM("Hi");
  kuka ku;
  ros::NodeHandle n("~");
    ros::Duration(5).sleep();
  auto joints_sub = n.subscribe("/iiwa/joint_states", 10,
                                                      &kuka::getJoints, &ku);
  std::string command_topic =
                  "/iiwa/PositionJointInterface_trajectory_controller/command";
  ros::Publisher cmd_pub =
                n.advertise<trajectory_msgs::JointTrajectory>(command_topic, 1);
  Perception d;
  Grip g;
  std::string colorInput;
  n.getParam("colorInput", colorInput);
  g.ToggleState(true);
  KDL::Frame cartpos;
  KDL::JntArray inv;
  ros::Duration(1).sleep();
  ros::spinOnce();
  KDL::JntArray jointpositions_new;
  trajectory_msgs::JointTrajectoryPoint pt;
  trajectory_msgs::JointTrajectory cmd;
  cmd = ku.driveRobot(ku.initializeHomePos());
  cmd_pub.publish(cmd);
  ros::Duration(3).sleep();

  KDL::Frame cartpos2;
  double cyl2_pick_traj[3][3] = {{0.429094, -0.309222, 0.720299},
                              {0.429094, -0.309222, 0.520299},
                              {0.429094, -0.309222, 0.720299}};
  double cyl2_drop_traj[3][3]={{0.429094, 0.17086, 0.720299},
                              {0.429094, 0.17086, 0.520299},
                              {0.429094, 0.17086, 0.720299}};
  double cyl0_pick_traj[3][3]={{0.490316, -0.216214, 0.720299},
                              {0.490316, -0.216214, 0.520299},
                              {0.490316, -0.216214, 0.720299}};
  double cyl0_drop_traj[3][3]={{0.526436, 0.187901, 0.720299},
                              {0.526436, 0.187901, 0.520299},
                              {0.526436, 0.187901, 0.720299}};
  double cyl1_pick_traj[3][3]={{0.34644, -0.188354, 0.720299},
                              {0.34644, -0.188354, 0.520299},
                              {0.34644, -0.188354, 0.720299}};
  double cyl1_drop_traj[3][3]={{0.346466, 0.276881, 0.720299},
                              {0.346466, 0.276881, 0.520299},
                              {0.346466, 0.276881, 0.720299}};

  KDL::Rotation rpy = KDL::Rotation::RPY(-3.14154, 0.0013407, 3.1415);


int f = d.colorThresholder(colorInput);

while (ros::ok()) {
    if (f == 2) {
      ROS_INFO_STREAM("Cylinder number"<< f+3 << "is" << colorInput);
      for (int i =0; i < 3; i++) {
        cartpos2.p[0] = cyl2_pick_traj[i][0];
        cartpos2.p[1] = cyl2_pick_traj[i][1];
        cartpos2.p[2] = cyl2_pick_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd = ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if (i == 1) {
          g.ToggleState(false);
        }
      }
      for (int i = 0; i < 3; i++) {
        cartpos2.p[0] = cyl2_drop_traj[i][0];
        cartpos2.p[1] = cyl2_drop_traj[i][1];
        cartpos2.p[2] = cyl2_drop_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd = ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if (i == 1) {
          g.ToggleState(true);
        }
      }
      cmd = ku.driveRobot(ku.initializeHomePos());
      cmd_pub.publish(cmd);
      ros::Duration(3).sleep();
      ROS_INFO_STREAM("PART KITTED SUCCESFULLY!!! KILLING NODE :)");
      ros::shutdown();
    } else if (f == 1) {
      for (int i = 0; i < 3; i++) {
        cartpos2.p[0] = cyl1_pick_traj[i][0];
        cartpos2.p[1] = cyl1_pick_traj[i][1];
        cartpos2.p[2] = cyl1_pick_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd = ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if (i == 1) {
          g.ToggleState(false);
        }
      }
      for (int i = 0; i < 3; i++) {
        cartpos2.p[0] = cyl1_drop_traj[i][0];
        cartpos2.p[1] = cyl1_drop_traj[i][1];
        cartpos2.p[2] = cyl1_drop_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd = ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if (i == 1) {
          g.ToggleState(true);
        }
      }
      cmd = ku.driveRobot(ku.initializeHomePos());
      cmd_pub.publish(cmd);
      ros::Duration(3).sleep();
      ROS_INFO_STREAM("PART KITTED SUCCESFULLY!!! KILLING NODE :)");
      ros::shutdown();
    } else if (f == 0) {
      for (int i = 0; i < 3; i++) {
        cartpos2.p[0] = cyl0_pick_traj[i][0];
        cartpos2.p[1] = cyl0_pick_traj[i][1];
        cartpos2.p[2] = cyl0_pick_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd = ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if (i == 1) {
          g.ToggleState(false);
        }
      }
      for (int i = 0; i < 3; i++) {
        cartpos2.p[0] = cyl0_drop_traj[i][0];
        cartpos2.p[1] = cyl0_drop_traj[i][1];
        cartpos2.p[2] = cyl0_drop_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd = ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if (i == 1) {
          g.ToggleState(true);
        }
      }
      cmd = ku.driveRobot(ku.initializeHomePos());
      cmd_pub.publish(cmd);
      ros::Duration(3).sleep();
      ROS_INFO_STREAM("PART KITTED SUCCESFULLY!!! KILLING NODE :)");
      ros::shutdown();
    } else {
      ROS_INFO_STREAM("INVALID COLOR INPUT");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
