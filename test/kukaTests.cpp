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
  * @file kukaTests.cpp
  *  @author Royneal Rayess [royneal] - driver
  * @author Bharat Mathur [bharatm11] - navigator
  * @date 13 Dec 2018
  * @copyright 2018 Royneal Rayess, Bharat Mathur
  * @brief This is the file for testing the methods of the kuka class
*/

// Bring in gtest
#include <gtest/gtest.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "kuka.hpp"

TEST(initializeTrajectoryPointTest, CheckJointNames) {
  ros::Time::init();
  trajectory_msgs::JointTrajectory obj;  // initialize to joint names
  kuka robot;
  obj = robot.initializeTrajectoryPoint();

  EXPECT_STREQ("iiwa_joint_1", obj.joint_names[0].c_str());
  EXPECT_STREQ("iiwa_joint_2", obj.joint_names[1].c_str());
  EXPECT_STREQ("iiwa_joint_3", obj.joint_names[2].c_str());
  EXPECT_STREQ("iiwa_joint_4", obj.joint_names[3].c_str());
  EXPECT_STREQ("iiwa_joint_5", obj.joint_names[4].c_str());
  EXPECT_STREQ("iiwa_joint_6", obj.joint_names[5].c_str());
  EXPECT_STREQ("iiwa_joint_7", obj.joint_names[6].c_str());
  EXPECT_EQ(0, obj.header.seq);
}

TEST(initializeTrajectoryPointTest, CheckTrajectoryheader) {
  ros::Time::init();
  trajectory_msgs::JointTrajectory obj;  // initialize to joint names
  kuka robot;
  obj = robot.initializeTrajectoryPoint();
  EXPECT_EQ(0, obj.header.seq);
}

TEST(initializeHomePosTest, CheckHomePos) {
  ros::Time::init();
  trajectory_msgs::JointTrajectoryPoint joint;
  kuka robot;
  joint = robot.initializeHomePos();
  EXPECT_EQ(0, joint.positions[0]);
  EXPECT_EQ(0, joint.positions[1]);
  EXPECT_EQ(0, joint.positions[2]);
  EXPECT_EQ(-1.57, joint.positions[3]);
  EXPECT_EQ(0, joint.positions[4]);
  EXPECT_EQ(1.57, joint.positions[5]);
  EXPECT_EQ(0, joint.positions[6]);
}

TEST(initializeJointsSubTest, CheckJointSub) {
  ros::Time::init();
  sensor_msgs::JointState obj;  // initialize to joint names
  kuka robot; ros::NodeHandle nh_;
  obj = robot.initializeJointsSub();
  EXPECT_EQ(0.2, obj.position[0]);
}

TEST(initializeJointsKDLTest, CheckJointsKDL) {
  ros::Time::init();
  KDL::JntArray obj;  // initialize to joint names
  kuka robot;
  obj = robot.initializeJointsKDL();
  EXPECT_EQ(0.2, obj(0));
}

TEST(makeChainTest, CheckChain) {
  ros::Time::init();
  KDL::Chain obj;  // initialize to joint names
  kuka robot;
  obj = robot.makeChain();
  int segmnt = obj.getNrOfSegments();
  EXPECT_EQ(8, segmnt);
}

TEST(EvaluateKinematics, CheckFK) {
  ros::Time::init();
  kuka ku;
  KDL::Frame obj = ku.evalKinematicsFK();
  std::cout << obj.p[0];
  EXPECT_NEAR(0.104714, obj.p[0], 0.1);
}

TEST(GetJointNumbersTest, CheckgetJointNums) {
  ros::Time::init();
  kuka robot;
  unsigned int obj = robot.getJointNums();
  EXPECT_EQ(7, obj);
}

TEST(normalizePointsTest, ChecknormalizedPoints) {
  ros::Time::init();
  kuka ku;
  KDL::JntArray joints = ku.initializeJointsKDL();
  trajectory_msgs::JointTrajectoryPoint obj;
  obj = ku.normalizePoints(joints);
  EXPECT_NEAR(0.2, obj.positions[1], 0.0001);
}

TEST(EvaluateKinematicsTest, CheckIK) {
  ros::Time::init();
  kuka ku;
  KDL::Frame cartpos = ku.evalKinematicsFK();
  KDL::JntArray jointpositions_new = ku.evalKinematicsIK(cartpos);
  EXPECT_NEAR(0.2, jointpositions_new(0), 0.001);
}

TEST(driveRobotTest, names) {
  ros::Time::init();
  kuka ku;
  KDL::Frame cartpos = ku.evalKinematicsFK();
  KDL::JntArray jointpositions_new = ku.evalKinematicsIK(cartpos);
  trajectory_msgs::JointTrajectoryPoint obj =
                                        ku.normalizePoints(jointpositions_new);
  trajectory_msgs::JointTrajectory obj2 = ku.driveRobot(obj);
  EXPECT_STREQ("iiwa_joint_1", obj2.joint_names[0].c_str());
  EXPECT_STREQ("iiwa_joint_2", obj2.joint_names[1].c_str());
  EXPECT_STREQ("iiwa_joint_3", obj2.joint_names[2].c_str());
  EXPECT_STREQ("iiwa_joint_4", obj2.joint_names[3].c_str());
  EXPECT_STREQ("iiwa_joint_5", obj2.joint_names[4].c_str());
  EXPECT_STREQ("iiwa_joint_6", obj2.joint_names[5].c_str());
  EXPECT_STREQ("iiwa_joint_7", obj2.joint_names[6].c_str());
}

TEST(driveRobotTest, properties) {
  ros::Time::init();
  kuka ku;
  KDL::Frame cartpos = ku.evalKinematicsFK();
  KDL::JntArray jointpositions_new = ku.evalKinematicsIK(cartpos);
  trajectory_msgs::JointTrajectoryPoint obj =
                                        ku.normalizePoints(jointpositions_new);
  trajectory_msgs::JointTrajectory obj2 = ku.driveRobot(obj);
  EXPECT_EQ(0, obj2.header.seq);
}

TEST(returnCurrJointsTest, joints) {
  ros::Time::init();
  kuka ku;
  KDL::JntArray obj =  ku.returnCurrJoints();
  EXPECT_NEAR(0.2, obj(0), 0.001);
}
