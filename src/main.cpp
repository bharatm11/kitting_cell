#include<iostream>
#include "kuka.hpp"

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/JointState.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<kdl/chain.hpp>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "Perception.hpp"
int main(int argc, char **argv) {
  ros::init(argc, argv, "kuka");
  ros::Time::init();
  int loop_freq = 100;
  float dt = (float) 1/loop_freq;
  ros::Rate loop_rate(loop_freq);
  ROS_INFO_STREAM("Hi");
  kuka ku;

  ros::NodeHandle n;
  auto joints_sub = n.subscribe("/iiwa/joint_states",10,  &kuka::getJoints, &ku);
  std::string command_topic = "/iiwa/PositionJointInterface_trajectory_controller/command";
  ros::Publisher cmd_pub = n.advertise<trajectory_msgs::JointTrajectory>(command_topic,1);
Perception d;
  KDL::Frame cartpos;
  ROS_INFO_STREAM("Hi");
  KDL::JntArray inv;
  ros::Duration(1).sleep();
  ros::spinOnce();
  KDL::JntArray jointpositions_new;
  trajectory_msgs::JointTrajectoryPoint pt;
  trajectory_msgs::JointTrajectory cmd;

  while (ros::ok()) {
    d.colorThresholder("red");
    cartpos = ku.evalKinematicsFK();
    jointpositions_new=ku.evalKinematicsIK(cartpos);
    trajectory_msgs::JointTrajectoryPoint pt = ku.normalizePoints(jointpositions_new);
    cmd = ku.driveRobot(pt);
    cmd_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
