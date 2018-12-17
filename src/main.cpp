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
#include <kdl/frames.hpp>
#include "Perception.hpp"
#include "Grip.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kuka");
  ros::Time::init();
  int loop_freq = 100;
  float dt = (float) 1/loop_freq;
  ros::Rate loop_rate(loop_freq);
  ROS_INFO_STREAM("Hi");
  kuka ku;
  ros::NodeHandle n("~");
    ros::Duration(5).sleep();
  auto joints_sub = n.subscribe("/iiwa/joint_states",10,  &kuka::getJoints, &ku);
  std::string command_topic = "/iiwa/PositionJointInterface_trajectory_controller/command";
  ros::Publisher cmd_pub = n.advertise<trajectory_msgs::JointTrajectory>(command_topic,1);
  Perception d;
  Grip g;
  std::string colorInput;
  n.getParam("colorInput",colorInput);
  g.ToggleState(true);
  KDL::Frame cartpos;
  KDL::JntArray inv;
  ros::Duration(1).sleep();
  ros::spinOnce();
  KDL::JntArray jointpositions_new;
  trajectory_msgs::JointTrajectoryPoint pt;
  trajectory_msgs::JointTrajectory cmd;


  cmd=ku.driveRobot(ku.initializeHomePos());
  cmd_pub.publish(cmd);
  ros::Duration(3).sleep();

  KDL::Frame cartpos2;
  double cyl2_pick_traj[3][3]={{0.429094,-0.309222,0.720299},
                              {0.429094,-0.309222,0.520299},
                              {0.429094,-0.309222,0.720299}};
  double cyl2_drop_traj[3][3]={{0.429094,0.17086,0.720299},
                              {0.429094,0.17086,0.520299},
                              {0.429094,0.17086,0.720299}};
  double cyl0_pick_traj[3][3]={{0.490316,-0.216214,0.720299},
                              {0.490316,-0.216214,0.520299},
                              {0.490316,-0.216214,0.720299}};
  double cyl0_drop_traj[3][3]={{0.526436,0.187901,0.720299},
                              {0.526436,0.187901,0.520299},
                              {0.526436,0.187901,0.720299}};
  double cyl1_pick_traj[3][3]={{0.34644,-0.188354,0.720299},
                              {0.34644,-0.188354,0.520299},
                              {0.34644,-0.188354,0.720299}};
  double cyl1_drop_traj[3][3]={{0.346466,0.276881,0.720299},
                              {0.346466,0.276881,0.520299},
                              {0.346466,0.276881,0.720299}};

  KDL::Rotation rpy=KDL::Rotation::RPY(-3.14154,0.0013407,3.1415);


int f = d.colorThresholder(colorInput);

while (ros::ok()) {

    if(f==2) {
      ROS_INFO_STREAM("Cylinder number"<<f+3<<"is"<<colorInput);
      for (int i =0; i<3;i++) {
        cartpos2.p[0]=cyl2_pick_traj[i][0];
        cartpos2.p[1]=cyl2_pick_traj[i][1];
        cartpos2.p[2]=cyl2_pick_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd=ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if(i==1) {
          g.ToggleState(false);
        }
      }
      for (int i =0; i<3;i++) {
        cartpos2.p[0]=cyl2_drop_traj[i][0];
        cartpos2.p[1]=cyl2_drop_traj[i][1];
        cartpos2.p[2]=cyl2_drop_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd=ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if(i==1) {
          g.ToggleState(true);
        }
      }
      cmd=ku.driveRobot(ku.initializeHomePos());
      cmd_pub.publish(cmd);
      ros::Duration(3).sleep();
      ROS_INFO_STREAM("PART KITTED SUCCESFULLY!!! KILLING NODE :)");
      ros::shutdown();
    } else if(f==1) {
      for (int i =0; i<3;i++) {
        cartpos2.p[0]=cyl1_pick_traj[i][0];
        cartpos2.p[1]=cyl1_pick_traj[i][1];
        cartpos2.p[2]=cyl1_pick_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd=ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if(i==1) {
          g.ToggleState(false);
        }
      }
      for (int i =0; i<3;i++) {
        cartpos2.p[0]=cyl1_drop_traj[i][0];
        cartpos2.p[1]=cyl1_drop_traj[i][1];
        cartpos2.p[2]=cyl1_drop_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd=ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if(i==1) {
          g.ToggleState(true);
        }
      }
      cmd=ku.driveRobot(ku.initializeHomePos());
      cmd_pub.publish(cmd);
      ros::Duration(3).sleep();
      ROS_INFO_STREAM("PART KITTED SUCCESFULLY!!! KILLING NODE :)");
      ros::shutdown();
    } else if(f==0) {
      for (int i =0; i<3;i++) {
        cartpos2.p[0]=cyl0_pick_traj[i][0];
        cartpos2.p[1]=cyl0_pick_traj[i][1];
        cartpos2.p[2]=cyl0_pick_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd=ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if(i==1) {
          g.ToggleState(false);
        }
      }
      for (int i =0; i<3;i++) {
        cartpos2.p[0]=cyl0_drop_traj[i][0];
        cartpos2.p[1]=cyl0_drop_traj[i][1];
        cartpos2.p[2]=cyl0_drop_traj[i][2];
        cartpos2.M = rpy;
        ros::spinOnce();
        cmd=ku.driveRobot(ku.normalizePoints(ku.evalKinematicsIK(cartpos2)));
        cmd_pub.publish(cmd);
        ros::Duration(3).sleep();
        if(i==1) {
          g.ToggleState(true);
        }
      }
      cmd=ku.driveRobot(ku.initializeHomePos());
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
