/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_apps
 * ROS package name: cob_teleop
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * Date of creation: June 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <ros/ros.h>
#include <joy/Joy.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

const int PUBLISH_FREQ = 20.0;

#define TORSO_TOPIC "torso_controller/command"
#define TRAY_TOPIC "tray_controller/command"
#define ARM_TOPIC "arm_controller/command"
using namespace std;

class TeleopCOB
{
public:
   //torso (lower neck and upper neck)
   double req_lower_tilt,req_lower_pan,req_upper_tilt,req_upper_pan; //positions
   double req_lower_tilt_vel,req_lower_pan_vel,req_upper_tilt_vel,req_upper_pan_vel;//velocities
   double lower_tilt_step,lower_pan_step,upper_tilt_step,upper_pan_step; //step variables
   int lower_neck_button,upper_neck_button; //buttons

   //tray
   double req_tray; //position
   double req_tray_vel; //velocity
   double tray_step;
   int tray_button;

   //base
   geometry_msgs::Twist cmd;
   double req_vx,req_vy,req_vw;
   double max_vx,max_vy,max_vw;
   int axis_vx,axis_vy,axis_vw;
  
   //arm
   double req_j1,req_j1_vel; //joint1 between link_0 and link_1 left_right movement
   double req_j2,req_j2_vel; //joint2 between link_1 and link_2 up_down movement
   int arm_joint12_button;

   double req_j3,req_j3_vel; //joint3 between link_2 and link_3 left_right
   double req_j4,req_j4_vel; //joint4 between link_3 and link_4 up_down 
   int arm_joint34_button;

   double req_j5,req_j5_vel; //joint5 between link_4 and link_5 left_right
   double req_j6,req_j6_vel; //joint6 between link_5 and link_6 up_down
   int arm_joint56_button;
 
   double req_j7,req_j7_vel; //joint7 between link_6 and link_7 left_right
   int arm_joint7_button;
   
   double arm_left_right_step; //arm left_right_step
   double arm_up_down_step; //arm up_down_step
 
   //signs
   int up_down,left_right;   //sign for movements of upper_neck and tray
   
  
   ros::NodeHandle n_;
   ros::Publisher torso_pub_;  //publish topic torso_controller/command
   ros::Publisher tray_pub_;  //publish topic tray_controller/command
   ros::Publisher vel_pub_;  //publish topic "cmd_vel"
   ros::Publisher arm_pub_;  //publish topic arm_controller/command
   ros::Subscriber joy_sub_;  //subscribe topic joy
  
   TeleopCOB();
   void init();
   void joy_cb(const joy::Joy::ConstPtr &joy_msg);
   void send_cmd_vel();
   ~TeleopCOB();

};

TeleopCOB::TeleopCOB():req_lower_tilt(0.0),req_lower_pan(0.0),req_upper_tilt(0.0),req_upper_pan(0.0),req_tray(0.0),req_vx(0.0),req_vy(0.0),req_vw(0.0),req_j1(0.0),req_j2(0.0),req_j3(0.0),req_j4(0.0),req_j5(0.0),req_j6(0.0),req_j7(0.0)
{
}

void TeleopCOB::init()
{
   //torso
   //button for lower neck
   n_.param("lower_neck_button",lower_neck_button,6);
   //button for upper neck
   n_.param("upper_neck_button",upper_neck_button,4);
  
   //tray
   n_.param("tray_button",tray_button,tray_button);

   //base
   cmd.linear.x = cmd.linear.y = cmd.angular.z = 0.0;
   n_.param("axis_vx",axis_vx,axis_vx);
   n_.param("axis_vy",axis_vy,axis_vy);
   n_.param("axis_vw",axis_vw,axis_vw);

   //arm
   n_.param("arm_joint12_button",arm_joint12_button,0); //button 0
   n_.param("arm_joint34_button",arm_joint34_button,1); //button 1
   n_.param("arm_joint56_button",arm_joint56_button,2); //button 2
   n_.param("arm_joint7_button",arm_joint7_button,3); //button 3
   
   //signs
   n_.param("up_down",up_down,1); //axis[1] tray--up/down; tilt--front/back, here we just name up_down
   n_.param("left_right",left_right,0);  //axis[0] pan--left/right

   n_.param("lower_tilt_step",lower_tilt_step,0.15);
   n_.param("lower_pan_step",lower_pan_step,0.05);

   n_.param("upper_tilt_step",upper_tilt_step,0.1);
   n_.param("upper_pan_step",upper_pan_step,0.05);
   
   n_.param("tray_step",tray_step,0.15);

   n_.param("max_vx", max_vx, 0.3);
   n_.param("max_vy", max_vy, 0.3);
   n_.param("max_vw", max_vw, 0.2);

   n_.param("arm_left_right_step",arm_left_right_step,0.1);
   n_.param("arm_up_down_step",arm_up_down_step,0.1);


   //test here!
   std::cout<<"init::lower_neck_button:"<<lower_neck_button<<endl;
   std::cout<<"init::upper_neck_button:"<<upper_neck_button<<endl;

   std::cout<<"init::tray_button:"<<tray_button<<endl;

   std::cout<<"init::axis_vx:"<<axis_vx<<endl;
   std::cout<<"init::axis_vy:"<<axis_vy<<endl;
   std::cout<<"init::axis_vw:"<<axis_vw<<endl;

   std::cout<<"init::arm_joint12_button:"<<arm_joint12_button<<endl;
   std::cout<<"init::arm_joint34_button:"<<arm_joint34_button<<endl;
   std::cout<<"init::arm_joint56_button:"<<arm_joint56_button<<endl;
   std::cout<<"init::arm_joint7_button:"<<arm_joint7_button<<endl;

   std::cout<<"init::up_down:"<<up_down<<endl;
   std::cout<<"init::left_right:"<<left_right<<endl;

   joy_sub_ = n_.subscribe("joy",10,&TeleopCOB::joy_cb,this);
   torso_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(TORSO_TOPIC,1);
   tray_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(TRAY_TOPIC,1);
   vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel",1);   
   arm_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>(ARM_TOPIC,1);
}

void TeleopCOB::joy_cb(const joy::Joy::ConstPtr &joy_msg)
{
   //torso
   //lower neck 
   if(lower_neck_button>=0 && lower_neck_button<(int)joy_msg->buttons.size() && joy_msg->buttons[lower_neck_button]==1)
   {
      //tilt
      if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]>0.0)
         req_lower_tilt_vel = (int)joy_msg->buttons[lower_neck_button]*lower_tilt_step;
      else if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]<0.0)
         req_lower_tilt_vel = -1*(int)joy_msg->buttons[lower_neck_button]*lower_tilt_step;
      else
         req_lower_tilt_vel = 0.0;
      //test
      std::cout<<"cb::lower neck tilt velocity:"<<req_lower_tilt_vel<<endl;
      //pan
      if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]<0.0)
         req_lower_pan_vel = (int)joy_msg->buttons[lower_neck_button]*lower_pan_step;
      else if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]>0.0)
         req_lower_pan_vel = -1*(int)joy_msg->buttons[lower_neck_button]*lower_pan_step;
      else
         req_lower_pan_vel = 0.0;
      //test
      std::cout<<"cb::lower neck pan velocity:"<<req_lower_pan_vel<<endl;
   }
   else
   {
      req_lower_tilt_vel = 0.0;
      req_lower_pan_vel = 0.0;  //button release
   }

   //upper neck
   if(upper_neck_button>=0 && upper_neck_button<(int)joy_msg->buttons.size() && joy_msg->buttons[upper_neck_button]==1)
   {
      //tilt
      if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]>0.0)
         req_upper_tilt_vel = (int)joy_msg->buttons[upper_neck_button]*upper_tilt_step;
      else if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]<0.0)
         req_upper_tilt_vel = -1*(int)joy_msg->buttons[upper_neck_button]*upper_tilt_step;
      else
         req_upper_tilt_vel = 0.0;
      //test
      std::cout<<"cb::upper neck tilt velocity:"<<req_upper_tilt_vel<<endl;
      //pan
      if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]<0.0)
         req_upper_pan_vel = (int)joy_msg->buttons[upper_neck_button]*upper_pan_step;
      else if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]>0.0)
         req_upper_pan_vel = -1*(int)joy_msg->buttons[upper_neck_button]*upper_pan_step;
      else
         req_upper_pan_vel = 0.0;
      //test
      std::cout<<"cb::upper neck pan velocity:"<<req_upper_pan_vel<<endl;
   }
   else
   {
      req_upper_tilt_vel = 0.0;
      req_upper_pan_vel = 0.0;  //button release
   }
    
   //tray
   if(tray_button>=0 && tray_button<(int)joy_msg->buttons.size() && joy_msg->buttons[tray_button]==1)
   {
      if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]>0.0)
         req_tray_vel = (int)joy_msg->buttons[tray_button]*tray_step;
      else if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]<0.0)
         req_tray_vel = -1*(int)joy_msg->buttons[tray_button]*tray_step;
      else
         req_tray_vel = 0.0;

      std::cout<<"cb::tray_button:"<<tray_button<<endl;
      std::cout<<"cb::joy_msg->buttons[tray_button]:"<<joy_msg->buttons[tray_button]<<endl;
   }
   else
   {
      req_tray_vel = 0.0;  //button release
   }  
 
   //base
   if(axis_vx>=0 && axis_vx<(int)joy_msg->get_axes_size())
     req_vx = joy_msg->axes[axis_vx]*max_vx;
   else
     req_vx = 0.0;

   if(axis_vy>=0 && axis_vy<(int)joy_msg->get_axes_size())
     req_vy = joy_msg->axes[axis_vy]*max_vy;
   else
     req_vy = 0.0;

   if(axis_vw>=0 && axis_vw<(int)joy_msg->get_axes_size())
     req_vw = joy_msg->axes[axis_vw]*max_vw;
   else
     req_vw = 0.0;

   //arm
   //joint12
   if(arm_joint12_button>=0 && arm_joint12_button<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint12_button]==1)
   {
      //joint 1 left or right
      if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]<0.0)
         req_j1_vel = -1*(int)joy_msg->buttons[arm_joint12_button]*arm_left_right_step;
      else if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]>0.0)
         req_j1_vel = (int)joy_msg->buttons[arm_joint12_button]*arm_left_right_step;
      else
         req_j1_vel = 0.0;
      //test
      std::cout<<"cb::arm joint1 velocity:"<<req_j1_vel<<endl;

      //joint 2 up or down
      if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]>0.0)
         req_j2_vel = -1*(int)joy_msg->buttons[arm_joint12_button]*arm_up_down_step;
      else if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]<0.0)
         req_j2_vel = (int)joy_msg->buttons[arm_joint12_button]*arm_up_down_step;
      else
         req_j2_vel = 0.0;
      //test
      std::cout<<"cb::arm joint2 velocity:"<<req_j2_vel<<endl;

   }
   else
   {
      req_j1_vel = 0.0;
      req_j2_vel = 0.0;  //button release
   } //arm_joint12

   //joint34
   if(arm_joint34_button>=0 && arm_joint34_button<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint34_button]==1)
   {
      //joint 3 left or right
      if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]<0.0)
         req_j3_vel = -1*(int)joy_msg->buttons[arm_joint34_button]*arm_left_right_step;
      else if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]>0.0)
         req_j3_vel = (int)joy_msg->buttons[arm_joint34_button]*arm_left_right_step;
      else
         req_j3_vel = 0.0;
      //test
      std::cout<<"cb::arm joint3 velocity:"<<req_j3_vel<<endl;

      //joint 4 up or down
      if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]>0.0)
         req_j4_vel = -1*(int)joy_msg->buttons[arm_joint34_button]*arm_up_down_step;
      else if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]<0.0)
         req_j4_vel = (int)joy_msg->buttons[arm_joint34_button]*arm_up_down_step;
      else
         req_j4_vel = 0.0;
      //test
      std::cout<<"cb::arm joint4 velocity:"<<req_j4_vel<<endl;

   }
   else
   {
      req_j3_vel = 0.0;
      req_j4_vel = 0.0;  //button release
   } //arm_joint34

   //joint56
   if(arm_joint56_button>=0 && arm_joint56_button<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint56_button]==1)
   {
      //joint 5 left or right
      if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]<0.0)
         req_j5_vel = -1*(int)joy_msg->buttons[arm_joint56_button]*arm_left_right_step;
      else if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]>0.0)
         req_j5_vel = (int)joy_msg->buttons[arm_joint56_button]*arm_left_right_step;
      else
         req_j5_vel = 0.0;
      //test
      std::cout<<"cb::arm joint5 velocity:"<<req_j5_vel<<endl;

      //joint 6 up or down
      if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]>0.0)
         req_j6_vel = -1*(int)joy_msg->buttons[arm_joint56_button]*arm_up_down_step;
      else if(up_down>=0 && up_down<(int)joy_msg->axes.size() && joy_msg->axes[up_down]<0.0)
         req_j6_vel = (int)joy_msg->buttons[arm_joint56_button]*arm_up_down_step;
      else
         req_j6_vel = 0.0;
      //test
      std::cout<<"cb::arm joint6 velocity:"<<req_j6_vel<<endl;

   }
   else
   {
      req_j5_vel = 0.0;
      req_j6_vel = 0.0;  //button release
   } //arm_joint56


   //joint7
   if(arm_joint7_button>=0 && arm_joint7_button<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint7_button]==1)
   {
      //joint 7 left or right
      if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]<0.0)
         req_j7_vel = -1*(int)joy_msg->buttons[arm_joint7_button]*arm_left_right_step;
      else if(left_right>=0 && left_right<(int)joy_msg->axes.size() && joy_msg->axes[left_right]>0.0)
         req_j7_vel = (int)joy_msg->buttons[arm_joint7_button]*arm_left_right_step;
      else
         req_j7_vel = 0.0;
      //test
      std::cout<<"cb::arm joint7 velocity:"<<req_j7_vel<<endl;

   }
   else
   {
      req_j7_vel = 0.0;
   } //arm_joint7

}//joy_cb

void TeleopCOB::send_cmd_vel()
{
  //torso
  {
   double dt = 1.0/double(PUBLISH_FREQ);
   double horizon = 3.0*dt;

   trajectory_msgs::JointTrajectory traj;
   traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
   traj.joint_names.push_back("torso_lower_neck_tilt_joint");
   traj.joint_names.push_back("torso_lower_neck_pan_joint");
   traj.joint_names.push_back("torso_upper_neck_tilt_joint");
   traj.joint_names.push_back("torso_upper_neck_pan_joint");
   traj.points.resize(1);
   //test here
   //fprintf(stdout,"req_lower_tilt_vel::%f",req_lower_tilt_vel);
   //std::cout<<"req_lower_pan_vel::"<<req_lower_pan_vel<<endl;
   traj.points[0].positions.push_back(req_lower_tilt + req_lower_tilt_vel*horizon);
   traj.points[0].velocities.push_back(req_lower_tilt_vel); //lower_neck_tilt
   traj.points[0].positions.push_back(req_lower_pan + req_lower_pan_vel*horizon);
   traj.points[0].velocities.push_back(req_lower_pan_vel);  //lower_neck_pan
   traj.points[0].positions.push_back(req_upper_tilt + req_upper_tilt_vel*horizon);
   traj.points[0].velocities.push_back(req_upper_tilt_vel); //upper_neck_tilt
   traj.points[0].positions.push_back(req_upper_pan + req_upper_pan_vel*horizon);
   traj.points[0].velocities.push_back(req_upper_pan_vel);  //upper_neck_pan
   traj.points[0].time_from_start = ros::Duration(horizon);

   torso_pub_.publish(traj);
 
   //update current position 
   req_lower_tilt += req_lower_tilt_vel*dt;
   req_lower_pan += req_lower_pan_vel*dt;
   req_upper_tilt += req_upper_tilt_vel*dt;
   req_upper_pan += req_upper_pan_vel*dt;
  } //torso
 
  //tray
  {
   double dt = 1.0/double(PUBLISH_FREQ);
   double horizon = 3.0*dt;

   trajectory_msgs::JointTrajectory traj;
   traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
   traj.joint_names.push_back("torso_tray_joint");
   traj.points.resize(1);
   traj.points[0].positions.push_back(req_tray + req_tray_vel*horizon);
   traj.points[0].velocities.push_back(req_tray_vel); 
   traj.points[0].time_from_start = ros::Duration(horizon);

   tray_pub_.publish(traj);
 
   //update current position 
   req_tray += req_tray_vel*dt;
  }

  //base
  cmd.linear.x = req_vx;
  cmd.linear.y = req_vy;
  cmd.angular.z = req_vw;
  vel_pub_.publish(cmd);

  //arm
  {
   double dt = 1.0/double(PUBLISH_FREQ);
   double horizon = 3.0*dt;

   trajectory_msgs::JointTrajectory traj;
   traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
   traj.joint_names.push_back("arm_1_joint");
   traj.joint_names.push_back("arm_2_joint");
   traj.joint_names.push_back("arm_3_joint");
   traj.joint_names.push_back("arm_4_joint");
   traj.joint_names.push_back("arm_5_joint");
   traj.joint_names.push_back("arm_6_joint");
   traj.joint_names.push_back("arm_7_joint");

   traj.points.resize(1);
   traj.points[0].positions.push_back(req_j1 + req_j1_vel*horizon);
   traj.points[0].velocities.push_back(req_j1_vel);  //joint1

   traj.points[0].positions.push_back(req_j2 + req_j2_vel*horizon);
   traj.points[0].velocities.push_back(req_j2_vel); //joint2

   traj.points[0].positions.push_back(req_j3 + req_j3_vel*horizon);
   traj.points[0].velocities.push_back(req_j3_vel); //joint3

   traj.points[0].positions.push_back(req_j4 + req_j4_vel*horizon);
   traj.points[0].velocities.push_back(req_j4_vel); //joint4

   traj.points[0].positions.push_back(req_j5 + req_j5_vel*horizon);
   traj.points[0].velocities.push_back(req_j5_vel); //joint5

   traj.points[0].positions.push_back(req_j6 + req_j6_vel*horizon);
   traj.points[0].velocities.push_back(req_j6_vel); //joint6

   traj.points[0].positions.push_back(req_j7 + req_j7_vel*horizon);
   traj.points[0].velocities.push_back(req_j7_vel); //joint7
   
   traj.points[0].time_from_start = ros::Duration(horizon);
   arm_pub_.publish(traj);
 
   //update current position 
   req_j1 += req_j1_vel*dt;
   req_j2 += req_j2_vel*dt;
   req_j3 += req_j3_vel*dt;
   req_j4 += req_j4_vel*dt;
   req_j5 += req_j5_vel*dt;
   req_j6 += req_j6_vel*dt;
   req_j7 += req_j7_vel*dt;
  } //arm

}

TeleopCOB::~TeleopCOB() {}

int main(int argc,char **argv)
{
   ros::init(argc,argv,"teleop_cob");
   TeleopCOB teleop_cob;
   ros::Rate pub_rate(PUBLISH_FREQ);
   teleop_cob.init();

   while(teleop_cob.n_.ok())
   {
     ros::spinOnce();
     teleop_cob.send_cmd_vel();
     pub_rate.sleep();
   }

   exit(0);
   return(0);
}

