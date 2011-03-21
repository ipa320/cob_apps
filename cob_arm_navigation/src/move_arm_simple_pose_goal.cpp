/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_apps
 * \note
 *   ROS package name: cob_arm_navigation
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: January 2011
 *
 * \brief
 *   Example client that moves to a simple pose goal.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  move_arm_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  motion_planning_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "base_link";//"torso_lift_link";
  desired_pose.link_name = "arm_7_link";//"r_wrist_roll_link";
  

/* OLD: possibly not working due to changes on lbr model
  //pos1: stretch-up
  desired_pose.pose.position.x = -0.06;
  desired_pose.pose.position.y = -0.21;
  desired_pose.pose.position.z = 1.92;
  desired_pose.pose.orientation.x = -1.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 0.02;
  
  //pos2: over-tablet
  desired_pose.pose.position.x = 0.52;
  desired_pose.pose.position.y = -0.01;
  desired_pose.pose.position.z = 0.92;
  desired_pose.pose.orientation.x = -0.72;
  desired_pose.pose.orientation.y = -0.17;
  desired_pose.pose.orientation.z = 0.19;
  desired_pose.pose.orientation.w = -0.63;
END_OLD */

/*
  //pos3: over-tablet
  desired_pose.pose.position.x = 0.544;
  desired_pose.pose.position.y =-0.267;
  desired_pose.pose.position.z = 0.935;
  desired_pose.pose.orientation.x = 0.576;
  desired_pose.pose.orientation.y = 0.421;
  desired_pose.pose.orientation.z = 0.475;
  desired_pose.pose.orientation.w =-0.512;
*/
  
  //pos3: test
  desired_pose.pose.position.x =-0.055;
  desired_pose.pose.position.y =-0.754;
  desired_pose.pose.position.z = 0.602;
  desired_pose.pose.orientation.x = 0.626;
  desired_pose.pose.orientation.y =-0.390;
  desired_pose.pose.orientation.z = 0.651;
  desired_pose.pose.orientation.w =-0.178;
	

  desired_pose.absolute_position_tolerance.x = 0.2;
  desired_pose.absolute_position_tolerance.y = 0.2;
  desired_pose.absolute_position_tolerance.z = 0.2;

  desired_pose.absolute_roll_tolerance = 0.4;
  desired_pose.absolute_pitch_tolerance = 0.4;
  desired_pose.absolute_yaw_tolerance = 0.4;
  
  move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}

