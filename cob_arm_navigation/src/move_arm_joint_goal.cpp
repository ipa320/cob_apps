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
 *   Example client that moves to a joint goal.
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

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_arm",true);

  ROS_INFO("waiting for action server...");
  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  move_arm_msgs::MoveArmGoal goalB;
  std::vector<std::string> names(7);
  names[0] = "arm_1_joint";//"r_shoulder_pan_joint";
  names[1] = "arm_2_joint";//"r_shoulder_lift_joint";
  names[2] = "arm_3_joint";//"r_upper_arm_roll_joint";
  names[3] = "arm_4_joint";//"r_elbow_flex_joint";
  names[4] = "arm_5_joint";//"r_forearm_roll_joint";
  names[5] = "arm_6_joint";//"r_wrist_flex_joint";
  names[6] = "arm_7_joint";//"r_wrist_roll_joint";

  goalB.motion_plan_request.group_name = "arm";//"right_arm";
  goalB.motion_plan_request.num_planning_attempts = 1;
  goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goalB.motion_plan_request.planner_id= std::string("");
  goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

  for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }
	
	//PR2:  
  //goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.0;
  //goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.2; //-1.0;
  //goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.15;

	//COB:
	//goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = 0.8;//pole-pos1
	//goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -1.0;//pole-pos2
	//goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = -0.78;
/*	
	//test
	//[[2.2031812731220626, -0.91217394752704806, -2.3337257293527092, -1.1578828463405637, 0.44081288873368774, 0.82864568416099793, -1.520311024214986]]
	goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = 2.203181273122062;
	goalB.motion_plan_request.goal_constraints.joint_constraints[1].position =-0.912173947527048;
	goalB.motion_plan_request.goal_constraints.joint_constraints[2].position =-2.333725729352709;
	goalB.motion_plan_request.goal_constraints.joint_constraints[3].position =-1.157882846340563;
	goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = 0.440812888733687;
	goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = 0.828645684160997;
	goalB.motion_plan_request.goal_constraints.joint_constraints[6].position =-1.520311024214986;
*/
	
	//goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = 1.5;



  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalB);
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

