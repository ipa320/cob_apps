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

  //pos3: brics_pregrasp
  desired_pose.pose.position.x = -0.44;
  desired_pose.pose.position.y = -0.327;
  desired_pose.pose.position.z = 0.701;
  desired_pose.pose.orientation.x = -0.463;
  desired_pose.pose.orientation.y = 0.664;
  desired_pose.pose.orientation.z = 0.567;
  desired_pose.pose.orientation.w = -0.149;

  //pos4: over_tablet
  /*desired_pose.pose.position.x = 0.515;
  desired_pose.pose.position.y = -0.177;
  desired_pose.pose.position.z = 0.999;
  desired_pose.pose.orientation.x = -0.457;
  desired_pose.pose.orientation.y = 0.566;
  desired_pose.pose.orientation.z = -0.422;
  desired_pose.pose.orientation.w = 0.541;*/

  //pos4: home
  /*desired_pose.pose.position.x = -0.215;
  desired_pose.pose.position.y = -0.790;
  desired_pose.pose.position.z = 1.608;
  desired_pose.pose.orientation.x = -0.111;
  desired_pose.pose.orientation.y = -0.368;
  desired_pose.pose.orientation.z = 0.923;
  desired_pose.pose.orientation.w = -0.014;*/
	
  


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

