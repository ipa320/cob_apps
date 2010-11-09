#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_arm",true);

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
	
	//over-tablet
	//-0.97877458873047019, -1.5948518814806336, 2.0263840730501208, 1.4992515760970839, 0.48346032199394173, 0.79316104671682552, -3.8301333079173459
	goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -0.98;
	goalB.motion_plan_request.goal_constraints.joint_constraints[1].position = -1.59;
	goalB.motion_plan_request.goal_constraints.joint_constraints[2].position = 2.03;
	goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = 1.50;
	goalB.motion_plan_request.goal_constraints.joint_constraints[4].position = 0.48;
	goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = 0.79;
	goalB.motion_plan_request.goal_constraints.joint_constraints[6].position = -3.83;


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

