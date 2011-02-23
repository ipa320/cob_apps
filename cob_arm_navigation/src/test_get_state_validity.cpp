#include <ros/ros.h>
#include <tf/tf.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <motion_planning_msgs/MultiDOFJointState.h>
#include <geometry_msgs/Pose2D.h>


int main(int argc, char **argv){
  ros::init (argc, argv, "get_state_validity_test");
  ros::NodeHandle rh;

  ros::service::waitForService("cob3_environment_server/get_state_validity");
  ros::ServiceClient check_state_validity_client_ = rh.serviceClient<planning_environment_msgs::GetStateValidity>("cob3_environment_server/get_state_validity");

  planning_environment_msgs::GetStateValidity::Request req;
  planning_environment_msgs::GetStateValidity::Response res;
  
  geometry_msgs::Pose2D platform_pose;
  platform_pose.x=-2.0;
  platform_pose.y=0.8;
  platform_pose.theta=-0.7;
  

  //fill start state...i.e. platform_position!!!
  motion_planning_msgs::MultiDOFJointState mdjs;
  mdjs.stamp=ros::Time::now();
  mdjs.joint_names.resize(1);
  mdjs.joint_names[0]="base_joint";
  mdjs.frame_ids.resize(1);
  mdjs.frame_ids[0]="map";
  mdjs.child_frame_ids.resize(1);
  mdjs.child_frame_ids[0]="base_footprint";
  mdjs.poses.resize(1);
  btQuaternion quat(btVector3(0.0, 0.0, 1.0), platform_pose.theta);
  mdjs.poses[0].position.x=platform_pose.x;
  mdjs.poses[0].position.y=platform_pose.y;
  mdjs.poses[0].position.z=0.0;
  mdjs.poses[0].orientation.x=quat.getX();
  mdjs.poses[0].orientation.y=quat.getY();
  mdjs.poses[0].orientation.z=quat.getZ();
  mdjs.poses[0].orientation.w=quat.getW();
  
  ROS_INFO("Platform_Pose pos(x,y,z): (%f,%f,%f)", mdjs.poses[0].position.x, mdjs.poses[0].position.y, mdjs.poses[0].position.z);
  ROS_INFO("Platform_Pose ori(x,y,z,w): (%f,%f,%f,%f)", mdjs.poses[0].orientation.x, mdjs.poses[0].orientation.y, mdjs.poses[0].orientation.z, mdjs.poses[0].orientation.w);
		
  req.robot_state.multi_dof_joint_state=mdjs;

  req.robot_state.joint_state.name.push_back("arm_1_joint");
  req.robot_state.joint_state.name.push_back("arm_2_joint");
  req.robot_state.joint_state.name.push_back("arm_3_joint");
  req.robot_state.joint_state.name.push_back("arm_4_joint");
  req.robot_state.joint_state.name.push_back("arm_5_joint");
  req.robot_state.joint_state.name.push_back("arm_6_joint");
  req.robot_state.joint_state.name.push_back("arm_7_joint");
  req.robot_state.joint_state.position.resize(7,0.0);

  //these set whatever non-zero joint angle values are desired
  req.robot_state.joint_state.position[0] = -0.3;
  req.robot_state.joint_state.position[1] = -0.78;
 
  req.robot_state.joint_state.header.stamp = ros::Time::now();
  req.check_collisions = true;
  if(check_state_validity_client_.call(req,res))
  {
    if(res.error_code.val == res.error_code.SUCCESS)
      ROS_INFO("Requested state is not in collision");
    else
      ROS_INFO("Requested state is in collision. Error code: %d",res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to check state validity failed %s",check_state_validity_client_.getService().c_str());
    return false;
  }
  ros::shutdown();
}
