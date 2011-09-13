#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <cob_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>


#include <kdl/chainfksolverpos_recursive.hpp>
#include <cob_mmcontroller/augmented_solver.h>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

using namespace std;
using namespace KDL;

class cob_config_controller
{
public:
	cob_config_controller();
private:
	JntArray parseJointStates(std::vector<std::string> names, std::vector<double> positions);
	void cartTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void baseTwistCallback(const nav_msgs::Odometry::ConstPtr& msg);
	bool SyncMMTriggerStart(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	bool SyncMMTriggerStop(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	void sendVel(JntArray q_t, JntArray q_dot, JntArray q_dot_base);
	void sendCartPose();
	void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

	ros::NodeHandle n;
	int zeroCounter;
	int zeroCounterTwist;

	//configuration
	std::string arm_base_name_;
	std::string arm_ee_name_;
	std::string kinematic_mode_;


	KDL::Chain arm_base_chain;
	KDL::Chain arm_chain;
	KDL::JntArray VirtualQ;
	KDL::JntArray q;
	KDL::JntArray q_last;
	bool started;
	KDL::Twist extTwist;
	ros::Time last;

	KDL::Frame base_odom_;
	KDL::Frame arm_pose_;

	ChainFkSolverPos_recursive *  fksolver1;
	augmented_solver * iksolver1v;//Inverse velocity solver

	bool RunSyncMM;
	ros::Publisher arm_pub_;  //publish topic arm_controller/command
	ros::Publisher base_pub_;  //publish topic base_controller/command
	ros::Publisher debug_cart_pub_;
	ros::Publisher cart_position_pub_;

	ros::ServiceServer serv_start;
	ros::ServiceServer serv_stop;

	ros::Subscriber sub;
	ros::Subscriber cart_vel_sub;
	ros::Subscriber plat_odom_sub;


};
