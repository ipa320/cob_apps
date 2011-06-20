#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <cob_mmcontroller/augmented_solver.h>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/jntarray.hpp>

using namespace std; 
using namespace KDL;
KDL::JntArray q;
KDL::Jacobian jacobian(7);
KDL::Chain chain;

ChainJntToJacSolver * jacobiansolver;

ros::Publisher manipulability_pub_;  //publish

JntArray parseJointStates(std::vector<std::string> names, std::vector<double> positions)
{
	JntArray q_temp(7);
	int count = 0;
    for(unsigned int i = 0; i < names.size(); i++)
    {
			if(strncmp(names[i].c_str(), "arm_", 4) == 0)
			{
				q_temp(count) = positions[i];
				count++;
      }
    }
	return q_temp;
}

void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::vector<std::string> names = msg->name;
	std::vector<double> positions = msg->position;

	q = parseJointStates(names,positions);
	jacobiansolver->JntToJac(q,jacobian);
	Eigen::Matrix<double,6,6> prod = jacobian.data * jacobian.data.transpose();
	float d = prod.determinant();
	std_msgs::Float64 f;
	f.data = sqrt(d);
	manipulability_pub_.publish(f);

	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_mm_controller");
	ros::NodeHandle n;
	KDL::Tree my_tree;
	ros::NodeHandle node;
	std::string robot_desc_string;
	node.param("/robot_description", robot_desc_string, string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
	      ROS_ERROR("Failed to construct kdl tree");
      	      return false;
	}
	my_tree.getChain("base_link","arm_7_link", chain);

	jacobiansolver = new KDL::ChainJntToJacSolver(chain);
	
	ros::Subscriber sub = n.subscribe("/joint_states", 1, controllerStateCallback);
	manipulability_pub_ = n.advertise<std_msgs::Float64>("/arm_controller/manipulability2", 1);

	ROS_INFO("Running cartesian velocity controller.");
	ros::spin();

	return 0;
}
