#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>

using namespace std;
using namespace KDL;
KDL::Chain chain;

ChainFkSolverPos_recursive *  fksolver1;//Forward position solver
ChainIkSolverVel_pinv * iksolver1v;//Inverse velocity solver
ChainIkSolverPos_NR * iksolverpos;//Maximum 100 iterations, stop at accuracy 1e-6

void controllerStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
  unsigned int nj = chain.getNrOfJoints();
  
  JntArray q(7);
  std::cout << "Numbers of joints " << nj << "\n";
  for(int i = 0; i < 7; i++)
    {
      q(i) = msg->actual.positions[i];
    }
  std::cout << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6)  << "\n";	
  Frame F_ist;
  fksolver1->JntToCart(q, F_ist);
  std::cout << F_ist <<"\n";
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

	fksolver1 = new ChainFkSolverPos_recursive(chain);//Forward position solver
	iksolver1v = new ChainIkSolverVel_pinv(chain);//Inverse velocity solver
	//iksolverpos = new ChainIkSolverPos_NR(chain,&fksolver1,&iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
	
	ros::Subscriber sub = n.subscribe("/arm_controller/controller_state", 1, controllerStateCallback);
	
	ROS_INFO("Running cartesian velocity controller.");
	ros::spin();

	return 0;
}
