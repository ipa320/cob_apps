#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <cob_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

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

bool RunSyncMM = false;
ros::Publisher arm_pub_;  //publish topic arm_controller/command


JntArray parseJointStates(std::vector<std::string> names, std::vector<double> positions)
{
	JntArray q(7);
	int count = 0;
  for(int i = 0; i < names.size(); i++)
    {
			if(strncmp(names[i].c_str(), "arm_", 4) == 0)
			{
				q(count) = positions[i];
				count++;
				//std::cout << names[i] << ": " << positions[i] << "\n";
      }
    }
	return q;
}

KDL::Twist getTwist(KDL::Frame F_ist)
{
	KDL::Twist zero;
	if(!RunSyncMM)
		return zero;
	else
	{
		KDL::Twist test;
		test.vel.x(0.1);
		return test;
	}	
}

bool SyncMMTrigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(RunSyncMM)
		RunSyncMM = false;
	else
		RunSyncMM = true;
	return true;
}

void sendVel(JntArray q, JntArray q_dot)
{
	double dt = 0.01;
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
	for(int i = 0; i < 7; i++)
	{ 
		traj.points[0].positions.push_back(q(i) + q_dot(i)*horizon);
		traj.points[0].velocities.push_back(q_dot(i));
	}
	arm_pub_.publish(traj);
}

void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  unsigned int nj = chain.getNrOfJoints();
  std::vector<std::string> names = msg->name;
	std::vector<double> positions = msg->position;
	JntArray q = parseJointStates(names,positions);
 	std::cout << "Joints: " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6)  << "\n";	
  JntArray q_out(7);
 	Frame F_ist;
  fksolver1->JntToCart(q, F_ist);
  std::cout << F_ist <<"\n";
	int ret = iksolver1v->CartToJnt(q, getTwist(F_ist), q_out);
	if(ret >= 0)
	{
		sendVel(q, q_out);
  	std::cout << q_out(0) << " " << q_out(1) << " " << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << " " << q_out(6)  << "\n";
	}	
	else
		std::cout << "Something went wrong" << "\n";	

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
	
	ros::Subscriber sub = n.subscribe("/joint_states", 1, controllerStateCallback);
	arm_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command",1);
	ros::ServiceServer serv = n.advertiseService("/mm/run", SyncMMTrigger);
	ROS_INFO("Running cartesian velocity controller.");
	ros::spin();

	return 0;
}
