#include "ros/ros.h"
#include "kinematics_msgs/GetPositionIK.h"
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <cob_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "augmented_solver.h"
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/frames.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>




using namespace std; 
using namespace KDL;
KDL::Chain chain;
KDL::Chain chain_base_arm0;
KDL::JntArray VirtualQ;
KDL::JntArray q;
Frame F_start;
bool started = false;
KDL::Twist extTwist;
KDL::Frame basePoseOdom;
KDL::Twist baseTwist;
ros::Time last;
double lastgradx = 0.0;
double lastgrady = 0.0;
double mytime;
ros::Time last2;

ChainFkSolverPos_recursive *  fksolver1;//Forward position solver
augmented_solver * iksolver1v;//Inverse velocity solver
ChainIkSolverPos_NR * iksolverpos;//Maximum 100 iterations, stop at accuracy 1e-6


bool RunSyncMM = false;
ros::Publisher arm_pub_;  //publish topic arm_controller/command
ros::Publisher base_pub_;  //publish topic base_controller/command
ros::Publisher debug_cart_pub_;



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
	if(!started)
	{
		VirtualQ = q_temp;
		started = true;
		last = ros::Time::now();

		ROS_INFO("Starting up controller with first configuration");
	}
	return q_temp;
}

void cartTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	extTwist.vel.x(msg->linear.x);
	extTwist.vel.y(msg->linear.y);
	extTwist.vel.z(msg->linear.z);

	extTwist.rot.x(msg->angular.x);
	extTwist.rot.x(msg->angular.y);
	extTwist.rot.x(msg->angular.z);
}

void baseTwistCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::PoseMsgToKDL(msg->pose.pose, basePoseOdom);
	if(RunSyncMM)
	{
		double vx = msg->twist.twist.linear.x;
		double vy = msg->twist.twist.linear.y;
		double omega = msg->twist.twist.angular.z;
		//std::cout << omega << "\n";
		Frame F_ist;
		fksolver1->JntToCart(q, F_ist, -1);
		//transformational part of base rotation
		KDL::Vector twist_Trans = (KDL::Rotation::EulerZYX(-omega, 0.0, 0.0) * F_ist.p) - F_ist.p;
		//rotational part of base rotation
		// 4. Get Rotational Twist

		// Create Transformation Matrix from ArmBase to Robot Base
		KDL::Rotation T_Rot_AB_RB(-0.683013,0.258819,-0.683013,
						0.183013,0.965926,0.183013,
						0.707107,0.00000,-0.707107);
		Frame F_armbase;
		ChainFkSolverPos_recursive fksolver_base_armv0(chain_base_arm0);//Forward position solver
		//std::cout << chain_base_arm0.getNrOfJoints() << "\n";
		fksolver_base_armv0.JntToCart(NULL, F_armbase, -1);
		//std::cout << F_armbase << "\n";
		// create transformation frame
		//  T_Trans.ReverseSign();
		KDL::Frame T_AB_RB(F_armbase.M);

		KDL::Vector twist_Rot = KDL::Vector(0,0, -omega);
		//KDL::Vector twist_Rot = F_ist.Inverse() * KDL::Vector(0,0, -omega);
		//std::cout << twist_Rot << "\n";
	
		//KDL::Twist RotTwist(twist_Trans, twist_Rot);
		KDL::Twist RotTwist(twist_Trans, KDL::Vector(0,0,0));

		baseTwist.vel.x(-vx);
		baseTwist.vel.y(-vy);

		baseTwist += RotTwist;
	}
}

KDL::Twist getTwist(KDL::Frame F_ist)
{
	KDL::Twist zero;
	if(!RunSyncMM)
		return zero;
	else
	{
		zero.vel.z(-0.1);
		return baseTwist;
	}	
}

KDL::Twist getTrajectoryTwist(double dt, Frame F_current)
{
	KDL::Twist circ;
	std::cout << "Time is " << dt << "\n";
	if(dt <= 7.0)
	{
		F_current.p.x(F_current.p.x() + basePoseOdom.p.x());
		F_current.p.y(F_current.p.y() + basePoseOdom.p.y());
		F_current.p.z(F_current.p.z() + basePoseOdom.p.z());
		double max_ang = 3.14/2.0;
		double max_time = 10.0;
		double soll_y = -0.6+(cos(max_ang*(dt/max_time)) * 0.6);
		double soll_x = sin(max_ang*(dt/max_time)) * 0.6;
		double soll_y_t1 = -0.6+(cos(max_ang*((dt+0.02)/max_time)) * 0.6);
		double soll_x_t1 = sin(max_ang*((dt+0.02)/max_time)) * 0.6;
		std::cout << "Soll x:" << soll_x << " y: " << soll_y << "\n";
		std::cout << "Diff x:" << F_current.p.x()-F_start.p.x() << " y: " << F_current.p.y()-F_start.p.y() << "\n";

		KDL::Frame F_soll = F_start;
		KDL::Frame F_soll2 = F_start;
		KDL::Frame F_diff = F_start;
		F_soll.p.x(F_start.p.x() + soll_x);
		F_soll.p.y(F_start.p.y() - soll_y);
		F_soll2.p.x(F_start.p.x() + soll_x_t1);
		F_soll2.p.y(F_start.p.y() - soll_y_t1);

		F_diff.p.x(F_current.p.x()-F_soll.p.x());
		F_diff.p.y(F_current.p.y()-F_soll.p.y());
		F_diff.p.z(F_current.p.z()-F_start.p.z());

		double twist_x = (F_current.p.x()-F_soll2.p.x());
		double twist_y = (F_current.p.y()-F_soll2.p.y());


		std::cout << "Twist x: " << twist_x << " y: " << twist_y << "\n";
		circ.vel.z(0.0);
		circ.vel.x(-twist_x);
		circ.vel.y(-twist_y);

		//DEBUG
		geometry_msgs::PoseArray poses;
		poses.poses.resize(3);
		tf::PoseKDLToMsg(F_current, poses.poses[0]);
		tf::PoseKDLToMsg(F_soll, poses.poses[1]);
		tf::PoseKDLToMsg(F_diff, poses.poses[2]);
		debug_cart_pub_.publish(poses);

	}
	else
		std::cout << "Finnished\n";
	return circ;
}



void Manipulability_potentialfield_lookup()
{
	double gradx;
	double grady;

	Frame F_ist;
  fksolver1->JntToCart(q, F_ist, -1);

	//std::cerr << "GlobPos: " << glcartpos << "\n";
	gradx = 0.420 - F_ist.p.x();
	grady = -0.711655  - F_ist.p.y();

	gradx = (gradx * gradx * gradx)/2;
	grady = (grady * grady * grady)/2;

	//max vels
	double maxvel = 0.2;
	if(gradx > maxvel)
		gradx = maxvel;
	if(gradx < -maxvel)
		gradx = -maxvel;
	if(grady > maxvel)
		grady = maxvel;
    if(grady < -maxvel)
		grady = -maxvel;
	if(abs(gradx) < 0.02)
		gradx = 0;
	if(abs(grady) < 0.02)
		grady = 0;

	//manage accelerations
	double maxacc = 0.01;
	if((gradx-lastgradx) > maxacc)
		gradx = lastgradx + maxacc;
	
	lastgradx = gradx;
	lastgrady = grady;

	geometry_msgs::Twist cmd;
	cmd.linear.x = gradx;
	cmd.linear.y = grady;
	if(RunSyncMM)
		base_pub_.publish(cmd);
}

bool SyncMMTrigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(RunSyncMM)
		RunSyncMM = false;
	else
	{
		mytime = 0.0;
		last2 = ros::Time::now();
		started = false;
		RunSyncMM = true;
		fksolver1->JntToCart(q, F_start);
		F_start.p.x(F_start.p.x() + basePoseOdom.p.x());
		F_start.p.y(F_start.p.y() + basePoseOdom.p.y());
	}	
	return true;
}

void sendVel(JntArray q_t, JntArray q_dot)
{
	ros::Time now = ros::Time::now();
	double dt = now.toSec() - last.toSec();
	last = now;
	double horizon = 3.0*dt;
	//std::cout << dt << "\n";

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
	bool nonzero = false;
	for(int i = 0; i < 7; i++)
	{ 
		if(q_dot(i) != 0.0)
		{
			traj.points[0].positions.push_back(VirtualQ(i) + q_dot(i)*horizon);
			traj.points[0].velocities.push_back(q_dot(i));
			VirtualQ(i) += q_dot(i)*dt;
			nonzero = true;
		}
	}
	traj.points[0].time_from_start = ros::Duration(horizon);
	if(nonzero)
		arm_pub_.publish(traj);
}

void controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::vector<std::string> names = msg->name;
	std::vector<double> positions = msg->position;
	q = parseJointStates(names,positions);
	if(RunSyncMM)
	{
		double dt = ros::Time::now().toSec() - last2.toSec();
		mytime += dt;
		last2 = ros::Time::now();
		unsigned int nj = chain.getNrOfJoints();
		//std::cout << "Joints: " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6)  << "\n";
		//std::cout << "VirtualJoints: " << VirtualQ(0) << " " << VirtualQ(1) << " " << VirtualQ(2) << " " << VirtualQ(3) << " " << VirtualQ(4) << " " << VirtualQ(5) << " " << VirtualQ(6)  << "\n";	
		JntArray q_out(7);
		JntArray q_base(3);
		JntArray q_dot_base(3);
		Frame F_ist;
		fksolver1->JntToCart(q, F_ist);
		KDL::Twist combined_twist = extTwist; //getTrajectoryTwist(mytime, F_ist);
		int ret = iksolver1v->CartToJnt(q, q_base, combined_twist, q_out, q_dot_base);
		if(ret >= 0)
		{
			sendVel(q, q_out);
			std::cout << q_out(0) << " " << q_out(1) << " " << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << " " << q_out(6)  << "\n";
			//send to base
			geometry_msgs::Twist cmd;
			cmd.linear.x = q_dot_base(0);
			cmd.linear.y = q_dot_base(1);
			cmd.angular.z = q_dot_base(2);
			base_pub_.publish(cmd);
		}	
		else
			std::cout << "Something went wrong" << "\n";	
	}
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
	my_tree.getChain("base_link","arm_0_link", chain_base_arm0);

	fksolver1 = new ChainFkSolverPos_recursive(chain);//Forward position solver
	iksolver1v = new augmented_solver(chain);//Inverse velocity solver
	//iksolverpos = new ChainIkSolverPos_NR(chain,&fksolver1,&iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
	
	ros::Subscriber sub = n.subscribe("/joint_states", 1, controllerStateCallback);
	ROS_INFO("Blub");
	ros::Subscriber cart_vel_sub = n.subscribe("/arm_controller/cart_command", 1, cartTwistCallback);
	ros::Subscriber plat_odom_sub = n.subscribe("/base_controller/odometry", 1, baseTwistCallback);

	arm_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);	
	base_pub_ = n.advertise<geometry_msgs::Twist>("/base_controller/command",1);
	debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/arm_controller/debug/cart",1);


	ros::ServiceServer serv = n.advertiseService("/mm/run", SyncMMTrigger);
	ROS_INFO("Running cartesian velocity controller.");
	ros::spin();

	return 0;
}
