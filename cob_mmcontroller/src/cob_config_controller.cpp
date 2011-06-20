#include <cob_mmcontroller/cob_config_controller.h>
cob_config_controller::cob_config_controller()
{
	started = false;
	RunSyncMM = false;
	//parsing urdf for KDL chain
	KDL::Tree my_tree;
	ros::NodeHandle node;
	std::string robot_desc_string;
	node.param("/robot_description", robot_desc_string, string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		  ROS_ERROR("Failed to construct kdl tree");
			  return;
	}
	my_tree.getChain("base_link","arm_7_link", chain);

	//Initializing configuration control solver
	iksolver1v = new augmented_solver(chain);//Inverse velocity solver
	fksolver1 = new ChainFkSolverPos_recursive(chain);

	//Initializing communication

	sub = n.subscribe("/joint_states", 1, &cob_config_controller::controllerStateCallback, this);
	cart_vel_sub = n.subscribe("/arm_controller/cart_command", 1, &cob_config_controller::cartTwistCallback, this);
	plat_odom_sub = n.subscribe("/base_controller/odometry", 1, &cob_config_controller::baseTwistCallback, this);

	ROS_INFO("Creating publishers");
	arm_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
	base_pub_ = n.advertise<geometry_msgs::Twist>("/base_controller/command",1);
	debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/arm_controller/debug/cart",1);
	cart_position_pub_ = n.advertise<geometry_msgs::Pose>("/arm_controller/cart_state",1);

	serv = n.advertiseService("/mm/run", &cob_config_controller::SyncMMTrigger, this);

	ROS_INFO("Running cartesian velocity controller.");
}


JntArray cob_config_controller::parseJointStates(std::vector<std::string> names, std::vector<double> positions)
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

void cob_config_controller::cartTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	extTwist.vel.x(msg->linear.x);
	extTwist.vel.y(msg->linear.y);
	extTwist.vel.z(msg->linear.z);

	extTwist.rot.x(msg->angular.x);
	extTwist.rot.x(msg->angular.y);
	extTwist.rot.x(msg->angular.z);
}

void cob_config_controller::baseTwistCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::PoseMsgToKDL(msg->pose.pose, base_odom_);
}


bool cob_config_controller::SyncMMTrigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(RunSyncMM)
		RunSyncMM = false;
	else
	{
		started = false;
		RunSyncMM = true;
	}	
	return true;
}

void cob_config_controller::sendVel(JntArray q_t, JntArray q_dot, JntArray q_dot_base)
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
	//send to base
	geometry_msgs::Twist cmd;
	cmd.linear.x = q_dot_base(0);
	cmd.linear.y = q_dot_base(1);
	cmd.angular.z = q_dot_base(2);
	base_pub_.publish(cmd);
}
void cob_config_controller::sendCartPose()
{
	KDL::Frame F_current;
	F_current = arm_pose_ * base_odom_;
	geometry_msgs::Pose pose;
	tf::PoseKDLToMsg(F_current, pose);
	cart_position_pub_.publish(pose);
}

void cob_config_controller::controllerStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	JntArray q_out(7);
	JntArray q_dot_base(3);
	std::vector<std::string> names = msg->name;
	std::vector<double> positions = msg->position;
	q = parseJointStates(names,positions);
	fksolver1->JntToCart(q, arm_pose_);
	sendCartPose();
	if(RunSyncMM)
	{
		int ret = iksolver1v->CartToJnt(q, extTwist, q_out, q_dot_base);
		if(ret >= 0)
		{
			sendVel(q, q_out, q_dot_base);
			std::cout << q_out(0) << " " << q_out(1) << " " << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << " " << q_out(6)  << "\n";
		}	
		else
			std::cout << "Something went wrong" << "\n";	
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_mm_controller");
	cob_config_controller cob_config_controller;
	ros::spin();

	return 0;
}
