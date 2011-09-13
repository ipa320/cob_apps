#include <cob_mmcontroller/cob_config_controller.h>
cob_config_controller::cob_config_controller()
{
	started = false;
	RunSyncMM = false;
	//parsing urdf for KDL chain
	KDL::Tree my_tree;
	ros::NodeHandle node;
	node.param("arm_base", arm_base_name_, std::string("arm_0_link"));
	node.param("arm_end_effector", arm_ee_name_, std::string("arm_7_link"));
	node.param("default_control_mode", kinematic_mode_, std::string("arm_base"));

	std::string robot_desc_string;
	node.param("/robot_description", robot_desc_string, string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		  ROS_ERROR("Failed to construct kdl tree");
			  return;
	}
	my_tree.getChain("base_link",arm_ee_name_, arm_base_chain);
	my_tree.getChain(arm_base_name_,arm_ee_name_, arm_chain);

	//Initializing configuration control solver
	iksolver1v = new augmented_solver(arm_base_chain);//Inverse velocity solver
	fksolver1 = new ChainFkSolverPos_recursive(arm_base_chain);

	//Initializing communication

	sub = n.subscribe("/joint_states", 1, &cob_config_controller::controllerStateCallback, this);
	cart_vel_sub = n.subscribe("/arm_controller/cart_command", 1, &cob_config_controller::cartTwistCallback, this);
	plat_odom_sub = n.subscribe("/base_controller/odometry", 1, &cob_config_controller::baseTwistCallback, this);

	ROS_INFO("Creating publishers");
	arm_pub_ = n.advertise<brics_actuator::JointVelocities>("/arm_controller/command_vel",1);
	base_pub_ = n.advertise<geometry_msgs::Twist>("/base_controller/command",1);
	debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/arm_controller/debug/cart",1);
	cart_position_pub_ = n.advertise<geometry_msgs::PoseStamped>("/arm_controller/cart_state",1);

	serv_start = n.advertiseService("/mm/start", &cob_config_controller::SyncMMTriggerStart, this);
	serv_stop = n.advertiseService("/mm/stop", &cob_config_controller::SyncMMTriggerStop, this);

	ROS_INFO("Running cartesian velocity controller.");
	zeroCounter = 0;
	zeroCounterTwist = 0;
}


JntArray cob_config_controller::parseJointStates(std::vector<std::string> names, std::vector<double> positions)
{
	JntArray q_temp(7);
	int count = 0;
	bool parsed = false;
	for(unsigned int i = 0; i < names.size(); i++)
    {
			if(strncmp(names[i].c_str(), "arm_", 4) == 0)
			{
				q_temp(count) = positions[i];
				count++;
				parsed = true;
			}
    }
	if(!parsed)
		return q_last;
	q_last = q_temp;
	//ROS_INFO("CurrentConfig: %f %f %f %f %f %f %f", q_temp(0), q_temp(1), q_temp(2), q_temp(3), q_temp(4), q_temp(5), q_temp(6));
	if(!started)
	{
		JntArray zero(7);
		sendVel(zero,zero,zero);
		VirtualQ = q_temp;
		started = true;
		last = ros::Time::now();

		ROS_INFO("Starting up controller with first configuration");
		std::cout << VirtualQ(0) << "\n";

	}
	return q_temp;
}

void cob_config_controller::cartTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	extTwist.vel.x(msg->linear.x);
	extTwist.vel.y(msg->linear.y);
	extTwist.vel.z(msg->linear.z);

	extTwist.rot.x(msg->angular.x);
	extTwist.rot.y(msg->angular.y);
	extTwist.rot.z(msg->angular.z);
}

void cob_config_controller::baseTwistCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::PoseMsgToKDL(msg->pose.pose, base_odom_);
}


bool cob_config_controller::SyncMMTriggerStart(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	ros::ServiceClient client = n.serviceClient<cob_srvs::Trigger>("/arm_controller/reset_brics_interface");
	cob_srvs::Trigger srv;
	client.call(srv);
	if(RunSyncMM)
	{
		ROS_INFO("Already started");
	}	
	else
	{
		ROS_INFO("Starting MM interface");
		started = false;
		RunSyncMM = true;
	}	
	return true;
}
bool cob_config_controller::SyncMMTriggerStop(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(RunSyncMM)
	{
		ROS_INFO("Stopping MM interface");
		RunSyncMM = false;
	}	
	else
	{
		ROS_INFO("Already stopped");
	}	
	return true;
}

void cob_config_controller::sendVel(JntArray q_t, JntArray q_dot, JntArray q_dot_base)
{
	ros::Time now = ros::Time::now();
	double dt = now.toSec() - last.toSec();
	last = now;
	double horizon = 3.0*dt;

	brics_actuator::JointVelocities target_joint_vel;
	target_joint_vel.velocities.resize(7);
	bool nonzero = false;
	for(unsigned int i=0; i<7; i++)
	{
		std::stringstream joint_name;
		joint_name << "arm_" << (i+1) << "_joint";
		target_joint_vel.velocities[i].joint_uri = joint_name.str();
		target_joint_vel.velocities[i].unit = "rad";
		if(q_dot(i) != 0.0)
		{
			target_joint_vel.velocities[i].value = q_dot(i);
			nonzero = true;
			zeroCounter = 0;
		}
		else
		{
			target_joint_vel.velocities[i].value = 0.0;
		}

	}
	if(zeroCounter <= 4)
	{
		zeroCounter++;
		if(!nonzero)
			std::cout << "Sending additional zero\n";
		nonzero = true;
	}
	if(!started)
		nonzero = true;
	if(nonzero)
	{
		arm_pub_.publish(target_joint_vel);
	}

	//send to base
	geometry_msgs::Twist cmd;
	if(q_dot_base(0) != 0.0 || q_dot_base(1) != 0.0 || q_dot_base(2) != 0.0)
	{
		cmd.linear.x = q_dot_base(0);
		cmd.linear.y = q_dot_base(1);
		cmd.angular.z = q_dot_base(2);
		base_pub_.publish(cmd);
	}
}
void cob_config_controller::sendCartPose()
{
	KDL::Frame F_current;
	F_current = arm_pose_ * base_odom_;
	geometry_msgs::PoseStamped pose;
	tf::PoseKDLToMsg(F_current, pose.pose);
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
		if(extTwist.vel.x() != 0.0 || extTwist.vel.y() != 0.0 || extTwist.vel.z() != 0.0)
		{
			zeroCounterTwist = 0;
			int ret = iksolver1v->CartToJnt(q, extTwist, q_out, q_dot_base);
			if(ret >= 0)
			{
				sendVel(q, q_out, q_dot_base);
				//std::cout << q_out(0) << " " << q_out(1) << " " << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << " " << q_out(6)  << "\n";
			}	
			else
				std::cout << "Something went wrong" << "\n";
		}
		else
		{
			if(zeroCounterTwist >= 4)
			{
				int ret = iksolver1v->CartToJnt(q, extTwist, q_out, q_dot_base);
				if(ret >= 0)
				{
					sendVel(q, q_out, q_dot_base);
					//std::cout << q_out(0) << " " << q_out(1) << " " << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << " " << q_out(6)  << "\n";
				}	
				else
					std::cout << "Something went wrong" << "\n";
			}
			zeroCounterTwist++;
		}	
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_mm_controller");
	cob_config_controller cob_config_controller;
	ros::spin();

	return 0;
}
