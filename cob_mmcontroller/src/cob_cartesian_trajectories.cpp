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
#include <geometry_msgs/Pose.h>


#include <kdl/chainfksolverpos_recursive.hpp>
#include <cob_mmcontroller/augmented_solver.h>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

using namespace KDL;

class cob_cartesian_trajectories
{
public:
	cob_cartesian_trajectories();

private:
	ros::NodeHandle n;
	KDL::Twist getTrajectoryTwist(double dt, Frame F_current);
	void cartStateCallback(const geometry_msgs::Pose::ConstPtr& msg);
	bool openDoorCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	ros::Subscriber cart_state_sub_;
	ros::Publisher cart_command_pub;
ros::ServiceServer serv;
	bool bRun;
	int testCounter;
	Frame F_start;

};


cob_cartesian_trajectories::cob_cartesian_trajectories()
{
	cart_state_sub_ = n.subscribe("/arm_controller/cart_state", 1, &cob_cartesian_trajectories::cartStateCallback, this);
	cart_command_pub = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
	serv = n.advertiseService("/mm/open_fridge", &cob_cartesian_trajectories::openDoorCB, this);
	bRun = false;
	testCounter = 0;
}

bool cob_cartesian_trajectories::openDoorCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(bRun)
	{
		ROS_ERROR("Already running trajectory");
		response.success.data = false;
		return true;
	}
	else
	{
		testCounter = 200;
		bRun = true;
		response.success.data = true;
	}	
	return true;
}

void cob_cartesian_trajectories::cartStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	if(bRun)
	{
		if(testCounter == 0)
		{
			ROS_INFO("finished trajectory");
			bRun = false;		
			return;
		}		
		geometry_msgs::Twist twist;
		twist.linear.z = 0.001;
		cart_command_pub.publish(twist);
		testCounter--;
	}
}

KDL::Twist cob_cartesian_trajectories::getTrajectoryTwist(double dt, Frame F_current)
{
	KDL::Twist circ;
	std::cout << "Time is " << dt << "\n";
	if(dt <= 7.0)
	{
		F_current.p.x(F_current.p.x());// + basePoseOdom.p.x());
		F_current.p.y(F_current.p.y());// + basePoseOdom.p.y());
		F_current.p.z(F_current.p.z());// + basePoseOdom.p.z());
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
		//debug_cart_pub_.publish(poses);

	}
	else
		std::cout << "Finnished\n";
	return circ;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_cartesian_trajectories");
	cob_cartesian_trajectories controller ;
	ros::spin();

	return 0;
}
