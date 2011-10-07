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
#include <visualization_msgs/Marker.h>


#include <kdl/chainfksolverpos_recursive.hpp>
#include <cob_mmcontroller/augmented_solver.h>
#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <cob_mmcontroller/OpenFridgeAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace KDL;


class cob_cartesian_trajectories
{
public:
	cob_cartesian_trajectories();

private:
	ros::NodeHandle n;
	actionlib::SimpleActionServer<cob_mmcontroller::OpenFridgeAction> as_;
	actionlib::SimpleActionServer<cob_mmcontroller::OpenFridgeAction> as2_;
	KDL::Twist getTwist(double dt, Frame F_current);
	void getSollLinear(double dt, double &sollx, double &solly, double &sollangle);
	void getSollCircular(double dt, double &sollx, double &solly, double &sollangle);
	void cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void moveCircActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal);
	void moveLinActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal);
	bool moveCircCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	bool moveLinCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response);
	void sendMarkers();
	bool start();
	ros::Subscriber cart_state_sub_;
	ros::Publisher cart_command_pub;
	ros::Publisher debug_cart_pub_;
	ros::Publisher map_pub_;
	ros::ServiceServer serv_linear;
	ros::ServiceServer serv_circular;
	std::vector<geometry_msgs::Point> trajectory_points;

	bool bRun;
	bool bStarted;
	double currentDuration;
	double targetDuration;

	ros::Time timer;
	ros::Time tstart;
	Frame F_start;
	std::string mode;

	geometry_msgs::PoseStamped current_hinge;

};


cob_cartesian_trajectories::cob_cartesian_trajectories() : as_(n, "moveCirc", boost::bind(&cob_cartesian_trajectories::moveCircActionCB, this, _1), false), as2_(n, "moveLin", boost::bind(&cob_cartesian_trajectories::moveLinActionCB, this, _1), false)
{
	cart_state_sub_ = n.subscribe("/arm_controller/cart_state", 1, &cob_cartesian_trajectories::cartStateCallback, this);
	cart_command_pub = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
	debug_cart_pub_ = n.advertise<geometry_msgs::PoseArray>("/mm/debug",1);
	serv_linear = n.advertiseService("/mm/move_lin", &cob_cartesian_trajectories::moveLinCB, this);
	serv_circular = n.advertiseService("/mm/move_circ", &cob_cartesian_trajectories::moveCircCB, this);
	map_pub_ = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
	bRun = false;
	as_.start();
	as2_.start();
	targetDuration = 0;
	currentDuration = 0;
	mode = "linear";
}

void cob_cartesian_trajectories::sendMarkers()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "trajectory_values";
	marker.id = 10;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.04;
	marker.scale.y = 0.2;
	marker.color.r = 1.0;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration(10.0);

	for(int i=0; i<trajectory_points.size(); i++)
	{
		//ROS_INFO("line %f %f %f %f\n", iX1, iY1, iX2, iY2);
		marker.points.push_back(trajectory_points[i]);
	}
	map_pub_.publish(marker);
}

void cob_cartesian_trajectories::moveCircActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal)
{
	mode = "circular";
	current_hinge = goal->hinge;
	if(start())
	{
		while(bRun)
		{
			//wait until finished
			sleep(1);
		}
		as_.setSucceeded();
	}
	return;

}
void cob_cartesian_trajectories::moveLinActionCB(const cob_mmcontroller::OpenFridgeGoalConstPtr& goal)
{
	mode = "linear";
	if(start())
	{
		while(bRun)
		{
			//wait until finished
			sleep(1);
		}
		as2_.setSucceeded();
	}
	return;

}

bool cob_cartesian_trajectories::moveCircCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	mode = "circular";
	return start();
}
bool cob_cartesian_trajectories::moveLinCB(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	mode = "linear";
	return start();
}

bool cob_cartesian_trajectories::start()
{
	if(bRun)
	{
		ROS_ERROR("Already running trajectory");
		return false;
	}
	else
	{
		bRun = true;
		bStarted = false;
		timer = ros::Time::now();
		tstart = ros::Time::now();
		targetDuration = 7;
		currentDuration = 0;
		trajectory_points.clear();
		return true;
	}	
}

//Pose is global pose with odometry
void cob_cartesian_trajectories::cartStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(bRun)
	{
		ros::Duration dt = ros::Time::now() - timer;
		timer = ros::Time::now();
		if((targetDuration-currentDuration) <= 0)
		{
			geometry_msgs::Twist twist;
			cart_command_pub.publish(twist);
			ROS_INFO("finished trajectory in %f", ros::Time::now().toSec() - tstart.toSec());
			bRun = false;
			bStarted = false;
			sendMarkers();
			return;
		}		
		KDL::Frame current;
		KDL::Frame myhinge;
		tf::PoseMsgToKDL(msg->pose, current);
		tf::PoseMsgToKDL(current_hinge.pose, myhinge);
		KDL::Vector unitz = myhinge.M.UnitZ();
		std::cout << "Radius because of Hinge: " << (myhinge.p - current.p) << "UnitZ of hinge: " << unitz.z() << "\n";
		geometry_msgs::Twist twist;
		KDL::Twist ktwist = getTwist(currentDuration, current);
		twist.linear.x =  ktwist.vel.x();
		twist.linear.y =  ktwist.vel.y();
		twist.linear.z =  ktwist.vel.z();

		twist.angular.x =  ktwist.rot.x();
		twist.angular.y =  ktwist.rot.y();
		twist.angular.z =  ktwist.rot.z();


		//twist.linear.z = -0.02;
		cart_command_pub.publish(twist);
		currentDuration+=dt.toSec();

		geometry_msgs::Point p;
		p.x = msg->pose.position.x;
		p.y = msg->pose.position.y;
		p.z = msg->pose.position.z;
		trajectory_points.push_back(p);
	}
	else
	{
		//publish zero	
		geometry_msgs::Twist twist;
		cart_command_pub.publish(twist);
	}
}

void cob_cartesian_trajectories::getSollLinear(double dt, double &sollx, double &solly, double &sollangle)
{
	double look_ahead = 0.1;
	sollx = ((dt+look_ahead)/targetDuration) * -0.1;
	solly = 0.0; //((dt+look_ahead)/targetDuration) * 0.6;
	sollangle = 0.0;
}
void cob_cartesian_trajectories::getSollCircular(double dt, double &sollx, double &solly, double &sollangle)
{
	double look_ahead = 0.02;
	double max_ang = 3.14/2;
	double radius = 0.3; //TODO: Radius einstellen

	sollx = sin(max_ang*((dt+look_ahead)/targetDuration)) * radius ;
	solly = radius-(cos(max_ang*((dt+look_ahead)/targetDuration)) * radius);
	solly *= -1;
	sollangle = -max_ang*((dt+look_ahead)/targetDuration);	
}

KDL::Twist cob_cartesian_trajectories::getTwist(double dt, Frame F_current)
{
	KDL::Frame F_soll = F_start;
	KDL::Frame F_diff = F_start;
	KDL::Twist lin;
	double start_roll, start_pitch, start_yaw = 0.0;
	double current_roll, current_pitch, current_yaw = 0.0;

	if(!bStarted)
	{
		F_start = F_current;
		bStarted = true;
	}
	double soll_x, soll_y, soll_angle;
	if(mode == "linear")
		getSollLinear(dt, soll_x, soll_y, soll_angle);
	else
		getSollCircular(dt, soll_x, soll_y, soll_angle);

	F_soll.p.x(F_start.p.x() + soll_x);
	F_soll.p.y(F_start.p.y() + soll_y);
	F_soll.p.z(F_start.p.z());
	
	F_start.M.GetRPY(start_roll, start_pitch, start_yaw);
	F_current.M.GetRPY(current_roll, current_pitch, current_yaw);

	std::cout << "AngleDiff: " << (current_yaw-start_yaw) << " vs " << (soll_angle) << " error: " << (soll_angle-current_yaw-start_yaw) << "\n";

	F_diff.p.x(F_soll.p.x()-F_current.p.x());
	F_diff.p.y(F_soll.p.y()-F_current.p.y());
	F_diff.p.z(F_soll.p.z()-F_current.p.z());

	lin.vel.x(0.5 * F_diff.p.x());
	lin.vel.y(0.5 * F_diff.p.y());
	lin.vel.z(0.0);
	lin.rot.x(0.0);
	lin.rot.y(0.0);
	/*if(fabs(soll_angle-current_yaw-start_yaw) < 0.6) 
		lin.rot.z((soll_angle-current_yaw-start_yaw));
	else
	{
		std::cout << "MAXROT\n";
		lin.rot.z(-0.6);
	}*/
	lin.rot.z(-0.4); //TODO: Rotationsgeschwindigkeit
		
	//DEBUG
	geometry_msgs::PoseArray poses;
	poses.poses.resize(3);
	tf::PoseKDLToMsg(F_current, poses.poses[0]);
	tf::PoseKDLToMsg(F_soll, poses.poses[1]);
	tf::PoseKDLToMsg(F_diff, poses.poses[2]);
	debug_cart_pub_.publish(poses);
	std::cout << "Twist x: " << 0.1 * F_diff.p.x() << " y: " << 0.1 * F_diff.p.y() << "\n";
	//

	return lin;
}

/*
KDL::Twist cob_cartesian_trajectories::getTrajectoryTwist(double dt, Frame F_current)
{
	if(!bStarted)
	{
		F_start = F_current;
		bStarted = true;
	}
	KDL::Twist circ;
	std::cout << "Time is " << dt << "\n";
	F_current.p.x(F_current.p.x());
	F_current.p.y(F_current.p.y());
	F_current.p.z(F_current.p.z());
	double max_ang = 0.4*3.14;
	double soll_y = 0.6-(cos(max_ang*(dt/targetDuration)) * 0.6);
	double soll_x = sin(max_ang*(dt/targetDuration)) * 0.6;
	//double soll_y_t1 = -0.6+(cos(max_ang*((dt+0.02)/max_time)) * 0.6);
	//double soll_x_t1 = sin(max_ang*((dt+0.02)/max_time)) * 0.6;
	std::cout << "Soll x:" << soll_x << " y: " << soll_y << "\n";
	std::cout << "Diff x:" << F_current.p.x()-F_start.p.x() << " y: " << F_current.p.y()-F_start.p.y() << "\n";

	KDL::Frame F_soll = F_start;
	KDL::Frame F_soll2 = F_start;
	KDL::Frame F_diff = F_start;
	F_soll.p.x(F_start.p.x() + soll_x);
	F_soll.p.y(F_start.p.y() - soll_y);
	//F_soll2.p.x(F_start.p.x() + soll_x_t1);
	//F_soll2.p.y(F_start.p.y() - soll_y_t1);

	F_diff.p.x(F_current.p.x()-F_soll.p.x());
	F_diff.p.y(F_current.p.y()-F_soll.p.y());
	F_diff.p.z(F_current.p.z()-F_start.p.z());

	double twist_x = (F_current.p.x()-F_soll.p.x());
	double twist_y = (F_current.p.y()-F_soll.p.y());


	std::cout << "Twist x: " << twist_x << " y: " << twist_y << "\n";
	circ.vel.z(0.0);
	circ.vel.x(twist_x);
	circ.vel.y(twist_y);

	//DEBUG
	geometry_msgs::PoseArray poses;
	poses.poses.resize(3);
	tf::PoseKDLToMsg(F_current, poses.poses[0]);
	tf::PoseKDLToMsg(F_soll, poses.poses[1]);
	tf::PoseKDLToMsg(F_diff, poses.poses[2]);
	debug_cart_pub_.publish(poses);


	return circ;
}
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_cartesian_trajectories");
	cob_cartesian_trajectories controller ;
	ros::spin();

	return 0;
}
