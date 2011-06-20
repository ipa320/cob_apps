#include "ros/ros.h"

class cob_cartesian_trajectories
{
public:
	cob_cartesian_trajectories();

private:
	KDL::Twist getTrajectoryTwist(double dt, Frame F_current);
	void cartStateCallback(const geometry_msgs::Pose::ConstPtr& msg);
	ros::Subscriber cart_state_sub_;

};

cob_cartesian_trajectories::cob_cartesian_trajectories()
{
	cart_state_sub_ = n.subscribe("/arm_controller/cart_state", 1, &cob_cartesian_trajectories::cartStateCallback, this);
}

void cartStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
{


}

KDL::Twist cob_cartesian_trajectories::getTrajectoryTwist(double dt, Frame F_current)
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
