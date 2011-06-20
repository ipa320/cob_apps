#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

ros::Publisher twist_pub_;

void spnavCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist new_twist;
	new_twist.linear = msg->linear;
	new_twist.linear.x /= 500;
	new_twist.linear.y /= 500;
	new_twist.linear.z /= 500;
	new_twist.angular = msg->angular;
	new_twist.angular.x /= 500;
	new_twist.angular.y /= 500;
	new_twist.angular.z /= 500;
	twist_pub_.publish(new_twist);
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "cob_spnav_filter");
	ros::NodeHandle n;
	ros::Subscriber spnav = n.subscribe("/spacenav/twist", 1, spnavCallback);
	twist_pub_ = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
	ros::spin();
	return 0;
}
