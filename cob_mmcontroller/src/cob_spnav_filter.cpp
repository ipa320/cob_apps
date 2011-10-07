#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cob_srvs/Trigger.h>
#include <deque>

ros::Publisher twist_pub_;
ros::ServiceServer serv;

std::deque<double> z_cache;
int num_cache = 6;
bool bRunning = false;
double offset = -2;

bool TwistInputTrigger(cob_srvs::Trigger::Request& request, cob_srvs::Trigger::Response& response)
{
	if(!bRunning)
	{
		bRunning = true;
		ROS_INFO("Starting wrench input");
	}
	else
	{
		bRunning = false;
		ROS_INFO("Stopping wrench input");
	}	
	return true;
} 

void spnavCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
if(bRunning)
{
  double average_z = 0.0;
  if(num_cache == 0)
    z_cache.pop_front();
  else
    num_cache--;
  z_cache.push_back(msg->wrench.force.z + offset);
  for(unsigned int i = 0; i < z_cache.size(); i++)
    average_z += z_cache.at(i);
  average_z /= z_cache.size();

	geometry_msgs::Twist new_twist;
	/*if(msg->wrench.force.x > 5.0)
	  new_twist.linear.x = 0.01;
	if(msg->wrench.force.x < 5.0)
	  new_twist.linear.x = -0.01;
	if(msg->wrench.force.y > 5.0)
	    new_twist.linear.y = 0.01;
	if(msg->wrench.force.y < 5.0)
	    new_twist.linear.y = -0.01;*/
	if(average_z < -1.0)
	    new_twist.linear.z = -0.01 * average_z;
	else
	  {
	    if(average_z > 1.0)
	      new_twist.linear.z = -0.01 * average_z;
	    else
	      new_twist.linear.z = 0.0;
	  }
	std::cout << "Sending twist of " << new_twist.linear.z << " current force is " << average_z << "\n";
	twist_pub_.publish(new_twist);
}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "cob_spnav_filter");
	ros::NodeHandle n;
	ros::Subscriber spnav = n.subscribe("/arm_controller/wrench", 1, spnavCallback);
	serv = n.advertiseService("/mm/twist_input_run", TwistInputTrigger);
	twist_pub_ = n.advertise<geometry_msgs::Twist>("/arm_controller/cart_command",1);
	ros::spin();
	return 0;
}
