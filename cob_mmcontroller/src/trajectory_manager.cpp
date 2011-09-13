#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <brics_actuator/JointPositions.h>


#define HZ 50

class trajectory_manager
{
private:
    ros::NodeHandle n_;
    ros::Publisher cart_vel_pub_;
    ros::Publisher joint_pos_pub_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber controller_state_;
    actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;
    std::string action_name_;
    bool executing_;
    trajectory_msgs::JointTrajectory traj_;
    double traj_time_;
    int current_point_;
    KDL::ChainFkSolverPos_recursive *  fksolver1_;//Forward position solver
    KDL::Chain chain_;
    KDL::JntArray q_current;
    KDL::JntArray startposition_;
public:
    trajectory_manager():as_(n_, "joint_trajectory_action", boost::bind(&trajectory_manager::executeTrajectory, this, _1)),
    action_name_("joint_trajectory_action")
    {
        cart_vel_pub_ = n_.advertise<geometry_msgs::Twist>("cart_twist", 1);
        joint_pos_pub_ = n_.advertise<brics_actuator::JointPositions>("target_joint_pos", 1);
        joint_state_pub_ = n_.advertise<sensor_msgs::JointState>("target_joint_state", 1);
        controller_state_ = n_.subscribe("state", 1, &trajectory_manager::state_callback, this);
        traj_time_ = 0.0;
        current_point_ = 0;
        executing_ = false;
        KDL::Tree my_tree;
        std::string robot_desc_string;
	    n_.param("/robot_description", robot_desc_string, std::string());
	    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
	    {
	        ROS_ERROR("Failed to construct kdl tree");
      	}
      	my_tree.getChain("base_link","arm_7_link", chain_);
      	fksolver1_ = new KDL::ChainFkSolverPos_recursive(chain_);
      	q_current = KDL::JntArray(7);
      	startposition_ = q_current;
	}

  void state_callback(const pr2_controllers_msgs::JointTrajectoryControllerStatePtr& message)
  {
    std::vector<double> positions = message->actual.positions;
    for(unsigned int i = 0; i < positions.size(); i++)
    {
      q_current(i) = positions[i];
    }
  }
  void executeTrajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal)
  {
        ROS_INFO("Received new goal trajectory with %d points",goal->trajectory.points.size());
        if(!executing_)
        {
            traj_ = goal->trajectory;
            traj_time_ = 0.0;
            current_point_ = 0;
            executing_ = true;
            startposition_ = q_current;
            std::cout << "Setting startposition to " << q_current(0) << " " << q_current(1) << "\n";
        }
        else //suspend current movement and start new one
        {
        }
        while(executing_)
        	sleep(1);
        as_.setSucceeded();
    }
    
    void run()
    {
        if(executing_)
        {
            //calculate current cartpos
            KDL::Frame F_ist;
		    fksolver1_->JntToCart(q_current, F_ist);
            //calculate target cartpos;
            KDL::Frame F_target;
            KDL::JntArray q_target(traj_.points[current_point_].positions.size());
            for (unsigned int i = 0; i < traj_.points[current_point_].positions.size(); i += 1)
            {
                q_target(i) = traj_.points[current_point_].positions[i];
            }
            fksolver1_->JntToCart(q_target, F_target);
            //calculate desired twist TODO
            double delta_time = 0.0;
            if(current_point_ != 0)
                delta_time = traj_.points[current_point_].time_from_start.toSec() - traj_.points[current_point_-1].time_from_start.toSec();
            else
            	delta_time = traj_.points[current_point_].time_from_start.toSec();

            geometry_msgs::Twist trajectory_twist;
            
            //calculate current intermediate joint position
            brics_actuator::JointPositions target_joint_position;
            sensor_msgs::JointState target_joint_state;
            target_joint_state.header.stamp = ros::Time();
            target_joint_position.positions.resize(traj_.points[current_point_].positions.size());
            target_joint_state.position.resize(traj_.points[current_point_].positions.size());
            for (unsigned int i = 0; i < traj_.points[current_point_].positions.size(); i += 1)
            {
            	target_joint_position.positions[i].unit = "rad";
                //position = current_time_inbetween * distance_to_travel/overall_time_inbetween
                target_joint_position.positions[i].value =  startposition_(i) + (traj_time_ * (traj_.points[current_point_].positions[i] - startposition_(i))/delta_time);
                if(current_point_ != 0)
                {
                   target_joint_position.positions[i].value = traj_.points[current_point_-1].positions[i] + ((traj_time_ - traj_.points[current_point_-1].time_from_start.toSec()) * (traj_.points[current_point_].positions[i] - traj_.points[current_point_-1].positions[i])/delta_time);
                }
                target_joint_state.position[i] = target_joint_position.positions[i].value;
            }
            //send everything
            cart_vel_pub_.publish(trajectory_twist);
            joint_pos_pub_.publish(target_joint_position);
            joint_state_pub_.publish(target_joint_state);
            
            traj_time_ += 1./HZ;
            //std::cerr << ".";
            if(traj_time_ >= traj_.points[current_point_].time_from_start.toSec())
			{
				//std::cout << "DEBUG: time from start" << traj_.points[current_point_].time_from_start.toSec() << " , Traj_time: " << traj_time_ << "\n";
				if(current_point_ == traj_.points.size()-1)
				{
					ROS_INFO("Trajecory finished");
					executing_ = false;
					return;
				}
				else
				{
					current_point_ +=1;
					//ROS_INFO("Next Point in trajectory");
				}
			}
        }
        
    }
    
};



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "trajectory_manager");
    trajectory_manager tm;
    ros::Rate loop_rate(HZ);
    while (ros::ok())
    {
        tm.run();
        ros::spinOnce();
        loop_rate.sleep();
    }  
	
}






