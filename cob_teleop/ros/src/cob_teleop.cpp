/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_apps
 * \note
 *   ROS package name: cob_teleop
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: June 2010
 *
 * \brief
 *   Implementation of teleoperation node.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <ros/ros.h>
#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

const int PUBLISH_FREQ = 20.0;

/*!
* \brief Implementation of teleoperation node.
*
* Sends direct commands to different components
*/
class TeleopCOB
{
	public:
		//torso(lower neck and upper neck)
		double req_lower_tilt_,req_lower_pan_,req_upper_tilt_,req_upper_pan_; //positions
		double req_lower_tilt_vel_,req_lower_pan_vel_,req_upper_tilt_vel_,req_upper_pan_vel_;//velocities
		double lower_tilt_step_,lower_pan_step_,upper_tilt_step_,upper_pan_step_; //step variables
		int lower_neck_button_,upper_neck_button_; //buttons

		//tray
		double req_tray_; //position
		double req_tray_vel_; //velocity
		double tray_step_;
		int tray_button_;

		//base
		double req_vx_,req_vy_,req_vth_;
		double vx_old_,vy_old_,vth_old_;
		double max_vx_,max_vy_,max_vth_;
		double max_ax_,max_ay_,max_ath_;
		int axis_vx_,axis_vy_,axis_vth_;

		//arm
		double req_j1_,req_j1_vel_; //joint1 between link_0 and link_1 left_right movement
		double req_j2_,req_j2_vel_; //joint2 between link_1 and link_2 up_down movement
		int arm_joint12_button_;

		double req_j3_,req_j3_vel_; //joint3 between link_2 and link_3 left_right
		double req_j4_,req_j4_vel_; //joint4 between link_3 and link_4 up_down 
		int arm_joint34_button_;

		double req_j5_,req_j5_vel_; //joint5 between link_4 and link_5 left_right
		double req_j6_,req_j6_vel_; //joint6 between link_5 and link_6 up_down
		int arm_joint56_button_;

		double req_j7_,req_j7_vel_; //joint7 between link_6 and link_7 left_right
		int arm_joint7_button_;

		double arm_left_right_step_; //arm left_right_step
		double arm_up_down_step_; //arm up_down_step

		//signs
		int up_down_,left_right_;   //sign for movements of upper_neck and tray
		
		//common
		int deadman_button_,run_button_;
		bool joy_active_,stopped_;
		double run_factor_, run_factor_param_;


		ros::NodeHandle n_;
		ros::Subscriber joy_sub_;  //subscribe topic joy
		ros::Subscriber joint_states_sub_;  //subscribe topic joint_states
		ros::Publisher torso_pub_;  //publish topic torso_controller/command
		ros::Publisher tray_pub_;  //publish topic tray_controller/command
		ros::Publisher arm_pub_;  //publish topic arm_controller/command
		ros::Publisher base_pub_;  //publish topic base_controller/command
		
		bool got_init_values_;
		double time_for_init_;
		std::vector<std::string> joint_names_;
		std::vector<double> joint_init_values_;

		TeleopCOB();
		void init();
		void joy_cb(const joy::Joy::ConstPtr &joy_msg);
		void joint_states_cb(const sensor_msgs::JointState::ConstPtr &joint_states_msg);
		void update();
		void update_torso();
		void update_tray();
		void update_arm();
		void update_base();
		void setInitValues();
		~TeleopCOB();
};

/*!
* \brief Constructor for TeleopCOB class.
*/
TeleopCOB::TeleopCOB()
{
	got_init_values_ = false;
	time_for_init_ = 0.0;
	joy_active_ = false;
	run_factor_ = 1.0;
	joint_names_.push_back("torso_tray_joint");
	joint_names_.push_back("torso_lower_neck_pan_joint");
	joint_names_.push_back("torso_lower_neck_tilt_joint");
	joint_names_.push_back("torso_upper_neck_pan_joint");
	joint_names_.push_back("torso_upper_neck_tilt_joint");
	joint_names_.push_back("arm_1_joint");
	joint_names_.push_back("arm_2_joint");
	joint_names_.push_back("arm_3_joint");
	joint_names_.push_back("arm_4_joint");
	joint_names_.push_back("arm_5_joint");
	joint_names_.push_back("arm_6_joint");
	joint_names_.push_back("arm_7_joint");
	
	joint_init_values_.resize(joint_names_.size());
}

/*!
* \brief Destructor for TeleopCOB class.
*/
TeleopCOB::~TeleopCOB()
{
}

/*!
* \brief Initializes node to get parameters, subscribe and publish to topics.
*/
void TeleopCOB::init()
{
	// common
	n_.param("run_factor",run_factor_param_,1.5);
	
	// assign buttons
	n_.param("lower_neck_button",lower_neck_button_,6);
	n_.param("upper_neck_button",upper_neck_button_,4);
	n_.param("tray_button",tray_button_,3);
	n_.param("arm_joint12_button",arm_joint12_button_,0);
	n_.param("arm_joint34_button",arm_joint34_button_,1);
	n_.param("arm_joint56_button",arm_joint56_button_,2);
	n_.param("arm_joint7_button",arm_joint7_button_,3);
	n_.param("deadman_button",deadman_button_,5);
	n_.param("run_button",run_button_,7);

	// assign axis
	n_.param("axis_vx",axis_vx_,1);
	n_.param("axis_vy",axis_vy_,0);
	n_.param("axis_vth",axis_vth_,2);
	n_.param("up_down",up_down_,5); //axis[5] tray--up/down; tilt--front/back, here we just name up_down
	n_.param("left_right",left_right_,4);  //axis[4] pan--left/right

	// define step sizes
	n_.param("lower_tilt_step",lower_tilt_step_,0.1); // rad/sec
	n_.param("lower_pan_step",lower_pan_step_,0.1); // rad/sec
	n_.param("upper_tilt_step",upper_tilt_step_,0.15); // rad/sec
	n_.param("upper_pan_step",upper_pan_step_,0.15); // rad/sec
	n_.param("tray_step",tray_step_,0.15); // rad/sec
	n_.param("arm_left_right_step",arm_left_right_step_,0.1); // rad/sec
	n_.param("arm_up_down_step",arm_up_down_step_,0.1); // rad/sec

	n_.param("max_vx", max_vx_, 0.3); // m/sec
	n_.param("max_ax", max_ax_, 0.5); // m/sec^2
	n_.param("max_vy", max_vy_, 0.2); // m/sec
	n_.param("max_ay", max_ay_, 0.5); // m/sec^2
	n_.param("max_vth", max_vth_, 0.3); // rad/sec
	n_.param("max_ath", max_ath_, 0.5); // rad/sec^2

	// output for debugging
	ROS_DEBUG("init::lower_neck_button: %d",lower_neck_button_);
	ROS_DEBUG("init::upper_neck_button: %d",upper_neck_button_);
	ROS_DEBUG("init::tray_button: %d",tray_button_);
	ROS_DEBUG("init::arm_joint12_button: %d",arm_joint12_button_);
	ROS_DEBUG("init::arm_joint34_button: %d",arm_joint34_button_);
	ROS_DEBUG("init::arm_joint56_button: %d",arm_joint56_button_);
	ROS_DEBUG("init::arm_joint7_button: %d",arm_joint7_button_);

	ROS_DEBUG("init::axis_vx: %d",axis_vx_);
	ROS_DEBUG("init::axis_vy: %d",axis_vy_);
	ROS_DEBUG("init::axis_vth: %d",axis_vth_);
	ROS_DEBUG("init::up_down: %d",up_down_);
	ROS_DEBUG("init::left_right: %d",left_right_);

	joy_sub_ = n_.subscribe("/joy",1,&TeleopCOB::joy_cb,this);
	joint_states_sub_ = n_.subscribe("/joint_states",1,&TeleopCOB::joint_states_cb,this);
	torso_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command",1);
	tray_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/tray_controller/command",1);
	arm_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
	base_pub_ = n_.advertise<geometry_msgs::Twist>("/base_controller/command",1);
}

/*!
* \brief Sets initial values for target velocities.
*/
void TeleopCOB::setInitValues()
{
	req_tray_ = joint_init_values_[0];
	req_tray_vel_ = 0.0;
	req_lower_pan_ = joint_init_values_[1];
	req_lower_pan_vel_ = 0.0;
	req_lower_tilt_ = joint_init_values_[2];
	req_lower_tilt_vel_ = 0.0;
	req_upper_pan_ = joint_init_values_[3];
	req_upper_pan_vel_ = 0.0;
	req_upper_tilt_ = joint_init_values_[4];
	req_upper_tilt_vel_ = 0.0;
	req_j1_ = joint_init_values_[5];
	req_j1_vel_ = 0.0;
	req_j2_ = joint_init_values_[6];
	req_j2_vel_ = 0.0;
	req_j3_ = joint_init_values_[7];
	req_j3_vel_ = 0.0;
	req_j4_ = joint_init_values_[8];
	req_j4_vel_ = 0.0;
	req_j5_ = joint_init_values_[9];
	req_j5_vel_ = 0.0;
	req_j6_ = joint_init_values_[10];
	req_j6_vel_ = 0.0;
	req_j7_ = joint_init_values_[11];
	req_j7_vel_ = 0.0;
	req_vx_ = req_vy_ = req_vth_ = 0.0;
	vx_old_ = vy_old_ = vth_old_ = 0.0;
	
	for (int i = 0; i<joint_names_.size(); i++ )
	{
		ROS_DEBUG("joint_name = %s, joint_init_value = %f",joint_names_[i].c_str(),joint_init_values_[i]);
	}
	
	got_init_values_ = true;
}

/*!
* \brief Executes the callback from the joint_states topic.
*
* Gets the current positions.
*
* \param msg JointState
*/
void TeleopCOB::joint_states_cb(const sensor_msgs::JointState::ConstPtr &joint_states_msg)
{
	if (!got_init_values_ && stopped_ && joy_active_)
	{
		ROS_DEBUG("joint_states_cb: getting init values");
		for (int j = 0; j<joint_names_.size(); j++ )
		{
			for (int i = 0; i<joint_states_msg->name.size(); i++ )
			{
				if (joint_states_msg->name[i] == joint_names_[j])
				{	
					joint_init_values_[j] = joint_states_msg->position[i];
					ROS_DEBUG("joint %s found. init value = %f",joint_names_[j].c_str(),joint_init_values_[j]);
					break;
				}
				else
				{
					//ROS_DEBUG("joint %s not found",joint_names_[j].c_str());
				}
			}
		}
		
		setInitValues();
	}
}

/*!
* \brief Executes the callback from the joystick topic.
*
* Gets the configuration
*
* \param joy_msg Joy
*/
void TeleopCOB::joy_cb(const joy::Joy::ConstPtr &joy_msg)
{
	// deadman button to activate joystick
	if(deadman_button_>=0 && deadman_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[deadman_button_]==1)
	{
		if (!joy_active_)
		{
			ROS_INFO("joystick is active");
			joy_active_ = true;
			got_init_values_ = false;
		}
	}
	else
	{
		ROS_DEBUG("joystick is not active");
		joy_active_ = false;
		return;
	}
	
	// run button
	if(run_button_>=0 && run_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[run_button_]==1)
	{
		run_factor_ = run_factor_param_;
	}
	else //button release
	{
		run_factor_ = 1.0;
	}

	//torso
	//lower neck 
	if(lower_neck_button_>=0 && lower_neck_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[lower_neck_button_]==1)
	{
		//pan
		if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
			req_lower_pan_vel_ = (int)joy_msg->buttons[lower_neck_button_]*lower_pan_step_*run_factor_;
		else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
			req_lower_pan_vel_ = -1*(int)joy_msg->buttons[lower_neck_button_]*lower_pan_step_*run_factor_;
		else
			req_lower_pan_vel_ = 0.0;
		ROS_DEBUG("cb::lower neck pan velocity: %f",req_lower_pan_vel_);
		
		//tilt
		if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
			req_lower_tilt_vel_ = (int)joy_msg->buttons[lower_neck_button_]*lower_tilt_step_*run_factor_;
		else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
			req_lower_tilt_vel_ = -1*(int)joy_msg->buttons[lower_neck_button_]*lower_tilt_step_*run_factor_;
		else
			req_lower_tilt_vel_ = 0.0;
		ROS_DEBUG("cb::lower neck tilt velocity: %f",req_lower_tilt_vel_);
	}
	else //button release
	{
		req_lower_pan_vel_ = 0.0;
		req_lower_tilt_vel_ = 0.0;
	}

	//upper neck
	if(upper_neck_button_>=0 && upper_neck_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[upper_neck_button_]==1)
	{
		//pan
		if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
			req_upper_pan_vel_ = (int)joy_msg->buttons[upper_neck_button_]*upper_pan_step_*run_factor_;
		else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
			req_upper_pan_vel_ = -1*(int)joy_msg->buttons[upper_neck_button_]*upper_pan_step_*run_factor_;
		else
			req_upper_pan_vel_ = 0.0;
		ROS_DEBUG("cb::upper neck pan velocity: %f",req_upper_pan_vel_);
	
		//tilt
		if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
			req_upper_tilt_vel_ = (int)joy_msg->buttons[upper_neck_button_]*upper_tilt_step_*run_factor_;
		else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
			req_upper_tilt_vel_ = -1*(int)joy_msg->buttons[upper_neck_button_]*upper_tilt_step_*run_factor_;
		else
			req_upper_tilt_vel_ = 0.0;
		ROS_DEBUG("cb::upper neck tilt velocity: %f",req_upper_tilt_vel_);
	}
	else //button release
	{
		req_upper_tilt_vel_ = 0.0;
		req_upper_pan_vel_ = 0.0;
	}

	//tray
	if(tray_button_>=0 && tray_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[tray_button_]==1)
	{
		if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
			req_tray_vel_ = (int)joy_msg->buttons[tray_button_]*tray_step_*run_factor_;
		else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
			req_tray_vel_ = -1*(int)joy_msg->buttons[tray_button_]*tray_step_*run_factor_;
		else
			req_tray_vel_ = 0.0;
		ROS_DEBUG("cb::tray velocity: %f",req_tray_vel_);
	}
	else //button release
	{
		req_tray_vel_ = 0.0;
	}

	//arm
	//joint12
	if(arm_joint12_button_>=0 && arm_joint12_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint12_button_]==1)
	{
		//joint 1 left or right
		if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
			req_j1_vel_ = -1*(int)joy_msg->buttons[arm_joint12_button_]*arm_left_right_step_*run_factor_;
		else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
			req_j1_vel_ = (int)joy_msg->buttons[arm_joint12_button_]*arm_left_right_step_*run_factor_;
		else
			req_j1_vel_ = 0.0;
		ROS_DEBUG("cb::arm joint1 velocity: %f",req_j1_vel_);

		//joint 2 up or down
		if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
			req_j2_vel_ = (int)joy_msg->buttons[arm_joint12_button_]*arm_up_down_step_*run_factor_;
		else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
			req_j2_vel_ = -1*(int)joy_msg->buttons[arm_joint12_button_]*arm_up_down_step_*run_factor_;
		else
			req_j2_vel_ = 0.0;
		ROS_DEBUG("cb::arm joint2 velocity: %f",req_j2_vel_);
	}
	else //button release
	{
		req_j1_vel_ = 0.0;
		req_j2_vel_ = 0.0;
	} //arm_joint12

	//joint34
	if(arm_joint34_button_>=0 && arm_joint34_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint34_button_]==1)
	{
		//joint 3 left or right
		if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
			req_j3_vel_ = -1*(int)joy_msg->buttons[arm_joint34_button_]*arm_left_right_step_*run_factor_;
		else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
			req_j3_vel_ = (int)joy_msg->buttons[arm_joint34_button_]*arm_left_right_step_*run_factor_;
		else
			req_j3_vel_ = 0.0;
		ROS_DEBUG("cb::arm joint3 velocity: %f",req_j3_vel_);

		//joint 4 up or down
		if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
			req_j4_vel_ = (int)joy_msg->buttons[arm_joint34_button_]*arm_up_down_step_*run_factor_;
		else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
			req_j4_vel_ = -1*(int)joy_msg->buttons[arm_joint34_button_]*arm_up_down_step_*run_factor_;
		else
			req_j4_vel_ = 0.0;
		ROS_DEBUG("cb::arm joint4 velocity: %f",req_j4_vel_);
	}
	else //button release
	{
		req_j3_vel_ = 0.0;
		req_j4_vel_ = 0.0;
	} //arm_joint34

	//joint56
	if(arm_joint56_button_>=0 && arm_joint56_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint56_button_]==1)
	{
		//joint 5 left or right
		if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
			req_j5_vel_ = -1*(int)joy_msg->buttons[arm_joint56_button_]*arm_left_right_step_*run_factor_;
		else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
			req_j5_vel_ = (int)joy_msg->buttons[arm_joint56_button_]*arm_left_right_step_*run_factor_;
		else
			req_j5_vel_ = 0.0;
		ROS_DEBUG("cb::arm joint5 velocity: %f",req_j5_vel_);

		//joint 6 up or down
		if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
			req_j6_vel_ = (int)joy_msg->buttons[arm_joint56_button_]*arm_up_down_step_*run_factor_;
		else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
			req_j6_vel_ = -1*(int)joy_msg->buttons[arm_joint56_button_]*arm_up_down_step_*run_factor_;
		else
			req_j6_vel_ = 0.0;
		ROS_DEBUG("cb::arm joint6 velocity: %f",req_j6_vel_);
	}
	else //button release
	{
		req_j5_vel_ = 0.0;
		req_j6_vel_ = 0.0;
	} //arm_joint56

	//joint7
	if(arm_joint7_button_>=0 && arm_joint7_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint7_button_]==1)
	{
		//joint 7 left or right
		if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
			req_j7_vel_ = -1*(int)joy_msg->buttons[arm_joint7_button_]*arm_left_right_step_*run_factor_;
		else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
			req_j7_vel_ = (int)joy_msg->buttons[arm_joint7_button_]*arm_left_right_step_*run_factor_;
		else
			req_j7_vel_ = 0.0;
		ROS_DEBUG("cb::arm joint7 velocity: %f",req_j7_vel_);
	}
	else //button release
	{
		req_j7_vel_ = 0.0;
	} //arm_joint7

	//base
	if(axis_vx_>=0 && axis_vx_<(int)joy_msg->get_axes_size())
		req_vx_ = joy_msg->axes[axis_vx_]*max_vx_*run_factor_;
	else
		req_vx_ = 0.0;

	if(axis_vy_>=0 && axis_vy_<(int)joy_msg->get_axes_size())
		req_vy_ = joy_msg->axes[axis_vy_]*max_vy_*run_factor_;
	else
		req_vy_ = 0.0;

	if(axis_vth_>=0 && axis_vth_<(int)joy_msg->get_axes_size())
		req_vth_ = joy_msg->axes[axis_vth_]*max_vth_*run_factor_;
	else
		req_vth_ = 0.0;
		
}//joy_cb

/*!
* \brief Main routine for updating all components.
*/
void TeleopCOB::update()
{	
	if (!joy_active_)
	{
		if (!stopped_)
		{
			// stop components: send zero for one time
			req_lower_pan_vel_ = 0.0;
			req_lower_tilt_vel_ = 0.0;
			req_upper_pan_vel_ = 0.0;
			req_upper_tilt_vel_ = 0.0;
			req_tray_vel_ = 0.0;
			req_j1_vel_ = 0.0;
			req_j2_vel_ = 0.0;
			req_j3_vel_ = 0.0;
			req_j4_vel_ = 0.0;
			req_j5_vel_ = 0.0;
			req_j6_vel_ = 0.0;
			req_j7_vel_ = 0.0;
			req_vx_ = vx_old_ = 0.0;
			req_vy_ = vy_old_ = 0.0;
			req_vth_ = vth_old_ = 0.0;

			update_torso();
			update_tray();
			update_arm();
			update_base();
			stopped_ = true;
			ROS_INFO("stopped all components");
		}
		return;
	}

	// set initial values
	if(!got_init_values_)
	{
		if (time_for_init_ < 5.0) // wait for 5 sec, then set init values to 0.0
		{
			ROS_DEBUG("still waiting for initial values, time_for_init_ = %f",time_for_init_);
			time_for_init_ = time_for_init_ + 1.0/PUBLISH_FREQ;
			return;
		}
		else
		{
			ROS_WARN("Timeout waiting for /joint_states message. Setting all init values to 0.0");
			setInitValues();
		}
	}
	
	update_torso();
	update_tray();
	update_arm();
	update_base();
	stopped_ = false;
}

/*!
* \brief Routine for updating the torso commands.
*/
void TeleopCOB::update_torso()
{
	//torso
	double dt = 1.0/double(PUBLISH_FREQ);
	double horizon = 3.0*dt;

	trajectory_msgs::JointTrajectory traj;
	traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
	traj.joint_names.push_back("torso_lower_neck_pan_joint");
	traj.joint_names.push_back("torso_lower_neck_tilt_joint");
	traj.joint_names.push_back("torso_upper_neck_pan_joint");
	traj.joint_names.push_back("torso_upper_neck_tilt_joint");
	traj.points.resize(1);
	traj.points[0].positions.push_back(req_lower_pan_ + req_lower_pan_vel_*horizon);
	traj.points[0].velocities.push_back(req_lower_pan_vel_);  //lower_neck_pan
	traj.points[0].positions.push_back(req_lower_tilt_ + req_lower_tilt_vel_*horizon);
	traj.points[0].velocities.push_back(req_lower_tilt_vel_); //lower_neck_tilt
	traj.points[0].positions.push_back(req_upper_pan_ + req_upper_pan_vel_*horizon);
	traj.points[0].velocities.push_back(req_upper_pan_vel_);  //upper_neck_pan
	traj.points[0].positions.push_back(req_upper_tilt_ + req_upper_tilt_vel_*horizon);
	traj.points[0].velocities.push_back(req_upper_tilt_vel_); //upper_neck_tilt
	traj.points[0].time_from_start = ros::Duration(horizon);

	torso_pub_.publish(traj);

	//update current position 
	req_lower_tilt_ += req_lower_tilt_vel_*dt;
	req_lower_pan_ += req_lower_pan_vel_*dt;
	req_upper_tilt_ += req_upper_tilt_vel_*dt;
	req_upper_pan_ += req_upper_pan_vel_*dt;
}

/*!
* \brief Routine for updating the tray commands.
*/
void TeleopCOB::update_tray()
{
	double dt = 1.0/double(PUBLISH_FREQ);
	double horizon = 3.0*dt;

	trajectory_msgs::JointTrajectory traj;
	traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
	traj.joint_names.push_back("torso_tray_joint");
	traj.points.resize(1);
	traj.points[0].positions.push_back(req_tray_ + req_tray_vel_*horizon);
	traj.points[0].velocities.push_back(req_tray_vel_); 
	traj.points[0].time_from_start = ros::Duration(horizon);

	tray_pub_.publish(traj);

	//update current position 
	req_tray_ += req_tray_vel_*dt;
}

/*!
* \brief Routine for updating the arm commands.
*/
void TeleopCOB::update_arm()
{
	double dt = 1.0/double(PUBLISH_FREQ);
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
	traj.points[0].positions.push_back(req_j1_ + req_j1_vel_*horizon);
	traj.points[0].velocities.push_back(req_j1_vel_);  //joint1
	traj.points[0].positions.push_back(req_j2_ + req_j2_vel_*horizon);
	traj.points[0].velocities.push_back(req_j2_vel_); //joint2
	traj.points[0].positions.push_back(req_j3_ + req_j3_vel_*horizon);
	traj.points[0].velocities.push_back(req_j3_vel_); //joint3
	traj.points[0].positions.push_back(req_j4_ + req_j4_vel_*horizon);
	traj.points[0].velocities.push_back(req_j4_vel_); //joint4
	traj.points[0].positions.push_back(req_j5_ + req_j5_vel_*horizon);
	traj.points[0].velocities.push_back(req_j5_vel_); //joint5
	traj.points[0].positions.push_back(req_j6_ + req_j6_vel_*horizon);
	traj.points[0].velocities.push_back(req_j6_vel_); //joint6
	traj.points[0].positions.push_back(req_j7_ + req_j7_vel_*horizon);
	traj.points[0].velocities.push_back(req_j7_vel_); //joint7
	traj.points[0].time_from_start = ros::Duration(horizon);
	
	arm_pub_.publish(traj);

	//update current position 
	req_j1_ += req_j1_vel_*dt;
	req_j2_ += req_j2_vel_*dt;
	req_j3_ += req_j3_vel_*dt;
	req_j4_ += req_j4_vel_*dt;
	req_j5_ += req_j5_vel_*dt;
	req_j6_ += req_j6_vel_*dt;
	req_j7_ += req_j7_vel_*dt;
}

/*!
* \brief Routine for updating the base commands.
*/
void TeleopCOB::update_base()
{
	double dt = 1.0/double(PUBLISH_FREQ);
	double vx,vy,vth = 0.0;
	
	// filter vx with ramp
	if ((req_vx_ - vx_old_)/dt > max_ax_)
	{
		vx = vx_old_ + max_ax_*dt;
	}
	else if ((req_vx_ - vx_old_)/dt < -max_ax_)
	{
		vx = vx_old_ - max_ax_*dt;
	}
	else
	{
		vx = req_vx_;
	}
	vx_old_ = vx;
	
	// filter vy with ramp
	if ((req_vy_ - vy_old_)/dt > max_ay_)
	{
		vy = vy_old_ + max_ay_*dt;
	}
	else if ((req_vy_ - vy_old_)/dt < -max_ay_)
	{
		vy = vy_old_ - max_ay_*dt;
	}
	else
	{
		vy = req_vy_;
	}
	vy_old_ = vy;

	// filter vth with ramp
	if ((req_vth_ - vth_old_)/dt > max_ath_)
	{
		vth = vth_old_ + max_ath_*dt;
	}
	else if ((req_vth_ - vth_old_)/dt < -max_ath_)
	{
		vth = vth_old_ - max_ath_*dt;
	}
	else
	{
		vth = req_vth_;
	}
	vth_old_ = vth;

	
	geometry_msgs::Twist cmd;
	cmd.linear.x = vx;
	cmd.linear.y = vy;
	cmd.angular.z = vth;

	base_pub_.publish(cmd);
}

/*!
* \brief Main loop of ROS node.
*
* Running with a specific frequency defined by loop_rate.
*/
int main(int argc,char **argv)
{
	ros::init(argc,argv,"teleop_cob");
	TeleopCOB teleop_cob;
	teleop_cob.init();

	ros::Rate loop_rate(PUBLISH_FREQ); //Hz
	while(teleop_cob.n_.ok())
	{
		ros::spinOnce();
		teleop_cob.update();
		loop_rate.sleep();
	}

	exit(0);
	return(0);
}
