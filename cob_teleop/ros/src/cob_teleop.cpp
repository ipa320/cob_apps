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
 * \author
 *   Changed by Sofie Nilsson Jan 2011:
 *   Read module configuration from parameter server. Joysick map to joints is still
 *   not as general as it should be.
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
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

#include <cob_srvs/Trigger.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

const int PUBLISH_FREQ = 20.0;

/*!
 * \brief Implementation of teleoperation node.
 *
 * Sends direct commands to different components
 */
class TeleopCOB
{
public:
	struct joint_module{
		std::string key;
		std::vector<std::string> joint_names;
		std::vector<double> req_joint_pos_;
		std::vector<double> req_joint_vel_;
		std::vector<double> steps;
		ros::Publisher module_publisher_;
		ros::Publisher module_publisher_brics_;
	};

	std::map<std::string,joint_module> joint_modules_; //std::vector<std::string> module_names;

	struct base_module_struct{
		std::vector<double> req_vel_;
		std::vector<double> vel_old_; //,vy_old_,vth_old_;
		std::vector<double> max_vel_; //max_vx_,max_vy_,max_vth_;
		std::vector<double> max_acc_; //max_ax_,max_ay_,max_ath_;
		ros::Publisher base_publisher_;
	} base_module_;

	bool has_base_module_;

	int lower_neck_button_,upper_neck_button_; //buttons
	int tray_button_;
	int axis_vx_,axis_vy_,axis_vth_;
	int arm_joint12_button_;
	int arm_joint34_button_;
	int arm_joint56_button_;
	int arm_joint7_button_;
	//signs
	int up_down_, left_right_;   //sign for movements of upper_neck and tray

	//common
	int deadman_button_, run_button_, stop_base_button_, recover_base_button_;
	bool joy_active_, stopped_;
	double run_factor_, run_factor_param_;


	ros::NodeHandle n_;
	ros::Subscriber joy_sub_;  //subscribe topic joy
	ros::Subscriber joint_states_sub_;  //subscribe topic joint_states

	bool got_init_values_;
	double time_for_init_;

	struct combined_joints_struct{
		std::vector<std::string> joint_names_;
		std::vector<double> joint_init_values_;
		std::vector<joint_module*> module_ref_;
	}combined_joints_;
	std::vector<std::string> joint_names_;
	std::vector<double> joint_init_values_;

	TeleopCOB();
	void waitForParameters();
	void getConfigurationFromParameters();
	void init();
	void joy_cb(const joy::Joy::ConstPtr &joy_msg);
	void joint_states_cb(const sensor_msgs::JointState::ConstPtr &joint_states_msg);
	void update();
	void update_joint_modules();
	void update_base();
	void setInitValues();
	~TeleopCOB();

private:
	bool assign_joint_module(std::string,XmlRpc::XmlRpcValue);
	bool assign_base_module(XmlRpc::XmlRpcValue);
};

void TeleopCOB::waitForParameters()
{
	while(!n_.hasParam("/robot_config/robot_modules"))
	{
		sleep(1); // sleep 1 s while waiting for parameter to be loaded
		ROS_WARN("no robot_module list loaded");
	} // block until robot modules are loded

	// get the list with modules that should be loaded
	XmlRpc::XmlRpcValue module_list;
	n_.getParam("/robot_config/robot_modules",module_list);
	ROS_ASSERT(module_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i=0;i<module_list.size();i++)
	{
		ROS_ASSERT(module_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		std::string s((std::string)module_list[i]);
		ROS_DEBUG("searching for module = %s", s.c_str());

		// block until required module is loaded
		while(!n_.hasParam("modules/"+s))
		{
			sleep(1); // sleep 1 s while waiting for parameter to be loaded
			ROS_WARN("required module not loaded");
		}
	}

	ROS_DEBUG("module list found");
}

void TeleopCOB::getConfigurationFromParameters()
{
	//std::map<std::string,joint_module> joint_modules; //std::vector<std::string> module_names;
	if(n_.hasParam("modules"))
	{
		XmlRpc::XmlRpcValue modules;
		ROS_DEBUG("modules found ");
		n_.getParam("modules", modules);
		if(modules.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			ROS_DEBUG("modules are of type struct with size %d",(int)modules.size());

			for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator p=modules.begin();p!=modules.end();++p)
			{
				std::string mod_name = p->first;
				ROS_DEBUG("module name: %s",mod_name.c_str());
				XmlRpc::XmlRpcValue mod_struct = p->second;
				if(mod_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
					ROS_WARN("invalid module, name: %s",mod_name.c_str());
				// search for joint_name parameter in current module struct to determine which type of module
				// only joint mods or wheel mods supported
				// which mens that is no joint names are found, then the module is a wheel module
				// TODO replace with build in find, but could not get it to work
				if(!assign_joint_module(mod_name, mod_struct))
				{
					// add wheel module struct
					ROS_DEBUG("wheel module found");
					assign_base_module(mod_struct);
				}
			}
		}
	}
}

/**
 * Tries to read joint module configurations from XmlRpcValue object.
 * If the module is a joint module, it contains a joint name array.
 * A typical joint module has the following configuration structure:
 * struct {
 * 	  joint_names: ['head_pan_joint','head_tilt_joint'],
 * 	  joint_step: 0.075
 * }
 * @param mod_struct configuration object struct
 * @return true if the configuration object hols a joint module config, else false
 */
bool TeleopCOB::assign_joint_module(std::string mod_name, XmlRpc::XmlRpcValue mod_struct)
{
	// search for joint_name parameter in current module struct to determine which type of module
	// only joint mods or wheel mods supported
	// which mens that is no joint names are found, then the module is a wheel module
	// TODO replace with build in find, but could not get it to work
	bool is_joint_module = false;
	joint_module tempModule;
	for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator ps=mod_struct.begin();ps!=mod_struct.end();++ps)
	{
		std::string par_name = ps->first;
		ROS_DEBUG("par name: %s",par_name.c_str());

		if(par_name.compare("joint_names")==0)
		{
			ROS_DEBUG("joint names found");
			XmlRpc::XmlRpcValue joint_names = ps->second;

			ROS_ASSERT(joint_names.getType() == XmlRpc::XmlRpcValue::TypeArray);
			ROS_DEBUG("joint_names.size: %d \n", joint_names.size());
			for(int i=0;i<joint_names.size();i++)
			{
				ROS_ASSERT(joint_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
				std::string s((std::string)joint_names[i]);
				ROS_DEBUG("joint_name found = %s",s.c_str());
				tempModule.joint_names.push_back(s);
			}
			// set size of other vectors according to the joint name vector
			tempModule.req_joint_pos_.resize(joint_names.size());
			tempModule.req_joint_vel_.resize(joint_names.size());

			is_joint_module = true;
			//break; // no need to continue searching if joint names are found
		}else if(par_name.compare("joint_step")==0){
			ROS_DEBUG("joint steps found");
			XmlRpc::XmlRpcValue joint_steps = ps->second;

			ROS_ASSERT(joint_steps.getType() == XmlRpc::XmlRpcValue::TypeArray);
			ROS_DEBUG("joint_steps.size: %d \n", joint_steps.size());
			for(int i=0;i<joint_steps.size();i++)
			{
				ROS_ASSERT(joint_steps[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				double step((double)joint_steps[i]);
				ROS_DEBUG("joint_step found = %f",step);
				tempModule.steps.push_back(step);
			}
		}
	}
	if(is_joint_module)
	{
		// assign publisher
		tempModule.module_publisher_ = n_.advertise<trajectory_msgs::JointTrajectory>(("/"+mod_name+"_controller/command"),1);
		tempModule.module_publisher_brics_ = n_.advertise<brics_actuator::JointVelocities>(("/"+mod_name+"_controller/command_vel"),1);
		// store joint module in collection
		ROS_DEBUG("head module stored");
		joint_modules_.insert(std::pair<std::string,joint_module>(mod_name,tempModule));
	}
	return is_joint_module;
}
/**
 * Tries to read base module configurations from XmlRpcValue object.
 * A base module is supposed to contain vectors 3 element vectors (x,y,th)
 * with max acceleration and velocity. Example:
 * struct {
 * 	   max_velocity: [0.3, 0.2, 0.3],
 * 	  max_acceleration: [0.5, 0.5, 0.7]
 * }
 * @param mod_struct configuration object struct
 * @return true no check is currently performed TODO check
 */
bool TeleopCOB::assign_base_module(XmlRpc::XmlRpcValue mod_struct)
{
	for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator ps=mod_struct.begin();ps!=mod_struct.end();++ps)
	{
		std::string par_name = ps->first;
		ROS_DEBUG("par name: %s",par_name.c_str());

		if(par_name.compare("max_velocity")==0)
		{
			ROS_DEBUG("max vel found");
			XmlRpc::XmlRpcValue max_vel = ps->second;

			ROS_ASSERT(max_vel.getType() == XmlRpc::XmlRpcValue::TypeArray);
			if(max_vel.size()!=3){ROS_WARN("invalid base parameter size");}
			ROS_DEBUG("max_vel.size: %d \n", max_vel.size());
			for(int i=0;i<max_vel.size();i++)
			{
				ROS_ASSERT(max_vel[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				double val = (double)max_vel[i];
				ROS_DEBUG("max vel value = %f",val);
				base_module_.max_vel_.push_back(val);
			}
		}
		else if(par_name.compare("max_acceleration")==0)
		{
			ROS_DEBUG("max acc found");
			XmlRpc::XmlRpcValue max_acc = ps->second;

			ROS_ASSERT(max_acc.getType() == XmlRpc::XmlRpcValue::TypeArray);
			if(max_acc.size()!=3){ROS_DEBUG("invalid base parameter size");}
			ROS_DEBUG("max_acc.size: %d \n", max_acc.size());
			for(int i=0;i<max_acc.size();i++)
			{
				ROS_ASSERT(max_acc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				double val = (double)max_acc[i];
				ROS_DEBUG("max acc value = %f", val);
				base_module_.max_acc_.push_back(val);
			}
		}
		else
		{
			ROS_WARN("unsupported base module parameter read");
		}
	}
	// make all the vectors the same length
	// the vector size is not completely safe since only warning is
	// issued if max value arrays has the wrong length
	base_module_.req_vel_.resize(base_module_.max_acc_.size());
	base_module_.vel_old_.resize(base_module_.max_acc_.size());
	base_module_.base_publisher_ = n_.advertise<geometry_msgs::Twist>("/base_controller/command",1);
	ROS_DEBUG("base module stored");
	has_base_module_ = true;
	return true;
}

/*!
 * \brief Constructor for TeleopCOB class.
 */
TeleopCOB::TeleopCOB()
{
	waitForParameters();
	getConfigurationFromParameters(); // assign configuration and subscribe to topics
	got_init_values_ = false;
	time_for_init_ = 0.0;
	joy_active_ = false;
	run_factor_ = 1.0;

	// add all found joint names to joint_names_vector, which is used to pass values to the state aggregator
	for(std::map<std::string,joint_module>::iterator module_it=joint_modules_.begin();module_it!=joint_modules_.end();++module_it){
		std::vector<std::string> names = (module_it->second).joint_names;
		for(int i=0; i<names.size();i++){
			joint_names_.push_back(names[i]);
			combined_joints_.joint_names_.push_back(names[i]); // puch joit name to combined collection
			combined_joints_.joint_init_values_.push_back(0.0); // be sure that a init value is related to the joint name
			combined_joints_.module_ref_.push_back((joint_module*)(&(module_it->second))); // store a reference to the module containing the joint
		}
	}
	joint_init_values_.resize(joint_names_.size());
}

/*!
 * \brief Destructor for TeleopCOB class.
 */
TeleopCOB::~TeleopCOB()
{
}

/*!
 * \brief Initializes node to get parameters, subscribe to topics.
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
	n_.param("stop_base_button",stop_base_button_,8);
	n_.param("recover_base_button",recover_base_button_,9);

	// assign axis
	n_.param("axis_vx",axis_vx_,1);
	n_.param("axis_vy",axis_vy_,0);
	n_.param("axis_vth",axis_vth_,2);
	n_.param("up_down",up_down_,5); //axis[5] tray--up/down; tilt--front/back, here we just name up_down
	n_.param("left_right",left_right_,4);  //axis[4] pan--left/right

	// output for debugging
	ROS_DEBUG("init::lower_neck_button: %d",lower_neck_button_);
	ROS_DEBUG("init::upper_neck_button: %d",upper_neck_button_);
	ROS_DEBUG("init::tray_button: %d",tray_button_);
	ROS_DEBUG("init::arm_joint12_button: %d",arm_joint12_button_);
	ROS_DEBUG("init::arm_joint34_button: %d",arm_joint34_button_);
	ROS_DEBUG("init::arm_joint56_button: %d",arm_joint56_button_);
	ROS_DEBUG("init::arm_joint7_button: %d",arm_joint7_button_);
	ROS_DEBUG("init::deadman_button: %d",deadman_button_);
	ROS_DEBUG("init::run_button: %d",run_button_);
	ROS_DEBUG("init::stop_base_button: %d",stop_base_button_);
	ROS_DEBUG("init::recover_base_button: %d",recover_base_button_);

	ROS_DEBUG("init::axis_vx: %d",axis_vx_);
	ROS_DEBUG("init::axis_vy: %d",axis_vy_);
	ROS_DEBUG("init::axis_vth: %d",axis_vth_);
	ROS_DEBUG("init::up_down: %d",up_down_);
	ROS_DEBUG("init::left_right: %d",left_right_);

	// TODO general!!!
	joy_sub_ = n_.subscribe("/joy",1,&TeleopCOB::joy_cb,this);
	joint_states_sub_ = n_.subscribe("/joint_states",1,&TeleopCOB::joint_states_cb,this);
}

/*!
 * \brief Sets initial values for target velocities.
 */
void TeleopCOB::setInitValues()
{

	// loop trough all the joints in the combined collection
	for(int i=0; i<combined_joints_.joint_init_values_.size();i++){
		//loop trough all the joints in module containing joint settings,
		//and try to find the one with a name matching the currently browsed joint
		//in the combined collection
		for(int j=0; j<combined_joints_.module_ref_[i]->joint_names.size();j++){
			// if the matching joint is found, assign value to pos command and stop looking for this name
			if(combined_joints_.module_ref_[i]->joint_names[j].compare(combined_joints_.joint_names_[i])==0){
				combined_joints_.module_ref_[i]->req_joint_pos_[j] = combined_joints_.joint_init_values_[i];
				combined_joints_.module_ref_[i]->req_joint_vel_[j] = 0.0; // initalize velocity cmd to 0
				break; // node found, break the search
			}
		}
	}

	// base init values (velocities) are already set to 0 by default

	got_init_values_ = true;
}

/*!
 * \brief Executes the callback from the joint_states topic. (published by joint state driver)
 *
 * Only used to get the initaial joint positions.
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
				ROS_DEBUG("joint names in init: %s should match %s",joint_names_[j].c_str(),joint_states_msg->name[i].c_str());
				if (joint_states_msg->name[i] == joint_names_[j])
				{
					joint_init_values_[j] = joint_states_msg->position[i];
					if(joint_names_[j]!=combined_joints_.joint_names_[j])
						ROS_ERROR("error in new joint name collection, name miss match.");
					combined_joints_.joint_init_values_[j] = joint_states_msg->position[i]; //new
					ROS_DEBUG("joint %s found. init value = %f",joint_names_[j].c_str(),joint_init_values_[j]);
					break;
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
	
	// recover base button
	if(recover_base_button_>=0 && recover_base_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[recover_base_button_]==1)
	{
		ros::ServiceClient client_init_base = n_.serviceClient<cob_srvs::Trigger>("/base_controller/init");
	
		ROS_INFO("Init base");
		cob_srvs::Trigger srv = cob_srvs::Trigger();
		if (client_init_base.call(srv))
		{
			ROS_INFO("Base init successfully");
		}
		else
		{
			ROS_ERROR("Failed to call service /base_controller/init");
		}
		
		ros::ServiceClient client_recover_base = n_.serviceClient<cob_srvs::Trigger>("/base_controller/recover");
	
		ROS_INFO("Recover base");
		if (client_recover_base.call(srv))
		{
			ROS_INFO("Base recovered successfully");
		}
		else
		{
			ROS_ERROR("Failed to call service /base_controller/recover");
		}
	}
	
	// stop base button
	if(stop_base_button_>=0 && stop_base_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[stop_base_button_]==1)
	{
		ros::ServiceClient client_stop_base = n_.serviceClient<cob_srvs::Trigger>("/base_controller/stop");
	
		ROS_INFO("Stop base");
		cob_srvs::Trigger srv = cob_srvs::Trigger();
		if (client_stop_base.call(srv))
		{
			ROS_INFO("Base stop successfully");
		}
		else
		{
			ROS_ERROR("Failed to call service /base_controller/stop");
		}
	}

	// TODO add map for buttons

	// head
	if(joint_modules_.find("head")!=joint_modules_.end())
	{
		if(upper_neck_button_>=0 &&
				upper_neck_button_<(int)joy_msg->buttons.size() &&
				joy_msg->buttons[upper_neck_button_]==1)
		{
			//pan (turn)
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
				joint_modules_["head"].req_joint_vel_[0] = (int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[0]*run_factor_;//upper_pan_step_*run_factor_;
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
				joint_modules_["head"].req_joint_vel_[0] = -1*(int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[0]*run_factor_;//upper_pan_step_*run_factor_;
			else
				joint_modules_["head"].req_joint_vel_[0]= 0.0;
			ROS_DEBUG("cb::upper neck pan velocity: %f",joint_modules_["head"].req_joint_vel_[0]);

			//tilt (nod)
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["head"].req_joint_vel_[1] = -1*(int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[1]*run_factor_;//upper_tilt_step_*run_factor_;
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["head"].req_joint_vel_[1] = (int)joy_msg->buttons[upper_neck_button_]*joint_modules_["head"].steps[1]*run_factor_;//upper_tilt_step_*run_factor_;
			else
				joint_modules_["head"].req_joint_vel_[1] = 0.0;
			ROS_DEBUG("cb::upper neck tilt velocity: %f",joint_modules_["head"].req_joint_vel_[1]);

		}
		else
		{
			joint_modules_["head"].req_joint_vel_[0] = 0.0;
			joint_modules_["head"].req_joint_vel_[1] = 0.0;
		}
	}
	if(joint_modules_.find("torso")!=joint_modules_.end())
	{
		// torso TODO update with general as well
		//lower neck
		if(lower_neck_button_>=0 &&
				lower_neck_button_<(int)joy_msg->buttons.size() &&
				joy_msg->buttons[lower_neck_button_]==1)
		{
			//pan
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
				joint_modules_["torso"].req_joint_vel_[0] = (int)joy_msg->buttons[lower_neck_button_]*joint_modules_["torso"].steps[0]*run_factor_; //req_lower_pan_vel_
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
				joint_modules_["torso"].req_joint_vel_[0] = -1*(int)joy_msg->buttons[lower_neck_button_]*joint_modules_["torso"].steps[0]*run_factor_; //req_lower_pan_vel_
			else
				joint_modules_["torso"].req_joint_vel_[0] = 0.0;
			ROS_DEBUG("cb::lower neck pan velocity: %f",joint_modules_["torso"].req_joint_vel_[0]);

			//tilt
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["torso"].req_joint_vel_[1] = (int)joy_msg->buttons[lower_neck_button_]*joint_modules_["torso"].steps[1]*run_factor_; //req_lower_tilt_vel_
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["torso"].req_joint_vel_[1] = -1*(int)joy_msg->buttons[lower_neck_button_]*joint_modules_["torso"].steps[1]*run_factor_;
			else
				joint_modules_["torso"].req_joint_vel_[1] = 0.0;
			ROS_DEBUG("cb::lower neck tilt velocity: %f",joint_modules_["torso"].req_joint_vel_[1]);
		}
		else //button release
		{
			joint_modules_["torso"].req_joint_vel_[0] = 0.0;
			joint_modules_["torso"].req_joint_vel_[1] = 0.0;
		}

		//upper neck
		if(upper_neck_button_>=0 &&
				upper_neck_button_<(int)joy_msg->buttons.size() &&
				joy_msg->buttons[upper_neck_button_]==1)
		{
			//pan
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
				joint_modules_["torso"].req_joint_vel_[2] = (int)joy_msg->buttons[upper_neck_button_]*joint_modules_["torso"].steps[2]*run_factor_;
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
				joint_modules_["torso"].req_joint_vel_[2] = -1*(int)joy_msg->buttons[upper_neck_button_]*joint_modules_["torso"].steps[2]*run_factor_;
			else
				joint_modules_["torso"].req_joint_vel_[2] = 0.0;
			ROS_DEBUG("cb::upper neck pan velocity: %f",joint_modules_["torso"].req_joint_vel_[2]);

			//tilt
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["torso"].req_joint_vel_[3] = (int)joy_msg->buttons[upper_neck_button_]*joint_modules_["torso"].steps[3]*run_factor_;
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["torso"].req_joint_vel_[3] = -1*(int)joy_msg->buttons[upper_neck_button_]*joint_modules_["torso"].steps[3]*run_factor_;
			else
				joint_modules_["torso"].req_joint_vel_[3] = 0.0;
			ROS_DEBUG("cb::upper neck tilt velocity: %f",joint_modules_["torso"].req_joint_vel_[3]);
		}
		else //button release
		{
			joint_modules_["torso"].req_joint_vel_[2] = 0.0;
			joint_modules_["torso"].req_joint_vel_[3] = 0.0;
		}
	}

	if(joint_modules_.find("tray")!=joint_modules_.end())
	{
		//tray
		if(tray_button_>=0 && tray_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[tray_button_]==1)
		{
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["tray"].req_joint_vel_[0] = (int)joy_msg->buttons[tray_button_]*joint_modules_["tray"].steps[0]*run_factor_;
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["tray"].req_joint_vel_[0] = -1*(int)joy_msg->buttons[tray_button_]*joint_modules_["tray"].steps[0]*run_factor_;
			else
				joint_modules_["tray"].req_joint_vel_[0] = 0.0;
			ROS_DEBUG("cb::tray velocity: %f",joint_modules_["tray"].req_joint_vel_[0]);
		}
		else //button release
		{
			joint_modules_["tray"].req_joint_vel_[0] = 0.0;
		}
	}
	if(joint_modules_.find("arm")!=joint_modules_.end())
	{
		//arm
		//publish_arm_ = false;

		//joint12
		if(joint_modules_["arm"].req_joint_vel_.size()>1 && arm_joint12_button_>=0 && arm_joint12_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint12_button_]==1)
		{
			//joint 1 left or right
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
				joint_modules_["arm"].req_joint_vel_[0] = -1*(int)joy_msg->buttons[arm_joint12_button_]*joint_modules_["arm"].steps[0]*run_factor_;
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
				joint_modules_["arm"].req_joint_vel_[0] = (int)joy_msg->buttons[arm_joint12_button_]*joint_modules_["arm"].steps[0]*run_factor_;
			else
				joint_modules_["arm"].req_joint_vel_[0] = 0.0;
			ROS_DEBUG("cb::arm joint1 velocity: %f",joint_modules_["arm"].req_joint_vel_[0]);

			//joint 2 up or down
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["arm"].req_joint_vel_[1] = (int)joy_msg->buttons[arm_joint12_button_]*joint_modules_["arm"].steps[1]*run_factor_;
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["arm"].req_joint_vel_[1] = -1*(int)joy_msg->buttons[arm_joint12_button_]*joint_modules_["arm"].steps[1]*run_factor_;
			else
				joint_modules_["arm"].req_joint_vel_[1] = 0.0;
			ROS_DEBUG("cb::arm joint2 velocity: %f",joint_modules_["arm"].req_joint_vel_[1]);
			//publish_arm_ = true;
		}
		else //button release
		{
			joint_modules_["arm"].req_joint_vel_[0] = 0.0;
			joint_modules_["arm"].req_joint_vel_[1] = 0.0;
		} //arm_joint12

		//joint34
		if(joint_modules_["arm"].req_joint_vel_.size()>3 && arm_joint34_button_>=0 && arm_joint34_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint34_button_]==1)
		{
			//joint 3 left or right
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
				joint_modules_["arm"].req_joint_vel_[2] = -1*(int)joy_msg->buttons[arm_joint34_button_]*joint_modules_["arm"].steps[2]*run_factor_;
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
				joint_modules_["arm"].req_joint_vel_[2] = (int)joy_msg->buttons[arm_joint34_button_]*joint_modules_["arm"].steps[2]*run_factor_;
			else
				joint_modules_["arm"].req_joint_vel_[2] = 0.0;
			ROS_DEBUG("cb::arm joint3 velocity: %f",joint_modules_["arm"].req_joint_vel_[2]);

			//joint 4 up or down
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["arm"].req_joint_vel_[3] = (int)joy_msg->buttons[arm_joint34_button_]*joint_modules_["arm"].steps[3]*run_factor_;
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["arm"].req_joint_vel_[3] = -1*(int)joy_msg->buttons[arm_joint34_button_]*joint_modules_["arm"].steps[3]*run_factor_;
			else
				joint_modules_["arm"].req_joint_vel_[3] = 0.0;
			ROS_DEBUG("cb::arm joint4 velocity: %f",joint_modules_["arm"].req_joint_vel_[3]);
			//publish_arm_ = true;
		}
		else //button release
		{
			joint_modules_["arm"].req_joint_vel_[2] = 0.0;
			joint_modules_["arm"].req_joint_vel_[3] = 0.0;
		} //arm_joint34

		//joint56
		if(joint_modules_["arm"].req_joint_vel_.size()>4 && arm_joint56_button_>=0 && arm_joint56_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint56_button_]==1)
		{
			//joint 5 left or right
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
				joint_modules_["arm"].req_joint_vel_[4] = -1*(int)joy_msg->buttons[arm_joint56_button_]*joint_modules_["arm"].steps[4]*run_factor_;
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
				joint_modules_["arm"].req_joint_vel_[4] = (int)joy_msg->buttons[arm_joint56_button_]*joint_modules_["arm"].steps[4]*run_factor_;
			else
				joint_modules_["arm"].req_joint_vel_[4] = 0.0;
			ROS_DEBUG("cb::arm joint5 velocity: %f",joint_modules_["arm"].req_joint_vel_[4]);

			//joint 6 up or down
			if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]>0.0)
				joint_modules_["arm"].req_joint_vel_[5] = (int)joy_msg->buttons[arm_joint56_button_]*joint_modules_["arm"].steps[5]*run_factor_;
			else if(up_down_>=0 && up_down_<(int)joy_msg->axes.size() && joy_msg->axes[up_down_]<0.0)
				joint_modules_["arm"].req_joint_vel_[5] = -1*(int)joy_msg->buttons[arm_joint56_button_]*joint_modules_["arm"].steps[5]*run_factor_;
			else
				joint_modules_["arm"].req_joint_vel_[5] = 0.0;
			ROS_DEBUG("cb::arm joint6 velocity: %f",joint_modules_["arm"].req_joint_vel_[5]);
			//publish_arm_ = true;
		}
		else //button release
		{
			joint_modules_["arm"].req_joint_vel_[4] = 0.0;
			joint_modules_["arm"].req_joint_vel_[5] = 0.0;
		} //arm_joint56

		//joint7
		if(joint_modules_["arm"].req_joint_vel_.size()>5 && arm_joint7_button_>=0 && arm_joint7_button_<(int)joy_msg->buttons.size() && joy_msg->buttons[arm_joint7_button_]==1)
		{
			//joint 7 left or right
			if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]<0.0)
			{
				joint_modules_["arm"].req_joint_vel_[6] = -1*(int)joy_msg->buttons[arm_joint7_button_]*joint_modules_["arm"].steps[6]*run_factor_;
				//publish_arm_ = true;
			}
			else if(left_right_>=0 && left_right_<(int)joy_msg->axes.size() && joy_msg->axes[left_right_]>0.0)
			{
				joint_modules_["arm"].req_joint_vel_[6] = (int)joy_msg->buttons[arm_joint7_button_]*joint_modules_["arm"].steps[6]*run_factor_;
				//publish_arm_ = true;
			}
			else
				joint_modules_["arm"].req_joint_vel_[6] = 0.0;
			ROS_DEBUG("cb::arm joint7 velocity: %f",joint_modules_["arm"].req_joint_vel_[6]);
		}
		else //button release
		{
			joint_modules_["arm"].req_joint_vel_[6] = 0.0;
		} //arm_joint7
	}
	//================base================
	if(has_base_module_ && base_module_.req_vel_.size()==3)
	{
		if(axis_vx_>=0 && axis_vx_<(int)joy_msg->get_axes_size())
			base_module_.req_vel_[0] = joy_msg->axes[axis_vx_]*base_module_.max_vel_[0]*run_factor_;
		else
			base_module_.req_vel_[0] = 0.0;

		if(axis_vy_>=0 && axis_vy_<(int)joy_msg->get_axes_size())
			base_module_.req_vel_[1] = joy_msg->axes[axis_vy_]*base_module_.max_vel_[1]*run_factor_;//req_vy_ = joy_msg->axes[axis_vy_]*max_vy_*run_factor_;
		else
			base_module_.req_vel_[1] = 0.0; //req_vy_ = 0.0;

		if(axis_vth_>=0 && axis_vth_<(int)joy_msg->get_axes_size())
			base_module_.req_vel_[2] = joy_msg->axes[axis_vth_]*base_module_.max_vel_[2]*run_factor_;//req_vth_ = joy_msg->axes[axis_vth_]*max_vth_*run_factor_;
		else
			base_module_.req_vel_[2] = 0.0; //req_vth_ = 0.0;
	}

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
			for(std::map<std::string,joint_module>::iterator module_it=joint_modules_.begin();module_it!=joint_modules_.end();++module_it)
			{
				for(int i=0; i<module_it->second.req_joint_vel_.size();i++)
				{
					module_it->second.req_joint_vel_[i] = 0.0;
				}
			}

			if(has_base_module_)
			{
				for(int i=0; i<3; i++){
					base_module_.req_vel_[i]=0;
					base_module_.vel_old_[i]=0;
				}
			}

			update_joint_modules();
			update_base();
			stopped_ = true;
			ROS_INFO("stopped all components");
		}
		return;
	}

	// set initial values
	if(!got_init_values_)
	{
		if (time_for_init_ < 1.0) // wait for 1 sec, then set init values to 0.0
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

	update_joint_modules();
	update_base();
	stopped_ = false;
}

void TeleopCOB::update_joint_modules()
{
	double dt = 1.0/double(PUBLISH_FREQ);
	double horizon = 3.0*dt;

	joint_module* jointModule;
	for(std::map<std::string,joint_module>::iterator it = joint_modules_.begin();it!=joint_modules_.end();++it)
	{
		jointModule = (joint_module*)(&(it->second));

		trajectory_msgs::JointTrajectory traj;
		traj.header.stamp = ros::Time::now()+ros::Duration(0.01);
		traj.points.resize(1);
		brics_actuator::JointVelocities cmd_vel;
		brics_actuator::JointValue joint_vel;
		joint_vel.timeStamp = traj.header.stamp;
		joint_vel.unit = "rad";
		for( int i = 0; i<jointModule->joint_names.size();i++)
		{
			// as trajectory message
			traj.joint_names.push_back(jointModule->joint_names[i]);
			traj.points[0].positions.push_back(jointModule->req_joint_pos_[i] + jointModule->req_joint_vel_[i]*horizon);
			traj.points[0].velocities.push_back(jointModule->req_joint_vel_[i]);  //lower_neck_pan
			// as brics message
			joint_vel.value = jointModule->req_joint_vel_[i];
			joint_vel.joint_uri = jointModule->joint_names[i];
			cmd_vel.velocities.push_back(joint_vel);
			// update current positions
			jointModule->req_joint_pos_[i] += jointModule->req_joint_vel_[i]*horizon;
		}

		traj.points[0].time_from_start = ros::Duration(horizon);

		jointModule->module_publisher_.publish(traj); // TODO, change
		jointModule->module_publisher_brics_.publish(cmd_vel);
	}
}

/*!
 * \brief Routine for updating the base commands.
 */
void TeleopCOB::update_base()
{
	if(!has_base_module_)
		return;
	double dt = 1.0/double(PUBLISH_FREQ);
	double v[] = {0.0,0.0,0.0};

	for( int i =0; i<3; i++)
	{
		// filter v with ramp
		if ((base_module_.req_vel_[i]-base_module_.vel_old_[i])/dt > base_module_.max_acc_[i])
		{
			v[i] = base_module_.vel_old_[i] + base_module_.max_acc_[i]*dt;
		}
		else if((base_module_.req_vel_[i]-base_module_.vel_old_[i])/dt < -base_module_.max_acc_[i])
		{
			v[i] = base_module_.vel_old_[i] - base_module_.max_acc_[i]*dt;
		}
		else
		{
			v[i] = base_module_.req_vel_[i];
		}
		base_module_.vel_old_[i] = v[i];
	}

	geometry_msgs::Twist cmd;
	cmd.linear.x = v[0]; //vx;
	cmd.linear.y = v[1]; //vy;
	cmd.angular.z = v[2]; //vth;

	base_module_.base_publisher_.publish(cmd);
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

