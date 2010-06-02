/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob3_driver
 * ROS package name: cob3_teleop
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * Date of creation: Jan 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        ros::Publisher topicPub_CmdVel;

	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_Joy;
        
        // service servers
        //--
        
        // service clients
        //--
        
        // global variables
		int linear_x, linear_y, angular_;
		double l_scale_, a_scale_;

        // Constructor
        NodeClass()
		{
            // initialize global variables
			n.param("axis_linear_x", linear_x, 1);
			n.param("axis_linear_y", linear_y, 0);
			n.param("axis_angular", angular_, 2);
			n.param("scale_angular", a_scale_, a_scale_);
			n.param("scale_linear", l_scale_, l_scale_);			
			
        	// implementation of topics to publish
            topicPub_CmdVel = n.advertise<geometry_msgs::Twist>("/base_controller/command", 50);

            // implementation of topics to subscribe
            topicSub_Joy = n.subscribe("joy", 10, &NodeClass::topicCallback_Joy, this);
            
            // implementation of service servers
			//--
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_Joy(const joy::Joy::ConstPtr& msg)
        {
            //ROS_INFO("received new velocity command [cmdVelX=%3.5f,cmdVelY=%3.5f,cmdVelTh=%3.5f]", 
            //         msg->linear.x, msg->linear.y, msg->angular.z);
                     
			geometry_msgs::Twist cmdVel;
			cmdVel.linear.x = l_scale_*msg->axes[linear_x];
			cmdVel.linear.y = l_scale_*msg->axes[linear_y];
			cmdVel.angular.z = a_scale_*msg->axes[angular_];
			ROS_INFO("new vel = [%3.5f,%3.5f,%3.5f]",cmdVel.linear.x, cmdVel.linear.y, cmdVel.angular.z);
			topicPub_CmdVel.publish(cmdVel);
        }

        // service callback functions
        // function will be called when a service is querried
		//--
        
        // other function declarations
		//--
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob3_teleop_platform");
    
    NodeClass nodeClass;

    // main loop
/*
 	ros::Rate loop_rate(10); // Hz 
    while(nodeClass.n.ok())
    {
        //--

        ros::spinOnce();
        loop_rate.sleep();
    }
*/  
    ros::spin();

    return 0;
}
