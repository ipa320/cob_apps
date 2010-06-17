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
 * ROS stack name: cob_driver
 * ROS package name: cob_simpledrive
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Mar 2010
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
// Headers provided by cob-packages which should be avoided/removed
#include <cob_utilities/IniFile.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/GetJointState.h>

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
        ros::Publisher topicPub_JointStateCmd;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_demoSubscribe;
        
        // service servers
        //--
            
        // service clients
        ros::ServiceClient srvClient_GetJointState;
        ros::ServiceClient srvClient_InitPltf;
        ros::ServiceClient srcClient_ShutdownPltf;
        
	    double m_dSpeedRadS;
        int iNumMotors;

        // Constructor
        NodeClass()
        {
            topicPub_JointStateCmd = n.advertise<sensor_msgs::JointState>("JointStateCmd", 1);
			
			srvClient_GetJointState = n.serviceClient<cob_srvs::GetJointState>("GetJointState");
			srvClient_InitPltf = n.serviceClient<cob_srvs::Trigger>("Init");
			srcClient_ShutdownPltf = n.serviceClient<cob_srvs::Trigger>("Shutdown");

            //topicSub_demoSubscribe = n.subscribe("demoSubscribe", 1, &NodeClass::topicCallback_demoSubscribe, this);

			m_dSpeedRadS = 6;
			iNumMotors = 8;
			     
		}
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 

        // service callback functions

        
        // other function declarations
        int init();

		int simpleDriveTest(int argc, char** argv);
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "drive_identification_node");

    NodeClass node;
	if(node.init() == 1) return 1;
	node.simpleDriveTest(argc, argv);
	
    
 /*
    while(node.n.ok())
    {

        ros::spinOnce();
    }
*/    
//    ros::spin();

    return 0;
}

//##################################
//#### function implementations ####
int NodeClass::init(){

	cob_srvs::Trigger data;
    srvClient_InitPltf.call(data);

    if(data.response.success != true) {
        ROS_ERROR("Failed to initialize Platform using base_drive_chain_node");
        return 1;
    } else ROS_INFO("Successfully initialized base_drive_chain_node");


	ROS_INFO("Simple_Drive_node init successful");
	
    return 0;
}

int NodeClass::simpleDriveTest(int argc, char** argv) {
	double startTime;
	sensor_msgs::JointState msgDriveCmd;
	cob_srvs::GetJointState srvGetJointState;
	msgDriveCmd.set_velocity_size(iNumMotors);

	switch (argc){
		
		case 10:
			startTime = ros::Time::now().toSec();
			while(ros::Time::now().toSec() - startTime < atof(argv[1])) {
        		for(int i = 0; i<iNumMotors; i++) {
		    		msgDriveCmd.velocity[i] = atof(argv[i+2]);
				}
				topicPub_JointStateCmd.publish(msgDriveCmd);
				srvClient_GetJointState.call(srvGetJointState);
				
				ROS_INFO_STREAM("Actual Vel for Motor " << 0 << " is " << srvGetJointState.response.jointstate.velocity[0]);
			}

			//stop drives again
			for(int i = 0; i<iNumMotors; i++) {
				msgDriveCmd.velocity[i] = 0;
    		}
			topicPub_JointStateCmd.publish(msgDriveCmd);
			break;

		default:
			ROS_ERROR("No input parameters set. Usage as follows:\nDURATION(s) SPEEDMOT1(radS) SPEEDMOT2 ... SPEEDMOT8");
			ROS_ERROR("Your inputs are:");			
			for(int i=0;i<argc;i++) {
				ROS_ERROR_STREAM(i << "  " << argv[i]);
			}
			break;
	}

	
    
	ROS_INFO("Successfully finished simpleDriveTest");	

	return 0;
}

