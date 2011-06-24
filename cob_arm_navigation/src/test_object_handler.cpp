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
 *   ROS package name: cob_arm_navigation
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: April 2011
 *
 * \brief
 *   This package provides services for handling a graspable object. 
 *   It reads data from the parameter_server in order to add it to or remove it from the environment_server as a known obstacle.
 * 	 Also it can attach such object to the robot in order to consider it as a part of the robot during the planning process.
 *
 ****************************************************************/




#include <ros/ros.h>
#include <cob_arm_navigation/HandleObject.h>

#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_object_handler");
  if (argc != 2)
  {
    ROS_INFO("usage: test_object_handler <object_name>");
    return 1;
  }

  ros::NodeHandle n;
  
  ros::service::waitForService("object_handler/add_object");
  ros::ServiceClient client = n.serviceClient<cob_arm_navigation::HandleObject>("object_handler/add_object");
  cob_arm_navigation::HandleObject::Request req;
  cob_arm_navigation::HandleObject::Response res;
  req.object.data = argv[1];

  if (client.call(req, res))
  {
    ROS_INFO("Result: %s", res.error_message.data.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
