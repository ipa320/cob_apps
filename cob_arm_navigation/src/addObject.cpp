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
 * \date Date of creation: January 2011
 *
 * \brief
 *   Adds an object model as a known obstacle to the environment server.
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




#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <mapping_msgs/CollisionObject.h>
//#include <tf/tf.h>
#include <gazebo/GetModelState.h>



int main(int argc, char** argv) {

  ros::init(argc, argv, "addObject");
  //const std::string frame_id = "/base_footprint";
  const std::string frame_id = "/map";

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 20);
  

  if (argc != 3){
    ROS_ERROR("Need a parameter name (1) and a model_name (2)");
    return -1;
  }
  std::string parameter_name = argv[1];
  std::string model_name = argv[2];
  ROS_INFO("Model-Name: %s", model_name.c_str());

  
  while(!nh.hasParam(parameter_name))
  {
	  ROS_WARN("waiting for parameter \"%s\"... ", parameter_name.c_str());
	  ros::Duration(0.5).sleep();
  }
  
  std::string model_parameter;
  if (nh.getParam(parameter_name, model_parameter))
  {
	ROS_INFO("Getting parameter successful!");
	ROS_DEBUG("Parameter: %s", model_parameter.c_str());
  }


  //parsing-scheme only valid for "box" (size: x,y,z)
  std::string pattern;
  std::size_t found_box, found_size, found_p, found_x, found_y, found_z;
  
  pattern = "geom:box";
  found_box=model_parameter.find(pattern);
  if (found_box!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_box));
  else
  {
	  ROS_ERROR("%s not found", pattern.c_str());
	  ROS_ERROR("I can't parse this model. Aborting!");
	  return -1;
  }
  
  
  
  pattern = "size";
  found_size=model_parameter.find(pattern);
  if (found_size!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_size));
  else
	ROS_ERROR("%s not found", pattern.c_str());

  pattern = ">";
  found_p=model_parameter.find(pattern, found_size);
  if (found_p!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());

  pattern = " ";
  found_x=model_parameter.find_first_not_of(pattern, found_p+1);
  if (found_x!=std::string::npos)
    ROS_DEBUG("first not \"%s\" found at: %d", pattern.c_str(), int(found_x));
  else
	ROS_ERROR("%s not found", pattern.c_str());

  pattern = " ";	
  found_p=model_parameter.find_first_of(pattern, found_x);
  if (found_p!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());

  size_t length_x = found_p - found_x;
  std::string x = model_parameter.substr(found_x, length_x);
  ROS_DEBUG("x: %s, real_length_x: %d", x.c_str(), int(length_x));

  double x_d = strtod(x.c_str(), NULL);
  ROS_INFO("x as double: %f", x_d);
 
 
 
  pattern = " ";
  found_y=model_parameter.find_first_not_of(pattern, found_p);
  if (found_y!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_y));
  else
	ROS_ERROR("%s not found", pattern.c_str());
	
  found_p=model_parameter.find_first_of(pattern, found_y);
  if (found_p!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());
  
  size_t length_y = found_p - found_y;
  std::string y = model_parameter.substr(found_y, length_y);
  ROS_DEBUG("y: %s, real_length_y: %d", y.c_str(), int(length_y));

  double y_d = strtod(y.c_str(), NULL);
  ROS_INFO("y as double: %f", y_d);
  
  
  
  
  pattern = " ";
  found_z=model_parameter.find_first_not_of(pattern, found_p);
  if (found_z!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_z));
  else
	ROS_ERROR("%s not found", pattern.c_str());
	
  pattern = "<";	
  found_p=model_parameter.find_first_of(pattern, found_z);
  if (found_p!=std::string::npos)
    ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());
  
  size_t length_z = found_p - found_z;
  std::string z = model_parameter.substr(found_z, length_z);
  ROS_DEBUG("z: %s, real_length_z: %d", z.c_str(), int(length_z));

  double z_d = strtod(z.c_str(), NULL);
  ROS_INFO("z as double: %f", z_d);
  
  
  
  
  
  
  
  
  //rosservice call /gazebo/get_model_state milk_box cob3::base_link
  ROS_INFO("waiting for service /gazebo/get_model_state....");
  ros::service::waitForService("/gazebo/get_model_state");
  
  ros::ServiceClient state_client = nh.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");
  
  gazebo::GetModelState state_srv;

  state_srv.request.model_name = model_name;
  if (state_client.call(state_srv))
  {
	ROS_INFO("ModelPose (x,y,z): (%f,%f,%f)", state_srv.response.pose.position.x, state_srv.response.pose.position.y, state_srv.response.pose.position.z);
  }
  else
  {
	ROS_ERROR("Failed to call service get_model_state");
	ros::shutdown();
  }
  
  
  
  
  //add to environment_server as a known obstacle
  mapping_msgs::CollisionObject collision_object;
  collision_object.id = model_name + "_object";
  collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  //collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  collision_object.header.frame_id = frame_id;
  collision_object.header.stamp = ros::Time::now();
  collision_object.shapes.resize(1);
  collision_object.poses.resize(1);
  
  //ToDo: figure out how *.model-size and *.urdf-extend are related
  //ToDo: figure out where the *.model origin is located (top,center,bottom?)
  collision_object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  collision_object.shapes[0].dimensions.push_back(x_d/2.0);
  collision_object.shapes[0].dimensions.push_back(y_d/2.0);
  collision_object.shapes[0].dimensions.push_back(z_d/2.0);
  
  collision_object.poses[0].position.x = state_srv.response.pose.position.x;
  collision_object.poses[0].position.y = state_srv.response.pose.position.y;
  collision_object.poses[0].position.z = state_srv.response.pose.position.z;
  collision_object.poses[0].orientation.x = state_srv.response.pose.orientation.x;
  collision_object.poses[0].orientation.y = state_srv.response.pose.orientation.y;
  collision_object.poses[0].orientation.z = state_srv.response.pose.orientation.z;
  collision_object.poses[0].orientation.w = state_srv.response.pose.orientation.w;
  
  
  
  
  object_in_map_pub_.publish(collision_object);
	
  ROS_INFO("Should have published");
 
  ros::shutdown();
}



