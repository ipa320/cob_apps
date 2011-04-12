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
#include <vector>

#include <mapping_msgs/CollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>
#include <tinyxml/tinyxml.h>
#include <urdf/model.h>
#include <planning_models/kinematic_model.h>
#include <tf/tf.h>
#include <planning_environment/util/construct_object.h>
#include <gazebo/GetModelState.h>
//#include <urdf/link.h>


double strict_str2double(const char* str)
{
    char* endptr;
    double value = strtod(str, &endptr);
    if (*endptr) return 0;
    return value;
}


/*
shapes::Shape* constructShape(const urdf::Geometry *geom)
{
  ROS_ASSERT(geom);
 
  shapes::Shape *result = NULL;
  if(geom->type == urdf::Geometry::BOX)
  {
	ROS_INFO("BOX");
    urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
    result = new shapes::Box(dim.x, dim.y, dim.z);
  }
  else if(geom->type == urdf::Geometry::SPHERE)
  {
	ROS_INFO("SPHERE");
    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
  }
  else if(geom->type == urdf::Geometry::CYLINDER)
  {
	ROS_INFO("CYLINDER");
    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius, dynamic_cast<const urdf::Cylinder*>(geom)->length);
  }
  else if(geom->type == urdf::Geometry::MESH)
  {
	//you can find the code in motion_planning_common/planning_models/kinematic_models.cpp
	ROS_INFO("MESH --- currently not supported");
  }
  else
  {
    ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
  }
    
  return result;
}
*/

int main(int argc, char** argv) {

  ros::init(argc, argv, "addObject");
  //const std::string frame_id = "/base_footprint";
  //const std::string frame_id = "/map";

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


  //std::string parameter_name = "milk_box";
  //std::string model_name = "milk_box";
  
  
  
  while(!nh.hasParam(parameter_name))
  {
	  ROS_WARN("waiting for parameter \"%s\"... ", parameter_name.c_str());
	  ros::Duration(0.5).sleep();
  }
  
  std::string model_parameter;
  if (nh.getParam(parameter_name, model_parameter))
  {
	ROS_INFO("Getting parameter successful!");
	ROS_INFO("Parameter: %s", model_parameter.c_str());
  }

  std::string pattern = "size";
  std::size_t found_size, found_p, found_x, found_y, found_z;
  
  found_size=model_parameter.find(pattern);
  if (found_size!=std::string::npos)
    ROS_INFO("first \"%s\" found at: %d", pattern.c_str(), int(found_size));
  else
	ROS_ERROR("%s not found", pattern.c_str());

  pattern = ">";
  found_p=model_parameter.find(pattern, found_size);
  if (found_p!=std::string::npos)
    ROS_INFO("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());

  pattern = " ";
  found_x=model_parameter.find_first_not_of(pattern, found_p);
  if (found_x!=std::string::npos)
    ROS_INFO("first not \"%s\" found at: %d", pattern.c_str(), int(found_x));
  else
	ROS_ERROR("%s not found", pattern.c_str());
	
  found_p=model_parameter.find_first_of(pattern, found_x);
  if (found_p!=std::string::npos)
    ROS_INFO("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());
	
  std::string x;
  size_t length_x;
  length_x = found_p - found_x;
  x = model_parameter.substr(found_x+1, length_x);
  ROS_INFO("x: %s, real_length_x: %d", x.c_str(), int(length_x));

  double x_d = strtod(x.c_str(), NULL);
  
  ROS_INFO("x as double: %f", x_d);
 
 
 
  pattern = " ";
  found_y=model_parameter.find_first_not_of(pattern, found_p);
  if (found_y!=std::string::npos)
    ROS_INFO("first \"%s\" found at: %d", pattern.c_str(), int(found_y));
  else
	ROS_ERROR("%s not found", pattern.c_str());
	
  found_p=model_parameter.find_first_of(pattern, found_y);
  if (found_p!=std::string::npos)
    ROS_INFO("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());
  
  std::string y;
  size_t length_y;
  length_y = found_p - found_y;
  y = model_parameter.substr(found_y, length_y);
  ROS_INFO("y: %s, real_length_y: %d", y.c_str(), int(length_y));

  double y_d = strtod(y.c_str(), NULL);
  
  ROS_INFO("y as double: %f", y_d);
  
  
  
  
  pattern = " ";
  found_z=model_parameter.find_first_not_of(pattern, found_p);
  if (found_z!=std::string::npos)
    ROS_INFO("first \"%s\" found at: %d", pattern.c_str(), int(found_z));
  else
	ROS_ERROR("%s not found", pattern.c_str());
	
  pattern = "<";	
  found_p=model_parameter.find_first_of(pattern, found_z);
  if (found_p!=std::string::npos)
    ROS_INFO("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
  else
	ROS_ERROR("%s not found", pattern.c_str());
  
  std::string z;
  size_t length_z;
  length_z = found_p - found_z;
  z = model_parameter.substr(found_z, length_z);
  ROS_INFO("z: %s, real_length_z: %d", z.c_str(), int(length_z));

  double z_d = strtod(z.c_str(), NULL);
  
  ROS_INFO("z as double: %f", z_d);
  
  
  
  
  
  
  
  
  //rosservice call /gazebo/get_model_state milk_box cob3::base_link
  
  
  
  
  
  
  
  
  
  /*
  std::vector< boost::shared_ptr< urdf::Link > > URDF_links;
  model.getLinks(URDF_links);
  std::map< std::string, boost::shared_ptr< urdf::Joint > > URDF_joints = model.joints_;
  std::map< std::string, boost::shared_ptr< urdf::Joint > >::iterator joints_it;


  //tranfo between links is in joints->origin!
  //access to Joint-Information
  for(joints_it=URDF_joints.begin() ; joints_it != URDF_joints.end(); joints_it++)
  {
    ROS_INFO("Joint name: %s", (*joints_it).first.c_str());
    ROS_INFO("\t origin: %f,%f,%f", (*joints_it).second->parent_to_joint_origin_transform.position.x, (*joints_it).second->parent_to_joint_origin_transform.position.y, (*joints_it).second->parent_to_joint_origin_transform.position.z);
  }

  ros::service::waitForService("/gazebo/get_model_state");
  ros::service::waitForService("/cob3_environment_server/get_state_validity");	//just to make sure that the environment_server is there!

  //access to tranformation /world to /root_link (table_top)
  ros::ServiceClient client = nh.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");
  gazebo::GetModelState srv;

  srv.request.model_name = model_name;
  if (client.call(srv))
  {
	ROS_INFO("URDFPose (x,y,z): (%f,%f,%f)", srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z);
  }
  else
  {
	ROS_ERROR("Failed to call service get_model_state");
	ros::shutdown();
  }
  
  mapping_msgs::CollisionObject collision_object;
  collision_object.id = model_name + "_object";
  collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  //collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  collision_object.header.frame_id = frame_id;
  collision_object.header.stamp = ros::Time::now();
  collision_object.shapes.resize(URDF_links.size());
  collision_object.poses.resize(URDF_links.size());
  

  joints_it=URDF_joints.begin();
  for(unsigned int i=0; i<URDF_links.size(); i++)
  {
	  urdf::Link current_link = *URDF_links[i];
	  ROS_INFO("Current Link: %s", current_link.name.c_str());

	  if(current_link.name == "dummy_link")
	  {
		  ROS_INFO("Dealing with dummy_link...");
		  continue;
	  }

	  boost::shared_ptr< urdf::Joint > current_parent_joint = current_link.parent_joint;
	  //ROS_INFO("Current Parent Joint: %s", current_parent_joint->name.c_str());

	  
	  //fill CollisionObject for each link
	  shapes::Shape *current_shape;
	  current_shape = constructShape(current_link.collision->geometry.get());
	  ROS_INFO("shape.type: %d", current_shape->type);
	  
	  //ROS_INFO("Position (x,y,z): (%f,%f,%f)", current_link.collision->origin.position.x, current_link.collision->origin.position.y, current_link.collision->origin.position.z);

	  tf::Pose pose;
	  tf::Transform world2dummy;
	  tf::Transform dummy2link;
	  world2dummy = tf::Transform(tf::Quaternion(srv.response.pose.orientation.x, srv.response.pose.orientation.y, srv.response.pose.orientation.z, srv.response.pose.orientation.w), tf::Vector3(srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z));
	  dummy2link = tf::Transform(tf::Quaternion(current_parent_joint->parent_to_joint_origin_transform.rotation.x, 
										  		current_parent_joint->parent_to_joint_origin_transform.rotation.y, 
										  		current_parent_joint->parent_to_joint_origin_transform.rotation.z, 
										  		current_parent_joint->parent_to_joint_origin_transform.rotation.w), 
						   		 tf::Vector3(current_parent_joint->parent_to_joint_origin_transform.position.x, 
									   		 current_parent_joint->parent_to_joint_origin_transform.position.y, 
									   		 current_parent_joint->parent_to_joint_origin_transform.position.z));
	  tf::Pose pose2;
	  pose2.mult(world2dummy, dummy2link);

	  tf::Stamped<tf::Pose> stamped_pose_in;
	  stamped_pose_in.stamp_ = ros::Time::now();
	  stamped_pose_in.frame_id_ = frame_id;
	  stamped_pose_in.setData(pose2);
	  


	  //// TODO: fill in collision_object
	  geometric_shapes_msgs::Shape msg_shape;
	  planning_environment::constructObjectMsg(current_shape, msg_shape);
	  
	  geometry_msgs::PoseStamped msg_pose_stamped;
	  //tf::poseStampedTFToMsg (transformed_pose, msg_pose_stamped);
	  tf::poseStampedTFToMsg (stamped_pose_in, msg_pose_stamped);
	  
	  collision_object.shapes[i] = msg_shape;
	  collision_object.poses[i] = msg_pose_stamped.pose;
	
	  object_in_map_pub_.publish(collision_object);
	
	  ROS_INFO("Should have published");
  }
  */
  ros::shutdown();
}



