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
 *   Adds cylinder as a known obstacle to the environment server.
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

#include <mapping_msgs/CollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "addCylinder");

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 10);

  ros::Duration(2.0).sleep();

  //add the cylinder into the collision space
  mapping_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "pole";
  cylinder_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  //cylinder_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  cylinder_object.header.frame_id = "/base_footprint";
  cylinder_object.header.stamp = ros::Time::now();
  geometric_shapes_msgs::Shape object;
  object.type = geometric_shapes_msgs::Shape::CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[0] = 0.1;
  object.dimensions[1] = 1.2;
  geometry_msgs::Pose pose;
  pose.position.x = -.1;
  pose.position.y = -.8;
  pose.position.z = .75;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "pole";
  object_in_map_pub_.publish(cylinder_object);

  ROS_INFO("Should have published");

  ros::Duration(2.0).sleep();

  ros::shutdown();
}

