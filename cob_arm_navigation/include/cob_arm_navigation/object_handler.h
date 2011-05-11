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



#ifndef OBJECT_HANDLER_INCLUDED
#define OBJECT_HANDLER_INCLUDED


#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <cob_arm_navigation/HandleObject.h>
#include <gazebo/GetModelState.h>
#include <mapping_msgs/CollisionObject.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <planning_environment_msgs/GetCollisionObjects.h>



const std::string frame_id = "/map";


class Object_Handler
{
private:
	ros::NodeHandle rh;

	ros::Publisher m_object_in_map_pub;
	ros::Publisher m_att_object_in_map_pub;

	ros::ServiceClient m_state_client;
	ros::ServiceClient m_collision_objects_client;
	
	ros::ServiceServer m_add_object_server;
	ros::ServiceServer m_remove_object_server;
	ros::ServiceServer m_attach_object_server;
	ros::ServiceServer m_detach_object_server;
	

public:	
	Object_Handler()
	{
		ROS_INFO("Object_Handler_constructor called");

		ROS_WARN("waiting for services...");
		ros::service::waitForService("/gazebo/get_model_state");
		ros::service::waitForService("/cob3_environment_server/get_collision_objects");
		ROS_INFO("...done!");
		
		
		m_object_in_map_pub  = rh.advertise<mapping_msgs::CollisionObject>("collision_object", 1);
		m_att_object_in_map_pub  = rh.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 1);

		
		m_state_client = rh.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");
		m_collision_objects_client = rh.serviceClient<planning_environment_msgs::GetCollisionObjects>("/cob3_environment_server/get_collision_objects");

		m_add_object_server = rh.advertiseService("/object_handler/add_object", &Object_Handler::add_object, this);
		m_remove_object_server = rh.advertiseService("/object_handler/remove_object", &Object_Handler::remove_object, this);
		m_attach_object_server = rh.advertiseService("/object_handler/attach_object", &Object_Handler::attach_object, this);
		m_detach_object_server = rh.advertiseService("/object_handler/detach_object", &Object_Handler::detach_object, this);
		ROS_INFO("object_handler ready...");
	}
	
	void run()
	{
		ROS_INFO("spinning...");
		ros::spin();
	}

private:	
	//implement callbacks here
	
	bool add_object(cob_arm_navigation::HandleObject::Request  &req,
					cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("add_object-service called!");
		ROS_INFO("Adding object %s ...",req.object.data.c_str());
		
		std::string parameter_name = req.object.data;	//this should always be identical to the model_name
		std::string model_name = req.object.data;
		ROS_INFO("Model-Name: %s", model_name.c_str());


		while(!rh.hasParam(parameter_name))
		{
		  ROS_WARN("waiting for parameter \"%s\"... ", parameter_name.c_str());
		  ros::Duration(0.5).sleep();
		}

		std::string model_parameter;
		if (rh.getParam(parameter_name, model_parameter))
		{
			ROS_INFO("Getting parameter successful!");
			ROS_DEBUG("Parameter: %s", model_parameter.c_str());
		}

		mapping_msgs::CollisionObject collision_object;

		//find out the geom::type of the model
		std::string pattern = "geom:box";
		std::size_t found_box = model_parameter.find(pattern);
		if (found_box!=std::string::npos)
		{
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_box));
			bool success;
			std::vector< double > dimensions;
			
			success = parse_box(model_parameter, dimensions);
			if(!success)
			{
				ROS_ERROR("Error while parsing a geom:box model. Aborting!");
		  
				res.success.data = false;
				res.error_message.data = "Error while parsing a geom:box model.";

				return false;
			}
			
			success = compose_box(model_name, dimensions, collision_object);
			
			if(!success)
			{
				ROS_ERROR("Error while composing the collision_object. Aborting!");
		  
				res.success.data = false;
				res.error_message.data = "Error while composing the collision_object.";

				return false;
			}
			collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
		}
		else
		{
		  ROS_ERROR("%s not found", pattern.c_str());
		  ROS_ERROR("I can't parse this model. Aborting!");
		  
		  res.success.data = false;
		  res.error_message.data = "No parser for the geom:type of this model available yet.";
		  
		  return false;
		}


		m_object_in_map_pub.publish(collision_object);

		ROS_INFO("Object added to environment server!");

		res.success.data = true;
		res.error_message.data = "Object added to environment server!";
		return true;
	}
	
	
	bool remove_object(cob_arm_navigation::HandleObject::Request  &req,
					   cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("remove_object-service called!");
		ROS_INFO("Removing object %s ...",req.object.data.c_str());
		
		std::string object_name = req.object.data;
		
		planning_environment_msgs::GetCollisionObjects srv;
		
		srv.request.include_points = false;
		
		if (m_collision_objects_client.call(srv))
		{
			ROS_INFO("get_collision_objects service_call successfull!");
			for(unsigned int i = 0; i < srv.response.collision_objects.size(); i++)
			{
				if(srv.response.collision_objects[i].id == object_name)
				{
					ROS_INFO("%s found!", object_name.c_str());
					mapping_msgs::CollisionObject collision_object = srv.response.collision_objects[i];
					
					collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
					
					m_object_in_map_pub.publish(collision_object);

					ROS_INFO("Object removed from environment server!");

					res.success.data = true;
					res.error_message.data = "Object removed from environment server!";
					return true;
				}
			}
			ROS_ERROR("Could not find object %s among known objects. Aborting!", object_name.c_str());
		  
			res.success.data = false;
			res.error_message.data = "Could not find object among known objects.";

			return false;
		}
		else
		{
			ROS_ERROR("Failed to call service get_collision_objects. Aborting!");
		  
			res.success.data = false;
			res.error_message.data = "Failed to call service get_collision_objects.";

			return false;
		}
		
		
		ROS_ERROR("You shouldn't be here!");
		
		return false;
	}
	
	
	bool attach_object(cob_arm_navigation::HandleObject::Request  &req,
					   cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("attach_object-service called!");
		ROS_INFO("Attaching object %s ...",req.object.data.c_str());
		
		std::string object_name = req.object.data;
		
		planning_environment_msgs::GetCollisionObjects srv;
		
		srv.request.include_points = false;
		
		if (m_collision_objects_client.call(srv))
		{
			ROS_INFO("get_collision_objects service_call successfull!");
			for(unsigned int i = 0; i < srv.response.collision_objects.size(); i++)
			{
				if(srv.response.collision_objects[i].id == object_name)
				{
					ROS_INFO("%s found!", object_name.c_str());
					
					mapping_msgs::AttachedCollisionObject att_object;
					att_object.object = srv.response.collision_objects[i];
					//attach it to the SDH
					att_object.link_name = "sdh_palm_link";
					att_object.touch_links.push_back("sdh_grasp_link");
					att_object.touch_links.push_back("sdh_finger_11_link");
					att_object.touch_links.push_back("sdh_finger_12_link");
					att_object.touch_links.push_back("sdh_finger_13_link");
					att_object.touch_links.push_back("sdh_finger_21_link");
					att_object.touch_links.push_back("sdh_finger_22_link");
					att_object.touch_links.push_back("sdh_finger_23_link");
					att_object.touch_links.push_back("sdh_thumb_1_link");
					att_object.touch_links.push_back("sdh_thumb_2_link");
					att_object.touch_links.push_back("sdh_thumb_3_link");
					
					att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;
					
					m_att_object_in_map_pub.publish(att_object);

					ROS_INFO("Object attached to robot!");

					res.success.data = true;
					res.error_message.data = "Object attached to robot!";
					return true;
				}
			}
			ROS_ERROR("Could not find object %s among known objects. Aborting!", object_name.c_str());
		  
			res.success.data = false;
			res.error_message.data = "Could not find object among known objects.";

			return false;
		}
		else
		{
			ROS_ERROR("Failed to call service get_collision_objects. Aborting!");
		  
			res.success.data = false;
			res.error_message.data = "Failed to call service get_collision_objects.";

			return false;
		}
		
		
		ROS_ERROR("You shouldn't be here!");
		
		return false;
	}
	
	
	bool detach_object(cob_arm_navigation::HandleObject::Request  &req,
					   cob_arm_navigation::HandleObject::Response &res )
	{
		ROS_INFO("detach_object-service called!");
		ROS_INFO("Detaching object %s ...",req.object.data.c_str());
		
		std::string object_name = req.object.data;
		
		planning_environment_msgs::GetCollisionObjects srv;
		
		srv.request.include_points = false;
		
		if (m_collision_objects_client.call(srv))
		{
			ROS_INFO("get_collision_objects service_call successfull!");
			for(unsigned int i = 0; i < srv.response.attached_collision_objects.size(); i++)
			{
				if(srv.response.attached_collision_objects[i].object.id == object_name)
				{
					ROS_INFO("%s found!", object_name.c_str());
					
					mapping_msgs::AttachedCollisionObject att_object = srv.response.attached_collision_objects[i];
					
					att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
					
					m_att_object_in_map_pub.publish(att_object);

					ROS_INFO("Object detached from robot!");

					res.success.data = true;
					res.error_message.data = "Object detached from robot!";
					return true;
				}
			}
			ROS_ERROR("Could not find object %s among attached objects. Aborting!", object_name.c_str());
		  
			res.success.data = false;
			res.error_message.data = "Could not find object among attached objects.";

			return false;
		}
		else
		{
			ROS_ERROR("Failed to call service get_collision_objects. Aborting!");
		  
			res.success.data = false;
			res.error_message.data = "Failed to call service get_collision_objects.";

			return false;
		}
		
		
		ROS_ERROR("You shouldn't be here!");
		
		return false;
	}
	
	
	
	//helper functions
	bool parse_box(std::string model_parameter, std::vector< double > &dimensions)
	{
		//parsing-scheme only valid for "box" (size: x,y,z)
		std::string pattern;
		std::size_t found_size, found_p, found_x, found_y, found_z;

		pattern = "size";
		found_size=model_parameter.find(pattern);
		if (found_size!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_size));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = ">";
		found_p=model_parameter.find(pattern, found_size);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";
		found_x=model_parameter.find_first_not_of(pattern, found_p+1);
		if (found_x!=std::string::npos)
			ROS_DEBUG("first not \"%s\" found at: %d", pattern.c_str(), int(found_x));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = " ";	
		found_p=model_parameter.find_first_of(pattern, found_x);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

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
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		found_p=model_parameter.find_first_of(pattern, found_y);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

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
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		pattern = "<";	
		found_p=model_parameter.find_first_of(pattern, found_z);
		if (found_p!=std::string::npos)
			ROS_DEBUG("first \"%s\" found at: %d", pattern.c_str(), int(found_p));
		else
		{
			ROS_ERROR("%s not found", pattern.c_str());
			return false;
		}

		size_t length_z = found_p - found_z;
		std::string z = model_parameter.substr(found_z, length_z);
		ROS_DEBUG("z: %s, real_length_z: %d", z.c_str(), int(length_z));

		double z_d = strtod(z.c_str(), NULL);
		ROS_INFO("z as double: %f", z_d);

		dimensions.push_back(x_d);
		dimensions.push_back(y_d);
		dimensions.push_back(z_d);
			
		return true;
	}
	
	
	bool compose_box(std::string model_name, std::vector< double > dimensions, mapping_msgs::CollisionObject &collision_object)
	{
		
		gazebo::GetModelState state_srv;

		state_srv.request.model_name = model_name;
		if (m_state_client.call(state_srv))
		{
			ROS_INFO("ModelPose (x,y,z): (%f,%f,%f)", state_srv.response.pose.position.x, state_srv.response.pose.position.y, state_srv.response.pose.position.z);
		}
		else
		{
			ROS_ERROR("Failed to call service get_model_state");
			return false;
		}

		collision_object.id = model_name;
		//collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
		//collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
		collision_object.header.frame_id = frame_id;
		collision_object.header.stamp = ros::Time::now();
		collision_object.shapes.resize(1);
		collision_object.poses.resize(1);

		//ToDo: figure out how *.model-size and *.urdf-extend are related
		//ToDo: figure out where the *.model origin is located (top,center,bottom?)
		collision_object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
		collision_object.shapes[0].dimensions.push_back(dimensions[0]/2.0);
		collision_object.shapes[0].dimensions.push_back(dimensions[1]/2.0);
		collision_object.shapes[0].dimensions.push_back(dimensions[2]/2.0);

		collision_object.poses[0].position.x = state_srv.response.pose.position.x;
		collision_object.poses[0].position.y = state_srv.response.pose.position.y;
		collision_object.poses[0].position.z = state_srv.response.pose.position.z;
		collision_object.poses[0].orientation.x = state_srv.response.pose.orientation.x;
		collision_object.poses[0].orientation.y = state_srv.response.pose.orientation.y;
		collision_object.poses[0].orientation.z = state_srv.response.pose.orientation.z;
		collision_object.poses[0].orientation.w = state_srv.response.pose.orientation.w;
		
		return true;
	}


	
	
	
};

#endif
