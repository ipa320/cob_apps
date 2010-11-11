#include <ros/ros.h>
#include <stdio.h>
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

shapes::Shape* constructShape(const urdf::Geometry *geom)
{
  ROS_ASSERT(geom);
 
  shapes::Shape *result = NULL;
  switch (geom->type)
  {
  case urdf::Geometry::SPHERE:
    {
      result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
      break;
    }
  case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
  case urdf::Geometry::CYLINDER:
    {
      result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
                                  dynamic_cast<const urdf::Cylinder*>(geom)->length);
      break;
    }
  case urdf::Geometry::MESH:
    {
		//you can find the code in motion_planning_common/planning_models/kinematic_models.cpp
	}
    break;
  default:
    ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
    break;
  }
    
  return result;
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "addURDF");
  //const std::string frame_id = "/base_footprint";
  const std::string frame_id = "/map";

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 10);
  

  if (argc != 3){
    ROS_ERROR("Need a urdf file as first argument and the model_name as in the launch file as second argument");
    return -1;
  }
  std::string urdf_file = argv[1];
  std::string model_name = argv[2];
  ROS_INFO("Model-Name: %s", model_name.c_str());

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  
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

  //access to tranformation /world to /root_link (table_top)
  ros::ServiceClient client = nh.serviceClient<gazebo::GetModelState>("/gazebo/get_model_state");
  gazebo::GetModelState srv;
  srv.request.model_name = model_name + "_model";
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
  

  //BEACTE: Reihenfolge Joints, Reihenfolge Links!!! Linkreihenfolge nicht wie in urdf-file! leave2root? (model.getParentJoint, model.getChildJoint nur deklariert, nicht implementiert)
  joints_it=URDF_joints.begin();
  for (unsigned int i=1; i<URDF_links.size(); i++)
  {
	  urdf::Link current_link = *URDF_links[i];
	  ROS_INFO("Current Link: %s", current_link.name.c_str());

	  boost::shared_ptr< urdf::Joint > current_parent_joint = current_link.parent_joint;
	  //ROS_INFO("Current Parent Joint: %s", current_parent_joint->name.c_str());

	  
	  //fill CollisionObject for each link
	  shapes::Shape *current_shape;
	  current_shape = constructShape(current_link.collision->geometry.get());
	  
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
	  //tf::Vector3 origin = pose2.getOrigin();
	  //ROS_INFO("Composed Pose: (%f, %f, %f)", origin.getX(), origin.getY(), origin.getZ());
	  //pose = tf::Transform(tf::Quaternion(current_link.collision->origin.rotation.x, current_link.collision->origin.rotation.y, current_link.collision->origin.rotation.z, current_link.collision->origin.rotation.w), tf::Vector3(current_link.collision->origin.position.x, current_link.collision->origin.position.y, current_link.collision->origin.position.z));
	  /*pose = tf::Transform(tf::Quaternion(current_parent_joint->parent_to_joint_origin_transform.rotation.x, 
										  current_parent_joint->parent_to_joint_origin_transform.rotation.y, 
										  current_parent_joint->parent_to_joint_origin_transform.rotation.z, 
										  current_parent_joint->parent_to_joint_origin_transform.rotation.w), 
						   tf::Vector3(current_parent_joint->parent_to_joint_origin_transform.position.x + srv.response.pose.position.x, 
									   current_parent_joint->parent_to_joint_origin_transform.position.y + srv.response.pose.position.y, 
									   current_parent_joint->parent_to_joint_origin_transform.position.z + srv.response.pose.position.z));*/
	  tf::Stamped<tf::Pose> stamped_pose_in;
	  stamped_pose_in.stamp_ = ros::Time::now();
	  stamped_pose_in.frame_id_ = frame_id;
	  stamped_pose_in.setData(pose2);
	  
	  //transformation necessary!!!
	  //use information from joints and get_model_state (see above!)


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

  ros::Duration(2.0).sleep();

  ros::shutdown();
}



