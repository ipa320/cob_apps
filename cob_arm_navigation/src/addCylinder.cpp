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

