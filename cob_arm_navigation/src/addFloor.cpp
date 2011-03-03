#include <ros/ros.h>

#include <mapping_msgs/CollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "addFloor");

  ros::NodeHandle nh;

  ros::service::waitForService("/cob3_environment_server/get_state_validity");	//just to make sure that the environment_server is there!

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 10);

  //add the cylinder into the collision space
  mapping_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "floor";
  cylinder_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  //cylinder_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  cylinder_object.header.frame_id = "/map";
  cylinder_object.header.stamp = ros::Time::now();
  geometric_shapes_msgs::Shape object;
  object.type = geometric_shapes_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = 10.0;
  object.dimensions[1] = 10.0;
  object.dimensions[2] = 0.01;
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = -0.005;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "floor";
  object_in_map_pub_.publish(cylinder_object);

  ROS_INFO("Should have published");

  ros::shutdown();
}

