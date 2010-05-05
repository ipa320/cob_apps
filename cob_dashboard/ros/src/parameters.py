#!/usr/bin/python

import rospy
from trajectory_msgs.msg import *

class armParameter:
	action_goal_topic = 'arm_controller/joint_trajectory_action'
	joint_names = ["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"]

	home = JointTrajectory()
	home.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0,0,0,0,0,0,0]
	point.velocities=[0,0,0,0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	home.points.append(point)

	folded = JointTrajectory()
	folded.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0,0.7,1.5,1.5,0.7,1.5,0]
	point.velocities=[0,0,0,0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	folded.points.append(point)

class armParameter_pr2:
	action_goal_topic = 'r_arm_controller/joint_trajectory_action'
	joint_names = ["r_shoulder_pan_joint","r_shoulder_lift_joint","r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint","r_wrist_flex_joint","r_wrist_roll_joint"]

	home = JointTrajectory()
	home.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0,0,0,0,0,0,0]
	point.velocities=[0,0,0,0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	home.points.append(point)

	folded = JointTrajectory()
	folded.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0,0.7,1.5,1.5,0.7,1.5,0]
	point.velocities=[0,0,0,0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	folded.points.append(point)
