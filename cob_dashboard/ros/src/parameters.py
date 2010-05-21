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
	

class torsoParameter:
	action_goal_topic = 'torso_controller/joint_trajectory_action'
	joint_names = ["torso_lower_neck_pan_joint","torso_lower_neck_tilt_joint","torso_upper_neck_pan_joint","torso_upper_neck_tilt_joint"]

	home = JointTrajectory()
	home.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0,0,0,0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	home.points.append(point)

	front = JointTrajectory()
	front.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0.0,0.05,0.0,0.1]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	front.points.append(point)
	
	back = JointTrajectory()
	back.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0.0,-0.05,0.0,-0.1]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	back.points.append(point)
	
	right = JointTrajectory()
	right.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[-0.05,0.0,-0.1,0.0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	right.points.append(point)
	
	left = JointTrajectory()
	left.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0.05,0.0,0.1,0.0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	left.points.append(point)

	nod = JointTrajectory()
	nod.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0,0.05,0,0.1]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	nod.points.append(point)
	point=JointTrajectoryPoint()
	point.positions=[0,0,0,0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(6)
	nod.points.append(point)
	point=JointTrajectoryPoint()
	point.positions=[0,0.05,0,0.1]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(9)
	nod.points.append(point)
	point=JointTrajectoryPoint()
	point.positions=[0,0,0,0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(12)
	nod.points.append(point)
	
	shake = JointTrajectory()
	shake.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0.05,0,0.1,0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	shake.points.append(point)
	point=JointTrajectoryPoint()
	point.positions=[-0.05,0,-0.1,0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	shake.points.append(point)
	point=JointTrajectoryPoint()
	point.positions=[0.05,0,0.1,0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	shake.points.append(point)
	point=JointTrajectoryPoint()
	point.positions=[0,0,0,0]
	point.velocities=[0,0,0,0]
	point.time_from_start=rospy.Duration(3)
	shake.points.append(point)
	
class trayParameter:
	action_goal_topic = 'tray_controller/joint_trajectory_action'
	joint_names = ["torso_tray_joint"]

	up = JointTrajectory()
	up.joint_names = joint_names
	point=JointTrajectoryPoint()
	point.positions=[0]
	point.velocities=[0]
	point.time_from_start=rospy.Duration(3)
	up.points.append(point)
	
	down = JointTrajectory()
	down.joint_names = joint_names
	point=JointTrajectoryPoint()
	#point.positions=[-3.1415926]
	point.positions=[0.5]
	point.velocities=[0]
	point.time_from_start=rospy.Duration(3)
	down.points.append(point)
