#!/usr/bin/python

import rospy
from trajectory_msgs.msg import *
from cob_msgs.msg import *

class torsoParameter:
	action_goal_topic = 'torso/JointTrajectory'
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
	action_goal_topic = 'tray/JointTrajectory'
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
	point.positions=[-3.1415926]
	point.velocities=[0]
	point.time_from_start=rospy.Duration(3)
	down.points.append(point)

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

class lbrParameter:
	action_goal_topic = 'lbr_controller/joint_trajectory_action'
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
	
class sdhParameter:
	action_goal_topic = 'sdh/JointCommand'
    #-->[not_used, joint_thumb1_thumb2, joint_thumb2_thumb3, joint_palm_finger11, joint_finger11_finger12, joint_finger12_finger13, not_used, joint_finger21_finger22, joint_finger22_finger23]
	joint_names = ["joint_palm_thumb1", "joint_thumb1_thumb2", "joint_thumb2_thumb3", "joint_palm_finger11", "joint_finger11_finger12", "joint_finger12_finger13", "joint_palm_finger21", "joint_finger21_finger22", "joint_finger22_finger23"]

	home = JointCommand()
	home.joint_names = joint_names
	home.positions=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	home.velocities=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

	cylClose = JointCommand()
	cylClose.joint_names = joint_names
	cylClose.positions=[0.0,0.0,1.0472,0.0,0.0,1.0472,0.0,0.0,1.0472]
	cylClose.velocities=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

	cylOpen = JointCommand()
	cylOpen.joint_names = joint_names
	cylOpen.positions=[0.0,-0.7854,1.0472,0.0,-0.7854,1.0472,0.0,-0.7854,1.0472]
	cylOpen.velocities=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

	spherClose = JointCommand()
	spherClose.joint_names = joint_names
	spherClose.positions=[0.0,-0.2618,1.0472,1.0472,-0.2618,1.0472,0.0,-0.2618,1.0472]
	spherClose.velocities=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

	spherOpen = JointCommand()
	spherOpen.joint_names = joint_names
	spherOpen.positions=[0.0,-0.7854,1.0472,1.0472,-0.7854,1.0472,0.0,-0.7854,1.0472]
	spherOpen.velocities=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	
	trainObjects = JointCommand()
	trainObjects.joint_names = joint_names
	trainObjects.positions=[0.0,-1.5700,-1.5700,1.0472,-1.5700,-1.5700,0.0,-1.5700,-1.5700]
	trainObjects.velocities=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	
	trainObjectsParallel = JointCommand()
	trainObjectsParallel.joint_names = joint_names
	trainObjectsParallel.positions=[0.0,-1.5700,-1.5700,0.0,-1.5700,-1.5700,0.0,-1.5700,-1.5700]
	trainObjectsParallel.velocities=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
