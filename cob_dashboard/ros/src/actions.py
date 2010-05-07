#!/usr/bin/python

import time

import roslib; roslib.load_manifest('cob_dashboard')
import rospy
import actionlib
from pr2_controllers_msgs.msg import *
from cob_srvs.srv import *
from cob_actions.msg import *

from parameters import *

class torso:
	def Stop(self):
		rospy.loginfo("torso: Stop")
		
		try:
			rospy.wait_for_service('torso/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			torso_stop = rospy.ServiceProxy('torso/Stop', Trigger)
			resp = torso_stop()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("torso: Init")
		
		try:
			rospy.wait_for_service('torso/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			torso_init = rospy.ServiceProxy('torso/Init', Trigger)
			resp = torso_init()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
	def MoveTraj(self,traj):
		rospy.loginfo("torso: MoveTraj")
		
		self.client = actionlib.SimpleActionClient(torsoParameter.action_goal_topic, JointTrajectoryAction)
		rospy.logdebug("waiting for torso action server to start")
		if not self.client.wait_for_server(rospy.Duration(5)):
			rospy.logerr("torso action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("torso action server ready")
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		self.client.send_goal(goal)

class tray:
	def Stop(self):
		rospy.loginfo("tray: Stop")
		
		try:
			rospy.wait_for_service('tray/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			tray_stop = rospy.ServiceProxy('tray/Stop', Trigger)
			resp = tray_stop()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("tray: Init")
		
		try:
			rospy.wait_for_service('tray/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			tray_init = rospy.ServiceProxy('tray/Init', Trigger)
			resp = tray_init()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e		
		
	def MoveTraj(self,traj):
		rospy.loginfo("tray: MoveTraj")
		
		self.client = actionlib.SimpleActionClient(trayParameter.action_goal_topic, JointTrajectoryAction)
		rospy.logdebug("waiting for tray action server to start")
		if not self.client.wait_for_server(rospy.Duration(5)):
			rospy.logerr("tray action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("tray action server ready")
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		self.client.send_goal(goal)

class arm:
	def Stop(self):
		rospy.loginfo("arm: Stop")
		
		try:
			rospy.wait_for_service('arm/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			arm_stop = rospy.ServiceProxy('arm/Stop', Trigger)
			resp = arm_stop()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("arm: Init")
		
		try:
			rospy.wait_for_service('arm/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			arm_init = rospy.ServiceProxy('arm/Init', Trigger)
			resp = arm_init()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			
	def MoveTraj(self,traj):
		rospy.loginfo("arm: MoveTraj")
		
		self.client = actionlib.SimpleActionClient(armParameter.action_goal_topic, JointTrajectoryAction)
		rospy.logdebug("waiting for arm action server to start")
		if not self.client.wait_for_server(rospy.Duration(5)):
			rospy.logerr("arm action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("arm action server ready")
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		self.client.send_goal(goal)
		
	def MoveArm3(self,name,name2):
		print "arm: MoveArm3"
		print name
		print name2

class arm_pr2:
	def MoveTraj(self,traj):
		rospy.logdebug("arm_pr2: MoveTraj")
		
		self.client = actionlib.SimpleActionClient(armParameter_pr2.action_goal_topic, JointTrajectoryAction)
		rospy.logdebug("waiting for arm_pr2 action server to start")
		if not self.client.wait_for_server(rospy.Duration(5)):
			rospy.logerr("arm_pr2 action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("arm_pr2 action server ready")
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		self.client.send_goal(goal)
	
class sdh:
	def Stop(self):
		rospy.loginfo("sdh: Stop")
		
		try:
			rospy.wait_for_service('sdh/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			sdh_stop = rospy.ServiceProxy('sdh/Stop', Trigger)
			resp = sdh_stop()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("sdh: Init")
		
		try:
			rospy.wait_for_service('sdh/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("service server not ready, aborting...")
			return
			
		try:
			sdh_init = rospy.ServiceProxy('sdh/Init', Trigger)
			resp = sdh_init()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e	
	def MoveCommand(self,command):
		rospy.loginfo("sdh: MoveCommand")
		
		self.client = actionlib.SimpleActionClient(sdhParameter.action_goal_topic, JointCommandAction)
		rospy.logdebug("waiting for sdh action server to start")
		if not self.client.wait_for_server(rospy.Duration(5)):
			rospy.logerr("sdh action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("sdh action server ready")
		#print command
		
		goal = JointCommandGoal()
		goal.command = command
		self.client.send_goal(goal)
