#!/usr/bin/python

import roslib; roslib.load_manifest('cob_dashboard')
import rospy
import actionlib
from pr2_controllers_msgs.msg import *

from parameters import *

class arm:
	def Stop(self):
		print "arm: Stop"
		
	def MoveTraj(self,traj):
		print "arm: MoveTraj"
		
		self.client = actionlib.SimpleActionClient(armParameter.action_goal_topic, JointTrajectoryAction)
		#print "waiting for action server to start"
		#self.client.wait_for_server()
		
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		self.client.send_goal(goal)
		
		#print "waiting for action result"
		#self.client.wait_for_result()
		#print self.client.get_result()
		
	def MoveArm3(self,name,name2):
		print "arm: MoveArm3"
		print name
		print name2

class arm_pr2:
	def MoveTraj(self,traj):
		print "arm_pr2: MoveTraj"
		
		self.client = actionlib.SimpleActionClient(armParameter_pr2.action_goal_topic, JointTrajectoryAction)
		#print "waiting for action server to start"
		#self.client.wait_for_server()
		
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		self.client.send_goal(goal)
		
		#print "waiting for action result"
		#self.client.wait_for_result()
		#print self.client.get_result()

class torso:
	def Stop(self):
		print "torso: Stop"
		
	def MoveTraj(self,traj):
		print "arm: MoveTraj"
		
		self.client = actionlib.SimpleActionClient(torsoParameter.action_goal_topic, JointTrajectoryAction)
		#print "waiting for action server to start"
		#self.client.wait_for_server()
		
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		self.client.send_goal(goal)
		
		#print "waiting for action result"
		#self.client.wait_for_result()
		#print self.client.get_result()
		
class sdh:
	def MovePos(self):
		print "MoveArm"
	def MovePos(self,name):
		print "MoveArm2"
		print name
	def MovePos(self,name,name2):
		print "MoveArm3"
		print name
		print name2
