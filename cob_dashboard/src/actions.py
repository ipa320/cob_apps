#!/usr/bin/python
#***************************************************************
#
# Copyright (c) 2010
#
# Fraunhofer Institute for Manufacturing Engineering	
# and Automation (IPA)
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Project name: care-o-bot
# ROS stack name: cob_apps
# ROS package name: cob_dashboard
#								
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#			
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# Date of creation: May 2010
# ToDo:
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fraunhofer Institute for Manufacturing 
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************

import roslib; roslib.load_manifest('cob_dashboard')
import rospy
import actionlib
from pr2_controllers_msgs.msg import *
from cob_srvs.srv import *
from cob_actions.msg import *

from parameters import *

class base:
	def Stop(self):
		rospy.loginfo("base: Stop")
		
		try:
			rospy.wait_for_service('base_controller/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("base service server not ready, aborting...")
			return
			
		try:
			base_stop = rospy.ServiceProxy('base_controller/Stop', Trigger)
			resp = base_stop()
			print resp
		except rospy.ServiceException, e:
			print "base service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("base: Init")
		
		try:
			rospy.wait_for_service('base_controller/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("base service server not ready, aborting...")
			return
			
		try:
			base_srvCall = rospy.ServiceProxy('base_controller/Init', Trigger)
			resp = base_srvCall()
			print resp
		except rospy.ServiceException, e:
			print "base service call failed: %s"%e
			
		try:
			rospy.wait_for_service('base_driver/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("base service server not ready, aborting...")
			return
			
		try:
			base_srvCall = rospy.ServiceProxy('base_driver/Init', Trigger)
			resp = base_srvCall()
			print resp
		except rospy.ServiceException, e:
			print "base service call failed: %s"%e

class torso:
	def Stop(self):
		rospy.loginfo("torso: Stop")
		
		try:
			rospy.wait_for_service('torso_controller/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("torso service server not ready, aborting...")
			return
			
		try:
			torso_stop = rospy.ServiceProxy('torso_controller/Stop', Trigger)
			resp = torso_stop()
			print resp
		except rospy.ServiceException, e:
			print "torso service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("torso: Init")
		
		try:
			rospy.wait_for_service('torso_controller/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("torso service server not ready, aborting...")
			return
			
		try:
			torso_srvCall = rospy.ServiceProxy('torso_controller/Init', Trigger)
			resp = torso_srvCall()
			print resp
		except rospy.ServiceException, e:
			print "torso service call failed: %s"%e
			
	def SetOperationMode(self,operationMode):
		rospy.loginfo("torso: SetOperationMode to -%s-",operationMode)
		rospy.set_param('torso_controller/OperationMode', operationMode)
		
	def MoveTraj(self,traj):
		rospy.loginfo("torso: MoveTraj")
		rospy.set_param('torso_controller/OperationMode', "position")
		
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
		goal.trajectory.header.stamp = rospy.Time.now()
		self.client.send_goal(goal)

class tray:
	def Stop(self):
		rospy.loginfo("tray: Stop")
		
		try:
			rospy.wait_for_service('tray_controller/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("tray service server not ready, aborting...")
			return
			
		try:
			tray_stop = rospy.ServiceProxy('tray_controller/Stop', Trigger)
			resp = tray_stop()
			print resp
		except rospy.ServiceException, e:
			print "tray service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("tray: Init")
		
		try:
			rospy.wait_for_service('tray_controller/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("tray service server not ready, aborting...")
			return
			
		try:
			tray_srvCall = rospy.ServiceProxy('tray_controller/Init', Trigger)
			resp = tray_srvCall()
			print resp
		except rospy.ServiceException, e:
			print "tray service call failed: %s"%e
		
	def SetOperationMode(self,operationMode):
		rospy.loginfo("tray: SetOperationMode to -%s-",operationMode)
		rospy.set_param('tray_controller/OperationMode', operationMode)
		
	def MoveTraj(self,traj):
		rospy.loginfo("tray: MoveTraj")
		rospy.set_param('tray_controller/OperationMode', "position")
		
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
		goal.trajectory.header.stamp = rospy.Time.now()
		self.client.send_goal(goal)

class arm:
	def Stop(self):
		rospy.loginfo("arm: Stop")
		
		try:
			rospy.wait_for_service('arm_controller/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("arm service server not ready, aborting...")
			return
			
		try:
			arm_stop = rospy.ServiceProxy('arm_controller/Stop', Trigger)
			resp = arm_stop()
			print resp
		except rospy.ServiceException, e:
			print "arm service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("arm: Init")
		
		try:
			rospy.wait_for_service('arm_controller/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("arm service server not ready, aborting...")
			return
			
		try:
			arm_srvCall = rospy.ServiceProxy('arm_controller/Init', Trigger)
			resp = arm_srvCall()
			print resp
		except rospy.ServiceException, e:
			print "arm service call failed: %s"%e
			
	def SetOperationMode(self,operationMode):
		rospy.loginfo("arm: SetOperationMode to -%s-",operationMode)
		rospy.set_param('arm_controller/OperationMode', operationMode)
	
	def MoveTraj(self,traj):
		rospy.loginfo("arm: MoveTraj")
		rospy.set_param('arm_controller/OperationMode', "position")
		
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
		goal.trajectory.header.stamp = rospy.Time.now()
		self.client.send_goal(goal)
		
	def MoveArm3(self,name,name2):
		print "arm: MoveArm3"
		print name
		print name2

class lbr:
	def Stop(self):
		rospy.logerr("Can't stop lbr, stop not implemented...")
	def Init(self):
		rospy.logerr("Can't initialize lbr automatically, please init manually...")
		print ("login via telnet: telnet 192.168.42.146")
	def MoveTraj(self,traj):
		rospy.loginfo("lbr: MoveTraj")
		pub = rospy.Publisher('/Trajectory', JointTrajectory)
		pub.publish(traj)
	
class sdh:
	def Stop(self):
		rospy.loginfo("sdh_controller: Stop")
		
		try:
			rospy.wait_for_service('sdh_controller/Stop',5)
		except rospy.ROSException, e:
			rospy.logerr("sdh service server not ready, aborting...")
			return
			
		try:
			sdh_stop = rospy.ServiceProxy('sdh_controller/Stop', Trigger)
			resp = sdh_stop()
			print resp
		except rospy.ServiceException, e:
			print "sdh service call failed: %s"%e
	
	def Init(self):
		rospy.loginfo("sdh_controller: Init")
		
		try:
			rospy.wait_for_service('sdh_controller/Init',5)
		except rospy.ROSException, e:
			rospy.logerr("sdh service server not ready, aborting...")
			return
			
		try:
			sdh_srvCall = rospy.ServiceProxy('sdh_controller/Init', Trigger)
			resp = sdh_srvCall()
			print resp
		except rospy.ServiceException, e:
			print "sdh service call failed: %s"%e	
	def MoveCommand(self,command):
		rospy.loginfo("sdh_controller: MoveCommand")
		
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
		
	def MoveTraj(self,traj):
		rospy.loginfo("sdh_controller: MoveTraj")
		
		self.client = actionlib.SimpleActionClient(sdhTrajParameter.action_goal_topic, JointTrajectoryAction)
		rospy.logdebug("waiting for sdh action server to start")
		if not self.client.wait_for_server(rospy.Duration(5)):
			rospy.logerr("sdh action server not ready within timeout, aborting...")
			return
		else:
			rospy.logdebug("sdh action server ready")
		#print traj
		
		goal = JointTrajectoryGoal()
		goal.trajectory = traj
		goal.trajectory.header.stamp = rospy.Time.now()
		self.client.send_goal(goal)
