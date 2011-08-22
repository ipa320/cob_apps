#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_apps
# \note
#   ROS package name: cob_generic_states
#
# \author
#   Author: Daniel Maeki
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: May 2011
#
# \brief
#   Implements generic states which can be used in multiple scenarios.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################


#------------------------------------------------------------------------------------------#
#-----	INFO						-------------------------------------------------------#

# \todo TODO merge the states 'linear_movement' and 'back_away'

#------------------------------------------------------------------------------------------#
#-----	IMPORT MODULES				-------------------------------------------------------#

import roslib
roslib.load_manifest('cob_generic_states')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

import tf
from tf.transformations import *
from actionlib_msgs.msg import *


#------------------------------------------------------------------------------------------#
#-----	SMACH STATES				-------------------------------------------------------#

class initialize(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['initialized', 'failed'],
			input_keys=['listener', 'message'],
			output_keys=['listener', 'message'])
		
		# self.listener = tf.TransformListener(True, rospy.Duration(10.0))

		# This state initializes all required components for executing a task.
		# However, this is not needed when running in simulation.

		# \todo TODO assign outcome 'failed'
		# \todo TODO check if tray is empty

	def execute(self, userdata):

		# userdata.listener = self.listener

		# print "self.listener =", self.listener # for debugging
		# print "userdata.listener =", userdata.listener # for debugging

		#########################
		# initialize components #
		#########################
		
		handle_head = sss.init("head")
		if handle_head.get_error_code() != 0:
			return 'failed'

		handle_torso = sss.init("torso")
		if handle_torso.get_error_code() != 0:
			return 'failed'
			
		handle_tray = sss.init("tray")
		if handle_tray.get_error_code() != 0:
			return 'failed'

		#handle_arm = sss.init("arm")
		#if handle_arm.get_error_code() != 0:
		#	return 'failed'

		handle_sdh = sss.init("sdh")
		#if handle_sdh.get_error_code() != 0:
		#	return 'failed'

		handle_base = sss.init("base")
		if handle_base.get_error_code() != 0:
			return 'failed'		
		
		######################
		# recover components #
		######################
		
		handle_head = sss.recover("head")
		if handle_head.get_error_code() != 0:
			return 'failed'		
		
		handle_torso = sss.recover("torso")
		if handle_torso.get_error_code() != 0:
			return 'failed'
		
		handle_tray = sss.recover("tray")
		if handle_tray.get_error_code() != 0:
			return 'failed'

		handle_arm = sss.recover("arm")
		#if handle_arm.get_error_code() != 0:
		#	return 'failed'

		#handle_sdh = sss.recover("sdh")
		#if handle_sdh.get_error_code() != 0:
		#	return 'failed'

		handle_base = sss.recover("base")
		if handle_base.get_error_code() != 0:
			return 'failed'

		# set light
		sss.set_light("green")

		userdata.message = []
		userdata.message.append(3)
		userdata.message.append("Finished initializing components")
		return 'initialized'

#------------------------------------------------------------------------------------------#

class interrupt(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['no_interruption', 'interrupted'],
			input_keys=['message'],
			output_keys=['message'])

		# Sync with scheduler
		# Checks if task has been interrupted.

		# \todo TODO check with 'master_node' for interruption

	def execute(self, userdata):

		print "\nHas task been interrupted?\n"
		while True:
			var = raw_input("1 = no, 2 = yes\n")
			if var == str(1) or var == str(2):
				break
		if var == str(1):
			userdata.message = []
			userdata.message.append(3)
			userdata.message.append("Task has not been interrupted, continuing task")
			return 'no_interruption'
		else:
			userdata.message = []
			userdata.message.append(4)
			userdata.message.append("Task has been interrupted")
			return 'interrupted'

#------------------------------------------------------------------------------------------#

# This state moves the robot to the given pose.
class approach_pose(smach.State):

	def __init__(self, pose = "", mode = "omni", move_second = "False"):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'],
			input_keys=['pose', 'message'],
			output_keys=['pose', 'message'])

		self.pose = pose
		self.mode = mode
		self.move_second = move_second

	def execute(self, userdata):

		# determine target position
		if self.pose != "":
			pose = self.pose
		elif type(userdata.pose) is str:
			pose = userdata.pose
		elif type(userdata.pose) is list:
			pose = []
			pose.append(userdata.pose[0])
			pose.append(userdata.pose[1])
			pose.append(userdata.pose[2])
		else: # this should never happen
			userdata.message = []
			userdata.message.append(5)
			userdata.message.append("Invalid userdata 'pose'")
			userdata.message.append(userdata.pose)
			return 'failed'

		# try reaching pose
		handle_base = sss.move("base", pose, False)
		move_second = self.move_second

		timeout = 0
		while True:
			if (handle_base.get_state() == 3) and (not move_second):
				# do a second movement to place the robot more exactly
				handle_base = sss.move("base", pose, False)
				move_second = True
			elif (handle_base.get_state() == 3) and (move_second):
				return 'succeeded'			

			# check if service is available
			service_full_name = '/base_controller/is_moving'
			try:
				rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
			except rospy.ROSException, e:
				error_message = "%s"%e
				rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
				return 'failed'
		
			# check if service is callable
			try:
				is_moving = rospy.ServiceProxy(service_full_name,Trigger)
				resp = is_moving()
			except rospy.ServiceException, e:
				error_message = "%s"%e
				rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
				return 'failed'
		
			# evaluate sevice response
			if not resp.success.data: # robot stands still
				if timeout > 10:
					sss.say(["I can not reach my target position because my path or target is blocked"],False)
					timeout = 0
				else:
					timeout = timeout + 1
					rospy.sleep(1)
			else:
				timeout = 0

#------------------------------------------------------------------------------------------#

class approach_pose_without_retry(smach.State):

	def __init__(self, pose = ""):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'],
			input_keys=['pose', 'message'],
			output_keys=['pose', 'message'])

		sub_move_base = rospy.Subscriber("/move_base/status", GoalStatusArray, self.cb_move_base)
		self.pose = pose

		# This state moves the robot to the given pose.

	def execute(self, userdata):

		# determine target position
		if self.pose != "":
			pose = self.pose
		elif type(userdata.pose) is str:
			pose = userdata.pose
		elif type(userdata.pose) is list:
			pose = []
			pose.append(userdata.pose[0])
			pose.append(userdata.pose[1])
			pose.append(userdata.pose[2])
		else: # this should never happen
			userdata.message = []
			userdata.message.append(5)
			userdata.message.append("Invalid userdata 'pose'")
			userdata.message.append(userdata.pose)
			return 'failed'

		# try reaching pose
		handle_base = sss.move("base", pose, False)
		move_second = False

		timeout = 0
		while True:
			if (handle_base.get_state() == 3) and (not move_second):
				# do a second movement to place the robot more exactly
				handle_base = sss.move("base", pose, False)
				move_second = True
			elif (handle_base.get_state() == 3) and (move_second):
				userdata.message = []
				userdata.message.append(3)
				userdata.message.append("Pose was succesfully reached")
				return 'succeeded'		

			# check if service is available
			service_full_name = '/base_controller/is_moving'
			try:
				rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
			except rospy.ROSException, e:
				error_message = "%s"%e
				rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
				return 'failed'
		
			# check if service is callable
			try:
				is_moving = rospy.ServiceProxy(service_full_name,Trigger)
				resp = is_moving()
			except rospy.ServiceException, e:
				error_message = "%s"%e
				rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
				return 'failed'
		
			# evaluate sevice response
			if not resp.success.data: # robot stands still
				if timeout > 10:
					sss.say(["I can not reach my target position because my path or target is blocked, I will abort."],False)
					rospy.wait_for_service('base_controller/stop',10)
					try:
						stop = rospy.ServiceProxy('base_controller/stop',Trigger)
						resp = stop()
					except rospy.ServiceException, e:
						error_message = "%s"%e
						rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
					return 'failed'
				else:
					timeout = timeout + 1
					rospy.sleep(1)
			else:
				timeout = 0

	def cb_move_base(self, msg):
		self.move_base_status = msg

#------------------------------------------------------------------------------------------#

class linear_movement(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'],
			input_keys=['message'],
			output_keys=['message'])

		# \todo TODO implement linear base moving

	def execute(self, userdata):

		print "\nApproach in a linear way\n"

		print "\nHas destination been reached?\n"
		while True:
			var = raw_input("1 = yes, 2 = no\n")
			if var == str(1) or var == str(2):
				break
		if var == str(1):
			userdata.message = []
			userdata.message.append(3)
			userdata.message.append("Destination has been reached")
			return 'succeeded'
		else:
			userdata.message = []
			userdata.message.append(2)
			userdata.message.append("Could not reach destination")
			return 'failed'

#------------------------------------------------------------------------------------------#

class back_away(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['backed_away', 'failed'],
			input_keys=['message'],
			output_keys=['message'])

		# \todo TODO implement linear base movement

	def execute(self, userdata):

		print "Backed away successfully?\n"
		while True:
			var = raw_input("1 = yes, 2 = no\n")
			if var == str(1) or var == str(2):
				break
		if var == str(1):
			userdata.message = []
			userdata.message.append(3)
			userdata.message.append("Backed away")
			return 'backed_away'
		else:
			userdata.message = []
			userdata.message.append(2)
			userdata.message.append("Failed to back away")
			return 'failed'

#------------------------------------------------------------------------------------------#

class message(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['no_message_sent', 'quit'],
			input_keys=['message'],
			output_keys=['message'])

		# Send message to master_node
		# Message types are:
		# 0 = QUIT
		# 1 = INFO
		# 2 = ERROR
		# 3 = STATUS
		# 4 = INTERRUPT
		# >5 = INVALID USERDATA

	def execute(self, userdata):

		while True:
			# userdata.message[0] = userdata.message[0] -1
			message = userdata.message
			if message[0] >= 0:
				if message[0] == 0:
					message[0] = "QUIT MESSAGE"
					print "\n", userdata.message, "\n"
					return 'quit'
				elif message[0] == 1:
					message[0] = "INFO MESSAGE"
					print "\n", userdata.message, "\n"
				elif message[0] == 2:
					message[0] = "ERROR MESSAGE"
					print "\n", userdata.message, "\n"
				elif message[0] == 3:
					message[0] = "STATUS MESSAGE"
					print "\n", userdata.message, "\n"
				elif message[0] == 4:
					message[0] = "INTERRUPT MESSAGE"
					print "\n", userdata.message, "\n"
				else:
					message[0] = "INVALID USERDATA MESSAGE"
					print "\n", userdata.message, "\n"
				userdata.message[0] = -1

#------------------------------------------------------------------------------------------#

