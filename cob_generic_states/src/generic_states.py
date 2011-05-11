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

# \todo TODO create class 'approach_pose_without_retry'
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


#------------------------------------------------------------------------------------------#
#-----	SMACH STATES				-------------------------------------------------------#

class initiate(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['initiated', 'failed'],
			input_keys=['listener', 'message'],
			output_keys=['listener', 'message'])
		
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))

		# This state initializes all required components for executing a task.
		# This is however not needed when running in simulation.

		# \todo TODO assign outcome 'failed'
		# \todo TODO check if tray is empty

	def execute(self, userdata):

		userdata.listener = self.listener

		print "userdata.listener =", userdata.listener # for debugging

		# initialize components
		sss.init("eyes")
		sss.init("torso")
		sss.init("tray")
		sss.init("arm")
		sss.init("sdh")
		sss.init("base")

		# move to initial positions
		handle_head = sss.move("eyes", "back", False)
		handle_torso = sss.move("torso", "home", False)
		handle_tray = sss.move("tray", "down", False)
		handle_arm = sss.move("arm", "folded", False)
		handle_sdh = sss.move("sdh", "cylclosed", False)

		# wait for initial movements to finish
		handle_head.wait()
		handle_torso.wait()
		handle_tray.wait()
		handle_arm.wait()
		handle_sdh.wait()

		userdata.message = []
		userdata.message.append(3)
		userdata.message.append("Finished initializing components")
		return 'initiated'

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

class approach_pose(smach.State):

	def __init__(self, pose = ""):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'],
			input_keys=['pose', 'message'],
			output_keys=['pose', 'message'])

		self.pose = pose

		# This state moves the robot to the given pose.

		# \todo TODO retry process is not working

	def execute(self, userdata):

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

#		sub_move_base = rospy.Subscriber("/move_base/status", GoalStatusArray, self.cb_move_base)

		# try reaching pose
		handle_base = sss.move("base", pose, False)
		sss.say(["i am moving now"],False)
		handle_base.wait()
		handle_base = sss.move("base", pose, False)
		handle_base.wait()
		return 'succeeded'
#		rospy.wait_for_service('/base_controller/is_base_moving', 3)

#		timeout = 0
#		while True:
#			if self.move_base_status.status_list.status == 3:
#				userdata.message = []
#				userdata.message.append(3)
#				userdata.message.append("Pose was succesfully reached")
#				return 'succeeded'
#			try:
#				ret = self.move_base_status.status_list.status
#			except rospy.callback, e:
#				# print "Failed to retrieve message: %s"%e # for debugging
#				userdata.message = []
#				userdata.message.append(2)
#				userdata.message.append("Pose could not be reached, failed to call service 'is_base_moving()'")
#				return 'failed'
#			if ret.value == False:
#				if timeout > 20:
#					sss.say(["I can not reach my target position because my path or target is blocked"],False)
#					timeout = 0
#				else:
#					timeout = timeout + 1
#					rospy.sleep(1)

#		timeout = 0
#		while True:
#			if handle_base.get_state() == 3:
#				userdata.message = []
#				userdata.message.append(3)
#				userdata.message.append("Pose was succesfully reached")
#				return 'succeeded'
#			try:
#				ret = self.is_base_moving()
#			except rospy.ServiceException,e:
#				# print "Service call failed: %s"%e # for debugging
#				userdata.message = []
#				userdata.message.append(2)
#				userdata.message.append("Pose could not be reached, failed to call service 'is_base_moving()'")
#				return 'failed'
#			if ret.value == False:
#				if timeout > 20:
#					sss.say(["I can not reach my target position because my path or target is blocked"],False)
#					timeout = 0
#				else:
#					timeout = timeout + 1
#					rospy.sleep(1)

#	def cb_move_base(self, msg):
#		self.move_base_status = msg

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
			outcomes=['send_success', 'send_failure', 'send_status', 'send_interrupt', 'no_message_sent'],
			input_keys=['message'],
			output_keys=['message'])

		# Send message to master_node
		# Message types are:
		# 1 = INFO
		# 2 = ERROR
		# 3 = STATUS
		# 4 = INTERRUPT
		# 5 = INVALID

	def execute(self, userdata):

		if userdata.message[0] == 1:
			userdata.message[0] = "INFO ---> "
			print "\n", userdata.message, "\n"
			return 'send_success'
		elif userdata.message[0] == 2:
			userdata.message[0] = "ERROR ---> "
			print "\n", userdata.message, "\n"
			return 'send_failure'
		elif userdata.message[0] == 3:
			userdata.message[0] = "STATUS ---> "
			print "\n", userdata.message, "\n"
			return 'send_status'
		elif userdata.message[0] == 4:
			userdata.message[0] = "INTERRUPT ---> "
			print "\n", userdata.message, "\n"
			return 'send_interrupt'
		elif userdata.message[0] == 5:
			userdata.message[0] = "INVALID ---> "
			print "\n", userdata.message, "\n"
			return 'send_failure'
		else: # this should never happen
			print "\nERROR ---> Invalid message type: ", userdata.message[0], "\n"
			print "Message = ", userdata.message, "\n"
			return 'no_message_sent'

#------------------------------------------------------------------------------------------#


