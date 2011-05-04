#!/usr/bin/python

#------------------------------------------------------------------------------------------#
#-----	INFO						-------------------------------------------------------#

# Script: Wimicare project: generic_states
# Author: Daniel Maeki (taj-dm)
# Assisting author(s): Florian Weisshardt


#------------------------------------------------------------------------------------------#
#-----	IMPORT MODULES				-------------------------------------------------------#

import roslib
roslib.load_manifest('cob_generic_states')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()


#------------------------------------------------------------------------------------------#
#-----	SMACH STATES				-------------------------------------------------------#

class initiate(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['initiated', 'failed'],
			input_keys=['initiate', 'message'],
			output_keys=['initiate', 'message'])

		# This state initializes all required components for executing a task.
		# This is however not needed when running in simulation.

	def execute(self, userdata):

		if userdata.initiate == 0:
			# initialize components
			sss.init("head")
			sss.init("torso")
			sss.init("tray")
			sss.init("arm")
			sss.init("sdh")
			sss.init("base")
			# move to initial positions
			handle_head = sss.move("head", "home", False)
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
			# initialization is only needed to run once
			userdata.initiate = 1
			userdata.message = []
			userdata.message.append(3)
			userdata.message.append("Finished initializing components")
			return 'initiated'
		elif userdata.initiate == 1:
			userdata.message = []
			userdata.message.append(3)
			userdata.message.append("The robot has been initialized")
			return 'initiated'
		else: # this should never happen
			userdata.message = []
			userdata.message.append(5)
			userdata.message.append("Invalid userdata 'initiate'")
			userdata.message.append(userdata.initiate)
			return 'failed'

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
		# If the robot failes to reach the given pose it will retry.

	def execute(self, userdata):

		# TODO fix the retry process for when approaching a pose
			
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

		# try first time
		handle_base = sss.move("base", pose, False)
		sss.say(["i am moving now"],False)
#		rospy.wait_for_service('/base_controller/is_base_moving', 3)

		handle_base.wait()
		handle_base = sss.move("base", pose, False)
		handle_base.wait()
		return 'succeeded'

#		timeout = 0
#		while True:
#			if handle_base.get_state() == 3:
#				userdata.message = []
#				userdata.message.append(1)
#				userdata.message.append("Pose was succesfully reached")
#				return 'succeeded'

#			try:
#				ret = self.is_base_moving()
#			except rospy.ServiceException,e:
#				print "Service call failed: %s"%e
#				userdata.message = []
#				userdata.message.append(2)
#				userdata.message.append('Pose was not reached succesfully')
#				return 'failed'

#			if ret.value == False: 
#			if True: ############################## change back ###########################
#				if timeout > 20:
#					sss.say(["I can not reach my target position because my path or target is blocked"],False)
#					timeout = 0
#				else:
#					timeout = timeout + 1
#					rospy.sleep(1)

#------------------------------------------------------------------------------------------#

#class approach_pose_without_retry(smach.State):
#
#	def __init__(self):
#
#		smach.State.__init__(self,
#			outcomes=['succeeded', 'failed'],
#			input_keys=['pose'],
#			output_keys=['message'])
#
#		self.is_base_moving = rospy.ServiceProxy('/base_controller/is_base_moving', ReturnBool)
#
#		# This state moves the robot to the given pose.
#		# If the robot failes to reach the given pose it will not retry.
#
#	def execute(self, userdata):
#
#		# try first time
#		# print userdata.pose
#		handle_base = sss.move("base",userdata.pose,False)
#		rospy.wait_for_service('/base_controller/is_base_moving', 3)
#
#		timeout = 0
#		while True:
#			if handle_base.get_state() == 3:
#				userdata.message = ['1','Pose was succesfully reached']
#				return 'succeeded'
#
#			try:
#				ret = self.is_base_moving()
#			except rospy.ServiceException,e:
#				print "Service call failed: %s"%e
#				userdata.message = ['2','Pose was not reached succesfully']
#				return 'failed'
#
#			#if ret.value == False:
#			if True: ############################## change back ###########################
#				if timeout > 10:
#					timeout = 0
#					return 'failed'
#				else:
#					timeout = timeout + 1
#					rospy.sleep(1)

#------------------------------------------------------------------------------------------#

class interrupt(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['no_interrupt', 'interrupted'],
			input_keys=['message'],
			output_keys=['message'])

		# Sync with scheduler
		# Checks if task has been interrupted.

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
			return 'no_interrupt'
		else:
			userdata.message = []
			userdata.message.append(4)
			userdata.message.append("Task has been interrupted")
			return 'interrupted'

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


