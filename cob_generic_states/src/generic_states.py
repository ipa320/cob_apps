#!/usr/bin/python

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
			outcomes=['initiated'],
			input_keys=['initiate'],
			output_keys=['initiate'])

		# This state initializes all required components for executing a task.
		# This is however not needed when running in simulation.

	def execute(self, userdata):
	
		print "\nInitializing components\n"
	
		if userdata.initiate == 0:
		
			# initialize components
			sss.init("head")
			sss.init("torso")
			sss.init("tray")
			sss.init("arm")
			sss.init("sdh")
			sss.init("base")

			# move to initial positions
			handle_head = sss.move("head", "back", False)
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
			userdata.initiate = userdata.initiate + 1

		print "\nFinished initializing components\n"
		return 'initiated'

#------------------------------------------------------------------------------------------#

class approach_pose(smach.State):

	def __init__(self, pose = ""):

		smach.State.__init__(
			self,
			outcomes=['success', 'failed'],
			input_keys=['pose'],
			output_keys=['message', 'pose'])

		self.pose = pose
		
		# This state moves the robot to the given pose.
		# If the robot failes to reach the given pose it will retry.

	def execute(self, userdata):
			
		if self.pose != "":
			print "empty"
			pose = self.pose
		elif type(userdata.pose) is str:
			print "string"
			print userdata.pose
			pose = userdata.pose
		elif type(userdata.pose) is list:
			print "list"
			pose = []
			pose.append(userdata.pose[0])
			pose.append(userdata.pose[1])
			pose.append(userdata.pose[2])
		else: #this should never happen
			return 'failed'
	
		print "pose = ", userdata.pose
	
		# try first time
		handle_base = sss.move("base",pose,False)
		sss.say(["i am moving now"],False)
#		rospy.wait_for_service('/base_controller/is_base_moving', 3)

		timeout = 0
		while True:
			# print handle_base.get_state()
			if handle_base.get_state() == 3:
				userdata.message = ['1','Pose was succesfully reached']
				return 'success'

#			try:
#				ret = self.is_base_moving()
#			except rospy.ServiceException,e:
#				print "Service call failed: %s"%e
#				userdata.message = ['2','Pose was not reached succesfully']
#				return 'failed'
			
			# if ret.value == False: 
			if True: ############################## change back ###########################
				if timeout > 20:
					sss.say(["I can not reach my target position because my path or target is blocked"],False)
					timeout = 0
				else:
					timeout = timeout + 1
					rospy.sleep(1)

#------------------------------------------------------------------------------------------#

class approach_pose_without_retry(smach.State):

	def __init__(self):
	
		smach.State.__init__(self,
			outcomes=['success', 'failed'],
			input_keys=['pose'],
			output_keys=['message'])
			
#		self.is_base_moving = rospy.ServiceProxy('/base_controller/is_base_moving', ReturnBool)

		# This state moves the robot to the given pose.
		# If the robot failes to reach the given pose it will not retry.

	def execute(self, userdata):
	
		# try first time
		# print userdata.pose
		handle_base = sss.move("base",userdata.pose,False)
#		rospy.wait_for_service('/base_controller/is_base_moving', 3)

		timeout = 0
		while True:
			if handle_base.get_state() == 3:
				userdata.message = ['1','Pose was succesfully reached']
				return 'success'

#			try:
#				ret = self.is_base_moving()
#			except rospy.ServiceException,e:
#				print "Service call failed: %s"%e
#				userdata.message = ['2','Pose was not reached succesfully']
#				return 'failed'
			
			#if ret.value == False:
			if True: ############################## change back ###########################
				if timeout > 10:
					timeout = 0
					return 'failed'
				else:
					timeout = timeout + 1
					rospy.sleep(1)

#------------------------------------------------------------------------------------------#

class interrupt(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['no_interrupt', 'interrupted'],
			output_keys=['message'])
			
		# Checks if task is interrupted.

	def execute(self, userdata):
			
		print "\nSync with scheduler"
		print "Has task been interrupted?\n"
		
		while True:
			var = raw_input("1 = no,   2 = yes\n")
			if var == str(1) or var == str(2):
				break
		if var == str(1):
			print "Continue task\n"
			return 'no_interrupt'
		else:
			print "ERROR: Task has been interrupted\n"
			userdata.message = ['1','Task has been interrupted']
			return 'interrupted'

#------------------------------------------------------------------------------------------#

class message(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['sent_succes', 'sent_fail'],
			input_keys=['message'],
			output_keys=['message'])

		# Send message to master_node

	def execute(self, userdata):
		
		if userdata.message[0] == ['1']:
			userdata.message[0] = "INFO ---> "
			print "\n", userdata.message, "\n"
			return 'sent_success'
		elif userdata.message[0] == ['2']:
			userdata.message[0] = "ERROR ---> "
			print "\n", userdata.message, "\n"
			return 'sent_fail'

#------------------------------------------------------------------------------------------#


