#!/usr/bin/python

######################### IMPORTS #########################

import roslib
roslib.load_manifest('cob_generic_states')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()


class approach_pose(smach.State):

	def __init__(self,pose = ""):

		smach.State.__init__(
			self,
			outcomes=['success', 'fail'],
			input_keys=['pose'],
			output_keys=['message'])

		self.pose = pose
		
		# ---

	def execute(self, userdata):
			
		if self.pose != "":
			pose = self.pose
		else:
			pose = userdata.pose
	
		# try first time
		handle_base = sss.move("base",pose,False)
		sss.say(["i am moving now"],False)
#		rospy.wait_for_service('/base_controller/is_base_moving', 3)

		timeout = 0
		while True:
			print handle_base.get_state()
			if handle_base.get_state() == 2: #3:
				userdata.message = ['info',' - pose was succesfully reached']
				return 'success'

#			try:
#				ret = self.is_base_moving()
#			except rospy.ServiceException,e:
#				print "Service call failed: %s"%e
#				userdata.message = ['error',' - pose was not reached succesfully']
#				return 'fail'
			
			#if ret.value == False: 
			if True: ############################## change back ###########################
				if timeout > 40:
					sss.say(["I can not reach my target position because my path or target is blocked"],False)
					timeout = 0
				else:
					timeout = timeout + 1
					rospy.sleep(1)

class approach_pose_without_retry(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['success', 'fail'],
			input_keys=['pose'],
			output_keys=['message'])
#		self.is_base_moving = rospy.ServiceProxy('/base_controller/is_base_moving', ReturnBool)

	def execute(self, userdata):
		# try first time
		print userdata.pose
		handle_base = sss.move("base",userdata.pose,False)
#		rospy.wait_for_service('/base_controller/is_base_moving', 3)

		timeout = 0
		while True:
			if handle_base.get_state() == 3:
				userdata.message = ['info',' - pose was succesfully reached']
				return 'success'

#			try:
#				ret = self.is_base_moving()
#			except rospy.ServiceException,e:
#				print "Service call failed: %s"%e
#				userdata.message = ['error',' - pose was not reached succesfully']
#				return 'fail'
			
			#if ret.value == False:
			if True: ############################## change back ###########################
				if timeout > 10:
					timeout = 0
					return 'failed'
				else:
					timeout = timeout + 1
					rospy.sleep(1)

class interrupt(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['no_interrupt', 'interrupted'],
			output_keys=['message'])
			
		# ---

	def execute(self, userdata):
		
		return 'no_interrupt'
			
#		print "\nSync with scheduler"
#		print "Has task been interrupted?"
#		while True:
#			var = raw_input("1 = no,   2 = yes\n")
#			if var == str(1) or var == str(2):
#				break
#		if var == str(1):
#			print "Continue task"
#			return 'no_interrupt'
#		else:
#			userdata.message = ['info',' - Task has been interrupted']
#			print "Task has been interrupted"
#			return 'interrupted'

class message(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['sent_succes', 'sent_fail'],
			input_keys=['message'])

		# ---

	def execute(self, userdata):

		print userdata.message
		return 'sent_fail'
#		if userdata.message == ['info']:
#			return 'sent_success'
#		elif userdata.message == ['error']:
#			return 'sent_fail'
