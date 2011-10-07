#!/usr/bin/env python

PKG="cob_script_server"
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

from simple_script_server import *
sss = simple_script_server()

## This test checks the correct call to commands from the cob_script_server. This does not cover the execution of the commands (this shoud be checked in the package where the calls refer to).
class TestMove(unittest.TestCase):
	def __init__(self, *args):
		super(TestMove, self).__init__(*args)
		rospy.init_node('test_move')
		self.cb_executed = False

	# test move base commands
	def test_move_base(self):
		self.move_base()

#	def test_move_base_omni(self): #FIXME fails because client is already in DONE state (mode="" and mode="omni" is the same)
#		self.move_base(mode="omni")

	def test_move_base_diff(self):
		self.move_base(mode="diff")

	def test_move_base_linear(self):
		self.move_base(mode="linear")
	
	def move_base(self,mode=None):
		if mode == None or mode == "" or mode == "omni":
			as_name = "/move_base"
		else:
			as_name = "/move_base_" + mode
		self.as_server = actionlib.SimpleActionServer(as_name, MoveBaseAction, execute_cb=self.base_cb, auto_start=False)
		self.as_server.start()
		self.cb_executed = False
		handle = sss.move("base","home",mode=mode)
		if not self.cb_executed:
			self.fail('Action Server not called. script server error_code: ' + str(handle.get_error_code()))

	def base_cb(self, goal):
		self.cb_executed = True
		result = MoveBaseResult()
		self.as_server.set_succeeded(result)

	# test move trajectory commands
	def test_move_traj(self):
		component_name = "arm" # testing for component arm
		as_name = "/" + component_name + "_controller/joint_trajectory_action"
		self.as_server = actionlib.SimpleActionServer(as_name, JointTrajectoryAction, execute_cb=self.traj_cb, auto_start=False)
		self.as_server.start()
		self.cb_executed = False
		handle = sss.move(component_name,"home")
		if not self.cb_executed:
			self.fail('Action Server not called. script server error_code: ' + str(handle.get_error_code()))

	def traj_cb(self, goal):
		self.cb_executed = True
		result = JointTrajectoryResult()
		self.as_server.set_succeeded(result)
	
	# move cartesian
#	def test_move_cart(self):
#		component_name = "arm" # testing for component arm
#		as_name = "/" + component_name + "_controller/move_cart"
#		self.as_server = actionlib.SimpleActionServer(as_name, MoveCartAction, execute_cb=self.cart_cb, auto_start=False)
#		self.as_server.start()
#		self.cb_executed = False
#		sss.move(component_name,["base_link",[0,0,0],[0,0,0]])
#		if not self.cb_executed:
#			self.fail('Action Server not called')

#	def cart_cb(self, goal):
#		self.cb_executed = True
#		result = MoveCartResult()
#		self.as_server.set_succeeded(result)

	# move planned
#	def test_move_planned(self):
#		component_name = "arm" # testing for component arm
#		as_name = "/" + component_name + "_controller/move_cart"
#		self.as_server = actionlib.SimpleActionServer(as_name, MoveCartAction, execute_cb=self.planned_cb, auto_start=False)
#		self.as_server.start()
#		self.cb_executed = False
#		sss.move(component_name,["base_link",[0,0,0],[0,0,0]])
#		if not self.cb_executed:
#			self.fail('Action Server not called')

#	def planned_cb(self, goal):
#		self.cb_executed = True
#		result = MoveCartResult()
#		self.as_server.set_succeeded(result)

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'move', TestMove)
