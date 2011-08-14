#!/usr/bin/env python

PKG="cob_script_server"
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

import actionlib

from simple_script_server import *

## This test checks the correct call to commands from the cob_script_server. This does not cover the execution of the commands (this shoud be checked in the package where the calls refer to).
class TestActionInterface(unittest.TestCase):
	def __init__(self, *args):
		super(TestActionInterface, self).__init__(*args)
		rospy.init_node('test_action_interface')
		self.cb_executed = False
		self.component_name = "arm" # testing for component arm
		self.client = actionlib.SimpleActionClient('/script_server', ScriptAction)

	# test trigger commands
	def test_init(self):
		goal = ScriptGoal()
		goal.function_name = "init"
		goal.component_name = "arm"
		self.script_action_trigger(goal)

	def test_stop(self):
		goal = ScriptGoal()
		goal.function_name = "stop"
		goal.component_name = "arm"
		self.script_action_trigger(goal)

	def test_recover(self):
		goal = ScriptGoal()
		goal.function_name = "recover"
		goal.component_name = "arm"
		self.script_action_trigger(goal)
		
	def script_action_trigger(self,goal):
		rospy.Service("/" + goal.component_name + "_controller/" + goal.function_name, Trigger, self.cb)
		self.cb_executed = False
		self.client.wait_for_server()
		self.client.send_goal(goal)
		self.client.wait_for_result()
		if not self.cb_executed:
			self.fail('Service Server not called')

	def cb(self,req):
		self.cb_executed = True
		res = TriggerResponse()
		res.success.data = True
		return res

	# test move base commands
	def test_move_base(self):
		goal = ScriptGoal()
		goal.function_name = "move"
		goal.component_name = "base"
		goal.parameter_name = "home"
		self.script_action_move_base(goal)

	def test_move_base_omni(self):
		goal = ScriptGoal()
		goal.function_name = "move"
		goal.component_name = "base"
		goal.parameter_name = "home"
		goal.mode = "omni"
		self.script_action_move_base(goal)

	def test_move_base_diff(self):
		goal = ScriptGoal()
		goal.function_name = "move"
		goal.component_name = "base"
		goal.parameter_name = "home"
		goal.mode = "diff"
		self.script_action_move_base(goal)

	def test_move_base_linear(self):
		goal = ScriptGoal()
		goal.function_name = "move"
		goal.component_name = "base"
		goal.parameter_name = "home"
		goal.mode = "linear"
		self.script_action_move_base(goal)

	def script_action_move_base(self,goal):
		if goal.mode == None or goal.mode == "" or goal.mode == "omni":
			as_name = "/move_base"
		else:
			as_name = "/move_base_" + goal.mode
		self.as_server = actionlib.SimpleActionServer(as_name, MoveBaseAction, execute_cb=self.base_cb, auto_start=False)
		self.as_server.start()
		self.cb_executed = False
		self.client.wait_for_server()
		self.client.send_goal(goal)
		self.client.wait_for_result()
		if not self.cb_executed:
			self.fail('Action Server not called')

	def base_cb(self, goal):
		self.cb_executed = True
		result = MoveBaseResult()
		self.as_server.set_succeeded(result)

	# test move trajectory commands
	#TODO

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'action_interface', TestActionInterface)
