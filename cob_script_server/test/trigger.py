#!/usr/bin/env python

PKG="cob_script_server"
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

from simple_script_server import *
sss = simple_script_server()

## This test checks the correct call to commands from the cob_script_server. This does not cover the execution of the commands (this shoud be checked in the package where the calls refer to).
class TestTrigger(unittest.TestCase):
	def __init__(self, *args):
		super(TestTrigger, self).__init__(*args)
		rospy.init_node('test_trigger')
		self.cb_executed = False
		self.component_name = "arm"  # testing for component arm

	def test_init(self):
		rospy.Service("/" + self.component_name + "_controller/init", Trigger, self.cb)
		self.cb_executed = False
		handle = sss.init(self.component_name)
		if not self.cb_executed:
			self.fail('Service Server not called. script server error_code: ' + str(handle.get_error_code()))

	def test_stop(self):
		rospy.Service("/" + self.component_name + "_controller/stop", Trigger, self.cb)
		self.cb_executed = False
		handle = sss.stop(self.component_name)
		if not self.cb_executed:
			self.fail('Service Server not called. script server error_code: ' + str(handle.get_error_code()))

	def test_recover(self):
		rospy.Service("/" + self.component_name + "_controller/recover", Trigger, self.cb)
		self.cb_executed = False
		handle = sss.recover(self.component_name)
		if not self.cb_executed:
			self.fail('Service Server not called. script server error_code: ' + str(handle.get_error_code()))

	def cb(self,req):
		self.cb_executed = True
		res = TriggerResponse()
		res.success.data = True
		return res

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'trigger', TestTrigger)
