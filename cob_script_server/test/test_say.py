#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_script_server')

import sys
import unittest

import rospy
import rostest

from simple_script_server import *
sss = simple_script_server()

class SayTest(unittest.TestCase):
	def __init__(self, *args):
		super(SayTest, self).__init__(*args)
		rospy.init_node('test_say_test')

	def test_say(self):
		sss.say(["hello"])

	def as_cb(self, goal):
		result = JointTrajectoryResult()
		#self.as_server.set_preempted(result)
		self.as_cb_executed = True
		self.traj = goal.trajectory
		print "action server callback"
		self.as_server.set_succeeded(result)

if __name__ == '__main__':
	try:
		rostest.run('rostest', 'test_say_test', SayTest, sys.argv)
	except KeyboardInterrupt, e:
		pass
	print "exiting"

