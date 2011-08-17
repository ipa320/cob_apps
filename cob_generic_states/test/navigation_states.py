#!/usr/bin/python

PKG = 'cob_generic_states'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import unittest

from generic_navigation_states import *

class TestStates(unittest.TestCase):
	def __init__(self, *args):
		super(TestStates, self).__init__(*args)
		rospy.init_node('test_states')

	def test_approach_pose(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		SM.userdata.pose = "home"

		# open the container
		with SM:
			smach.StateMachine.add('APPROACH_POSE', approach_pose(),
				transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			self.fail(error_message)

	def test_approach_pose_without_retry(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		SM.userdata.pose = "home"

		# open the container
		with SM:
			smach.StateMachine.add('APPROACH_POSE_WITHOUT_RETRY', approach_pose_without_retry(),
				transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			self.fail(error_message)


# main
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'navigation', TestStates)
