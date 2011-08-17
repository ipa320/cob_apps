#!/usr/bin/python

PKG = 'cob_generic_states'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import unittest

from geometry_msgs import *

from generic_manipulation_states import *

class TestStates(unittest.TestCase):
	def __init__(self, *args):
		super(TestStates, self).__init__(*args)
		rospy.init_node('test_states')

	def test_grasp_top(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		SM.userdata.object_pose = PoseStamped()

		# open the container
		with SM:
			smach.StateMachine.add('GRASP_TOP', grasp_top(),
				transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			self.fail(error_message)

	def test_grasp_side(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		SM.userdata.object_pose = PoseStamped()

		# open the container
		with SM:
			smach.StateMachine.add('GRASP_SIDE', grasp_side(),
				transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
			self.fail(error_message)


# main
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'manipulation', TestStates)
