#!/usr/bin/python

PKG = 'cob_generic_states'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import unittest

from generic_manipulation_states import *

def test_manipulation_states():
	rospy.init_node('test_manipulation_states')


	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
	SM.userdata.pose = "home"

	# open the container
	with SM:

		# add states to the container
		smach.StateMachine.add('GRASP_TOP', grasp_top(),
			transitions={'succeeded':'GRASP_SIDE', 'failed':'GRASP_SIDE'})

		smach.StateMachine.add('GRASP_SIDE', grasp_side(),
			transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('TEST_MANIPULATION_STATES', SM, 'SM')
	smach_viewer.start()

	SM.execute()

	# stop SMACH viewer
	rospy.spin()
	# smach_thread.stop()
	smach_viewer.stop()

# main
if __name__ == '__main__':
	test_manipulation_states()
