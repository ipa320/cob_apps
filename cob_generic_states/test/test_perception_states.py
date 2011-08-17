#!/usr/bin/python

import roslib; roslib.load_manifest('cob_generic_states')
import rospy
import smach
import smach_ros

from generic_perception_states import *

class evaluate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		#self.fail()
		return 'failed'


def test_perception_states():
	rospy.init_node('test_perception_states')


	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
	SM.userdata.pose = "home"

	# open the container
	with SM:

		# add states to the container
		smach.StateMachine.add('INITIALIZE', initialize(),
			transitions={'succeeded':'overall_succeeded', 'failed':'EVALUATE'})

		smach.StateMachine.add('EVALUATE', evaluate(),
			transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})

	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('TEST_PERCEPTION_STATES', SM, 'SM')
	smach_viewer.start()

	SM.execute()

	# stop SMACH viewer
	rospy.spin()
	# smach_thread.stop()
	smach_viewer.stop()

# main
if __name__ == '__main__':
	test_perception_states()
