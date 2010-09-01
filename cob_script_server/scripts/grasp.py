#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script


import tf
from geometry_msgs.msg import *

class GraspScript(script):
		
	def Initialize(self):
		# initialize components (not needed for simulation)
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.init("base")
		
		# move to initial positions
		handle01 = self.sss.move("arm","folded",False)
		self.sss.move("torso","home",False)
		self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle01.wait()
		print "Please localize the robot with rviz"
		self.sss.wait_for_input()
		self.sss.move("base","home")
		
	def Run(self): 
		listener = tf.TransformListener()
		transformer = tf.TransformerROS()
		br = tf.TransformBroadcaster()
	
		# prepare for grasping
		self.sss.move("base","kitchen")
		handle01 = self.sss.move("arm","pregrasp",False)
		self.sss.move("sdh","cylopen")
		handle01.wait()

		# get cup position (from camera)
		#caculate tranformations, we need cup coord in sdh_palm_link
		cup = PointStamped()
		cup.header.stamp = rospy.Time.now()
		cup.header.frame_id = "/map"
		cup.point.x = -3
		cup.point.y = 0
		cup.point.z = 0.98

		self.sss.sleep(2)
		
		if not self.sss.simulate:
			try:
				listener.waitForTransform('/map', '/arm_7_link', rospy.Time(0), rospy.Duration(5))
				(trans,rot) = listener.lookupTransform('/map', '/arm_7_link', rospy.Time(0))
				cup_arm = listener.transformPoint('/arm_7_link',cup)

				print cup_arm
				
				#self.sss.move_cart_rel("arm",[[cup_arm.point.x, cup_arm.point.y, cup_arm.point.z], [0, 0, 0]])
				self.sss.move_cart_rel("arm",[[-0.1874598889600168, -0.011396993072336177, -0.019201736647769573], [0, 0, 0]])

			except (tf.LookupException, tf.ConnectivityException):
				print "tf exception"
	

		# place cup on tray
		#...

		# deliver cup to home
		#say("here's your drink")

		# move components back to initial positions
		
if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
