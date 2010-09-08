#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *

class MyScript(script):

	def Initialize(self):
		pass

	def Run(self):
		listener = tf.TransformListener(True, rospy.Duration(10.0))
		self.sss.sleep(2)
		cup = PointStamped()
		cup.header.stamp = rospy.Time.now()
		cup.header.frame_id = "/map"
		cup.point.x = -2.95
		cup.point.y = 0.1
		cup.point.z = 0.98
		self.sss.sleep(2)
		
		self.sss.move("arm","pregrasp")
		
		if not self.sss.simulate:
			self.sss.sleep(2)
			#listener.waitForTransform('/map', '/arm_7_link', rospy.Time(0), rospy.Duration(5))
			cup_arm = listener.transformPoint('/arm_7_link',cup)

			print cup_arm
			
			#self.sss.move_cart_rel("arm",[[cup_arm.point.x, cup_arm.point.y, cup_arm.point.z-0.4], [0, 0, 0]])
			#self.sss.move_cart_rel("arm",[[0.0, 0.0, 0.2], [0, 0, 0]])
			
			#self.sss.move_cart_rel("arm",[[0.5, -0.12, -0.4], [0, 0, 0]])
			self.sss.move_cart_rel("arm",[[0.2, -0.1, -0.2], [0, 0, 0]])
		
		#self.sss.move_cart_rel("arm",[[0.0, 0.2, 0.0], [0.0, 0, 0]])
		#self.sss.move_cart_rel("arm",[[0.0, 0.0, 0.1], [0, 0, 0]])
		#self.sss.move_cart_rel("arm",[[-0.2, 0.0, 0.0], [0, 0, 0]])
		#self.sss.move_cart_rel("arm",[[0.0, 0.0, -0.1], [0, 0, 0]])
		#self.sss.move_cart_rel("arm",[[0.2, 0.0, 0.0], [0, 0, 0]])

if __name__ == "__main__":
	SCRIPT = MyScript()
	SCRIPT.Start()
