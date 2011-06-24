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
		handle_arm = self.sss.move("arm","folded",False)
		handle_torso = self.sss.move("torso","home",False)
		handle_sdh = self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle_arm.wait()
		handle_torso.wait()
		handle_sdh.wait()
		if not self.sss.parse:
			print "Please localize the robot with rviz"
		self.sss.wait_for_input()
		
	def Run(self): 
		listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2)
	
		# prepare for grasping
		self.sss.move("base","kitchen")
		self.sss.move("arm","pregrasp")
		handle_sdh = self.sss.move("sdh","cylopen",False)
		handle_sdh.wait()

		# caculate tranformations, we need cup coordinates in arm_7_link coordinate system
		cup = PointStamped()
		cup.header.stamp = rospy.Time.now()
		cup.header.frame_id = "/map"
		cup.point.x = -3.0
		cup.point.y = 0.08
		cup.point.z = 1.02
		rospy.sleep(2)
		
		if not self.sss.parse:
			cup = listener.transformPoint('/arm_7_link',cup)
			rospy.sleep(2)
			# transform grasp point to sdh center
			cup.point.z = cup.point.z - 0.2

		# move in front of cup
		pregrasp_distance = 0.2
		grasp_offset = 0.05 # offset between arm_7_link and sdh_grasp_link
		self.sss.move_cart_rel("arm",[[cup.point.x, cup.point.y, cup.point.z-grasp_offset-pregrasp_distance], [0, 0, 0]])
		# move to cup
		self.sss.move_cart_rel("arm",[[0.0, 0.0, pregrasp_distance], [0, 0, 0]])
		# grasp cup
		self.sss.move("sdh","china_cup")
		# lift cup
		self.sss.move_cart_rel("arm",[[0.2, -0.1, -0.2], [0, 0, 0]])
	

		# place cup on tray
		handle01 = self.sss.move("arm","grasp-to-tray",False)
		self.sss.move("tray","up")
		handle01.wait()
		self.sss.move("arm","tray")
		self.sss.move("sdh","cylopen")
		self.sss.move_cart_rel("arm",[[0.0, 0.0, -0.1], [0, 0, 0]])
		handle01 = self.sss.move("arm","tray-to-folded",False)
		self.sss.sleep(4)
		self.sss.move("sdh","cylclosed",False)
		handle01.wait()

		# deliver cup to order position
		self.sss.move("base","order")
		self.sss.say("Here's your drink.")
		self.sss.move("torso","nod")

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
