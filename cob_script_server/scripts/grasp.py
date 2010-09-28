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
		#self.sss.init("tray")
		#self.sss.init("torso")
		#self.sss.init("arm")
		#self.sss.init("sdh")
		#self.sss.init("base")
		
		# move to initial positions
		handle01 = self.sss.move("arm","folded",False)
		self.sss.move("torso","home",False)
		self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle01.wait()
		print "Please localize the robot with rviz"
		self.sss.wait_for_input()
		#self.sss.move("base","home")
		
	def Run(self): 
		listener = tf.TransformListener(True, rospy.Duration(10.0))
	
		# prepare for grasping
		self.sss.move("base","kitchen")
		handle01 = self.sss.move("arm","pregrasp",False)
		self.sss.move("sdh","cylopen")
		handle01.wait()

		# caculate tranformations, we need cup coordinates in arm_7_link coordinate system
		cup = PointStamped()
		cup.header.stamp = rospy.Time.now()
		cup.header.frame_id = "/map"
		cup.point.x = -2.95
		cup.point.y = 0.1
		cup.point.z = 0.98
		self.sss.sleep(2)
		
		if not self.sss.simulate:
			cup = listener.transformPoint('/arm_7_link',cup)

		#print "cup: ", cup		
		self.sss.move_cart_rel("arm",[[cup.point.x, cup.point.y, cup.point.z-0.4], [0, 0, 0]])
		self.sss.move_cart_rel("arm",[[0.0, 0.0, 0.2], [0, 0, 0]])
	

		# place cup on tray
		self.sss.move("sdh","cup")
		handle01 = self.sss.move("arm","grasp-to-tablet",False)
		self.sss.move("tray","up")
		handle01.wait()
		self.sss.move("sdh","cylopen")
		self.sss.move_cart_rel("arm",[[0.0, 0.0, -0.2], [0, 0, 0]])
		handle01 = self.sss.move("arm","tablet-to-folded",False)
		self.sss.sleep(4)
		self.sss.move("sdh","cylclosed",False)
		handle01.wait()

		# deliver cup to home
		self.sss.move("base","order")
		#say("here's your drink")
		self.sss.move("torso","nod")

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
