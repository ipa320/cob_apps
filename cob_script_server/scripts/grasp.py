#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

class GraspScript(script):
		
	def Initialize(self):
		# initialize components (not needed for simulation)
		#self.sss.init("tray")
		#self.sss.init("torso")
		#self.sss.init("arm")
		#self.sss.init("sdh")
		#self.sss.init("base")
		
		self.sss.move("arm","pregrasp")
		
		self.sss.move_cart_rel("arm",[[0,0,0.1],[0,0,0]])
		self.sss.wait_for_input()
		
		self.sss.move_cart_rel("arm",[[0,0,-0.1],[0,0,0]])
		self.sss.wait_for_input()		
		
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
		# prepare for grasping
		self.sss.move("base","kitchen")
		handle01 = self.sss.move("arm","pregrasp",False)
		self.sss.move("sdh","cylopen")
		handle01.wait()

		# get cup position (from camera)
		#caculate tranformations, we need cup coord in sdh_palm_link
		#self.sss.move_cart_rel([x,y,z],[r,p,y])

		# place cup on tray
		#...

		# deliver cup to home
		#say("here's your drink")

		# move components back to initial positions
		
if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
