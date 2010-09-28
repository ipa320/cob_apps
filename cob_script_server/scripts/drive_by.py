#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *

class GraspScript(script):
		
#	def Initialize(self):
		# initialize components (not needed for simulation)
		#self.sss.init("tray")
		#self.sss.init("torso")
		#self.sss.init("arm")
		#self.sss.init("sdh")
		#self.sss.init("base")
		
		# move to initial positions
		#handle01 = self.sss.move("arm","folded",False)
		#self.sss.move("torso","home",False)
		#self.sss.move("sdh","home",False)
		#self.sss.move("tray","down")
		#handle01.wait()
		#print "Please localize the robot with rviz"
		#self.sss.wait_for_input()
		#self.sss.move("base","home")
		
	def Run(self): 
		self.sss.move("base","table1")
		self.sss.move("base","table2")

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
