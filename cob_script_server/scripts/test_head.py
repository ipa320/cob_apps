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
		
		self.sss.init("head")
		
		# move to initial positions
		handle01 = self.sss.move("head","home")
		handle01.wait()
		
	def Run(self): 
		handle01=self.sss.move("head","nod");
		handle01.wait()
		handle01=self.sss.move("head","shake");
		handle01.wait()

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
