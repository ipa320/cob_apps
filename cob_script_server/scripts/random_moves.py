#!/usr/bin/python

import time
from random import *

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

class RandomMoves(script):

	def Initialize(self):
		#self.sss.init("sdh")
		#self.sss.init("tray")
		self.sss.init("torso")
		#self.sss.init("arm")	

	def Run(self):
		seed()
		maxVal = 0.1
		print "start"
		self.sss.move("sdh","home",False)
		self.sss.move("torso","home",False)
		handle01 = self.sss.move("arm","folded",False)
		self.sss.move("tray","up")
		handle01.wait()
		for i in range(1,2):
			r1 = (random()-0.5)*2*maxVal;
			r2 = (random()-0.5)*2*maxVal;
			self.sss.move("torso",[[0.5*r1,0.5*r2,r1,r2]])
			#self.sss.move("arm","pregrasp")
			#self.sss.move_cart_rel("arm",[0.0, 0.0, 0.1], [0.0, 0.0, 0.0])
			#self.sss.move("sdh","cylopen")
			#self.sss.move("sdh","cylclosed")
			#self.sss.move("arm","folded")
			self.sss.sleep(1)
		self.sss.move("sdh","home")
		self.sss.wait_for_input()
		self.sss.move("torso","home",False)
		
if __name__ == "__main__":
	SCRIPT = RandomMoves()
	SCRIPT.Start()
