#!/usr/bin/python

import time
from random import *

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import ssscript

class RandomMoves(ssscript):

	def Initialize(self):
		print "init"
		#self.sss.init("sdh")
		#self.sss.init("tray")
		#self.sss.init("torso")
		#self.sss.init("arm")	

	def run(self): 
		seed()
		maxVal = 0.1
		print "start"
		#self.sss.move("sdh","home")
		#self.sss.move("torso","home")
		#self.sss.move("arm","folded")
		#self.sss.move("tray","up")
		for i in range(1,4):
			r1 = (random()-0.5)*2*maxVal;
			r2 = (random()-0.5)*2*maxVal;
			#self.sss.move("torso",[[0.5*r1,0.5*r2,r1,r2]])
			#self.sss.move("arm","pregrasp")
			self.sss.move_cart_rel("arm",[0.0, 0.0, 0.1], [0.0, 0.0, 0.0])
			#self.sss.move("sdh","cylopen")
			#self.sss.move("sdh","cylclosed")
			#self.sss.move("arm","folded")
			self.sss.sleep(1)
		self.sss.move("sdh","home")
		self.sss.move("torso","home")
		print "finished"
		
if __name__ == "__main__":
	SCRIPT = RandomMoves()
	SCRIPT.Start('random_moves_script')
