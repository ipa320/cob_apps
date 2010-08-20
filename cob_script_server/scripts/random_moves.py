#!/usr/bin/python

import time
from random import *

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import ssscript

class RandomMoves(ssscript):

	def Initialize(self):
		self.sss.init("sdh")
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")	

	def run(self): 
		seed()
		maxVal = 0.1
		print "start"
		self.sss.move("torso","home")
		for i in range(1,9):
			r1 = (random()-0.5)*2*maxVal;
			r2 = (random()-0.5)*2*maxVal;
			r3 = (random()-0.5)*2*maxVal;
			r4 = (random()-0.5)*2*maxVal;
			self.sss.move("torso",[[r1,r2,r3,r4]])
			#self.sss.move("torso","asdf")
			self.sss.sleep(1)
		print "finished"
		
if __name__ == "__main__":
	SCRIPT = RandomMoves()
	SCRIPT.Start('random_moves_script')
