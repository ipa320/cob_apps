#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from script_utils import *

class GetDrink:
	def __init__(self):
		rospy.init_node('test_script')
		self.sss = simple_script_server()

	def Initialize(self):
		self.sss.Init("tray")
		self.sss.Init("torso")
		self.sss.Init("arm")
		self.sss.Init("sdh")
		
	def run(self): # init poses
		
		
		self.sss.Move("tray","down",False)
		self.sss.Move("sdh","home",False)
		self.sss.Move("arm","folded")

		#grasp
		handle01 = self.sss.Move("arm","pregrasp",False)
		self.sss.Move("sdh","cylopen")
		handle01.wait()
		self.sss.Move("arm","grasp")
		self.sss.Move("sdh","cylclosed")

		#place on tablet
		handle02 = self.sss.Move("arm","grasp-to-tablet",False)
		self.sss.Move("tray","up",False)
		handle02.wait()
		print handle02.get_error_code()
		self.sss.Move("sdh","cylopen")
		
		#move back to save poses
		handle03 = self.sss.Move("arm","tablet-to-folded",False)
		self.sss.sleep(5.23)
		self.sss.Move("sdh","home",False)
		handle03.wait()
		
		print "finished"
		
if __name__ == "__main__":
	SCRIPT = GetDrink()
	#SCRIPT.Initialize()
	SCRIPT.run()
