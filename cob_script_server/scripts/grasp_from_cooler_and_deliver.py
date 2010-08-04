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
		self.sss.Init("tray")
		self.sss.Init("torso")
		self.sss.Init("arm")
		self.sss.Init("sdh")

	def Initialize(self):
		# Move all components to starting positions
		self.sss.Move("tray","down",False)
		self.sss.Move("sdh","home",False)
		self.sss.Move("torso","back",False)
		self.sss.Move("arm","folded")
		self.sss.SetLight("green")
		
		
	def run(self): 
		
		rospy.loginfo("Grasping water from cooler...")

		# start grasping, assuming that Care-O-bot stands right in front of the water cooler
		handle01 = self.sss.Move("arm","pregrasp",False)
		self.sss.Move("sdh","coolerbuttonup")
		handle01.wait()

		# lay finger on cooler button and get water
		self.sss.Move("arm","coolerbuttonup")
		self.sss.Move("sdh","coolerbuttondown")
		self.sss.sleep(5)
		self.sss.Move("sdh","coolerbuttonup")
		
		# move hand to cup and grasp it
		handle01 = self.sss.Move("sdh","cylclosed",False)
		self.sss.Move("arm","coolerpregrasp")
		handle01.wait()
		self.sss.Move("sdh","coolercupopen")
		self.sss.Move("arm","coolergrasp")
		self.sss.Move("sdh","coolercupclosed")

		# draw hand back and place cup on tablet
		self.sss.Move("arm","coolerpostgrasp")
		handle01 = self.sss.Move("arm","coolerpostgrasp-to-tablet",False)
		self.sss.sleep(2)
		self.sss.Move("torso","front",False)
		self.sss.Move("tray","up")
		handle01.wait()
		self.sss.Move("arm","tablet")
		self.sss.Move("sdh","cuprelease")

		# draw hand back and fold arm
		self.sss.Move("arm","tablet-to-folded")
		
		
if __name__ == "__main__":
	SCRIPT = GetDrink()
	SCRIPT.Initialize()
	SCRIPT.run()
