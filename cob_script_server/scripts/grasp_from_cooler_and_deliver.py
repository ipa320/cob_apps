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

	def initialize(self):
		# Move all components to starting positions
		self.sss.Move("tray","down",False)
		self.sss.Move("sdh","home",False)
		self.sss.Move("torso","back",False)
		self.sss.Move("arm","folded")
		self.sss.SetLight("green")
		
		
	def grasp_from_cooler(self): 
		rospy.loginfo("Grasping water from cooler...")

		# start grasping, assuming that Care-O-bot stands right in front of the water cooler
		handle01 = self.sss.Move("arm","pregrasp",False)
		self.sss.Move("sdh","coolerbuttonup")
		handle01.wait()

		# lay finger on cooler button and get water
		self.sss.Move("arm","coolerbutton")
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
		return 0
	
	def drive_to_cooler(self):
		rospy.loginfo("Driving to water cooler ...")

		# get watercooler position from parameters
		position_param = "script_sever/base/watercooler"
		if not rospy.has_param(position_param):
			rospy.lorerr("parameter %s does not exist on ROS Parameter Server, aborting...",position_param)
			return -2
		else:
			position = rospy.get_param(position_param)
			handle01 = self.sss.Move("torso","back",False)
			self.util.movePlatformiWait(position)
			handle01.wait()
		return 0

	def deliver_drink
		rospy.loginfo("Delivering drink ...")
		
		# drive to delivery position at the table
		position_param = "script_sever/base/table"
		if not rospy.has_param(position_param):
			rospy.lorerr("parameter %s does not exist on ROS Parameter Server, aborting...",position_param)
			return -2
		else:
			position = rospy.get_param(position_param)
			handle01 = self.sss.Move("torso","front",False)
			self.util.movePlatformiWait(position)
			handle01.wait()

		# offer drink
		handle01 = self.sss.Move("torso","left",False)
		### Speak
		handle01.wait()
		self.sss.Move("torso","right")
		self.sss.Move("torso","front")
		
		# check if drink has been taken and thank user
		taken_param = "/deliver_drink/drink_has_been_taken"
		if not rospy.has_param(taken_param):
			rospy.lorerr("parameter %s does not exist on ROS Parameter Server, aborting...",taken_param)
			return -2
		else:
			for timeout in range(0,10)
				if rospy.get_param(taken_param):
					handle01 = self.sss.Move("torso","bow",False)
					### Speak
					handle01.wait()
				self.sss.Sleep(0.5)
				
		# back away and fold tablet if possible
		position_param = "script_sever/base/backaway"
		if not rospy.has_param(position_param):
			rospy.lorerr("parameter %s does not exist on ROS Parameter Server, aborting...",position_param)
			return -2
		else:
			position = rospy.get_param(position_param)
			self.util.movePlatformiWait(position)
		if rospy.get_param(taken_param):
			self.sss.Move("tablet","down",False)
		return 0
		

if __name__ == "__main__":
	SCRIPT = GetDrink()
	SCRIPT.Initialize()
	SCRIPT.run()
