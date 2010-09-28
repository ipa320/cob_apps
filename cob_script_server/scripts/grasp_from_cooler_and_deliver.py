#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

import simple_script_server
import script_utils
import sys
sys.path.insert(0,'/home/cob/svn/cob/Tests/GO/')
from ScriptUtils import *
from ScriptParameter import *
del sys.path[0]

class GraspFromCoolerAndDeliver(ssscript):

	def Initialize(self, servers):
		self.su = script_utils.script_utils()
		self.util = ScriptUtils(servers)
		self.sss.Init("tray")
		self.sss.Init("torso")
		self.sss.Init("arm")
		self.sss.Init("sdh")
		# Move all components to starting positions
		self.sss.Move("tray","down",False)
		self.sss.Move("sdh","home",False)
		self.sss.Move("torso","back",False)
		self.su.MoveLED("arm","folded")
		self.sss.SetLight("green")
		
		
	def GraspFromCooler(self): 
		rospy.loginfo("Grasping water from cooler...")

		# start grasping, assuming that Care-O-bot stands right in front of the water cooler
		handle01 = self.sss.Move("arm","pregrasp",False)
		self.su.MoveLED("sdh","coolerbuttonup")
		handle01.wait()

		# lay finger on cooler button and get water
		self.su.MoveLED("arm","coolerbutton")
		self.sss.Move("sdh","coolerbuttondown")
		self.sss.sleep(3)
		self.sss.Move("sdh","coolerbuttonup")
		
		# move hand to cup and grasp it
		handle01 = self.sss.Move("arm","coolerpregrasp",False)
		self.sss.sleep(0.5)
		self.sss.Move("sdh","cylclosed")
		handle01.wait()
		self.sss.Move("sdh","coolercupopen")
		self.su.MoveLED("arm","coolergrasp")
		self.sss.Move("sdh","coolercupclosed")

		# draw hand back and place cup on tablet
		self.su.MoveLED("arm","coolerpostgrasp")
		handle01 = self.sss.Move("arm","coolerpostgrasp-to-tablet",False)
		self.sss.sleep(2)
		self.sss.Move("torso","front",False)
		self.sss.Move("tray","up")
		handle01.wait()
		self.su.MoveLED("arm","tablet")
		self.sss.Move("sdh","cuprelease")

		# draw hand back and fold arm
		handle01 = self.sss.Move("arm","tablet-to-folded",False)
		self.sss.sleep(2)
		self.sss.Move("sdh","home")
		handle01.wait()
		return 0
	
	def DriveToCooler(self):
		rospy.loginfo("Driving to water cooler ...")

		# get watercooler position from parameters
		position_param = "script_server/base/watercooler_mm_deg"
		if not rospy.has_param(position_param):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",position_param)
			return -2
		else:
			position = rospy.get_param(position_param)
			handle01 = self.sss.Move("torso","back",False)
			self.util.movePlatformWait(*position)
			handle01.wait()
		return 0

	def DeliverDrink(self):
		rospy.loginfo("Delivering drink ...")
		
		# drive to delivery position at the table
		position_param = "script_server/base/table_mm_deg"
		if not rospy.has_param(position_param):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",position_param)
			return -2
		else:
			position = rospy.get_param(position_param)
			handle01 = self.sss.Move("torso","front",False)
			self.util.movePlatformWait(*position)
			handle01.wait()

		# offer drink
		handle01 = self.sss.Move("torso","left",False)
		self.su.SpeakRandom("offer")
		handle01.wait()
		self.sss.Move("torso","right")
		self.sss.Move("torso","front")
		
		# check if drink has been taken and thank user
		taken_param = self.sss.wait_for_input()
		if taken_param == "\n":
			handle01 = self.sss.Move("torso","bow",False)
			self.su.SpeakRandom("taken")
			handle01.wait()
			self.sss.sleep(0.5)
				
		# back away and fold tablet if possible
		position_param = "script_server/base/watercooler_mm_deg"
		if not rospy.has_param(position_param):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",position_param)
			return -2
		else:
			position = rospy.get_param(position_param)
			self.util.movePlatformWait(*position)
		if taken_param == "\n":
			self.sss.Move("tray","down",False)
		return 0
		
	def run(self):
		self.Initialize()
		self.DriveToCooler()
		self.GraspFromCooler()
		self.DeliverDrink()
		self.sss.Move("sdh","coolercupopen")
		self.su.MoveLED("arm","coolergrasp")
		self.sss.wait_for_input()
		self.sss.Move("sdh","coolercupclosed")
		self.su.MoveLED("arm","grasp-to-tablet")
		self.su.MoveLED("arm","tablet-to-folded")

if __name__ == "__main__":
	servers = Servers()
	SCRIPT = GraspFromCoolerAndDeliver(servers)
	SCRIPT.Start('grasp_deliver')
	#SCRIPT.Initialize()
	SCRIPT.run()
