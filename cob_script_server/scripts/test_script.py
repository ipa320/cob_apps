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
		
	def run(self): 
		
		rospy.loginfo("Testing Sound modes...")
		for i in range(1):
			rospy.loginfo("Speaking with default mode")
			self.sss.Speak("sentence1")

			rospy.loginfo("Speaking Cepstral English")
			self.sss.Speak("sentence1","CEPS_EN")

			rospy.loginfo("Speaking Cepstral German")
			self.sss.Speak("sentence1","CEPS_DE")

			rospy.loginfo("Speaking WAV German")
			self.sss.Speak("sentence1","WAV_DE")

			rospy.loginfo("Speaking WAV English")
			self.sss.Speak("sentence1","WAV_EN")

			rospy.loginfo("Speaking Festival English")
			self.sss.Speak("sentence1","FEST_EN")

			rospy.loginfo("Setting sound to 'mute'")
			self.sss.Speak("sentence1","MUTE")

			rospy.loginfo("Selecting invalid mode")
			self.sss.Speak("sentence1","BLOBB")

			time.sleep(2)
			rospy.loginfo("\n\n")

		#for i in range(10):
		#	print "start"
		#	self.sss.Speak_Str("Festival Englisch","FEST_EN")
		#	time.sleep(0.5)
		#	self.sss.Speak_Str("Cepstral Deutsch","CEPS_DE")
		#	time.sleep(0.5)
		#	self.sss.Speak_Str("Cepstral Englisch","CEPS_EN")
		#	time.sleep(3)
		
		
		# init poses
#		self.sss.Move("base","pos1",False)
#		self.sss.Move("tray","down",False)
#		self.sss.Move("sdh","home",False)
#		self.sss.Move("arm","folded")

		#grasp
#		self.sss.Move("base","pos2",False)
#		handle01 = self.sss.Move("arm","pregrasp",False)
#		self.sss.Move("sdh","cylopen")
#		handle01.wait()
#		self.sss.Move("arm","grasp")
#		self.sss.Move("sdh","cylclosed")

		#place on tablet
#		self.sss.Move("base","pos3",False)
#		handle02 = self.sss.Move("arm","grasp-to-tablet",False)
#		self.sss.Move("tray","up",False)
#		handle02.wait()
#		print handle02.get_error_code()
#		self.sss.Move("sdh","cylopen")
		
		#move back to save poses
#		self.sss.Move("base","pos4",False)
#		handle03 = self.sss.Move("arm","tablet-to-folded",False)
#		self.sss.sleep(5.23)
#		self.sss.Move("sdh","home",False)
#		handle03.wait()
		
		print "finished"
		
if __name__ == "__main__":
	SCRIPT = GetDrink()
	#SCRIPT.Initialize()
	SCRIPT.run()
