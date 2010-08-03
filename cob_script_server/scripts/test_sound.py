#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from script_utils import *

class Test_Sound:
	def __init__(self):
		rospy.init_node('test_sound')
		self.sss = simple_script_server()

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
			self.sss.Speak("sentence1","SPAM")

			time.sleep(2)
			rospy.loginfo("\n\n")

		print "finished"
		
if __name__ == "__main__":
	SCRIPT = Test_Sound()
	SCRIPT.run()
