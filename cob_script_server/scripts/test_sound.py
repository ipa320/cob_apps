#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from script_utils import *

from sound_play.libsoundplay import SoundClient


class Test_Sound(script):
	def Run(self):
		rospy.loginfo("Testing Sound modes...")
		for i in range(1):
			self.sss.say(["Hello, my name is Care-O-bot."])
			self.sss.sleep(5)
			
			self.sss.say(["Hello, my name is Care-O-bot.","How are you?"])
			self.sss.sleep(5)
			
			self.sss.say("sent00")
			self.sss.sleep(5)
			
			self.sss.say("sent00","de")
			self.sss.sleep(5)

			self.sss.say(123)
			self.sss.say([123])
			
#			rospy.loginfo("Speaking with default mode")
#			self.sss.Speak("sent00")
#
#			rospy.loginfo("Speaking Cepstral English")
#			self.sss.Speak("sent01","CEPS_EN")
#
#			rospy.loginfo("Speaking Cepstral German")
#			self.sss.Speak("sent01","CEPS_DE")
#
#			rospy.loginfo("Speaking WAV German")
#			self.sss.Speak("grasp_tutorial_01","WAV_DE")
#
#			rospy.loginfo("Speaking WAV English")
#			self.sss.Speak("sent01","WAV_EN")
#
#			rospy.loginfo("Speaking Festival English")
#			self.sss.Speak("sent01","FEST_EN")
#
#			rospy.loginfo("Setting sound to 'mute'")
#			self.sss.Speak("sent01","MUTE")
#			self.sss.Speak("sent01","CEPS_EN")
#
#			rospy.loginfo("Selecting invalid mode")
#			self.sss.Speak("sent01","SPAM")

			time.sleep(2)
			rospy.loginfo("\n\n")

		print "finished"
		
if __name__ == "__main__":
	SCRIPT = Test_Sound()
	SCRIPT.Start('test_sound')
