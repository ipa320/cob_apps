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
		if not self.sss.parse:
			rospy.loginfo("Testing Sound modes...")
			rospy.loginfo("If you can't hear something, check soundcard number (#card) with:")
			rospy.loginfo("    cat /proc/asound/cards")
			rospy.loginfo("Initialize card with:")
			rospy.loginfo("    alsactrl init #card")
		for i in range(1):
			self.sss.say(["Hello, my name is Care-O-bot."])
			
			self.sss.say(["Hello, my name is Care-O-bot.","How are you?"])
			
			self.sss.say("sent00")
			
			self.sss.say("sent00","de")

			self.sss.say(123)
			self.sss.say([123])

			self.sss.say(["Hello, my name is Care-O-bot.","How are you?"],False)
			self.sss.sleep(1)
			self.sss.say(["This is a non blocking voice."])

			rospy.set_param("script_server/sound/language","de")
			self.sss.play("grasp_tutorial_01")
		
			rospy.set_param("script_server/sound/language","en")
			self.sss.play("grasp_tutorial_01")

if __name__ == "__main__":
	SCRIPT = Test_Sound()
	SCRIPT.Start()
