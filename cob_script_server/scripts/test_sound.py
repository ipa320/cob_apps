#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from script_utils import *


class Test_Sound(script):
	def Run(self):
		if not self.sss.parse:
			rospy.loginfo("Testing Sound modes...")
		for i in range(1):
			self.sss.say(["Hello, my name is Care-O-bot."])
			
			self.sss.say(["Hello, my name is Care-O-bot.","How are you?"])
			
			self.sss.say("sent00")
			
			self.sss.say("sent00","de")

			self.sss.say(123)
			self.sss.say([123])

			self.sss.say(['Once Upon A Time, there was a Shoemaker named Zerbo. He very carefully handcrafted every pair of shoes. Each pair of shoes was made especially for each person, and they were made to fit perfectly.'],False)

			self.sss.sleep(1)
			self.sss.say(["This is a non blocking voice which should be played in parallel to the previous text."])

			rospy.set_param("script_server/sound/language","de")
			self.sss.play("grasp_tutorial_01")
		
			rospy.set_param("script_server/sound/language","en")
			self.sss.play("grasp_tutorial_01")

if __name__ == "__main__":
	SCRIPT = Test_Sound()
	SCRIPT.Start()
