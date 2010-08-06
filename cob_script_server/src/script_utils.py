#!/usr/bin/python

import simple_script_server
import random
import rospy

class script_utils:
	def __init__(self):
		print "script_utils: init"
		self.sss = simple_script_server.simple_script_server()
		random.seed()
		
	def Home(self): # init poses
		self.sss.Move("tray","down",False)
		self.sss.Move("sdh","home",False)
		self.sss.Move("arm","folded")
		
	def MoveLED(self, component_name, parameter_name):
		self.sss.SetLight("yellow")
		return_value = self.sss.Move(component_name, parameter_name)
		self.sss.SetLight("green")
		return return_value

	def SpeakRandom(self,parameter_name,mode="DEFAULT"):
		# get phrase list from parameter server
		if not rospy.has_param("/script_server/sound/rand/" + parameter_name):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","/script_server/sound/rand/" + parameter_name)
			return 2
		phrase_list = rospy.get_param("/script_server/sound/rand/" + parameter_name)
		# choose one phrase randomly
		phrase = random.choice(phrase_list)
		# speak phrase
		return self.sss.Speak(phrase,mode)
