#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

import simple_script_server

class GetDrink:
	def __init__(self):
		rospy.init_node('test_script')
		self.sss = simple_script_server.simple_script_server()

	def Initialize(self):
		self.sss.Init("tray")
		self.sss.Init("torso")
		self.sss.Init("arm")
		self.sss.Init("sdh")
		
	def run(self): 
		
		print "start"
		self.sss.SpeakStr("Hallo","FEST_EN")
		# init poses
		handle01 = self.sss.Move("arm","folded",False)
		self.sss.Move("torso","home",False)
		self.sss.Move("sdh","home",False)
		self.sss.Move("tray","down")
		handle01.wait()
#		self.sss.Move("base","home")
#		self.sss.wait_for_input()

		#test
#		self.sss.Move("arm","home")
#		self.sss.MoveCartRel("arm", [0.0, 0.0, 0.0], [0.0, 0.0, 90.0/180.0*3.1415926])
#		self.sss.Speak("sentence1","WAV_DE")
#		self.sss.Speak("sentence1","FEST_EN")

		#grasp
#		self.sss.Move("base","kitchen")
		handle01 = self.sss.Move("arm","pregrasp",False)
		self.sss.Move("sdh","cylopen")
		handle01.wait()
		#self.sss.Move("arm","grasp")
		self.sss.MoveCartRel("arm", [-0.2, 0.0, 0.0], [0.0, 0.0, 0.0])
		self.sss.Move("sdh","cylclosed")

		#place on tablet
#		self.sss.wait_for_input()
		handle01 = self.sss.Move("arm","grasp-to-tablet",False)
		self.sss.Move("tray","up",False)
		handle01.wait()
#		print handle01.get_error_code()
		self.sss.Move("sdh","cylopen")
		
		#move back to save poses
		handle03 = self.sss.Move("arm","tablet-to-folded",False)
		self.sss.sleep(3)
		self.sss.Move("sdh","home",False)
		handle03.wait()
		
		#deliver
#		self.sss.Move("base","order")
		self.sss.Move("torso","nod")
		self.sss.sleep(3)
		
		#drive back to home
		self.sss.Move("tray","down",False)
#		self.sss.Move("base","home")
		
		print "finished"
		
if __name__ == "__main__":
	SCRIPT = GetDrink()
	#SCRIPT.Initialize()
	SCRIPT.run()
