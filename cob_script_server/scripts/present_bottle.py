#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy
from std_msgs.msg import *

import simple_script_server

class GetDrink:
	def __init__(self):
		rospy.init_node('test_script')
		self.sss = simple_script_server.simple_script_server()
		rospy.Subscriber("/phidgetAO", Int16, self.phidget_callback)
		self.phidget_data = 0

	def Initialize(self):
		self.sss.Init("tray")
		self.sss.Init("torso")
		self.sss.Init("arm")
		self.sss.Init("sdh")
		
		
	def phidget_callback(self,msg):
		self.phidget_data = msg.data
		
	def run(self): 
		
		print "start"
		
		usr_in = self.sss.wait_for_input()
		
		
		self.sss.SpeakStr("I will get the bottle for you.","FEST_EN")
		self.sss.Move("base","kitchen")

		arm_pre_g = self.sss.Move("arm","pregrasp",False)
		self.sss.SetLight("red")
		tray_up = self.sss.Move("tray","up",False)
		self.sss.Move("sdh","cylopen")
		arm_pre_g.wait()
		tray_up.wait()
		
		#self.sss.wait_for_input()
		
		self.sss.Move("arm","grasp")
		self.sss.Move("sdh","cylclosed")
		
		#self.sss.wait_for_input()
		
		self.sss.Move("arm","grasp-to-tablet")
		self.sss.Move("arm","tablet")
		self.sss.Move("sdh","cylopen")
		
		#self.sss.wait_for_input()
		
		arm_folded = self.sss.Move("arm","tablet-to-folded",False)
		self.sss.sleep(3)
		self.sss.Move("sdh","home")
		arm_folded.wait()
		
		#self.sss.wait_for_input()
		
#		self.sss.SpeakStr("Here is your drink!","FEST_EN")
		nod = self.sss.Move("torso","nod",False)
		self.sss.SetLight("green")
		nod.wait()
	
		#getting input from phidgetao
		#while (self.phidget_data > 100):
		#	self.sss.sleep(1)
		
		user_input = self.sss.wait_for_input()
		
		self.sss.Move("tray","down",False)
		self.sss.SpeakStr("Enjoy!","FEST_EN")
		
		self.sss.sleep(10)
		self.sss.Move("base","home")
		
		print "finished"
		
if __name__ == "__main__":
	SCRIPT = GetDrink()
	#SCRIPT.Initialize()
	SCRIPT.run()
