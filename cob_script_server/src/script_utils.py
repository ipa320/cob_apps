#!/usr/bin/python

from simple_script_server import *

class script_utils:
	def __init__(self):
		print "script_utils: init"
		self.sss = simple_script_server()
		
	def home(self): # init poses
		self.sss.Move("tray","down",False)
		self.sss.Move("sdh","home",False)
		self.sss.Move("arm","folded")
		
	def MoveLED(self, component_name, parameter_name):
		self.SetLight("yellow")
		return_value = self.Move(component_name, parameter_name)
		self.SetLight("green")
		return return_value
