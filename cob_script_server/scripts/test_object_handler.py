#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import *
from script_utils import *

class Test_Object_Handler(script):
	def Run(self):
		if not self.sss.parse:
			rospy.loginfo("Testing Object_Handler modes...")
			
		#self.sss.add_object("milk_box")
		#self.sss.attach_object("milk_box")
		#self.sss.detach_object("milk_box")
		self.sss.remove_object("milk_box")

if __name__ == "__main__":
	SCRIPT = Test_Object_Handler()
	SCRIPT.Start()
