#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

class TestScript(script):
		
	def Initialize(self):
		self.sss.init("arm")
		
	def Run(self): 
		###
		#the cartesian position needs to be with respect to /base_footprint
		#
		#ADD-ON: use another parameter to specify the reference frame and do transformation in cob_script_server::move_cart_planned
		###
		self.sss.move_cart("arm",["base_link",[1.2, 0.0, 1.0],[0, -1.5708, 0]])


		
if __name__ == "__main__":
	SCRIPT = TestScript()
	SCRIPT.Start()

