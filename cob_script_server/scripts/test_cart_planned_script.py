#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

class TestScript(script):
		
	def Initialize(self):
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		self.sss.init("sdh")

		
	def Run(self): 
		# init poses
		handle01 = self.sss.move("arm","home",False)
		handle01.wait()
		
		###
		#the cartesian position needs to be with respect to /base_footprint
		#
		#ADD-ON: use another parameter to specify the reference frame and do transformation in cob_script_server::move_cart_planned
		###
		self.sss.move_cart_planned("arm",[[-0.180,-0.180, 1.808],[-0.212,-0.296,-0.708, 0.605]])	#wave_in (base_footprint)


		
if __name__ == "__main__":
	SCRIPT = TestScript()
	SCRIPT.Start()

