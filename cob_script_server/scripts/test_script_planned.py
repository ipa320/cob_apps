#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

class TestScript(script):
		
	def Initialize(self):
		#self.sss.init("tray")
		#self.sss.init("torso")
		self.sss.init("arm")
		#self.sss.init("sdh")

	def Run(self): 
		# init poses
		handle01 = self.sss.move("arm","home",False)
		#self.sss.move("torso","home",False)
		#self.sss.move("sdh","home",False)
		#self.sss.move("tray","down")
		handle01.wait()
		#self.sss.move("base","home")

		#planned motion
		self.sss.move_planned("arm","pregrasp")

		self.sss.move_planned("arm","overtablet")


		
if __name__ == "__main__":
	SCRIPT = TestScript()
	SCRIPT.Start()
