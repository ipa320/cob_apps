#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

class MyScript(script):

	def Initialize(self):
		rospy.loginfo("Initializing all components...")

	def Run(self):
		handle01=self.sss.move("arm","home",False)
		self.sss.move("torso","home",False)
		self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle01.wait()

		self.sss.move("arm","pregrasp")
		self.sss.sleep(2)

		self.sss.move("tray","up")

		self.sss.move("sdh","cylopen")

		self.sss.move("arm","grasp")
		self.sss.sleep(2)

		self.sss.move("sdh","cylclosed")
		self.sss.sleep(1)

		self.sss.move("arm","overtablet")
		self.sss.sleep(3)

		self.sss.move("sdh","cylopen")

		self.sss.move("arm","tablet-to-folded")

if __name__ == "__main__":
	SCRIPT = MyScript()
	SCRIPT.Start()
