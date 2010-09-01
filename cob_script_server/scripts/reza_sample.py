#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

class MyScript(script):

	#def Initialize(self):
		# empty

	def Run(self):
		self.sss.sleep(1)
		retVal = self.sss.wait_for_input()

if __name__ == "__main__":
	SCRIPT = MyScript()
	SCRIPT.Start()
