#!/usr/bin/python


import roslib
roslib.load_manifest('cob_script_server')
import rospy
import actionlib
from pr2_controllers_msgs.msg import *
from cob_srvs.srv import *
from cob_msgs.msg import *

from simple_script_server import *
import time

class script_server:
	def __init__(self):
		self.ns_global_prefix = "/script_server"
		self.sss = simple_script_server()
		self.move_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, MoveAction, self.execute)
		time.sleep(1)

#------------------- Actionlib section -------------------#		
	def execute(self, server_goal):
		server_result = MoveActionResult().result
		server_result.return_value = self.sss.Move(server_goal.component_name.data,server_goal.parameter_name.data)
		
		if server_result.return_value == 0:
			print "success"
			self.move_action_server.set_succeeded(server_result)
		else:
			print "error"
			self.move_action_server.set_aborted(server_result)

if __name__ == '__main__':
	rospy.init_node('script_server')
	script_server()
	rospy.loginfo("script_server is running")
	rospy.spin()
