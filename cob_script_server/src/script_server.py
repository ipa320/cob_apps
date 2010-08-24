#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_apps
# \note
#   ROS package name: cob_script_server
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implementation of ROS node for script_server.
#
#################################################################
# <a href="todo.html">Todo list</a>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import actionlib

from cob_msgs.msg import *
from simple_script_server import script

## Script server class which inherits from script class.
#
# Implements actionlib interface for the script server.
#
class script_server(script):
	## Initializes the actionlib interface of the script server.
	#
	def __init__(self):
		self.ns_global_prefix = "/script_server"
		self.move_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, MoveAction, self.execute_cb)
		time.sleep(1)

#------------------- Actionlib section -------------------#
	## Executes actionlib callbacks.
	#
	# \param server_goal MoveActionGoal
	#
	def execute_cb(self, server_goal):
		server_result = MoveActionResult().result
		server_result.return_value = self.sss.move(server_goal.component_name.data,server_goal.parameter_name.data)
		
		if server_result.return_value == 0:
			print "success"
			self.move_action_server.set_succeeded(server_result)
		else:
			print "error"
			self.move_action_server.set_aborted(server_result)

## Main routine for running the script server
#
if __name__ == '__main__':
	rospy.init_node('script_server')
	script_server()
	rospy.loginfo("script_server is running")
	rospy.spin()
