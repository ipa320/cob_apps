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
#   Implements action_handle functionalities.
#
#################################################################
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

# System imports
import types

# ROS imports
import roslib
roslib.load_manifest('cob_script_server')
import rospy

# care-o-bot imports
from cob_msgs.msg import *

# graph includes
import pygraphviz as pgv

graph=""
graph_wait_list=[]
function_counter = 0
ah_counter = 0
graph = pgv.AGraph()
graph.node_attr['shape']='box'
last_node = "Start"

#------------------- action_handle section -------------------#	
## Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class action_handle:
	## Initializes the action handle.
	def __init__(self, function_name, component_name, parameter_name, blocking, parse):
		self.parent_node = ""
		self.error_code = -1
		self.wait_log = False
		self.function_name = function_name
		self.component_name = component_name
		self.parameter_name = parameter_name
		self.state = ScriptState.UNKNOWN
		self.blocking = blocking
		self.parse = parse
		self.level = int(rospy.get_param("/script_server/level",100))
		self.state_pub = rospy.Publisher("/script_server/state", ScriptState)
		self.AppendNode(blocking)

	# Sets the graph.
	def set_graph(self,graph):
		rospy.set_param("/script_server/graph", graph.string())

	# Gets the graph.
	def get_graph(self):
		# check, if graph is available on parameter server
		if rospy.has_param('script_server/graph'):
			dotcode = rospy.get_param("script_server/graph")
			graph=pgv.AGraph(dotcode)
		else:
			graph=pgv.AGraph()
			graph.add_node('no graph available')
		return graph

	## Sets the actionlib client.
	def set_client(self,client):
		self.client = client

	## Sets the execution state to active, if not paused
	def set_active(self):
		self.check_pause()
		self.state = ScriptState.ACTIVE
		self.error_code = -1
		self.PublishState()
		
		global ah_counter
		ah_counter += 1
		
	## Checks for pause
	def check_pause(self):
		param_string = "/script_server/pause"
		while bool(rospy.get_param(param_string,False)):
			rospy.logwarn("Script is paused...")
			self.state = ScriptState.PAUSED
			self.PublishState()
			rospy.sleep(1)
		if self.state == ScriptState.PAUSED:
			rospy.loginfo("...continuing script")
		
	## Sets the execution state to succeeded.
	def set_succeeded(self):
		self.state = ScriptState.SUCCEEDED
		self.error_code = 0
		self.PublishState()
		
		global ah_counter
		ah_counter -= 1
		
	## Sets the execution state to failed.
	def set_failed(self,error_code):
		self.state = ScriptState.FAILED
		self.error_code = error_code
		self.PublishState()

		global ah_counter
		ah_counter -= 1
		
	## Gets the state of an action handle.
	def get_state(self):
		return self.state

	## Gets the error code of an action handle.
	def get_error_code(self):
		return self.error_code
	
	## Returns the graphstring.
	def GetGraphstring(self):
		global function_counter
		if type(self.parameter_name) is types.StringType:
			graphstring = str(function_counter)+"_"+self.function_name+"_"+self.component_name+"_"+self.parameter_name
		else:
			graphstring = str(function_counter)+"_"+self.function_name+"_"+self.component_name
		return graphstring

	## Gets level of function name.
	def GetLevel(self,function_name):
		if (function_name == "move"):
			level = 0
		elif (function_name == "init"):
			level = 1
		elif (function_name == "stop"):
			level = 1
		elif (function_name == "sleep"):
			level = 2
		else:
			level = 100
		return level
		
	## Appends a registered function to the graph.
	def AppendNode(self, blocking=True):
		global graph
		global last_node
		global graph_wait_list
		global function_counter
		graphstring = self.GetGraphstring()
		if self.parse:
			if ( self.level >= self.GetLevel(self.function_name)):
				#print "adding " + graphstring + " to graph"
				graph.add_edge(last_node, graphstring)
				for waiter in graph_wait_list:
					graph.add_edge(waiter, graphstring)
				self.graph_wait_list=[]
				if blocking:
					self.last_node = graphstring
				else:
					self.parent_node = graphstring
			#else:
				#print "not adding " + graphstring + " to graph"
		#else:
			#self.PublishState()
		function_counter += 1
		
	## Publishs the state of the action handle
	def PublishState(self):
		script_state = ScriptState()
		script_state.header.stamp = rospy.Time.now()
		script_state.number = self.function_counter
		script_state.function_name = self.function_name
		script_state.component_name = self.component_name
		script_state.full_graph_name = self.GetGraphstring()
		if ( type(self.parameter_name) is str ):
			script_state.parameter_name = self.parameter_name
		else:
			script_state.parameter_name = ""
		script_state.state = self.state
		script_state.error_code = self.error_code
		self.state_pub.publish(script_state)
		
	## Handles wait.
	#
	# This function is meant to be uses directly in the script.
	#
	# \param duration Duration for timeout.
	def wait(self, duration=None):
		global ah_counter
		ah_counter += 1
		self.blocking = True
		self.wait_for_finished(duration,True)

	## Handles inside wait.
	#
	# This function is meant to be uses inside the simple_script_server.
	#
	# \param duration Duration for timeout.
	def wait_inside(self, duration=None):
		if self.blocking:
			self.wait_for_finished(duration,True)
		else:
			thread.start_new_thread(self.wait_for_finished,(duration,False,))
		return self.error_code
	
	## Waits for the action to be finished.
	#
	# If duration is specified, waits until action is finished or timeout is reached.
	#
	# \param duration Duration for timeout.
	# \param logging Enables or disables logging for this wait.
	def wait_for_finished(self, duration, logging):
		global graph_wait_list
		if(self.parse):
			if(self.parent_node != ""):
				self.graph_wait_list.append(self.parent_node)
			return

		if self.error_code <= 0:			
			if duration is None:
				if logging:
					rospy.loginfo("Wait for <<%s>> reaching <<%s>>...",self.component_name, self.parameter_name)
				self.client.wait_for_result()
			else:
				if logging:
					rospy.loginfo("Wait for <<%s>> reached <<%s>> (max %f secs)...",self.component_name, self.parameter_name,duration)
				if not self.client.wait_for_result(rospy.Duration(duration)):
					if logging:
						rospy.logerr("Timeout while waiting for <<%s>> to reach <<%s>>. Continuing...",self.component_name, self.parameter_name)
					self.error_code = 10
					return
			if logging:
				rospy.loginfo("...<<%s>> reached <<%s>>",self.component_name, self.parameter_name)
		else:
			rospy.logwarn("Execution of <<%s>> to <<%s>> was aborted, wait not possible. Continuing...",self.component_name, self.parameter_name)
			self.set_failed(self.error_code)
			return
			
		self.set_succeeded() # full success
