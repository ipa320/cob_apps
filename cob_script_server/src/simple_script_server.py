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
#   Implements script server functionalities.
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

import time
import os
import sys
import types
import thread
import commands

# ROS imports
import roslib
roslib.load_manifest('cob_script_server')
import rospy
import actionlib

# msg imports
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from move_base_msgs.msg import *
from move_arm_msgs.msg import *
from motion_planning_msgs.msg import *
from tf.transformations import *
from std_msgs.msg import String

# care-o-bot includes
from cob_msgs.msg import *
from cob_light.msg import *
from cob_sound.msg import *
from cob_script_server.msg import *
from cob_srvs.srv import *

# graph includes
import pygraphviz as pgv

graph=""
graph_wait_list=[]
function_counter = 0
ah_counter = 0
graph = pgv.AGraph()
graph.node_attr['shape']='box'
last_node = "Start"

## Script class from which all script inherit.
#
# Implements basic functionalities for all scripts.
class script():
	def __init__(self):
		# use filename as nodename
		filename = os.path.basename(sys.argv[0])
		self.basename, extension = os.path.splitext(filename)
		rospy.init_node(self.basename)
		self.graph_pub = rospy.Publisher("/script_server/graph", String)

	## Dummy function for initialization
	def Initialize(self):
		pass

	## Dummy function for main run function
	def Run(self):
		pass

	## Function to start the script
	#
	# First does a simulated turn and then calls Initialize() and Run().
	def Start(self):
		self.Parse()
		global ah_counter
		ah_counter = 0
		self.sss = simple_script_server()
		rospy.loginfo("Starting <<%s>> script...",self.basename)
		self.Initialize()
		self.Run()
		# wait until last threaded action finishes
		rospy.loginfo("Wait for script to finish...")
		while ah_counter != 0:
			rospy.sleep(1)
		rospy.loginfo("...script finished.")
	
	## Function to generate graph view of script.
	#
	# Starts the script in simulation mode and calls Initialize() and Run().
	def Parse(self):
		rospy.loginfo("Start parsing...")
		global graph
		global function_counter
		function_counter = 0
		# run script in simulation mode
		self.sss = simple_script_server(parse=True)
		self.Initialize()
		self.Run()
		
		# save graph on parameter server for further processing
#		self.graph = graph
		rospy.set_param("/script_server/graph", graph.string())
		self.graph_pub.publish(graph.string())
		rospy.loginfo("...parsing finished")
		function_counter = 0
		return graph.string()

## Simple script server class.
#
# Implements the python interface for the script server.
class simple_script_server:
	## Initializes simple_script_server class.
	#
	# \param parse Defines wether to run script in simulation for graph generation or not
	def __init__(self, parse=False):
		global graph
		self.ns_global_prefix = "/script_server"
		self.wav_path = ""
		self.parse = parse
		
		# init light publisher
		self.pub_light = rospy.Publisher('light_controller/command', Light)

		rospy.sleep(1) # we have to wait here until publisher is ready, don't ask why

    #------------------- Init section -------------------#
	## Initializes different components.
	#
	# Based on the component, the corresponding init service will be called.
	#
	# \param component_name Name of the component.
	def init(self,component_name,blocking=True):
		return self.trigger(component_name,"init",blocking)

	## Stops different components.
	#
	# Based on the component, the corresponding stop service will be called.
	#
	# \param component_name Name of the component.
	def stop(self,component_name):
		return self.trigger(component_name,"stop")

	## Recovers different components.
	#
	# Based on the component, the corresponding recover service will be called.
	#
	# \param component_name Name of the component.
	def recover(self,component_name):
		return self.trigger(component_name,"recover")

	## Deals with all kind of trigger services for different components.
	#
	# Based on the component and service name, the corresponding trigger service will be called.
	#
	# \param component_name Name of the component.
	# \param service_name Name of the trigger service.
	# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
	def trigger(self,component_name,service_name,blocking=True, planning=False):
		ah = action_handle(service_name, component_name, "", blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		rospy.loginfo("<<%s>> <<%s>>", service_name, component_name)
		rospy.loginfo("Wait for <<%s>> to <<%s>>...", component_name, service_name)
		service_full_name = "/" + component_name + "_controller/" + service_name
		
		# check if service is available
		try:
			rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
		except rospy.ROSException, e:
			error_message = "%s"%e
			rospy.logerr("...<<%s>> service of <<%s>> not available, error: %s",service_name, component_name, error_message)
			ah.set_failed(4)
			return ah
		
		# check if service is callable
		try:
			init = rospy.ServiceProxy(service_full_name,Trigger)
			#print init()
			resp = init()
		except rospy.ServiceException, e:
			error_message = "%s"%e
			rospy.logerr("...calling <<%s>> service of <<%s>> not successfull, error: %s",service_name, component_name, error_message)
			ah.set_failed(10)
			return ah
		
		# evaluate sevice response
		if not resp.success.data:
			rospy.logerr("...<<%s>> <<%s>> not successfull, error: %s",service_name, component_name, resp.error_message.data) 
			ah.set_failed(10)
			return ah
		
		# full success
		rospy.loginfo("...<<%s>> is <<%s>>", component_name, service_name)
		ah.set_succeeded() # full success
		return ah

#------------------- Move section -------------------#
	## Deals with all kind of movements for different components.
	#
	# Based on the component, the corresponding move functions will be called.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move(self,component_name,parameter_name,blocking=True, mode=None):
		if component_name == "base":
			return self.move_base(component_name,parameter_name,blocking, mode)
		elif component_name == "arm" and mode=="planned":
			return self.move_planned(component_name,parameter_name,blocking)
		else:
			return self.move_traj(component_name,parameter_name,blocking)

	## Deals with movements of the base.
	#
	# A target will be sent to the actionlib interface of the move_base node.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_base(self,component_name,parameter_name,blocking, mode):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		else:
			#print i,"type1 = ", type(i)
			DOF = 3
			if not len(param) == DOF: # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,DOF,len(param))
				print "parameter is:",param
				ah.set_failed(3)
				return ah
			else:
				for i in param:
					#print i,"type2 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
						print "parameter is:",param
						ah.set_failed(3)
						return ah
					else:
						rospy.logdebug("accepted parameter %f for %s",i,component_name)

		# convert to pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "/map"
		pose.pose.position.x = param[0]
		pose.pose.position.y = param[1]
		pose.pose.position.z = 0.0
		q = quaternion_from_euler(0, 0, param[2])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		
		# call action server
		if(mode == None):
			action_server_name = "/move_base"
		elif(mode == "omni"):
			action_server_name = "/move_base"
		elif(mode == "diff"):
			action_server_name = "/move_base_diff"
		elif(mode == "linear"):
			action_server_name = "/move_base_linear"
		else:
			rospy.logerr("no valid navigation mode given for %s, aborting...",component_name)
			print "navigation mode is:",mode
			ah.set_failed(33)
			return ah
		
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = MoveBaseGoal()
		client_goal.target_pose = pose
		#print client_goal
		self.client.send_goal(client_goal)
		ah.set_client(self.client)

		ah.wait_inside()

		return ah

	## Deals with all kind of trajectory movements for different components.
	#
	# A trajectory will be sent to the actionlib interface of the corresponding component.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_traj(self,component_name,parameter_name,blocking):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		
		# get joint_names from parameter server
		param_string = self.ns_global_prefix + "/" + component_name + "/joint_names"
		if not rospy.has_param(param_string):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
				ah.set_failed(2)
				return ah
		joint_names = rospy.get_param(param_string)
		
		# check joint_names parameter
		if not type(joint_names) is list: # check list
				rospy.logerr("no valid joint_names for %s: not a list, aborting...",component_name)
				print "joint_names are:",joint_names
				ah.set_failed(3)
				return ah
		else:
			for i in joint_names:
				#print i,"type1 = ", type(i)
				if not type(i) is str: # check string
					rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",component_name)
					print "joint_names are:",param
					ah.set_failed(3)
					return ah
				else:
					rospy.logdebug("accepted joint_names for component %s",component_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name

		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

		traj = []

		for point in param:
			#print point,"type1 = ", type(point)
			if type(point) is str:
				if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + point):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + point)
					ah.set_failed(2)
					return ah
				point = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + point)
				point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
				#print point
			elif type(point) is list:
				rospy.logdebug("point is a list")
			else:
				rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			# here: point should be list of floats/ints
			#print point
			if not len(point) == len(joint_names): # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(point))
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			for value in point:
				#print value,"type2 = ", type(value)
				if not ((type(value) is float) or (type(value) is int)): # check type
					#print type(value)
					rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
					print "parameter is:",param
					ah.set_failed(3)
					return ah
			
				rospy.logdebug("accepted value %f for %s",value,component_name)
			traj.append(point)

		rospy.logdebug("accepted trajectory for %s",component_name)
		
		# convert to ROS trajectory message
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()+rospy.Duration(0.5)
		traj_msg.joint_names = joint_names
		point_nr = 0
		for point in traj:
			point_nr = point_nr + 1
			point_msg = JointTrajectoryPoint()
			point_msg.positions = point
			point_msg.time_from_start=rospy.Duration(3*point_nr) # this value is set to 3 sec per point. \todo TODO: read from parameter
			traj_msg.points.append(point_msg)

		# call action server
		action_server_name = "/" + component_name + '_controller/joint_trajectory_action'
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, JointTrajectoryAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)
		
		# set operation mode to position
		if not component_name == "arm":
			self.set_operation_mode(component_name,"position")
		
		# sending goal
		client_goal = JointTrajectoryGoal()
		client_goal.trajectory = traj_msg
		#print client_goal
		self.client.send_goal(client_goal)
		ah.set_client(self.client)

		ah.wait_inside()
		return ah
		
	def move_planned(self, component_name, parameter_name, blocking=True):
		ah = action_handle("move_planned", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move planned <<%s>> to <<%s>>",component_name,parameter_name)
		
		# get joint_names from parameter server
		param_string = self.ns_global_prefix + "/" + component_name + "/joint_names"
		if not rospy.has_param(param_string):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
				ah.set_failed(2)
				return ah
		joint_names = rospy.get_param(param_string)
		
		# check joint_names parameter
		if not type(joint_names) is list: # check list
				rospy.logerr("no valid joint_names for %s: not a list, aborting...",component_name)
				print "joint_names are:",joint_names
				ah.set_failed(3)
				return ah
		else:
			for i in joint_names:
				#print i,"type1 = ", type(i)
				if not type(i) is str: # check string
					rospy.logerr("no valid joint_names for %s: not a list of strings, aborting...",component_name)
					print "joint_names are:",param
					ah.set_failed(3)
					return ah
				else:
					rospy.logdebug("accepted joint_names for component %s",component_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name

		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

		traj = []

		for point in param:
			#print point,"type1 = ", type(point)
			if type(point) is str:
				if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + point):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + point)
					ah.set_failed(2)
					return ah
				point = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + point)
				point = point[0] # \todo TODO: hack because only first point is used, no support for trajectories inside trajectories
				#print point
			elif type(point) is list:
				rospy.logdebug("point is a list")
			else:
				rospy.logerr("no valid parameter for %s: not a list of lists or strings, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			# here: point should be list of floats/ints
			#print point
			if not len(point) == len(joint_names): # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(point))
				print "parameter is:",param
				ah.set_failed(3)
				return ah

			for value in point:
				#print value,"type2 = ", type(value)
				if not ((type(value) is float) or (type(value) is int)): # check type
					#print type(value)
					rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
					print "parameter is:",param
					ah.set_failed(3)
					return ah
			
				rospy.logdebug("accepted value %f for %s",value,component_name)
			traj.append(point)

		rospy.logdebug("accepted trajectory for %s",component_name)
		
		# convert to ROS Move arm message
		motion_plan = MotionPlanRequest()
		motion_plan.group_name = "arm"
		motion_plan.num_planning_attempts = 1
		motion_plan.allowed_planning_time = rospy.Duration(5.0)

		motion_plan.planner_id= ""
		motion_plan.goal_constraints.joint_constraints=[]
		
		for i in range(len(joint_names)):
			new_constraint = JointConstraint()
			new_constraint.joint_name = joint_names[i]
			new_constraint.position = 0.0
			new_constraint.tolerance_below = 0.4
			new_constraint.tolerance_above = 0.4
			motion_plan.goal_constraints.joint_constraints.append(new_constraint)
		#no need for trajectories anymore, since planning (will) guarantee collision-free motion!
		traj_endpoint = traj[len(traj)-1]
		for k in range(len(traj_endpoint)):
			#print "traj_endpoint[%d]: %f", k, traj_endpoint[k]
			motion_plan.goal_constraints.joint_constraints[k].position = traj_endpoint[k]


		# call action server
		action_server_name = "/move_arm"
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, MoveArmAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)
		
		# set operation mode to position
		#self.set_operation_mode(component_name,"position")
		
		# sending goal
		client_goal = MoveArmGoal()
		client_goal.planner_service_name = "ompl_planning/plan_kinematic_path"		#choose planner
		#client_goal.planner_service_name = "cob_prmce_planner/plan_kinematic_path"
		client_goal.motion_plan_request = motion_plan
		#print client_goal
		self.client.send_goal(client_goal)
		ah.set_client(self.client)

		ah.wait_inside()
		return ah

	def move_cart_rel(self, component_name, parameter_name=[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]], blocking=True):
		ah = action_handle("move_rel", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		param = parameter_name

		# convert to Pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "/arm_7_link"
		#pose.header.frame_id = "/sdh_palm_link"
		pose.pose.position.x = param[0][0]
		pose.pose.position.y = param[0][1]
		pose.pose.position.z = param[0][2]
		q = quaternion_from_euler(param[1][0], param[1][1], param[1][2])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		#print pose

		# call action server
		action_server_name = "/" + component_name + '_controller/move_cart_rel'
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, MoveCartAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = MoveCartGoal()
		client_goal.goal_pose = pose
		#print client_goal
		self.client.send_goal(client_goal)
		ah.set_client(self.client)

		ah.wait_inside()
		return ah
		
	## Move to a simple pose goal - planned
	# 
	# - recieves a cartesian pose (position, orientation) for 'arm_7_link' given in 'base_footprint' reference coordinate system
	# - sends the goal to move_arm
	# - move_arm will call the IK-solver
	# - if configuration is found, the specified planner is called to find a collision-free trajectory
	# - if successful the motion is executed
	#
	# ADD-ON: use another parameter to specify the reference frame and do transformation in cob_script_server::move_cart_planned
	##	
	def move_cart_planned(self, component_name, parameter_name=[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]], blocking=True):
#		if component_name != "arm":
#			print "ERROR: You can only use move_cart_planned for 'arm'"
#		else:
#			
		ah = action_handle("move_cart_planned", component_name, "cartesian", blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		# convert to ROS Move arm message
		motion_plan = MotionPlanRequest()
		motion_plan.group_name = "arm"
		motion_plan.num_planning_attempts = 1
		motion_plan.planner_id= ""
		motion_plan.allowed_planning_time = rospy.Duration(5.0)

		pose_constraint = SimplePoseConstraint()
		pose_constraint.header.stamp = rospy.Time.now()
		pose_constraint.header.frame_id = "base_footprint"
		pose_constraint.link_name = "arm_7_link"
		
		pose_constraint.absolute_position_tolerance.x = 0.1;
		pose_constraint.absolute_position_tolerance.y = 0.1;
		pose_constraint.absolute_position_tolerance.z = 0.1;

		pose_constraint.absolute_roll_tolerance = 0.1;
		pose_constraint.absolute_pitch_tolerance = 0.1;
		pose_constraint.absolute_yaw_tolerance = 0.1;
		
		# convert to Pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "base_footprint"
		pose.pose.position.x = parameter_name[0][0]
		pose.pose.position.y = parameter_name[0][1]
		pose.pose.position.z = parameter_name[0][2]
		q = quaternion_from_euler(parameter_name[1][0], parameter_name[1][1], parameter_name[1][2])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		pose_constraint.pose = pose.pose
		
		### can't use this in python
		###move_arm_msgs.addGoalConstraintToMoveArmGoal(pose_constraint, motion_plan)
		
		#convert to PositionConstraint
		position_constraint = PositionConstraint()
		position_constraint.header = pose_constraint.header
		position_constraint.link_name = pose_constraint.link_name
		position_constraint.position = pose_constraint.pose.position
		
		position_constraint.constraint_region_shape.type = 1	#geometric_shapes_msgs.Shape.Box
		position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
		position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
		position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)
		
		position_constraint.constraint_region_orientation.x = 0.0
		position_constraint.constraint_region_orientation.y = 0.0
		position_constraint.constraint_region_orientation.z = 0.0
		position_constraint.constraint_region_orientation.w = 1.0
		
		position_constraint.weight = 1.0
		
		
		#convert to OrientationConstraint
		orientation_constraint = OrientationConstraint()
		orientation_constraint.header = pose_constraint.header
		orientation_constraint.link_name = pose_constraint.link_name
		orientation_constraint.orientation = pose_constraint.pose.orientation
		#orientation_constraint.type = pose_constraint.orientation_constraint_type
		
		orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
		orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
		orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
		
		orientation_constraint.weight = 1.0
		
		#append()
		motion_plan.goal_constraints.joint_constraints = []
		motion_plan.goal_constraints.position_constraints = []
		motion_plan.goal_constraints.position_constraints.append(position_constraint)
		motion_plan.goal_constraints.orientation_constraints = []
		motion_plan.goal_constraints.orientation_constraints.append(orientation_constraint)	
		motion_plan.goal_constraints.visibility_constraints = []
		

		# call action server
		action_server_name = "/move_arm"
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, MoveArmAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)
		
		# sending goal
		client_goal = MoveArmGoal()
		client_goal.planner_service_name = "ompl_planning/plan_kinematic_path"		#choose planner
		#client_goal.planner_service_name = "cob_prmce_planner/plan_kinematic_path"
		client_goal.motion_plan_request = motion_plan
		client_goal.disable_ik = False
		#print client_goal
		self.client.send_goal(client_goal)
		ah.set_client(self.client)

		ah.wait_inside()
		return ah
		
	
	## Set the operation mode for different components.
	#
	# Based on the component, the corresponding set_operation_mode service will be called.
	#
	# \param component_name Name of the component.
	# \param mode Name of the operation mode to set.
	# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
	def set_operation_mode(self,component_name,mode,blocking=True, planning=False):
		#rospy.loginfo("setting <<%s>> to operation mode <<%s>>",component_name, mode)
		rospy.set_param("/" + component_name + "_controller/OperationMode",mode) # \todo TODO: remove and only use service call
		#rospy.wait_for_service("/" + component_name + "_controller/set_operation_mode")
		try:
			set_operation_mode = rospy.ServiceProxy("/" + component_name + "_controller/set_operation_mode", SetOperationMode)
			req = SetOperationModeRequest()
			req.operation_mode.data = mode
			#print req
			resp = set_operation_mode(req)
			#print resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
#------------------- LED section -------------------#
	## Set the color of the cob_light component.
	#
	# The color is given by a parameter on the parameter server.
	#
	# \param parameter_name Name of the parameter on the parameter server which holds the rgb values.
	def set_light(self,parameter_name,blocking=False):
		ah = action_handle("set", "light", parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		rospy.loginfo("Set light to <<%s>>",parameter_name)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/light/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/light/" + parameter_name)
				return 2
			param = rospy.get_param(self.ns_global_prefix + "/light/" + parameter_name)
		else:
			param = parameter_name
			
		# check color parameters
		if not type(param) is list: # check outer list
			rospy.logerr("no valid parameter for light: not a list, aborting...")
			print "parameter is:",param
			ah.error_code = 3
			return ah
		else:
			if not len(param) == 3: # check dimension
				rospy.logerr("no valid parameter for light: dimension should be 3 (r,g,b) and is %d, aborting...",len(param))
				print "parameter is:",param
				ah.error_code = 3
				return ah
			else:
				for i in param:
					#print i,"type1 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						#print type(i)
						rospy.logerr("no valid parameter for light: not a list of float or int, aborting...")
						print "parameter is:",param
						ah.error_code = 3
						return ah
					else:
						rospy.logdebug("accepted parameter %f for light",i)
		
		# convert to light message
		color = Light()
		color.header.stamp = rospy.Time.now()
		if type(parameter_name) is str:
			color.name.data = parameter_name
		else:
			color.name.data = "unspecified"
		color.r = param[0]
		color.g = param[1]
		color.b = param[2]

		# publish color		
		self.pub_light.publish(color)
		
		ah.set_succeeded()
		ah.error_code = 0
		return ah

#-------------------- Sound section --------------------#
	## Say some text.
	#
	# The text to say may be given by a list of strings or a single string which points to a parameter on the ROS parameter server.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use for the TTS system
	def say(self,parameter_name,blocking=True):
		component_name = "sound"
		ah = action_handle("say", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
			
		text = ""
		
		# get values from parameter server
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
				ah.set_failed(2)
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check parameters
		if not type(param) is list: # check list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		else:
			for i in param:
				#print i,"type1 = ", type(i)
				if not type(i) is str:
					rospy.logerr("no valid parameter for %s: not a list of strings, aborting...",component_name)
					print "parameter is:",param
					ah.set_failed(3)
					return ah
				else:
					text = text + i + " "
					rospy.logdebug("accepted parameter <<%s>> for <<%s>>",i,component_name)

		rospy.loginfo("Saying <<%s>>",text)
		
		# call action server
		action_server_name = "/sound_controller/say"
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, SayAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		client_goal = SayGoal()
		client_goal.text.data = text
		#print client_goal
		self.client.send_goal(client_goal)
		ah.set_client(self.client)

		ah.wait_inside()
		return ah

	## Play a sound file.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use
	def play(self,parameter_name,blocking=True):
		component_name = "sound"
		ah = action_handle("play", component_name, parameter_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		if self.wav_path == "":
			wav_path = commands.getoutput("rospack find cob_script_server")
		else:
			wav_path = self.wav_path
		filename = wav_path + "/common/files/" + language + "/" + parameter_name + ".wav"
		
		rospy.loginfo("Playing <<%s>>",filename)
		#self.soundhandle.playWave(filename)
		
		#\todo TODO: check if file exists
		# if filename exists:
		#	do ...
		# else 
		#	ah.set_fail(3)
		#	return ah
		
		if blocking:
			os.system("aplay -q " + filename)
		else:
			os.system("aplay -q " + filename + "&")
		ah.set_succeeded()
		return ah
		
	def set_wav_path(self,parameter_name,blocking=True):
		if type(parameter_name) is str:
			self.wav_path = parameter_name
		else:
			rospy.logerr("invalid wav_path parameter specified, aborting...")
			print "parameter is:", parameter_name
			ah.set_failed(2)
			return ah		
		
#-------------------- Object_Handler section --------------------#

	## Add an object to the environment_server.
	#
	# \param object_name name of the object
	def add_object(self,object_name,blocking=True):
		component_name = "object_handler"
		ah = action_handle("add", component_name, "add_" + object_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		try:
			adder = rospy.ServiceProxy("/object_handler/add_object", HandleObject)
			req = HandleObjectRequest()
			req.object.data = object_name
			#print req
			res = adder(req)
			#print resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			ah.set_failed(1)
			return ah
		
		ah.set_succeeded()
		return ah


	## Remove an object from the environment_server.
	#
	# \param object_name name of the object
	def remove_object(self,object_name,blocking=True):
		component_name = "object_handler"
		ah = action_handle("remove", component_name, "remove_" + object_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		try:
			remover = rospy.ServiceProxy("/object_handler/remove_object", HandleObject)
			req = HandleObjectRequest()
			req.object.data = object_name
			#print req
			res = remover(req)
			#print resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			ah.set_failed(1)
			return ah
		
		ah.set_succeeded()
		return ah


	## Attach a known object to the robot's SDH.
	#
	# \param object_name name of the object
	def attach_object(self,object_name,blocking=True):
		component_name = "object_handler"
		ah = action_handle("attach", component_name, "attach_" + object_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		try:
			attacher = rospy.ServiceProxy("/object_handler/attach_object", HandleObject)
			req = HandleObjectRequest()
			req.object.data = object_name
			#print req
			res = attacher(req)
			#print resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			ah.set_failed(1)
			return ah
		
		ah.set_succeeded()
		return ah


	## Detach an attached object from the robot's SDH.
	#
	# \param object_name name of the object
	def detach_object(self,object_name,blocking=True):
		component_name = "object_handler"
		ah = action_handle("attach", component_name, "detach_" + object_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		try:
			detacher = rospy.ServiceProxy("/object_handler/detach_object", HandleObject)
			req = HandleObjectRequest()
			req.object.data = object_name
			#print req
			res = detacher(req)
			#print resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			ah.set_failed(1)
			return ah
		
		ah.set_succeeded()
		return ah
				

#------------------- General section -------------------#
	## Sleep for a certain time.
	#
	# \param duration Duration in seconds to sleep.
	#
	def sleep(self,duration):
		ah = action_handle("sleep", "", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		rospy.loginfo("Wait for %f sec",duration)
		rospy.sleep(duration)
		
		ah.set_succeeded()

	## Waits for user input.
	#
	# Waits either for a user input or until timeout is reached.
	#
	# \param duration Duration in seconds for timeout.
	# 
	# \todo TODO: implement waiting for timeout
	def wait_for_input(self,duration=0):
		ah = action_handle("wait", "input", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		if (duration != 0):
			rospy.logerr("Wait with duration not implemented yet") # \todo TODO: implement waiting with duration
		
		rospy.loginfo("Wait for user input...")
		retVal = raw_input()
		rospy.loginfo("...got string <<%s>>",retVal)
		ah.set_succeeded()
		return retVal

#------------------- action_handle section -------------------#	
## Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class action_handle:
	## Initializes the action handle.
	def __init__(self, function_name, component_name, parameter_name, blocking, parse):
		global graph
		global function_counter
		self.parent_node = ""
		self.error_code = -1
		self.wait_log = False
		self.function_counter = function_counter
		self.function_name = function_name
		self.component_name = component_name
		self.parameter_name = parameter_name
		self.state = ScriptState.UNKNOWN
		self.blocking = blocking
		self.parse = parse
		self.level = int(rospy.get_param("/script_server/level",100))
		self.state_pub = rospy.Publisher("/script_server/state", ScriptState)
		self.AppendNode(blocking)

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
		return self.client.get_state()

	## Gets the error code of an action handle.
	def get_error_code(self):
		return self.error_code
	
	## Returns the graphstring.
	def GetGraphstring(self):
		if type(self.parameter_name) is types.StringType:
			graphstring = str(self.function_counter)+"_"+self.function_name+"_"+self.component_name+"_"+self.parameter_name
		else:
			graphstring = str(self.function_counter)+"_"+self.function_name+"_"+self.component_name
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
		global graph_wait_list
		global function_counter
		global last_node
		graphstring = self.GetGraphstring()
		if self.parse:
			if ( self.level >= self.GetLevel(self.function_name)):
				#print "adding " + graphstring + " to graph"
				graph.add_edge(last_node, graphstring)
				for waiter in graph_wait_list:
					graph.add_edge(waiter, graphstring)
				graph_wait_list=[]
				if blocking:
					last_node = graphstring
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
				graph_wait_list.append(self.parent_node)
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
					self.set_failed(10)
					return
			# check state of action server
			#print self.client.get_state()
			if self.client.get_state() != 3:
				if logging:
					rospy.logerr("...<<%s>> could not reach <<%s>>, aborting...",self.component_name, self.parameter_name)
				self.set_failed(11)
				return

			if logging:
				rospy.loginfo("...<<%s>> reached <<%s>>",self.component_name, self.parameter_name)
		else:
			rospy.logwarn("Execution of <<%s>> to <<%s>> was aborted, wait not possible. Continuing...",self.component_name, self.parameter_name)
			self.set_failed(self.error_code)
			return
			
		self.set_succeeded() # full success
