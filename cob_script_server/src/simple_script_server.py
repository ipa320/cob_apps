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
from sound_play.libsoundplay import SoundClient

# care-o-bot imports
from cob_msgs.msg import *
from cob_srvs.srv import *

# script server imports
import action_handle

# graph includes
import pygraphviz as pgv

## Script class from which all script inherit.
#
# Implements basic functionalities for all scripts.
class script():
	def __init__(self):
		self.graph=""
		self.graph_wait_list=[]
		self.function_counter = 0
		self.ah_counter = 0
		self.graph = pgv.AGraph()
		self.graph.node_attr['shape']='box'
		self.last_node = "Start"
	
		# use filename as nodename
		filename = os.path.basename(sys.argv[0])
		self.basename, extension = os.path.splitext(filename)
		rospy.init_node(self.basename)
		self.graph_pub = rospy.Publisher("/script_server/graph", String)

	# Sets the graph.
	def set_graph(self,graph):
		self.graph = graph

	# Gets the graph.
	def get_graph(self):
		return self.graph

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
		self.graph = self.sss.action_handle.get_graph()
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
		rospy.set_param("/script_server/graph", self.graph.string())
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
		self.ns_global_prefix = "/script_server"
		self.parse = parse
		
		# sound
		self.soundhandle = SoundClient()
		
		# light
		self.pub_light = rospy.Publisher('light_controller/command', Light)

		# base
		self.pub_base = rospy.Publisher('base_controller/command', Twist)
		rospy.sleep(1) # we have to wait here until publisher is ready, don't ask why

#------------------- Init section -------------------#
	## Initializes different components.
	#
	# Based on the component, the corresponding init service will be called.
	#
	# \param component_name Name of the component.
	def init(self,component_name,blocking=True):
		self.trigger(component_name,"init",blocking)

	## Stops different components.
	#
	# Based on the component, the corresponding stop service will be called.
	#
	# \param component_name Name of the component.
	def stop(self,component_name):
		self.trigger(component_name,"stop")

	## Recovers different components.
	#
	# Based on the component, the corresponding recover service will be called.
	#
	# \param component_name Name of the component.
	def recover(self,component_name):
		self.trigger(component_name,"recover")

	## Deals with all kind of trigger services for different components.
	#
	# Based on the component and service name, the corresponding trigger service will be called.
	#
	# \param component_name Name of the component.
	# \param service_name Name of the trigger service.
	# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
	def trigger(self,component_name,service_name,blocking=True):
		ah = action_handle.action_handle(service_name, component_name, "", blocking, self.parse)
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
	def move(self,component_name,parameter_name,blocking=True):
		if component_name == "base":
			return self.move_base(component_name,parameter_name,blocking)
		else:
			return self.move_traj(component_name,parameter_name,blocking)

	#todo: decide about success/failure return value
	def move_planned(self, component_name, parameter_name, blocking=True):
		if(self.parse):
			return -1		

		if component_name == "arm":
			rospy.loginfo("Move Arm Planned!")
			client = actionlib.SimpleActionClient('/move_arm', MoveArmAction)
			client.wait_for_server()

			joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]

			goal = MoveArmGoal()
			# Fill in the goal here
			goal.motion_plan_request.group_name = "arm"
			goal.motion_plan_request.num_planning_attempts = 1
			goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)

			goal.motion_plan_request.planner_id= ""
			#choose planner
			goal.planner_service_name = "ompl_planning/plan_kinematic_path"
			#goal.planner_service_name = "cob_prmce_planner/plan_kinematic_path"
			goal.motion_plan_request.goal_constraints.joint_constraints=[]
			
			for i in range(len(joint_names)):
				new_constraint = JointConstraint()
				new_constraint.joint_name = joint_names[i]
				new_constraint.position = 0.0
				new_constraint.tolerance_below = 0.1
				new_constraint.tolerance_above = 0.1
				goal.motion_plan_request.goal_constraints.joint_constraints.append(new_constraint)

			# get joint values from parameter server
			if type(parameter_name) is str:
				if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
					rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
			else:
				param = parameter_name
				rospy.loginfo("Getting joint values from parameter server failed")

			# check trajectory parameters
			if not type(param) is list: # check outer list
					rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
					print "parameter is:",param
			else:
				for i in param:
					#print i,"type1 = ", type(i)
					if not type(i) is list: # check inner list
						rospy.logerr("no valid parameter for %s: not a list of lists, aborting...",component_name)
						print "parameter is:",param
					else:
						if not len(i) == len(joint_names): # check dimension
							rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(i))
							print "parameter is:",param
						else:
							for j in i:
								#print j,"type2 = ", type(j)
								if not ((type(j) is float) or (type(j) is int)): # check type
									#print type(j)
									rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
									print "parameter is:",param
								else:
									rospy.logdebug("accepted parameter %f for %s",j,component_name)

			#fill into message
			#pregrasp hardcoded			
			#goal.motion_plan_request.goal_constraints.joint_constraints[0].position = 	-1.2986303567886353
			#goal.motion_plan_request.goal_constraints.joint_constraints[1].position = 	-1.9999999245021005
			#goal.motion_plan_request.goal_constraints.joint_constraints[2].position = 	-2.0263538360595703
			#goal.motion_plan_request.goal_constraints.joint_constraints[3].position = 	-1.3672049045562744
			#goal.motion_plan_request.goal_constraints.joint_constraints[4].position = 	0.88282591104507446
			#goal.motion_plan_request.goal_constraints.joint_constraints[5].position = 	1.0767384767532349
			#goal.motion_plan_request.goal_constraints.joint_constraints[6].position = 	-2.2612252235412598

			#no need for trajectories anymore, since planning (will) guarantee collision-free motion!
			traj_endpoint = param[len(param)-1]
			for k in range(len(traj_endpoint)):
				#print "traj_endpoint[%d]: %f", k, traj_endpoint[k]
				goal.motion_plan_request.goal_constraints.joint_constraints[k].position = traj_endpoint[k]

			#print "goal_position: "
			#print goal.motion_plan_request.goal_constraints.joint_constraints
	
			finished_within_time = False
			client.send_goal(goal)
			finished_within_time = client.wait_for_result(rospy.Duration(200.0))
			if finished_within_time:
				rospy.loginfo("Planned motion finished within time!")
			else:
				rospy.loginfo("This all takes too long...!")
		else:
			rospy.loginfo("Planned motion only available for component 'arm'! Aborting...")


	## Deals with movements of the base.
	#
	# A target will be sent to the actionlib interface of the move_base node.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_base(self,component_name,parameter_name,blocking):
		ah = action_handle.action_handle("move", component_name, parameter_name, blocking, self.parse)
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
		action_server_name = "/move_base"
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

	## Deals with direct movements of the base (without planning and collision checking).
	#
	# A target will be sent directly to the base_controller node.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_base_direct(self,component_name,parameter_name,blocking=False):
		ah = action_handle.action_handle("move_direct", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		param = parameter_name
		
		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.set_failed(3)
				return ah
		else:
			#print i,"type1 = ", type(i)
			DOF = 4
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

		# sending goal
		twist = Twist()
		twist.linear.x = param[0]
		twist.linear.y = param[1]
		twist.angular.z = param[2]
		self.pub_base.publish(twist)
		
		# drive for some time
		rospy.sleep(param[3])

		# stop base
		twist.linear.x = 0
		twist.linear.y = 0
		twist.angular.z = 0
		self.pub_base.publish(twist)
		
		ah.set_succeeded()
		return ah

	## Deals with all kind of trajectory movements for different components.
	#
	# A trajectory will be sent to the actionlib interface of the corresponding component.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_traj(self,component_name,parameter_name,blocking):
		ah = action_handle.action_handle("move", component_name, parameter_name, blocking, self.parse)
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
				point = point[0] # \todo hack because only first point is used, no support for trajectories inside trajectories
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
			point_msg.time_from_start=rospy.Duration(3*point_nr) # this value is set to 3 sec per point. \todo: read from parameter
			traj_msg.points.append(point_msg)

		# call action server
		operation_mode_name = "/" + component_name + '_controller/OperationMode'
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
		self.set_operation_mode(component_name,"position")
		
		# sending goal
		client_goal = JointTrajectoryGoal()
		client_goal.trajectory = traj_msg
		#print client_goal
		self.client.send_goal(client_goal)
		ah.set_client(self.client)

		ah.wait_inside()
		return ah

	def move_cart_rel(self, component_name, parameter_name=[[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]], blocking=True):
		ah = action_handle.action_handle("move_rel", component_name, parameter_name, blocking, self.parse)
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
	
	## Set the operation mode for different components.
	#
	# Based on the component, the corresponding set_operation_mode service will be called.
	#
	# \param component_name Name of the component.
	# \param mode Name of the operation mode to set.
	# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
	def set_operation_mode(self,component_name,mode,blocking=False):
		#rospy.loginfo("setting <<%s>> to operation mode <<%s>>",component_name, mode)
		rospy.set_param("/" + component_name + "_controller/OperationMode",mode) # \todo remove and only use service call
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
		ah = action_handle.action_handle("light", "", parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		rospy.loginfo("Set light to %s",parameter_name)
		
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
		ah = action_handle.action_handle("say", component_name, parameter_name, False, self.parse)
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
		if blocking:
			os.system("echo " + text + " | text2wave | aplay -q")
		else:
			self.soundhandle.say(text)
			#os.system("echo " + text + " | text2wave | aplay -q &")
		ah.set_succeeded()
		return ah

	## Play a sound file.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use
	def play(self,parameter_name,blocking=True):
		component_name = "sound"
		ah = action_handle.action_handle("play", component_name, parameter_name, False, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		language = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/language","en")
		wav_path = commands.getoutput("rospack find cob_script_server")
		filename = wav_path + "/common/files/" + language + "/" + parameter_name + ".wav"
		
		rospy.loginfo("Playing <<%s>>",filename)
		#self.soundhandle.playWave(filename)
		if blocking:
			os.system("aplay -q " + filename)
		else:
			os.system("aplay -q " + filename + "&")
		ah.set_succeeded()
		return ah

#------------------- General section -------------------#
	## Sleep for a certain time.
	#
	# \param duration Duration in seconds to sleep.
	#
	def sleep(self,duration):
		ah = action_handle.action_handle("sleep", "", str(duration), True, self.parse)
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
	# \todo implement waiting for timeout
	def wait_for_input(self,duration=0):
		ah = action_handle.action_handle("wait", "input", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		if (duration != 0):
			rospy.logerr("Wait with duration not implemented yet") # \todo implement waiting with duration
		
		rospy.loginfo("Wait for user input...")
		retVal = raw_input()
		rospy.loginfo("...got string <<%s>>",retVal)
		ah.set_succeeded()
		return retVal
