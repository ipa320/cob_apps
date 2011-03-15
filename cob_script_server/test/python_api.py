#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_script_server')

import sys
import unittest

import rospy
import rostest
from trajectory_msgs.msg import *
from simple_script_server import *
from pr2_controllers_msgs.msg import *
from cob_srvs.srv import *

class PythonAPITest(unittest.TestCase):
	def __init__(self, *args):
		super(PythonAPITest, self).__init__(*args)
		rospy.init_node('test_python_api_test')
		self.sss=simple_script_server()
		self.as_cb_executed = False
		self.ss_stop_cb_executed = False
		self.ns_global_prefix = "/script_server"

	def test_python_api(self):
		# get parameters
		try:
			# component
			if not rospy.has_param('~command'):
				self.fail('Parameter command does not exist on ROS Parameter Server')
			command = rospy.get_param('~command')
		except KeyError, e:
			self.fail('Parameters not set properly')		

		# choose command to test
		if command == "move":
			component_name = "arm"
			# init action server (as)
			as_name = "/" + component_name + "_controller/joint_trajectory_action"
			self.as_server = actionlib.SimpleActionServer(as_name, JointTrajectoryAction, execute_cb=self.as_cb)
			#execute test (as all components have the same trajectory interface, we only test for arm)
			self.move_test(component_name,"home") # test trajectories with a single point (home)
			self.move_test(component_name,"grasp-to-tray") # test trajectories with multiple points (grasp-to-tray)
			self.move_test(component_name,"wave") # test trajectories out of other points (wave)
		elif command == "stop":
			# prepare service server (ss)
			ss_name = "/arm_controller/" + command
			self.ss_cb_executed = False
			rospy.Service(ss_name, Trigger, self.ss_cb)
			# call sss function
			self.sss.stop("arm")
			# check result
			if not self.ss_cb_executed:
				self.fail('Service Server not called')
		elif command == "init":
			# prepare service server (ss)
			ss_name = "/arm_controller/" + command
			self.ss_cb_executed = False
			rospy.Service(ss_name, Trigger, self.ss_cb)
			# call sss function
			self.sss.init("arm")
			# check result
			if not self.ss_cb_executed:
				self.fail('Service Server not called')
		elif command == "recover":
			# prepare service server (ss)
			ss_name = "/arm_controller/" + command
			self.ss_cb_executed = False
			rospy.Service(ss_name, Trigger, self.ss_cb)
			# call sss function
			self.sss.recover("arm")
			# check result
			if not self.ss_cb_executed:
				self.fail('Service Server not called')
		else:
			self.fail("Command not known to test script")

	def move_test(self,component_name, parameter_name):
		self.as_cb_executed = False
		
		# call sss functions
		self.sss.move(component_name,parameter_name)		
		
		# check result
		if not self.as_cb_executed:
			self.fail('No goal received at action server')
		
		# get joint_names from parameter server
		param_string = self.ns_global_prefix + "/" + component_name + "/joint_names"
		if not rospy.has_param(param_string):
			error_msg = "parameter " + param_string +" does not exist on ROS Parameter Server"
			self.fail(error_msg)
		joint_names = rospy.get_param(param_string)
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			param_string = self.ns_global_prefix + "/" + component_name + "/" + parameter_name
			if not rospy.has_param(param_string):
				error_msg = "parameter " + param_string + " does not exist on ROS Parameter Server"
				self.fail(error_msg)
			param = rospy.get_param(param_string)
		else:
			param = parameter_name
			
		# check trajectory parameters
		if not type(param) is list: # check outer list
				error_msg = "no valid parameter for " + component_name + ": not a list, aborting..."
				self.fail(error_msg)
		
		traj = []
		
		for point in param:
			#print point,"type1 = ", type(point)
			if type(point) is str:
				if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + point):
					error_msg = "parameter " + self.ns_global_prefix + "/" + component_name + "/" + point + " does not exist on ROS Parameter Server, aborting..."
					self.fail(error_msg)
				point = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + point)
				point = point[0] # \todo hack because only first point is used, no support for trajectories inside trajectories
				#print point
			elif type(point) is list:
				rospy.logdebug("point is a list")
			else:
				error_msg = "no valid parameter for " + component_name + ": not a list of lists or strings, aborting..."
				self.fail(error_msg)
		
			# here: point should be list of floats/ints
			#print point
			if not len(point) == len(joint_names): # check dimension
				error_msg = "no valid parameter for " + component_name + ": dimension should be " + len(joint_names) + " and is " + len(point) + ", aborting..."
				self.fail(error_msg)

			for value in point:
				#print value,"type2 = ", type(value)
				if not ((type(value) is float) or (type(value) is int)): # check type
					#print type(value)
					error_msg = "no valid parameter for " + component_name + ": not a list of float or int, aborting..."
					self.fail(error_msg)
			traj.append(point)
		
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()+rospy.Duration(0.5)
		traj_msg.joint_names = joint_names
		point_nr = 0
		for point in traj:
			point_msg = JointTrajectoryPoint()
			point_msg.positions = point
			point_msg.time_from_start=rospy.Duration(3*point_nr) # this value is set to 3 sec per point. \todo: read from parameter
			traj_msg.points.append(point_msg)
		
		#print traj_msg
		#print self.traj
		
		# check amount of trajectory points
		if not (len(traj_msg.points) == len(self.traj.points)):
			self.fail('Not the same amount of points in trajectory')
		
		# check points
		#print traj_msg.points
		#print self.traj.points
		for i in range(len(traj_msg.points)):
			# check dimension of points
			#print traj_msg.points[i].positions
			#print self.traj.points[i].positions
			if not (len(traj_msg.points[i].positions) == len(self.traj.points[i].positions)):
				self.fail('Not the same amount of values in point')
			
			# check values of points
			for j in range(len(traj_msg.points[i].positions)):
				if not (traj_msg.points[i].positions[j] == self.traj.points[i].positions[j]):
					self.fail('Not the same values in point')
		
	def as_cb(self, goal):
		result = JointTrajectoryResult()
		#self.as_server.set_preempted(result)
		self.as_cb_executed = True
		self.traj = goal.trajectory
		print "action server callback"
		self.as_server.set_succeeded(result)
	
	def ss_cb(self,req):
		self.ss_cb_executed = True
		res = TriggerResponse()
		res.success.data = True
		return res

if __name__ == '__main__':
	try:
		rostest.run('rostest', 'test_python_apt_test', PythonAPITest, sys.argv)
	except KeyboardInterrupt, e:
		pass
	print "exiting"

