#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import actionlib
from cob_msgs.msg import *
from cob_srvs.srv import *
#from cob_actions.msg import *
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

class simple_script_server:
	def __init__(self):
		self.ns_global_prefix = "/script_server"

#------------------- Init section -------------------#
	def Init(self,component_name):
		rospy.loginfo("Waiting for %s_controller init...", component_name)
		service_name = component_name + "_controller/Init"
		try:
			rospy.wait_for_service(service_name,rospy.get_param('server_timeout',1))
		except rospy.ROSException, e:
			print "Service not available: %s"%e
			return False
		try:
			init = rospy.ServiceProxy(service_name,Trigger)
			print init()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return False
		return True

#------------------- Move section -------------------#
	def Move(self,component_name,parameter_name,blocking=True):
		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		ah = action_handle()
		ah.component_name = component_name
		ah.parameter_name = parameter_name
		
		# selecting component
		if component_name == "tray":
			joint_names = ["torso_tray_joint"]
		elif component_name == "torso":
			joint_names = ["torso_lower_neck_pan_joint","torso_lower_neck_tilt_joint","torso_upper_neck_pan_joint","torso_upper_neck_tilt_joint"]
		elif component_name == "arm":
			joint_names = ["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"]
		elif component_name == "sdh":
			joint_names = ["sdh_thumb_2_joint", "sdh_thumb_3_joint", "sdh_finger_11_joint", "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_21_joint", "sdh_finger_22_joint", "sdh_finger_23_joint"]
		else:
			rospy.logerr("component %s not kown to script_server",component_name)
			ah.error_code = 1
			return ah
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.error_code = 2
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.error_code = 3
				return ah
		else:
			for i in param:
				#print i,"type1 = ", type(i)
				if not type(i) is list: # check inner list
					rospy.logerr("no valid parameter for %s: not a list of lists, aborting...",component_name)
					print "parameter is:",param
					ah.error_code = 3
					return ah
				else:
					if not len(i) == len(joint_names): # check dimension
						rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(i))
						print "parameter is:",param
						ah.error_code = 3
						return ah
					else:
						for j in i:
							#print j,"type2 = ", type(j)
							if not ((type(j) is float) or (type(j) is int)): # check type
								print type(j)
								rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
								print "parameter is:",param
								ah.error_code = 3
								return ah
							else:
								rospy.logdebug("accepted parameter %f for %s",j,component_name)
		
		# convert to trajectory message
		traj = JointTrajectory()
		traj.header.stamp = rospy.Time.now()
		traj.joint_names = joint_names
		point_nr = 0
		for i in param:
			point_nr = point_nr + 1
			point = JointTrajectoryPoint()
			point.positions = i
			point.time_from_start=rospy.Duration(3*point_nr) # this value is set to 3 sec per point. TODO: read from parameter
			traj.points.append(point)
		
		# call action server
		operation_mode_name = "/" + component_name + '_controller/OperationMode'
		action_server_name = "/" + component_name + '_controller/joint_trajectory_action'
		rospy.set_param(operation_mode_name, "position")
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, JointTrajectoryAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.error_code = 4
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		self.check_pause()
		client_goal = JointTrajectoryGoal()
		client_goal.trajectory = traj
		#print client_goal
		self.client.send_goal(client_goal)
		ah.error_code = 0 # full success
		ah.client = self.client

		if blocking:
			ah.wait()
		else:
			rospy.logdebug("actionlib client not waiting for result, continuing...")
		
		return ah
			
#------------------- LED section -------------------#
	def SetLight(self,parameter_name):
		rospy.loginfo("Set light to %s",parameter_name)
		pub = rospy.Publisher('light_controller/command', Light)
		time.sleep(0.5) # we have to wait here until publisher is ready, don't ask why
		
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
				return 3
		else:
			if not len(param) == 3: # check dimension
				rospy.logerr("no valid parameter for light: dimension should be 3 (r,g,b) and is %d, aborting...",len(param))
				print "parameter is:",param
				return 3
			else:
				for i in param:
					#print i,"type1 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						print type(i)
						rospy.logerr("no valid parameter for light: not a list of float or int, aborting...")
						print "parameter is:",param
						return 3
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
		pub.publish(color)
		
		return 0 # full success

#------------------- General section -------------------#
	def sleep(self,duration):
		rospy.loginfo("Wait for %f sec",duration)
		time.sleep(duration)

	def check_pause(self):
		""" check if pause is globally set. If yes, enter a wait loop until
		the parameter is reset """
		pause_was_active = False

		while rospy.get_param("/script_server/pause"):
			if not pause_was_active:
				rospy.loginfo("ActionServer set to pause mode. Waiting for resume...")
				pause_was_active = True
			time.sleep(1)

		if pause_was_active:
			rospy.loginfo("Resuming...")
			return 1
		else:
			return 0

#------------------- action_handle section -------------------#	
class action_handle:
	def __init__(self):
		self.error_code = -1
		self.component_name = None
		self.parameter_name = None
	
	def wait(self,duration=None):
		if self.error_code == 0:			
			if duration is None:
				rospy.loginfo("Wait for <<%s>> reaching <<%s>>...",self.component_name, self.parameter_name)
				self.client.wait_for_result()
			else:
				rospy.loginfo("Wait for <<%s>> reached <<%s>> (max %f secs)...",self.component_name, self.parameter_name,duration)
				if not self.client.wait_for_result(rospy.Duration(duration)):
					rospy.logerr("Timeout while waiting for <<%s>> to reach <<%s>>. Continuing...",self.component_name, self.parameter_name)
					error_code = 10
					return
			rospy.loginfo("...<<%s>> reached <<%s>>",self.component_name, self.parameter_name)
		else:
			rospy.logwarn("Execution of action was aborted, wait not possible. Continuing...")
		return self.error_code
	
	def get_error_code(self):
		return self.error_code
			
if __name__ == '__main__':
	rospy.init_node('simple_script_server')
	simple_script_server()
	rospy.loginfo("simple_script_server is running")
	rospy.spin()
