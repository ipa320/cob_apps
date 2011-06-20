#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script
from kinematics_msgs.srv import *

import tf
from tf.transformations import *
from geometry_msgs.msg import *

class GraspScript(script):

	def init_ik_interface(self):
		rospy.wait_for_service('/arm_controller/get_ik')
		rospy.wait_for_service('/arm_controller/get_fk_tcp')
		self.iks = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
		self.fks = rospy.ServiceProxy('/arm_controller/get_fk_tcp', GetPositionFK)


	def callIKSolver(self, current_pose, goal_pose):
		req = GetPositionIKRequest()
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped.pose = goal_pose
		resp = self.iks(req)
		return (resp.solution.joint_state.position, resp.error_code)

	def getEmptyPose(self):
		relpos = PoseStamped()
		relpos.pose.position.x = 0.0
		relpos.pose.position.y = 0.0
		relpos.pose.position.z = 0.0
		relpos.pose.orientation.x = 0.0
		relpos.pose.orientation.y = 0.0
		relpos.pose.orientation.z = 0.0
		relpos.pose.orientation.w = 1.0	
		return relpos	

	def Initialize(self):
		self.init_ik_interface()
		# initialize components (not needed for simulation)
		#self.sss.init("tray")
		#self.sss.init("torso")
		#self.sss.init("arm")
		#self.sss.init("sdh")
		#self.sss.init("base")

	def calcRelIK(self, start, goalpose, goalrotation = [0,0,0]):
		result = []
		print "Calling IK Server 1"
		q = quaternion_from_euler(goalrotation[0], goalrotation[1], goalrotation[2])
		
		req = GetPositionFKRequest()
		req.robot_state.joint_state.position = start
		resp = self.fks(req)
		abspos = resp.pose_stamped[0]
		abspos.pose.position.x = abspos.pose.position.x + goalpose[0]
		abspos.pose.position.y = abspos.pose.position.y + goalpose[1]
		abspos.pose.position.z = abspos.pose.position.z + goalpose[2]
		qrel = tf.transformations.quaternion_multiply([abspos.pose.orientation.x, abspos.pose.orientation.y, abspos.pose.orientation.z, abspos.pose.orientation.w], [q[0], q[1], q[2], q[3]])
		#print qrel
		abspos.pose.orientation.x = qrel[0]
		abspos.pose.orientation.y = qrel[1]
		abspos.pose.orientation.z = qrel[2]
		abspos.pose.orientation.w = qrel[3]
		(out, error_code) = self.callIKSolver(start, abspos.pose)
		for o in out:
			result.append(o)		
		return (result, error_code)

				
	def Run(self):
		if(not self.sss.parse): 
			self.listener = tf.TransformListener()
			time.sleep(0.5)
			startposition = [0.32801351164082243, -1.2319244977553321, -0.020446793812738041, 1.7740684566996034, 0.44454129082455984, 1.5863009631773928, 0.66069169492063817]

			#calculating ik's
			(grasp_empty_cup_position, error_code) = self.calcRelIK(startposition, [-0.1,0.0,0.0])		
			if(error_code.val == error_code.SUCCESS):
				print grasp_empty_cup_position
			else:
				print "Ik 1 Failed"
			(grasp_cup_out_of_cooler_position, error_code) = self.calcRelIK(grasp_empty_cup_position, [0.1,0.0,0,1])		
			if(error_code.val == error_code.SUCCESS):
				print grasp_empty_cup_position
			else:
				print "Ik 2 Failed"

			self.sss.move("arm", [startposition])
			self.sss.move("sdh", [[0.0,1.0472,0.0,0.0,1.0472,0.0,1.0472]])
			self.sss.move("arm", [grasp_empty_cup_position])
			self.sss.move("arm", [grasp_cup_out_of_cooler_position])
				
			#self.sss.move("arm", grasp_empty_cup_position
			#self.sss.move_cart_rel("arm",[[0.0, 0.0, 0.0], [0, 0, 0]])
		


		# prepare for grasping
		#self.sss.move("base","kitchen")
		#self.sss.move("base","rc_table_planning")
		#self.sss.move_planned("arm","pregrasp")
		#self.sss.move("sdh","cylopen")

		# caculate tranformations, we need cup coordinates in arm_7_link coordinate system
		#cup = PointStamped()
		#cup.header.stamp = rospy.Time.now()
		#cup.header.frame_id = "/map"
		#cup.point.x = -2.95
		#cup.point.y = 0.1
		#cup.point.z = 0.98
		#cup.point.x = 2.48
		#cup.point.y = 1.22
		#cup.point.z = 0.85
		#self.sss.sleep(2)
		
		#if not self.sss.parse:
		#	cup = listener.transformPoint('/arm_7_link',cup)

		#print "cup: ", cup		
		#self.sss.move_cart_rel("arm",[[cup.point.x, cup.point.y, cup.point.z-0.2], [0, 0, 0]])
		#self.sss.move_cart_rel("arm",[[0.0, 0.0, 0.2], [0, 0, 0]])
		#self.sss.move_planned("arm", "grasp")
		#self.sss.move("sdh","cup")cylclosed
		#self.sss.move("sdh","cylclosed")
	

		# place cup on tray
		#handle01 = self.sss.move("arm","grasp-to-tablet",False)
		#self.sss.move("tray","up")
		#handle01.wait()
		#self.sss.move_planned("arm","grasp-to-tablet")
		#self.sss.move("sdh","cylopen")
		#self.sss.move_cart_rel("arm",[[0.0, 0.0, -0.2], [0, 0, 0]])
		#handle01 = self.sss.move("arm","tablet-to-folded",False)
		#self.sss.move_planned("arm","tablet-to-folded")
		#self.sss.sleep(4)
		#self.sss.move("sdh","cylclosed",False)
		#handle01.wait()

		# deliver cup to home
		#self.sss.move("base","table")
		#self.sss.move("base","rc_entrance")
		#say("here's your drink")
		#self.sss.move("torso","nod")

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
