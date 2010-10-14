#!/usr/bin/python
# -*- coding: utf-8 -*-

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *
from gazebo_plugins.msg import *

oneTime = 0;

class GraspScript(script):
		
	def Initialize(self):
		# initialize components (not needed for simulation)
		#self.sss.init("tray")
		#self.sss.init("torso")
		#self.sss.init("arm")
		#self.sss.init("sdh")
		#self.sss.init("base")
		
		self.thumb_sub = rospy.Subscriber("/sdh_thumb_2_bumper/state",ContactsState,self.callback)
		
		# move to initial positions
		
		handle01 = self.sss.move("arm","folded",False)
		self.sss.move("torso","home",False)
		self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle01.wait()
		if not self.sss.parse:
			print "Please localize the robot with rviz"
		self.sss.wait_for_input()
		#self.sss.move("base","home")
		
	def Run(self): 
		listener = tf.TransformListener(True, rospy.Duration(10.0))
	
		handle01 = self.sss.move("base","table_cup_start")
	
		# prepare for grasping
		handle01 = self.sss.move("arm","pregrasp_tablecup",False)
		self.sss.move("sdh","table_cup_open")
		handle01.wait()
		
		self.sss.move("base","table_cup_end")
		
	def callback(self,data):
		global oneTime
		if oneTime==0:
			self.sss.move("sdh","cup")
			#self.sss.sleep(2.0)
			handle01 = self.sss.move("arm","overtablet",False)
			self.sss.move("tray","up")
			handle01.wait()
			self.sss.move("sdh","cylopen")
			self.sss.move_cart_rel("arm",[[0.0, 0.0, -0.2], [0, 0, 0]])
			handle01 = self.sss.move("arm","tablet-to-folded",False)
			self.sss.move("sdh","cylclosed",False)
			handle01.wait()
			oneTime = 1;

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
