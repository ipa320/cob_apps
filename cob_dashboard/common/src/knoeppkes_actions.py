#!/usr/bin/python

import roslib; roslib.load_manifest('cob3_dashboard')

from home import *
from folded import *
from simple_trajectory import *
from simple_trajectory2 import *
from torso_home import *
from torso_front import *
from torso_traj1 import *

class arm:
	def Home(self):
		home()

	def Folded(self):
		folded() 
	   
	def SimpleTrajectory(self):
		simple_trajectory()
		
	def SimpleTrajectory2(self):
		simple_trajectory2()

class torso:
	def Home(self):
		torso_home()
		print "torso.Home"
   
	def Front(self):
		torso_front()
		print "torso.Front"

	def Back(self):
		print "torso.Back"

	def Left(self):
		print "torso.Left"

	def Right(self):
		print "torso.Right"

	def Traj1(self):
		torso_traj1()
		print "torso.Traj1"

class sdh:
	def Home(self):
		print "sdh.Home"
   
	def Close(self):
		print "sdh.Close"
		
class cob3:
	arm=arm()
	torso=torso()
	sdh=sdh()
