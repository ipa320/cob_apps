#!/usr/bin/python

import roslib; roslib.load_manifest('cob3_dashboard')

from home import *
from folded import *
from simple_trajectory import *
from simple_trajectory2 import *

class arm:
	def Home(self):
		home()

	def Folded(self):
		folded() 
	   
	def SimpleTrajectory(self):
		simple_trajectory()
		
	def SimpleTrajectory2(self):
		simple_trajectory2()

class sdh:
	def Home(self):
		print "sdh.Home"
   
	def Close(self):
		print "sdh.Close"
		
class cob3:
	arm=arm()
	sdh=sdh()
