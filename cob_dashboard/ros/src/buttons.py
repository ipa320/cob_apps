#!/usr/bin/python

from actions import *
from parameters import *

arm=arm()
arm_pr2=arm_pr2()
torso=torso()

panels = [  
  ( "arm", [ 
	 ( "Stop", arm.Stop, ()),
	 ( "Home", arm.MoveTraj, (armParameter.home,)),
	 ( "Folded", arm.MoveTraj, (armParameter.folded,)),
	 ( "moveArm3", arm.MoveArm3, ("dadsasasfd","asdf")),
	 ]),
  ( "pr2_r_arm", [ 
	 ( "Home", arm_pr2.MoveTraj, (armParameter_pr2.home,)),
	 ( "Folded", arm_pr2.MoveTraj, (armParameter_pr2.folded,)),
	 ]),
  ( "torso", [ 
	 ( "Home", torso.MoveTraj, (torsoParameter.home,)),
	 ( "Front", torso.MoveTraj, (torsoParameter.front,)),
	 ]),
   ]
