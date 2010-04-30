#!/usr/bin/python

from actions import *
from parameters import *

arm=arm()

panels = [  
  ( "arm", [ 
	 ( "Stop", arm.Stop, ()),
	 ( "Home", arm.MoveTraj, (armParameter.home,)),
	 ( "Folded", arm.MoveTraj, (armParameter.folded,)),
	 ( "moveArm3", arm.MoveArm3, ("dadsasasfd","asdf")),
	 ]),

   ]
