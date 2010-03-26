#!/usr/bin/python

from knoeppkes_actions import *

panels = [  
  ( "arm", [ 
	 ( "home", cob3.arm.Home),
	 ( "folded", cob3.arm.Folded),
	 ( "simple_trajectory", cob3.arm.SimpleTrajectory),
	 ( "simple_trajectory2", cob3.arm.SimpleTrajectory2),
	 ]),

  ( "torso", [
	( "home",  cob3.torso.Home),
	( "front",  cob3.torso.Front),
	( "back",  cob3.torso.Back),
	( "left",  cob3.torso.Left),
	( "right",  cob3.torso.Right),
	( "traj1",  cob3.torso.Traj1),
   ]),

  ( "sdh", [
	( "home",  cob3.sdh.Home),
	( "close",  cob3.sdh.Close),
   ]),

   ]
