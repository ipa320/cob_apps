#!/usr/bin/python

from knoeppkes_actions import *

panels = [  
  ( "arm", [ 
	 ( "home", cob3.arm.Home),
	 ( "folded", cob3.arm.Folded),
	 ( "simple_trajectory", cob3.arm.SimpleTrajectory),
	 ( "simple_trajectory2", cob3.arm.SimpleTrajectory2),
	 ]),

  ( "sdh", [
	( "home",  cob3.sdh.Home),
	( "close",  cob3.sdh.Close),
   ]),

   ]
