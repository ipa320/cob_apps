#!/usr/bin/python

from actions import *
from parameters import *

torso=torso()
tray=tray()
arm=arm()
arm_pr2=arm_pr2()
sdh=sdh()

panels = [  
  ( "torso", [ 
	( "stop", torso.Stop, ()),
	( "init", torso.Init, ()),
	( "home", torso.MoveTraj, (torsoParameter.home,)),
	( "front", torso.MoveTraj, (torsoParameter.front,)),
	( "back", torso.MoveTraj, (torsoParameter.back,)),
	( "left", torso.MoveTraj, (torsoParameter.left,)),
	( "right", torso.MoveTraj, (torsoParameter.right,)),
	( "shake", torso.MoveTraj, (torsoParameter.shake,)),
	( "nod", torso.MoveTraj, (torsoParameter.nod,)),
	]),
  ( "tray", [ 
  	( "stop", tray.Stop, ()),
	( "init", tray.Init, ()),
	( "up", tray.MoveTraj, (trayParameter.up,)),
	( "down", tray.MoveTraj, (trayParameter.down,)),
	]),
  ( "arm", [ 
	( "stop", arm.Stop, ()),
	( "init", arm.Init, ()),
	( "home", arm.MoveTraj, (armParameter.home,)),
	( "folded", arm.MoveTraj, (armParameter.folded,)),
	( "moveArm3", arm.MoveArm3, ("dadsasasfd","asdf")),
	]),
#  ( "pr2_r_arm", [ 
#	( "Home", arm_pr2.MoveTraj, (armParameter_pr2.home,)),
#	( "Folded", arm_pr2.MoveTraj, (armParameter_pr2.folded,)),
#	]),
  ( "sdh", [ 
  	( "stop", sdh.Stop, ()),
	( "init", sdh.Init, ()),
	( "home", sdh.MoveCommand, (sdhParameter.home,)),
	( "cylClose", sdh.MoveCommand, (sdhParameter.cylClose,)),
	( "cylOpen", sdh.MoveCommand, (sdhParameter.cylOpen,)),
	( "spherClose", sdh.MoveCommand, (sdhParameter.spherClose,)),
	( "spherOpen", sdh.MoveCommand, (sdhParameter.spherOpen,)),
	])
  ]
