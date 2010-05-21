#!/usr/bin/python

from actions import *
from parameters import *

torso=torso()
tray=tray()
arm=arm()
lbr=lbr()
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
#	( "moveArm3", arm.MoveArm3, ("dadsasasfd","asdf")),
	]),
  ( "lbr", [ 
  	( "stop", lbr.Stop, ()),
	( "init", lbr.Init, ()),
	( "home", lbr.MoveTraj, (lbrParameter.home,)),
	( "folded", lbr.MoveTraj, (lbrParameter.folded,)),
#	( "foldedTopregrasp", lbr.MoveTraj, (lbrParameter.foldedTopregrasp,)),
	( "pregrasp", lbr.MoveTraj, (lbrParameter.pregrasp,)),
	( "grasp", lbr.MoveTraj, (lbrParameter.grasp,)),
#	( "graspTOtablet", lbr.MoveTraj, (lbrParameter.graspTOtablet,)),
	( "overTablet", lbr.MoveTraj, (lbrParameter.overTablet,)),
	( "tablet", lbr.MoveTraj, (lbrParameter.tablet,)),
#	( "tabletTOfolded", lbr.MoveTraj, (lbrParameter.tabletTOfolded,)),
	( "coolerButton", lbr.MoveTraj, (lbrParameter.coolerButton,)),
	( "coolerPreGrasp", lbr.MoveTraj, (lbrParameter.coolerPreGrasp,)),
	( "coolerGrasp", lbr.MoveTraj, (lbrParameter.coolerGrasp,)),
	( "coolerPostGrasp", lbr.MoveTraj, (lbrParameter.coolerPostGrasp,)),
#	( "cupTOtablet", lbr.MoveTraj, (lbrParameter.cupTOtablet,)),
	]),
  ( "lbr traj", [ 
  	( "stop", lbr.Stop, ()),
  	( "init", lbr.Init, ()),
	( "foldedTopregrasp", lbr.MoveTraj, (lbrParameter.foldedTopregrasp,)),
	( "graspTOtablet", lbr.MoveTraj, (lbrParameter.graspTOtablet,)),
	( "cupTOtablet", lbr.MoveTraj, (lbrParameter.cupTOtablet,)),
	( "tabletTOfolded", lbr.MoveTraj, (lbrParameter.tabletTOfolded,)),
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
	( "trainObjects", sdh.MoveCommand, (sdhParameter.trainObjects,)),
	( "trainObjectsParallel", sdh.MoveCommand, (sdhParameter.trainObjectsParallel,)),
	( "cupHome", sdh.MoveCommand, (sdhParameter.cupHome,)),
	( "cupClose", sdh.MoveCommand, (sdhParameter.cupClose,)),
	( "cupOpen", sdh.MoveCommand, (sdhParameter.cupOpen,)),
	( "cupRelease", sdh.MoveCommand, (sdhParameter.cupRelease,)),
	( "coolerButtonUp", sdh.MoveCommand, (sdhParameter.coolerButtonUp,)),
	( "coolerButtonDown", sdh.MoveCommand, (sdhParameter.coolerButtonDown,)),
	( "coolerCupOpen", sdh.MoveCommand, (sdhParameter.coolerCupOpen,)),
	( "coolerCupClose", sdh.MoveCommand, (sdhParameter.coolerCupClose,)),
	]),
  ( "sdh sim", [ 
  	( "stop", sdh.Stop, ()),
	( "init", sdh.Init, ()),
	( "home", sdh.MoveTraj, (sdhTrajParameter.home,)),
	( "cylClose", sdh.MoveTraj, (sdhTrajParameter.cylClose,)),
	])
  ]
