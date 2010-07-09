#!/usr/bin/python
#***************************************************************
#
# Copyright (c) 2010
#
# Fraunhofer Institute for Manufacturing Engineering	
# and Automation (IPA)
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Project name: care-o-bot
# ROS stack name: cob_apps
# ROS package name: cob_dashboard
#								
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#			
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# Date of creation: May 2010
# ToDo:
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fraunhofer Institute for Manufacturing 
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************

from actions import *
from parameters import *

base=base()
torso=torso()
tray=tray()
arm=arm()
lbr=lbr()
sdh=sdh()

panels = [  
  ( "base", [ 
#	( "stop", base.Stop, ()),
	( "init", base.Init, ()),
	]),
  ( "torso", [ 
	( "stop", torso.Stop, ()),
	( "init", torso.Init, ()),
	( "Mode: Joy", torso.SetOperationMode, ("velocity",)),
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
	( "Mode: Joy", tray.SetOperationMode, ("velocity",)),
	( "up", tray.MoveTraj, (trayParameter.up,)),
	( "down", tray.MoveTraj, (trayParameter.down,)),
	]),
  ( "arm", [ 
	( "stop", arm.Stop, ()),
	( "init", arm.Init, ()),
	( "Mode: Joy", arm.SetOperationMode, ("velocity",)),
	( "home", arm.MoveTraj, (armParameter.home,)),
	( "folded", arm.MoveTraj, (armParameter.folded,)),
	( "pregrasp", arm.MoveTraj, (armParameter.pregrasp,)),
	( "grasp", arm.MoveTraj, (armParameter.grasp,)),
	( "intermediateback", arm.MoveTraj, (armParameter.intermediateback,)),
	( "intermediatefront", arm.MoveTraj, (armParameter.intermediatefront,)),
	( "overtablet", arm.MoveTraj, (armParameter.overtablet,)),
	]),
  ( "arm traj", [ 
	( "stop", arm.Stop, ()),
	( "init", arm.Init, ()),
	( "Mode: Joy", arm.SetOperationMode, ("velocity",)),
	( "graspTOtablet", arm.MoveTraj, (armParameter.graspTOtablet,)),
	( "tabletTOfolded", arm.MoveTraj, (armParameter.tabletTOfolded,)),
	]),
  ( "lbr", [ 
	( "home", lbr.MoveTraj, (lbrParameter.home,)),
	( "folded", lbr.MoveTraj, (lbrParameter.folded,)),
	]),
#  ( "sdh", [ 
#  	( "stop", sdh.Stop, ()),
#	( "init", sdh.Init, ()),
#	( "home", sdh.MoveCommand, (sdhParameter.home,)),
#	( "cylClose", sdh.MoveCommand, (sdhParameter.cylClose,)),
#	( "cylOpen", sdh.MoveCommand, (sdhParameter.cylOpen,)),
#	( "spherClose", sdh.MoveCommand, (sdhParameter.spherClose,)),
#	( "spherOpen", sdh.MoveCommand, (sdhParameter.spherOpen,)),
#	( "trainObjects", sdh.MoveCommand, (sdhParameter.trainObjects,)),
#	( "trainObjectsParallel", sdh.MoveCommand, (sdhParameter.trainObjectsParallel,)),
#	( "cupHome", sdh.MoveCommand, (sdhParameter.cupHome,)),
#	( "cupClose", sdh.MoveCommand, (sdhParameter.cupClose,)),
#	( "cupOpen", sdh.MoveCommand, (sdhParameter.cupOpen,)),
#	( "cupRelease", sdh.MoveCommand, (sdhParameter.cupRelease,)),
#	( "coolerButtonUp", sdh.MoveCommand, (sdhParameter.coolerButtonUp,)),
#	( "coolerButtonDown", sdh.MoveCommand, (sdhParameter.coolerButtonDown,)),
#	( "coolerCupOpen", sdh.MoveCommand, (sdhParameter.coolerCupOpen,)),
#	( "coolerCupClose", sdh.MoveCommand, (sdhParameter.coolerCupClose,)),
#	]),
  ( "sdh sim", [ 
  	( "stop", sdh.Stop, ()),
	( "init", sdh.Init, ()),
	( "home", sdh.MoveTraj, (sdhTrajParameter.home,)),
	( "cylClose", sdh.MoveTraj, (sdhTrajParameter.cylClose,)),
	( "cylOpen", sdh.MoveTraj, (sdhTrajParameter.cylOpen,)),
	( "spherClose", sdh.MoveTraj, (sdhTrajParameter.spherClose,)),
	( "spherOpen", sdh.MoveTraj, (sdhTrajParameter.spherOpen,)),
	])
  ]
