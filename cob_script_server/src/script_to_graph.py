#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_apps
# \note
#   ROS package name: cob_script_server
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Generates a visual graph (*.png) out of a script.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import sys
import types
import string
import os
from simple_script_server import script

# graph includes
import pygraphviz as pgv


if __name__ == "__main__":
	if (len(sys.argv) <= 1 ):
		print "Error: wrong number of input arguments"
		print "usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]"
		# \todo What does "level" do?
		sys.exit(1)
	elif (len(sys.argv) == 2):
		filename = sys.argv[1]
		level = 100
	elif (len(sys.argv) == 3):
		filename = sys.argv[1]
		level = int(sys.argv[2])
	else:
		print "Error: to many arguments"
		print "usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]"
		sys.exit(1)
	
	print "Script file = ", filename
	print "Graph level = ", level
	rospy.set_param("/script_server/level",level)
	
	filename_splitted = string.split(filename, "/")
	#print filename_splitted
	scriptfile = filename_splitted[-1]
	if(len(filename_splitted) > 1):
		filename_splitted.pop(len(filename_splitted)-1)
		scriptdir = ""
		for name in filename_splitted:
			scriptdir += name + "/"
		#print scriptdir
		sys.path.insert(0,scriptdir)
	scriptfile_woext = string.split(scriptfile, ".")[0]
	#print scriptfile_woext

	try:
		__import__(scriptfile_woext, globals(), locals(), [], -1)
		scriptmodule = sys.modules[scriptfile_woext]
		for classname in dir(scriptmodule):
			subclass = scriptmodule.__getattribute__(classname)
			if(isinstance(subclass, types.ClassType)):
				if(issubclass(subclass, script)):
					if(classname != "script"):
						s = subclass()
						dotcode = s.Parse()
						print "dotcode = ",dotcode
						graph=pgv.AGraph(dotcode)
						graph.layout('dot')
						basename, extension = os.path.splitext(filename)
						if (level == 100):
							graph.draw(basename + ".png")
						else:
							graph.draw(basename + "_" + str(level) + ".png")
	except ImportError:
		print "Unable to import script file"
		print "usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]"

