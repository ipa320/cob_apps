#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import sys
import types
import string
import os
from simple_script_server import script


if __name__ == "__main__":
	if (len(sys.argv) <= 1 ):
		print "Error: wrong number of input arguments"
		print "usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]"
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
						model = s.Parse()
						#print "s = ",s
						print s.graph.string()
						s.graph.layout('dot')
						basename, extension = os.path.splitext(filename)
						if (level == 100):
							s.graph.draw(basename + ".png")
						else:
							s.graph.draw(basename + "_" + str(level) + ".png")
	except ImportError:
		print "Unable to import script file"
		print "usage: rosrun cob_script_server script_to_graph.py <<SCRIPTFILE>> [level]"

