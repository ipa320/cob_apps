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
#   Live visualization of a scipt.
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

import gtk
import gtk.gdk
import roslib; roslib.load_manifest('cob_script_server')
import rospy
import xdot
from std_msgs.msg import String
from cob_msgs.msg import *
from cob_script_server.msg import *

import pygraphviz as pgv

gtk.gdk.threads_init()


# check, if graph is available on parameter server
if rospy.has_param('script_server/graph'):
	dotcode = rospy.get_param("script_server/graph")
	G=pgv.AGraph(dotcode)
else:
	G=pgv.AGraph()
	G.add_node('no graph available')
	dotcode = G.string()

## Graph callback.
def graph_cb(msg):
	print "new graph received"
	global dotcode
	global G
	dotcode = msg.data
	print dotcode
	G=pgv.AGraph(dotcode)
	
	# update vizualisation
	gtk.gdk.threads_enter()
	widget.set_dotcode(dotcode)
	widget.zoom_to_fit()
	gtk.gdk.threads_leave()
	
	
## State callback.
def state_cb(msg):
	global widget
	
	# modify active node
	active_node = msg.full_graph_name
	rospy.loginfo("Received state <<%s>> from node <<%s>>",str(msg.state),active_node)
	try:
		n=G.get_node(active_node)
	except:
		rospy.logwarn("Node <<%s>> not found in graph",active_node)
		return
	n.attr['style']='filled'
	if msg.state == ScriptState.UNKNOWN:
		n.attr['fillcolor']='white'
	elif msg.state == ScriptState.ACTIVE:
		n.attr['fillcolor']='yellow'
	elif msg.state == ScriptState.SUCCEEDED:
		n.attr['fillcolor']='green'
	elif msg.state == ScriptState.FAILED:
		n.attr['fillcolor']='red'
	elif msg.state == ScriptState.PAUSED:
		n.attr['fillcolor']='orange'
	else:
		n.attr['fillcolor']='blue'
	dotcode = G.string()

	# update vizualisation
	gtk.gdk.threads_enter()
	widget.set_dotcode(dotcode)
	widget.zoom_to_fit()
	gtk.gdk.threads_leave()
		

# create gtk window
window = gtk.Window()
window.set_title('script viewer')
window.set_default_size(600, 800)
vbox = gtk.VBox()
window.add(vbox)

widget = xdot.DotWidget()
widget.set_dotcode(dotcode)
widget.zoom_to_fit()


vbox.pack_start(widget)

window.show_all()

window.connect('destroy', gtk.main_quit)

rospy.init_node('script_viewer', anonymous=True)
rospy.Subscriber("/script_server/graph", String, graph_cb)
rospy.Subscriber("/script_server/state", ScriptState, state_cb)
gtk.main()
