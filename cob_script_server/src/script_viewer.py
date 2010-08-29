#!/usr/bin/env python

import gtk
import gtk.gdk
import roslib; roslib.load_manifest('cob_script_server')
import rospy
import xdot
from std_msgs.msg import String
from cob_msgs.msg import *

import pygraphviz as pgv

gtk.gdk.threads_init()


# check, if graph is available on parameter server
if rospy.has_param('script_server/graph'):
	dotcode = rospy.get_param("script_server/graph")
	G=pgv.AGraph(dotcode)
else:
	G=pgv.AGraph()
	G.add_node('no grap available')
	dotcode = G.string()

def ros_cb(msg):
	print "topic received"
	global widget
	
	# modify active node
	active_node = msg.full_graph_name
	print active_node
	n=G.get_node(active_node)
	print n
	n.attr['style']='filled'
	n.attr['fillcolor']='green'
	dotcode = G.string()

	# update vizualisation
	gtk.gdk.threads_enter()
	widget.set_dotcode(dotcode)
	widget.zoom_to_fit()
	gtk.gdk.threads_leave()
		

# create gtk window
window = gtk.Window()
window.set_title('cob script server viewer')
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
#rospy.Subscriber("/script_server/graph", String, ros_cb)
rospy.Subscriber("/script_server/state", ScriptState, ros_cb)
gtk.main()
