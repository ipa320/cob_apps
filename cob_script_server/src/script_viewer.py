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
	print "active_node = ",active_node
	n=G.get_node(active_node)
	#print n
	n.attr['style']='filled'
	print msg.state
	if msg.state == ScriptState.UNKNOWN:
		n.attr['fillcolor']='white'
	elif msg.state == ScriptState.ACTIVE:
		n.attr['fillcolor']='yellow'
	elif msg.state == ScriptState.SUCCEEDED:
		print "jeeeahhh success"
		n.attr['fillcolor']='green'
	elif msg.state == ScriptState.FAILED:
		n.attr['fillcolor']='red'
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
rospy.Subscriber("/script_server/graph", String, graph_cb)
rospy.Subscriber("/script_server/state", ScriptState, state_cb)
gtk.main()
