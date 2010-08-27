#!/usr/bin/env python

import gtk
import gtk.gdk
import roslib; roslib.load_manifest('cob_script_server')
import rospy
import xdot
from std_msgs.msg import String

import pygraphviz as pgv

gtk.gdk.threads_init()



G=pgv.AGraph()
G.add_node('nothing received')
#G.add_edge('b','c')
#G.node_attr['shape']='square'
#G.node_attr['style']='filled'


dotcode = G.string()
#dotcode = """
#digraph G {
#  Hello [URL="http://en.wikipedia.org/wiki/Hello"]
#  World [URL="http://en.wikipedia.org/wiki/World"]
#    Hello -> World
#}
#"""

def roscb(msg):
	print "topic received"
	global widget
	dotcode = msg.data
	gtk.gdk.threads_enter()
	widget.set_dotcode(dotcode)
	widget.zoom_to_fit()
	gtk.gdk.threads_leave()
		

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

rospy.init_node('scriptviewer', anonymous=True)
rospy.Subscriber("/script_server/graph", String, roscb)
gtk.main()


