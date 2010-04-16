#!/usr/bin/python
import roslib; roslib.load_manifest('cob_tactiletools')
import rospy
import pygtk
pygtk.require('2.0')
import gtk, gobject, cairo
import threading
import time

#Initializing the gtk's thread engine
gtk.threads_init()

class Screen(gtk.DrawingArea):

	# Draw in response to an expose-event
	__gsignals__ = { "expose-event": "override" }
	sizex = 3
	sizey = 5

	# Handle the expose-event by drawing
	def do_expose_event(self, event):
		# Create the cairo context
		cr = self.window.cairo_create()
		print "Width: ", self.allocation.width
		print "Height: ", self.allocation.height
		# Restrict Cairo to the exposed area; avoid extra work
		cr.rectangle(event.area.x, event.area.y, event.area.width, event.area.height)
		cr.clip()
		self.draw(cr, self.allocation.width,self.allocation.height )

	def draw(self, cr, width, height):
		# Fill the background with gray
		color = 0.5
		xw = height/(self.sizex+1)
		print height, ", ", self.sizex
		yw = width/(self.sizey+1)
		print "Divider: ", xw, ", ", yw
		for i in range(0,self.sizey):
			for j in range(0,self.sizex):
				if(color==0.0):
					color=1.0
				else:
					color=0.0
				cr.set_source_rgb(color, 1-color, 0.5)
				print (j)*yw,":", (i)*xw,":", (j+1)*yw,":", (i+1) * xw
				cr.rectangle((j)*xw, (i)*yw, xw, yw)
				cr.fill()
	def updateTactileMatrix(self, string):
		print "Got something: ", string
		self.queue_draw()


class DataGet(threading.Thread):
	stopthread = threading.Event()	
	def run(self):
		global sc1
		while not self.stopthread.isSet() :
			gtk.threads_enter()
			sc1.updateTactileMatrix("test")
			gtk.threads_leave()
			time.sleep(0.5)
	def stop(self):
		"""Stop method, sets the event to terminate the thread's main loop"""
		self.stopthread.set()

# GTK mumbo-jumbo to show the widget in a window and quit when it's closed
def main_quit(obh, obb):
	global dget
	#Stopping the thread and the gtk's main loop
	dget.stop()
	gtk.main_quit()

try:
	window = gtk.Window()
	winwidth = 400
	winheight = 400
	window.set_size_request(winwidth,winheight)
	window.set_title("TactileSensorData")
	sc1 = Screen()
	sc1.set_size_request((winwidth/3)-10, (winheight/2)-5)
	sc2 = Screen()
	sc2.set_size_request((winwidth/3)-10, (winheight/2)-5)
	sc3 = Screen()
	sc3.set_size_request((winwidth/3)-10, (winheight/2)-5)
	button4 = gtk.Button("box.pack4")
	button5 = gtk.Button("box.pack5")
	button6 = gtk.Button("box.pack6")
	hbox1=gtk.HBox(False ,10 )
	hbox1.pack_start(sc1,False, True, 0)
	hbox1.pack_start(sc2,False, True, 0)
	hbox1.pack_start(sc3,False, True, 0)
	hbox2=gtk.HBox(True,0)
	hbox2.pack_start(button4,False, False, 0)
	hbox2.pack_start(button5,False, False, 0)
	hbox2.pack_start(button6,False, False, 0)
	vbox=gtk.VBox(True,0)
	vbox.pack_start(hbox1,False, False, 0)
	vbox.pack_start(hbox2,False, False, 0)
	window.add(vbox)
	window.show_all()
	window.connect("delete-event", main_quit)
	window.present()
	dget = DataGet()
	dget.start()

	gtk.main()
except KeyboardInterrupt:
    main_quit("tut", "tut")
