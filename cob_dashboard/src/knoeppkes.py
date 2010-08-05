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

from Tkinter import *
from buttons import *
import thread
import pygtk
pygtk.require('2.0')
import gtk
import roslib
import os 

def start(func, args):
  print "starting", func
#	func(*args)
  thread.start_new_thread(func,args)

def startGTK(widget, data):
  data()

class General(Frame):
  def __init__(self, master=None):
    width = 5
    self.charge = StringVar()
    self.charge.set("0")

    Frame.__init__(self, master, border=2, relief=GROOVE)
    label = Label(self, text="general", bg="#bfdfff")
    label.grid(sticky=W+E, padx=0, pady=0, columnspan=2)

    label = Label(self, text="state:")
    label.grid(sticky=W, padx=3, pady=3, row=1, column=0)
    charge = Entry(self, width=width, justify=RIGHT,
        textvariable=self.charge)
    charge.grid(sticky=W+E, padx=3, pady=3, row=1, column=1)
    
    button = Button(self, text="quit", command=quit)
    button.grid(sticky=W+E, padx=3, pady=3, row=10, column=0)

class Panel(Frame):
  def __init__(self, master=None, labeltext=""):
    Frame.__init__(self, master, border=2, relief=GROOVE)
    label = Label(self, text=labeltext, bg="#bfdfff")
    label.grid(sticky=W+E, padx=0, pady=0)

  def addButton(self, *args, **keys):
    button = Button(self, *args, **keys)
    button.grid(sticky=W+E, padx=3, pady=3)

class GtkGeneralPanel(gtk.Frame):
  def __init__(self):
    gtk.Frame.__init__(self)
    self.set_label("general")
    self.set_shadow_type(gtk.SHADOW_IN)
    self.vbox = gtk.VBox(False, 0)
    self.add(self.vbox)
    hbox=gtk.HBox(True, 0)
    image = gtk.Image()
    image.set_from_file(roslib.packages.get_pkg_dir("cob_dashboard") + "/share/icons/batti-040.png")
    hbox.pack_start(image, False, False, 0)
    label = gtk.Label("40 %")
    hbox.pack_start(label, False, False, 0)
    self.vbox.pack_start(hbox, False, False, 5)    
    hbox=gtk.HBox(True, 0)
    image = gtk.Image()
    image.set_from_file(roslib.packages.get_pkg_dir("cob_dashboard") + "/share/icons/weather-clear.png")
    hbox.pack_start(image, False, False, 0)
    label = gtk.Label("Status OK")
    hbox.pack_start(label, False, False, 0)
    self.vbox.pack_start(hbox, False, False, 5)    

    but = gtk.Button("Init all")
    #but.connect("clicked", lambda w: gtk.main_quit())
    self.vbox.pack_start(but, False, False, 5)    

    but = gtk.Button(stock=gtk.STOCK_QUIT	)
    but.connect("clicked", lambda w: gtk.main_quit())
    self.vbox.pack_start(but, False, False, 5)    

class GtkPanel(gtk.Frame):
  def __init__(self, master=None, labeltext=""):
    gtk.Frame.__init__(self)
    self.set_label(labeltext)
    self.set_shadow_type(gtk.SHADOW_IN)
    self.vbox = gtk.VBox(False, 0)
    self.add(self.vbox)

  def addButton(self, text, command):
    but = gtk.Button(text)  
    but.connect("clicked", startGTK, command)
    #but.set_size_request(120,-1)
    self.vbox.pack_start(but, False, False, 5)
        

class Knoeppkes(Frame):
  def delete_event(self, widget, event, data=None):
    gtk.main_quit()
    return False
  def __init__(self):
    # init ros node
    rospy.init_node('cob_dashboard')
  
    self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
    self.window.connect("delete_event", self.delete_event)
    self.window.set_title("cob_dashboard")	
    self.window.set_size_request(1000, 500)
    vbox = gtk.VBox(False, 1)
    self.hbox = gtk.HBox(True, 10)
    vbox.pack_start(self.hbox, True, True, 0)
    gpanel = GtkGeneralPanel()
    self.hbox.pack_start(gpanel,True, True, 3)
    for pname, actions in panels:
      panel = GtkPanel(self, pname)
      for aname, func, args in actions:
        panel.addButton(text=aname, command=lambda f=func, a=args: start(f, a))
      self.hbox.pack_start(panel,True, True, 3)
    
    self.status_bar = gtk.Statusbar()  
    context_id = self.status_bar.get_context_id("Statusbar")
    string = "Connected to $ROS_MASTER_URI=" + os.environ.get("ROS_MASTER_URI")    
    self.status_bar.push(context_id, string)
    vbox.pack_start(self.status_bar, False, False, 0)     
    self.window.add(vbox)    
    self.window.show_all()
    gtk.gdk.threads_init() 
    gtk.main()
    # init GUI
    #Frame.__init__(self)
    #self.grid()
    #self.master.title("cob_dashboard")
    #col = 0

    #panel = General(self)
    #panel.grid(row=0, column=col, padx=3, pady=3, sticky=N)
    #col += 1

    #for pname, actions in panels:
    #  panel = Panel(self, pname)
    #  for aname, func, args in actions:
    #    panel.addButton(text=aname, command=lambda f=func, a=args: start(f, a))
    #  panel.grid(row=0, column=col, padx=3, pady=3, sticky=N)
    #  col += 1

app = Knoeppkes()
#app.mainloop()
