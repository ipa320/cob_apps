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
#   ROS package name: cob_dashboard
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implementation of ROS node for dashboard.
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
roslib.load_manifest('cob_dashboard')
from cob_relayboard.msg import EmergencyStopState
from buttons import *
import thread
import pygtk
pygtk.require('2.0')
import gtk
import roslib
import os
import pynotify
import sys 

planning_enabled = False
base_diff_enabled = False

#Initializing the gtk's thread engine
gtk.gdk.threads_init()

## Executes a button click in a new thread
def start(func, args):
  global planning_enabled
  global base_diff_enabled
  largs = list(args)
  if(largs[0] == "arm"):
    if(planning_enabled):
      largs.append("planned")
  if(largs[0] == "base"):
    if(base_diff_enabled):
  	largs.append("diff")	
  #print "Args", tuple(largs)
  thread.start_new_thread(func,tuple(largs))

def startGTK(widget, data):
  data()

## Class for general gtk panel implementation
class GtkGeneralPanel(gtk.Frame):
  def __init__(self):
    gtk.Frame.__init__(self)
    if not pynotify.init ("cob_dashboard"):
      sys.exit (1)
    self.em_stop = False
    self.set_label("general")
    self.set_shadow_type(gtk.SHADOW_IN)
    self.vbox = gtk.VBox(False, 0)
    self.add(self.vbox)
    #hbox=gtk.HBox(True, 0)
    #image = gtk.Image()
    #image.set_from_file(roslib.packages.get_pkg_dir("cob_dashboard") + "/common/files/icons/batti-040.png")
    #hbox.pack_start(image, False, False, 0)
    #label = gtk.Label("40 %")
    #hbox.pack_start(label, False, False, 0)
    #self.vbox.pack_start(hbox, False, False, 5)    
    hbox=gtk.HBox(True, 0)
    self.status_image = gtk.Image()
    self.status_image.set_from_file(roslib.packages.get_pkg_dir("cob_dashboard") + "/common/files/icons/weather-clear.png")
    hbox.pack_start(self.status_image, False, False, 0)
    self.status_label = gtk.Label("Status OK")
    hbox.pack_start(self.status_label, False, False, 0)
    self.vbox.pack_start(hbox, False, False, 5)    

    #but = gtk.Button("Init all")
    #but.connect("clicked", lambda w: gtk.main_quit())
    #self.vbox.pack_start(but, False, False, 5)
    
    plan_check = gtk.CheckButton("Planning")#
    plan_check.connect("toggled", self.planned_toggle)
    self.vbox.pack_start(plan_check, False, False, 5)

    base_mode_check = gtk.CheckButton("Base Diff")
    base_mode_check.connect("toggled", self.base_mode_toggle)
    self.vbox.pack_start(base_mode_check, False, False, 5)
    
    but = gtk.Button(stock=gtk.STOCK_QUIT	)
    but.connect("clicked", lambda w: gtk.main_quit())
    self.vbox.pack_start(but, False, False, 5)

  def setEMStop(self, em):
    if(em):
      #print "Emergency Stop Active"
      self.status_image.set_from_file(roslib.packages.get_pkg_dir("cob_dashboard") + "/common/files/icons/weather-storm.png")
      self.status_label.set_text("EM Stop !")
      if(self.em_stop == False):
        self.em_stop = True
        n = pynotify.Notification("Emergency Stop issued!", "", "dialog-warning")
        n.set_timeout(1)
        n.show()
    else:
      #print "Status OK"
      #self.status_image.set_from_file(roslib.packages.get_pkg_dir("cob_dashboard") + "/common/files/icons/weather-clear.png")
      self.status_label.set_text("Status OK")
      if(self.em_stop == True):
        self.em_stop = False
        n = pynotify.Notification("Emergency Stop released!", "", "dialog-ok")
        n.set_timeout(1)
        n.show()
        
  def planned_toggle(self, b):
    global planning_enabled
    if(planning_enabled):
      planning_enabled = False
    else:
      planning_enabled = True  

  def base_mode_toggle(self, b):
    global base_diff_enabled
    if(base_diff_enabled):
      base_diff_enabled = False
    else:
      base_diff_enabled = True       
		    

## Class for gtk panel implementation
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
        
## Implementation of knoeppkes dashboard
class Knoeppkes():
  def delete_event(self, widget, event, data=None):
    gtk.main_quit()
    return False

  def emcb(self, msg):
    self.gpanel.setEMStop(msg.emergency_state)
    
  def __init__(self):
    # init ros node
    rospy.init_node('cob_knoeppkes')
    rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.emcb)
  
    self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
    self.window.connect("delete_event", self.delete_event)
    self.window.set_title("cob_dashboard")	
    self.window.set_size_request(1000, 500)
    vbox = gtk.VBox(False, 1)
    self.hbox = gtk.HBox(True, 10)
    vbox.pack_start(self.hbox, True, True, 0)
    self.gpanel = GtkGeneralPanel()
    self.hbox.pack_start(self.gpanel,True, True, 3)
    b = buttons()
    panels = b.panels
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
  
app = Knoeppkes()
