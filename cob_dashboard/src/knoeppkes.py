#!/usr/bin/python
# -*- coding: utf-8 -*-

from Tkinter import *
import roslib
roslib.load_manifest('cob_dashboard')
roslib.load_manifest('cob_knoeppkes')
roslib.load_manifest('cob_script_server')
from buttons import *
import thread

def start(func, args):
#	print "starting", func
#	func(*args)
	thread.start_new_thread(func,args)

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

class Knoeppkes(Frame):
  def __init__(self):
    # init ros node
    rospy.init_node('cob_knoeppkes')
  
    # init GUI
    Frame.__init__(self)
    self.grid()
    self.master.title("cob_knoeppkes")
    col = 0

    panel = General(self)
    panel.grid(row=0, column=col, padx=3, pady=3, sticky=N)
    col += 1

	# get buttons
    buttonss = buttons()
    panels = buttonss.panels
    #print panels

    for pname, actions in panels:
      panel = Panel(self, pname)
      for aname, func, args in actions:
        panel.addButton(text=aname, command=lambda f=func, a=args: start(f, a))
      panel.grid(row=0, column=col, padx=3, pady=3, sticky=N)
      col += 1

app = Knoeppkes()
app.mainloop()
