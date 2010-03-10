#!/usr/bin/python
# -*- coding: utf-8 -*-
from Tkinter import *

#class arm:
#	def __init__(self, master=None):
#		print "start"

from knoeppkes_buttons import *

def start(func):
	print "starting", func
	func()
#	Async(func, ())

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
    Frame.__init__(self)
    self.grid()
    self.master.title("cob3_dashboard")
    col = 0

    panel = General(self)
    panel.grid(row=0, column=col, padx=3, pady=3, sticky=N)
    col += 1

    for pname, actions in panels:
      panel = Panel(self, pname)
      for aname, func in actions:
        panel.addButton(text=aname, command=lambda f=func: start(f))
      panel.grid(row=0, column=col, padx=3, pady=3, sticky=N)
      col += 1

app = Knoeppkes()
app.mainloop()
