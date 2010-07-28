#!/usr/bin/python
# coding: utf-8

import time
import thread
import sys

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import actionlib

class CobTaskException(Exception):
	def __init__(self, value):
		self.value = value
	def __str__(self):
		return repr(self.value)



class CobTask:
	""" A CobTask is a list of CobActions or other CobTasks. Each list element
	consists of a unique identifier, the CobAction itself and a dictionary which
	relates each possible return value of the current CobAction to the identifier
	of the next CobAction to be processed. The run-routine also provides means to
	check, if the script is generally in PAUSE-status."""

	def __init__(self,description):
		self.description = "execute a task with Care-O-bot"
		self.actionList = []
		self.currentItem = ""
		self.pause = False
		self.editReturnValues = True
	
	def run(self):
		""" The elements of the actionList are executed one by one. Depending on the
		return value, the appropriate reaction is chosen and the next element is
		processed """
		
		rospy.loginfo(self.getDescription())
	
		while True:
			# Get next element to be processed and check for existance
			currentElement = self.__getElement(self.currentItem)
			rospy.logdebug("Processing item '%s'.",self.currentItem)
			rospy.loginfo(currentElement[1].getDescription())
			if currentElement == None:
				errorstring = "CobAction with identifier '" + self.currentItem + "' was not found in this CobTask!"
				rospy.logerr(errorstring)
				raise CobTaskException(errorstring)
				
			# Sleep if global pause is on
			self.__waitOnPause()
			
			# Start executing the CobAction whith the given parameters
			returnValue = apply(currentElement[1].run, currentElement[2])
			
			# Sleep if global pause is on
			if self.__waitOnPause() == 1:
				print "TODO: Implement repetition of the action, if pause was on"
			
			# Give the user the possibility to change the return value for testing
			print "TODO: send parameter for editing return value to parameter server"
			if self.editReturnValues:
				print "The processed function returned value >" + str(returnValue) + "<. ",
				print "Please type the return value (integer) you need for testing! ",
				print "Your options are:"
				print currentElement[3]
				returnValue = int(sys.stdin.readline())
			
			# Get the next item to process based on the return value
			# First check if a reaction is available
			if not returnValue in currentElement[3]:
				rospy.logerr("CobAction %s returned value '%d'. No reaction defined for that value!",self.currentItem,returnValue)
				rospy.logerr("Task will be aborted!")
				nextItem = "_abort_"
			else:
				nextItem = currentElement[3][returnValue]
			
			# If the reaction is to takte the "next" item, check if one is there	
			if nextItem == "_next_":
				idx = self.__getIndex(self.currentItem)
				# check if a next element exists
				if not len(self.actionList) > idx+1:
					rospy.logwarn("Cannot process '_next_' item, as CobAction '%s' is last in the list", self.currentItem)
					rospy.logwarn("If this is intended, please use '_end_' or '_abort_' keyword!")
					return 0
				# Change identifier to the identifier of the next element
				nextItem = self.actionList[idx+1][0]
					
			# Process keywords for special reactions
			if nextItem == "_end_":
				rospy.loginfo("Task execution ended sucessfully")
				return 0
			elif nextItem == "_abort_":
				rospy.loginfo("Task was aborted due to a failure")
				return 1
			
			# In standard case set nextItem as currentItem
			rospy.logdebug("The next CobAction to be processed is '%s'.",nextItem)
			self.currentItem = nextItem
						
		
	def addAction(self,identifier, action, parameterTuple, background=False, reactionDict=None):
		""" generate one row in the actionList which is a list itself
		->	[uniqueIdentifier, CobAction, backgroundFlag, {returnValue: next uniqueIdentifier,...}"""
	
		# check if the identifier is really unique
		if identifier in ("_next_","_end_","_abort_"):
			errorstring = "identifier in CobTask must not be '" + identifier + "' as this is a reserved word!"
			rospy.logerr(errorstring)
			raise CobTaskException(errorstring)
		for element in self.actionList:
			if element[0] == identifier:
				errorstring = "CobTask identifier '" + identifier + "' is already existing!"
				rospy.logerr(errorstring)
				raise CobTaskException(errorstring)

		# add element to list
		if reactionDict == None:
			reactionDict2 = {0:"_next_"}
		else:
			reactionDict2 = reactionDict
		self.actionList.append([identifier, action, parameterTuple, reactionDict2])
			
		# check if this was the first element. If yes, set this item as currentItem
		if len(self.actionList) == 1:
			self.currentItem = identifier
		rospy.logdebug("Element '%s' sucessfully added",identifier)
		
	def setDescription(self,description):
		self.description = description
		
	def getDescription(self):
		return self.description
		
	def printActionList(self,indent=0):
		""" print action list in a human readable form including subtasks """
		if indent == 0:
			print "\nTask structure: ",self.getDescription()
		for element in self.actionList:
			# add blanks and tree structure
			print "   ",
			for i in range(indent):
				print "|  ",
			print "|--",
			# description of the element and possible reactions
			print element[0] + "  ---->  " + str(element[3].values())
			# if the element is a task, display it's actions recursively
			if isinstance(element[1],CobTask):
				element[1].printActionList(indent+1)
				
		print "\n"

		
	def __getElement(self,identifier):
		for element in self.actionList:
			if element[0] == identifier:
				return element
		return None
		
	def __hasElement(self,identifier):
		for element in self.actionList:
			if element[0] == identifier:
				return True
		return False
		
	def __getIndex(self,identifier):
		for i in range(len(self.actionList)):
			if self.actionList[i][0] == identifier:
				return i
		return None
		
	def __waitOnPause(self):
		print "TODO: Shift pause parameter to parameter server!"
		cycleTimeOnPause = 1 #sec
		pauseWasActive = False
				
		while self.pause:
			if not pauseWasActive:
				rospy.loginfo("Skript entered Pause mode - Waiting for release")
			else:
				rospy.logdebug("Skript is still in Pause mode - Waiting for release")
			pauseWasActive = True
			time.sleep(cycleTimeOnPause)
			
		if pauseWasActive:
			rospy.loginfo("Pause was released - Continuing script")
			return 1
		return 0
			
		
	def validateReactionReferences(self):
		""" Check if all references in the reaction dicts lead someware"""
		noOfWrongReferences = 0
		rospy.loginfo("The references in this CobTask are being checked...")
		
		# Check over all element
		for element in self.actionList:
			# Check over all references in one reactionDict
			for reference in element[3].values():
				if not self.__hasElement(reference) and reference not in ("_next_","_end_","_abort_"):
					rospy.logwarn("CobTask element '%s' refers to non-existing element '%s'!", element[0], reference)
					noOfWrongReferences = noOfWrongReferences + 1
			# If the CobAction is itself a CobTask, check recursively
			if isinstance(element[1],CobTask):
				noOfWrongReferences = noOfWrongReferences + element[1].validateReactionReferences()
		
		if noOfWrongReferences > 0:
			rospy.logwarn("%d references in CobTask are non-existing and will lead to exceptions when executed!",noOfWrongReferences)
		else:
			rospy.loginfo("References in CobTask seem to be valid!")
			
		return noOfWrongReferences
					 
		

class CobAction:
	""" this is the ordinary class for wrapping actions. Each main function is
	associated with one or more error handling functions which are executed, if the
	main functions return a value different from 0. The run-routine also provides
	means to check, if the script is generally in PAUSE-status."""

	def __init__(self,mainFunctionTuple, errorFunctionTupleDict=None, revokeGoalOnPause=False):
		self.mainFunctionTuple = mainFunctionTuple
		self.errorFunctionTupleDict = errorFunctionTupleDict
		self.revokeGoalOnPause = revokeGoalOnPause
		self.description = "execute an action using ROS actionlib"
	
	def run(self,parameterTuple):
		print "DUMMY: This routine has to be filled!"
		return 0
		
	def setDescription(self,description):
		self.description = description
		
	def getDescription(self):
		return self.description



class MoveJointAction(CobAction):
	""" This class extends CobAction and focusses on moving joints. Parameters can
	be trajectories or positions wrapped as trajectories"""

	def __init__(self):
		self.description = "Move joint to specified position"
		
if __name__ == '__main__':
	action1 = CobAction(None)
	action2 = MoveJointAction()
	task1 = CobTask("Dies ist ein Testtask")
	task2 = CobTask("Der alles entscheidenede uebergeordnete Testtask")
	
	task1.addElement("act1",action1,("Hello World",))
	task1.addElement("act2",action2,("Hello World",),{0:"_end_"})
	
	task2.addElement("act3",action1,("Hello World",),{0:"_next_",2:"_abort_"})
	task2.addElement("tsk1",task1,(),{0:"_end_",1:"act3"})
	
	task2.validateReactionReferences()
	task2.printActionList()
	#rospy.loginfo("Trying to execute this crap...")
	#task2.run()
