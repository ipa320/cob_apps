#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy


import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import *


from simple_script_server import script

class CalibCam(script):

	def Initialize(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/stereo/right/camera/image_raw",Image,self.callback)
		self.cv_image = cv.CreateImage((1,1), 1 , 3)
		self.bridge1 = CvBridge()
		self.image_sub1 = rospy.Subscriber("/stereo/left/camera/image_raw",Image,self.callback1)
		self.cv_image1 = cv.CreateImage((1,1), 1 , 3)
		self.bridge2 = CvBridge()
		self.image_sub2 = rospy.Subscriber("/swissranger/intensity/image_raw",Image,self.callback2)
		self.cv_image2 = cv.CreateImage((1,1), 1 , 3)
		cv.NamedWindow("right", 1)
		cv.NamedWindow("left", 1)
		cv.NamedWindow("sr", 1)

	def Run(self):
		print "start"
		file_path = "./"
		nr_images = 14

		self.sss.wait_for_input()

		# start calbration routine
		if not self.sss.parse:
			for i in range(1,nr_images):
				cv.SaveImage(file_path+'right'+str(i)+'.png',self.cv_image)
				cv.SaveImage(file_path+'left'+str(i)+'.png',self.cv_image1)
				cv.SaveImage(file_path+'sr'+str(i)+'.png',self.cv_image2)
				self.sss.wait_for_input()
				print "next image"
		print "finished"
		
	def callback(self,data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
			cv.ShowImage("right",self.cv_image)
			cv.WaitKey(3)
		except CvBridgeError, e:
			print e
		
	def callback1(self,data):
		try:
			self.cv_image1 = self.bridge1.imgmsg_to_cv(data, "bgr8")
			cv.ShowImage("left",self.cv_image1)
			#cv.WaitKey(3)
		except CvBridgeError, e:
			print e
		
	def callback2(self,data):
		try:
			self.cv_image2 = self.bridge2.imgmsg_to_cv(data, "mono8")
			cv.ShowImage("sr",self.cv_image2)
			#cv.WaitKey(3)
		except CvBridgeError, e:
			print e
		
if __name__ == "__main__":
	SCRIPT = CalibCam()
	SCRIPT.Start()
