#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import tf
from random import *
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import simple_script_server

class CalibCam:
	def __init__(self):
		rospy.init_node('calib_script')
		self.sss = simple_script_server.simple_script_server()
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/stereo/right/image_color",Image,self.callback)
		self.sss.Init("torso")

	def callback(self,data):
		try:
  			self.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
  			print e

		cv.ShowImage("Image window", self.cv_image)
		cv.WaitKey(3)

	def run(self): 
		
		seed()
		maxVal = 0.1
		listener = tf.TransformListener()
		print "start"
		self.sss.Move("torso","home")
		for i in range(1,9):
			r1 = (random()-0.5)*2*maxVal;
			r2 = (random()-0.5)*2*maxVal;
			r3 = (random()-0.5)*2*maxVal;
			r4 = (random()-0.5)*2*maxVal;
			self.sss.Move("torso",[[r1,r2,r3,r4]])
			self.sss.Sleep(1)
			try:
				(trans,rpy) = listener.lookupTransform('/base_link', '/head_color_camera_r_link', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException):
				print "tf exception"
			cyaw = cos(rpy[2])
			syaw = sin(rpy[2])
			cpitch = cos(rpy[1])
			spitch = sin(rpy[1])
			croll = cos(rpy[0])
			sroll = sin(rpy[0])
			R11 = cyaw*cpitch
			R12 = cyaw*spitch*sroll-syaw*croll
			R13 = cyaw*spitch*croll+syaw*sroll
			R21 = syaw*cpitch
			R22 = syaw*spitch*sroll+cyaw*croll
			R23 = syaw*spitch*croll-cyaw*sroll
			R31 = -spitch
			R32 = cpitch*sroll
			R33 = cpitch*croll
			fout = open('calpic'+str(i)+'.coords','w')
			fout.write(str(R11)+' '+str(R12)+' '+str(R13)+' '+str(trans[0])+'\n'+str(R21)+' '+str(R22)+' '+str(R23)+' '+str(trans[1])+'\n'+str(R31)+' '+str(R32)+' '+str(R33)+' '+str(trans[2]))
			fout.close()
			cv.SaveImage('calpic'+str(i)+'.png',self.cv_image)
		print "finished"
		
if __name__ == "__main__":
	SCRIPT = CalibCam()
	SCRIPT.run()
