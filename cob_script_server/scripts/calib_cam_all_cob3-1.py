#!/usr/bin/python

import time
from random import *

import roslib
roslib.load_manifest('cob_script_server')
import rospy

import tf
from tf.transformations import euler_from_quaternion

import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import *


from simple_script_server import script

class CalibCam(script):

	def Initialize(self):
		self.bridge = CvBridge()
		self.image_sub_r = rospy.Subscriber("/stereo/right/image_color",Image,self.callback_r)
		self.cv_image_r = cv.CreateImage((1,1), 1 , 3)
		self.image_sub_l = rospy.Subscriber("/stereo/left/image_color",Image,self.callback_l)
		self.cv_image_l = cv.CreateImage((1,1), 1 , 3)
		self.image_sub_3d = rospy.Subscriber("/cam3d/rgb/image_color",Image,self.callback_3d)
		self.cv_image_3d = cv.CreateImage((1,1), 1 , 3)
		self.sss.init("head")
		self.sss.init("torso")
		self.sss.init("sdh")
		self.image_number_offset = 1
		self.listener = tf.TransformListener()
		self.file_path = "./"

	def Run(self):
		print "start"
		seed()
		maxVal_pan = 0.125 #max: 0.125
		maxVal_tilt = 0.20 #max: 0.20
		nr_images = 12

#joint_names: ['torso_lower_neck_pan_joint', 'torso_lower_neck_tilt_joint', 'torso_upper_neck_pan_joint', 'torso_upper_neck_tilt_joint']
#lower right  positions: [-0.10956629365682602, -0.049855709075927734, -0.16463811695575714, -0.074878953397274017]


		# move components to initial position
		self.sss.move("head","back")
		#self.sss.move("head","front")
		#self.sss.move("arm","calib")
		self.sss.move("torso","home")
		#self.sss.move("sdh","home")

		self.sss.wait_for_input()
		#self.sss.move("sdh","calib")
		#self.sss.wait_for_input()

		for j in range(1,4):
			if (j==1):
				# close (~ 0.75m)
				print "Move checkerboard to a close distance and strike any key when ready."
				self.sss.wait_for_input()
				maxVal_pan = 0.125 #max: 0.125
				maxVal_tilt = 0.15 #max: 0.20
			elif (j==2):
				# intermediate (~1.20m)
				print "Move checkerboard to an intermediate distance and strike any key when ready."
				self.sss.wait_for_input()
				maxVal_pan = 0.125 #max: 0.125
				maxVal_tilt = 0.20 #max: 0.20
			elif (j==3):
				# far (~1.90m)
				print "Move checkerboard to a far distance and strike any key when ready."
				self.sss.wait_for_input()
				maxVal_pan = 0.125 #max: 0.125
				maxVal_tilt = 0.20 #max: 0.20

			# start calbration routine
			for i in range(1,nr_images+1):
				if i==1:
					r1 = maxVal_pan
					r2 = maxVal_tilt
				elif i==2:
					r1 = 0
					r2 = maxVal_tilt
				elif i==3:
					r1 = -maxVal_pan
					r2 = maxVal_tilt
				elif i==4:
					r1 = -maxVal_pan
					r2 = 0
				elif i==5:
					r1 = 0
					r2 = 0
				elif i==6:
					r1 = maxVal_pan
					r2 = 0
				elif i==7:
					r1 = maxVal_pan
					r2 = -maxVal_tilt
				elif i==8:
					r1 = 0
					r2 = -maxVal_tilt
				elif i==9:
					r1 = -maxVal_pan
					r2 = -maxVal_tilt
				else:	
					r1 = (random()-0.5)*2*maxVal_pan;
					r2 = (random()-0.5)*2*maxVal_tilt;
				self.sss.move("torso",[[1./3.*r1,1./3.*r2,2./3.*r1,2./3.*r2]])
				self.sss.sleep(1)
				if not self.sss.parse:
					self.captureImage()

		self.sss.move("torso","home")
		print "Prepare checkerboard for further views and hit <Enter> for each image capture. Press any other key + <Enter> to finish.\n"

		key = self.sss.wait_for_input()
		if not self.sss.parse:
			while (key==''):
				self.captureImage()
				key = self.sss.wait_for_input()

		print "finished"

	def captureImage(self):
		try:
			(trans,rot) = self.listener.lookupTransform('/base_link', '/head_axis_link', rospy.Time(0))
			rpy = euler_from_quaternion(rot)
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
			fout = open(self.file_path+'calpic'+str(self.image_number_offset)+'.coords','w')
			fout.write(str(R11)+' '+str(R12)+' '+str(R13)+' '+str(trans[0]*1000)+'\n'+str(R21)+' '+str(R22)+' '+str(R23)+' '+str(trans[1]*1000)+'\n'+str(R31)+' '+str(R32)+' '+str(R33)+' '+str(trans[2]*1000))
			fout.close()
		except (tf.LookupException, tf.ConnectivityException):
			print "tf exception"

		self.sss.sleep(1)
		cv.SaveImage(self.file_path+'right_'+str(self.image_number_offset)+'.png',self.cv_image_r)
		cv.SaveImage(self.file_path+'left_'+str(self.image_number_offset)+'.png',self.cv_image_l)
		cv.SaveImage(self.file_path+'cam3d_'+str(self.image_number_offset)+'.png',self.cv_image_3d)
		self.sss.sleep(1)
		self.image_number_offset = self.image_number_offset + 1
		
	def callback_r(self,data):
		try:
			self.cv_image_r = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e

	def callback_l(self,data):
		try:
			self.cv_image_l = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e

	def callback_3d(self,data):
		try:
			self.cv_image_3d = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e

if __name__ == "__main__":
	SCRIPT = CalibCam()
	SCRIPT.Start()
