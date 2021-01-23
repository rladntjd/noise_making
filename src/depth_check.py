#!/usr/bin/env python

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import sin, cos, sqrt, pi
import numpy as np

class Mission:
	def __init__(self):
		# Subscriber
		cv.namedWindow("Image window", 1)
		cv.namedWindow("Depth window", 1)
		rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depthCb)
		rospy.Subscriber('/camera/color/image_raw', Image, self.colorCb)
		self.depth = np.empty((480,640), dtype = np.uint16)
		self.color = np.empty((480,640), dtype = np.uint8)
		self.bridge = CvBridge()
	def depthCb(self, msg):
		#try:
		depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
		self.depth = np.array(depth)
		self.depth = (self.depth)
		cv.imshow("Depth window", depth)
		cv.imwrite("depth.png", depth)
		cv.waitKey(1)
		#print(self.depth[100][100])
		#print(type(int(self.depth[240][400])))
		#print(int(self.depth[240][200]))
	def colorCb(self, msg):
		#try:
		color = self.bridge.imgmsg_to_cv2(msg, "8UC3")
		self.color = np.array(color)
		self.color = (self.color)
		cv.imshow("Image window", color)
		cv.imwrite("img.png", color)
		cv.waitKey(1)
		#print(self.depth[100][100])
		#print(type(int(self.depth[240][400])))
		#print(int(self.depth[240][200]))
		
	def main(self):
		#print(self.depth[100,100])
		#pass
		a = self.depth[240][200]
		b = self.depth[240][400]
		
		if (a == b):
			print("True")
		else :
			print(self.depth[240][200])
			pass
		

if __name__ == '__main__':
	# Initialize node
	rospy.init_node('main', anonymous=True, disable_signals=True)
	mission = Mission()

	rate = rospy.Rate(20.0)

	try:

		while not rospy.is_shutdown():
			mission.main()
			rate.sleep()

	except rospy.ROSInterruptException:
		pass
