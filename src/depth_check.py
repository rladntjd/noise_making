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
		rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depthCb)
		self.depth = np.empty((480,640), dtype = np.uint8)
		self.bridge = CvBridge()
	def depthCb(self, msg):
		#try:
		depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
		self.depth = np.array(depth)
		self.depth = (self.depth)
		#print(self.depth[100][100])
		#print(type(int(self.depth[240][400])))
		#print(int(self.depth[240][200]))
		
	def main(self):
		#print(self.depth[100,100])
		#pass
		a = int(self.depth[240][200])
		b = int(self.depth[240][400])
		
		if (a == b):
			print("True")
		else :
			print(int(self.depth[240][200]))
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
