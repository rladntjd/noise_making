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
        self.f = open("std_data", 'w')
		self.f2 = open("std_data_pixel", 'w')
		self.width_s = 0 #has to be changed by depth image
		self.width_e = 0 #has to be changed by depth image
		self.height_s = 0
		self.height_e = 0
		s1 = ""# memory allocation for pixel's position?
	def depthCb(self, msg):
		depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
		self.depth = np.array(depth)
		self.depth = (self.depth) 
		std = np.std(self.depth[self.height_s:self.height_e][self.width_s:self.width_e])
        mean = np.mean(self.depth[self.height_s:self.height_e][self.width_s:self.width_e])
        writing_data = "std : %f, mean : %f\n" %(mean	std)
        self.f.write(writing_data)
	def main(self):
		#print(self.depth[100,100])
		a = int(self.depth[240][200])
		b = int(self.depth[240][400])
		
		if (a == b):
			print("True")
		else :
			print(int(self.depth[240][200]))
			pass

    def final(self):
		#mean , std calculation for pixels
		#mean, std write
        self.f.close()
		self.f2.close()

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
    finally :
        mission.final()
