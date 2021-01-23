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
		self.f2 = open("mean_data_pixel", 'w')
		self.width_s = 100 #has to be changed by depth image
		self.width_e = 200 #has to be changed by depth image
		self.height_s = 100
		self.height_e = 200
		self.div_num = 2
		for i in range((self.width_e -  self.width_e)/self.div_num):
    		for j in range((self.height_e - self.height_s)/self.div_num):
    			s1 = 'self.pixel_data_%d%d = []'%(i,j)
				exec(s1)
	def depthCb(self, msg):
		depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
		self.depth = np.array(depth)
		self.depth = (self.depth) 
		std = np.std(self.depth[self.height_s:self.height_e][self.width_s:self.width_e])
        mean = np.mean(self.depth[self.height_s:self.height_e][self.width_s:self.width_e])
        writing_data = "std : %f, mean : %f\n" %(mean	std)
        self.f.write(writing_data)
		for i in range((self.width_e -  self.width_e)/self.div_num):
			for j in range((self.height_e - self.height_s)/self.div_num):
				s1 = 'self.pixel_data_%d%d.append(self.depth[i][j])'%(i,j)
				exec(s1)
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
    	pixel_data_std = ""
		pixel_data_mean = ""
		#mean , std calculation for pixels
		#mean, std write
		for i in range((self.width_e -  self.width_e)/self.div_num):
    		for j in range((self.height_e - self.height_s)/self.div_num):
				s1 = 'pixel_data_std += str(np.std(self.pixel_data_%d%d))'%(i,j)
				pixel_data_std += "\t"
				s2 = 'pixel_data_mean += str(np.mean(self.pixel_data_%d%d))'%(i,j)
				pixel_data_mean += "\t"
				exec(s1)
				exec(s2)
			pixel_data_std += "\n"
			pixel_data_mean += "\n"
		self.f2.write(pixel_data_std)
		self.f3.write(pixel_data_mean)
        self.f.close()
		self.f2.close()
		self.f3.close()

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
