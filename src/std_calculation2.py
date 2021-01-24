#!/usr/bin/env python
import rospy
import cv2 as cv
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from math import sin, cos, sqrt, pi
import numpy as np

class Mission:
    def __init__(self):
        # Subscriber
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depthCb)
        self.depth = np.empty((480,640), dtype = np.uint16)
        self.bridge = CvBridge()
        self.f = open('ex_2_10pl_std_data.txt', 'w')
        self.f2 = open('ex_2_10pl_std_data_pixel.txt', 'w')
        self.test_pub = rospy.Publisher('/testing_frequency', Int32, queue_size=10000)
        self.f3 = open('ex_2_10pl_mean_data_pixel.txt', 'w')
        self.width_s = 248 #has to be changed by depth image
        self.width_e = 211 #has to be changed by depth image
        self.height_s = 428
        self.height_e = 319
        self.div_num = 8
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                s1 = 'self.pixel_data_%d%d = []' %(i,j)  # x, y coordinate
                exec(s1)
    def depthCb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        test_msg = Int32()
        test_msg.data = 1
        self.test_pub.publish(test_msg)
        self.depth = np.array(depth)
        self.depth = (self.depth.astype(np.int64)) 
        std = np.std(self.depth[self.height_s:self.height_e,self.width_s:self.width_e])
        #print(self.depth[self.height_s:self.height_e,self.width_s:self.width_e])
        mean = np.mean(self.depth[self.height_s:self.height_e,self.width_s:self.width_e])
        writing_data = "%f    %f    \n" %(mean,std)
        print(writing_data)
        print(std)
        print(mean)
        print((self.depth[100,200]+1))
        self.f.write(writing_data)
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                s2 = 'self.pixel_data_%d%d.append(self.depth[j,i])' %(i,j)
                exec(s2)
    def main(self):
        #print(self.depth[100,100])
        pass

    def final(self):
        pixel_data_std = ""
        pixel_data_mean = ""
        #mean , std calculation for pixels
        #mean, std write
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                s1 = 'pixel_data_std += str(np.std(self.pixel_data_%d%d[:,:]))'%(i,j)
                pixel_data_std += "\t"
                s2 = 'pixel_data_mean += str(np.mean(self.pixel_data_%d%d[:,:]))'%(i,j)
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
