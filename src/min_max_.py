#!/usr/bin/env python
import rospy
import cv2 as cv
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from math import sin, cos, sqrt, pi
import numpy as np

def check(item):
    if item >= 11000:
        return True
    else :
        return False

class Mission:
    def __init__(self):
        # Subscriber
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depthCb)
        self.depth = np.empty((480,640), dtype = np.uint16)
        self.bridge = CvBridge()
        self.f = open('in_pl_1_50_filtered_max_data.txt', 'w')
        self.f1 = open('in_pl_1_50_filtered_min_data.txt', 'w')
        self.test_pub = rospy.Publisher('/testing_frequency', Int32, queue_size=10000)
        self.step = 0
        self.process = {0:[319,213,346,242,1], # 1 material 50~6
                        1:[330,215,351,240,1], # 2 material 50~6
                        2:[329,216,350,234,1], #3 material 50~6
                        3:[332,210,358,246,1], #4 material 50~6
                        4:[241,135,379,307,5], #1 material 1
                        5:[152,58,409,367,10], #1 material 50
                        6:[246,133,380,301,5], #2 material 1
                        7:[171,47,397,366,10], #2 material 50
                        8:[247,140,384,298,5], #3 material 1
                        9:[171,57,429,354,10], #3 material 50
                        10:[242,137,365,299,5], #4 material 1
                        11:[174,62,403,349,10]}.get(self.step) #4 material 50
        print(self.process)
        self.width_s = self.process[0] #has to be changed by depth image
        print(self.width_s)
        self.width_e = self.process[2] #has to be changed by depth image
        self.height_s = self.process[1]
        self.height_e = self.process[3]
        self.div_num = self.process[4]
        self.count = 0
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                s2 = 'self.pixel_data_%d_%d = []' %(i*self.div_num,j*self.div_num)  # x, y coordinate
                exec(s2)
    def depthCb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        test_msg = Int32()
        test_msg.data = 1
        mean_data = []
        self.depth = np.array(depth)
        self.depth = (self.depth.astype(np.int64)) 
        
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                if (self.depth[j*self.div_num + self.height_s,i*self.div_num + self.width_s] != 0):
                    s3 = 'self.pixel_data_%d_%d.append(self.depth[j*self.div_num + self.height_s,i*self.div_num + self.width_s])' %(i*self.div_num,j*self.div_num)  
                    exec(s3)

    def main(self):
        
        pass

    def final(self):
        pixel_max_data = ""
        pixel_min_data = ""
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                s1 = 'pixel_max_data += str(max(self.pixel_data_%d_%d))' %(i*self.div_num,j*self.div_num)
                exec(s1)
                s2 = 'pixel_min_data += str(min(self.pixel_data_%d_%d))' %(i*self.div_num,j*self.div_num)
                exec(s2)
                pixel_max_data += '\t'
                pixel_min_data += '\t'
            pixel_max_data += "\n"
            pixel_min_data += '\t'
            
        self.f.write(pixel_max_data)
        self.f1.write(pixel_min_data)
        
        self.f.close()
        self.f1.close()
        

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
