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
        self.image_pub = rospy.Publisher('/image/example', Image)
        self.depth = np.empty((480,640), dtype = np.uint16)
        self.bridge = CvBridge()
        self.step = 7
        self.process = {0:[248,211,428,319,6], # 2
                        1:[292,247,386,301,4], #4
                        2:[305,244,368,280,3], #6
                        3:[309,238,363,266,2], #8
                        4:[311,238,354,263,1], #10
                        5:[314,234,355,264,1], #2~10
                        6:[315,234,355,262,1],
                        7:[0,0,640,480,3]}.get(self.step)
        print(self.process)
        self.width_s = self.process[0] #has to be changed by depth image
        print(self.width_s)
        self.width_e = self.process[2] #has to be changed by depth image
        self.height_s = self.process[1]
        self.height_e = self.process[3]
        self.div_num = self.process[4]
        self.count = 0
    def depthCb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        self.depth = np.array(depth)
        #self.depth = (self.depth.astype(np.int16)) 
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                self.depth[j,i] += 1
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.depth, encoding="passthrough"))
    def main(self):
        #print(self.depth[100,100])
        pass

    def final(self):
        pass
        #print(pixel_data_mean)

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
