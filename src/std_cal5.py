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
        self.f = open('check_raw_filtered_std_data.txt', 'w')
        self.f2 = open('check_raw_filtered_std_data_pixel.txt', 'w')
        self.test_pub = rospy.Publisher('/testing_frequency', Int32, queue_size=10000)
        self.f3 = open('check_raw_filtered_mean_data_pixel.txt', 'w')
        self.f4 = open('check_raw_zero_value.txt', 'w')
        self.f5 = open('check_raw_over_value.txt', 'w')
        self.step = 1
        self.process = {1:[324,215,355,254,1]}.get(self.step)
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
                s1 = 'self.filtered_pixel_data_%d_%d = []' %(i*self.div_num,j*self.div_num)  # x, y coordinate
                s2 = 'self.pixel_data_%d_%d = []' %(i*self.div_num,j*self.div_num)  # x, y coordinate
                exec(s2)
                exec(s1)
    def depthCb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        test_msg = Int32()
        test_msg.data = 1
        mean_data = []
        #self.test_pub.publish(test_msg)
        self.depth = np.array(depth)
        self.depth = (self.depth.astype(np.int64)) 

        #print(writing_data)
        #print(std)
        #print(mean)
        #print((self.depth[100,200]+1))
        
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                if (self.depth[j*self.div_num + self.height_s,i*self.div_num + self.width_s] != 0) and (self.depth[j*self.div_num + self.height_s,i*self.div_num + self.width_s] < 11000):
                    mean_data.append(self.depth[j*self.div_num + self.height_s,i*self.div_num + self.width_s])
                    s3 = 'self.filtered_pixel_data_%d_%d.append(self.depth[j*self.div_num + self.height_s,i*self.div_num + self.width_s])' %(i*self.div_num,j*self.div_num)  
                    exec(s3)
                s2 = 'self.pixel_data_%d_%d.append(self.depth[j*self.div_num + self.height_s,i*self.div_num + self.width_s])' %(i*self.div_num,j*self.div_num)
                #if depth[j*self.div_num,i*self.div_num] > 20000:
                #    self.count += 1
                    
                exec(s2)
        std = np.std(mean_data)
        #print(self.depth[self.height_s:self.height_e,self.width_s:self.width_e])
        mean = np.mean(mean_data)
        writing_data = "%f    %f    \n" %(mean,std)
        self.f.write(writing_data)
        #print(self.count)
        self.count += 1
    def main(self):
        #print(self.depth[100,100])
        pass

    def final(self):
        pixel_data_std = ""
        pixel_data_mean = ""
        pixel_zero_data = ""
        pixel_over_value = []
        pixel_over_data = ""
        #mean , std calculation for pixels
        #mean, std write
        for i in range((self.width_e -  self.width_s)/self.div_num):
            for j in range((self.height_e - self.height_s)/self.div_num):
                s1 = 'pixel_data_std += str(np.std(self.filtered_pixel_data_%d_%d))' %(i*self.div_num,j*self.div_num)
                pixel_data_std += '\t'
                s2 = 'pixel_data_mean += str(np.mean(self.filtered_pixel_data_%d_%d))' %(i*self.div_num,j*self.div_num)
                pixel_data_mean += '\t'
                s3 = 'pixel_zero_data += str((self.pixel_data_%d_%d.count(0)))' %(i*self.div_num,j*self.div_num)
                pixel_zero_data += '\t'
                s4 = 'pixel_over_value = list(filter(check,self.pixel_data_%d_%d))' %(i*self.div_num,j*self.div_num)
                pixel_over_data += str(len(pixel_over_value))+'\t'
                exec(s1)
                exec(s2)
                exec(s3)
                exec(s4)
            pixel_data_std += "\n"
            pixel_data_mean += "\n"
            pixel_zero_data += "\n"
            pixel_over_data += "\n"
        
        pixel_zero_data += "\n"
        pixel_over_data += "\n"
        pixel_zero_data += str(self.count)
        pixel_over_data += str(self.count)
        self.f2.write(pixel_data_std)
        self.f3.write(pixel_data_mean)
        self.f.close()
        self.f2.close()
        self.f3.close()
        self.f4.write(pixel_zero_data)
        self.f4.close()
        self.f5.write(pixel_over_data)
        self.f5.close()
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
