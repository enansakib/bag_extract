#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 25 12:04:34 2022

@author: enan
"""
import rospy
import os 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class BagExtractor:

    def __init__(self):
        rospy.init_node('to_png')

        self.img_counter = 0
        self.out_folder = '/home/enan/TestDemetriImages'   
        os.makedirs(self.out_folder, exist_ok=True)          
        self.save_flag = False
        self.rate = rospy.Rate(50)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/stereo/right/image_raw', Image, self.imageCallBack, queue_size=3) # /stereo/right/image_raw;; /camera_front_right/image_raw

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.delete_param('/save_flag')
            print("Rospy shutting down.")


    def imageCallBack(self, img_topic):
        try:
            self.img_raw = self.bridge.imgmsg_to_cv2(img_topic, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        if rospy.has_param('/save_flag'):
            self.save_flag = rospy.get_param("/save_flag")

        if self.img_raw is None:
            print('frame dropped.')
        else:        
            self.imageProcessor()
            


    def imageProcessor(self):   
        # print("test")

        input_key = cv2.waitKey(1) & 0xFF
        self.img_counter += 1
        
        if input_key == ord('q'):
            rospy.delete_param('/save_flag')
            print('Quitting the program.')
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Q is pressed.")


        # cv2.imshow('Output', self.img_raw)        
        if self.save_flag:
            print("saving.")
            cv2.imwrite(os.path.join(self.out_folder, 'img_{:03d}.png'.format(self.img_counter) ) , self.img_raw)
        
        self.rate.sleep()

    
BagExtractor()