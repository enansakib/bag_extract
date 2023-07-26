#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 25 12:04:34 2022

@author: enan
"""
import rospy
import os 
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class BagExtractor:

    def __init__(self):
        rospy.init_node('to_png')

        self.img_counter_left = 0
        self.img_counter_right = 0
        self.img_counter = 0
        self.out_folder = '/home/dtkutzke/2023_01_30_Aqua_Raw_Images/11_24_55/'  
        os.makedirs(self.out_folder, exist_ok=True)          
        self.save_flag = False
        #self.rate = rospy.Rate(50)
        self.bridge = CvBridge()
        self.image_sub_left = message_filters.Subscriber('/camera_front_left/image_raw', Image) # /stereo/right/image_raw;; /camera_front_right/image_raw
        self.image_sub_right = message_filters.Subscriber('/camera_front_right/image_raw', Image)
        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub_left, self.image_sub_right], queue_size=10, allow_headerless=True, slop=0.001)
        self.time_sync.registerCallback(self.imageCallback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.delete_param('/save_flag')
            print("Rospy shutting down.")

    def imageCallback(self, img_topic_left, img_topic_right):
        try:
            self.img_raw_left = self.bridge.imgmsg_to_cv2(img_topic_left, "rgb8")
        except CvBridgeError as e:
            print(e)
        try:
            self.img_raw_right = self.bridge.imgmsg_to_cv2(img_topic_right, "rgb8")
        except CvBridgeError as e:
            print(e)
                
        if rospy.has_param('/save_flag'):
            self.save_flag = rospy.get_param("/save_flag")

        if self.img_raw_left is None or self.img_raw_right is None:
            print('frame dropped.')
        else:        
            self.imageProcessor()    

    def imageProcessor(self):
        # print("test")

        input_key = cv2.waitKey(1) & 0xFF
        
        if input_key == ord('q'):
            rospy.delete_param('/save_flag')
            print('Quitting the program.')
            #cv2.destroyAllWindows()
            rospy.signal_shutdown("Q is pressed.")


        #cv2.imshow('Output', self.img_raw_left)        
        if self.save_flag:
            self.img_counter += 1
            print("saving...")
            cv2.imwrite(os.path.join(self.out_folder, 'img_{:03d}_left.png'.format(self.img_counter) ) , self.img_raw_left)
            cv2.imwrite(os.path.join(self.out_folder, 'img_{:03d}_right.png'.format(self.img_counter) ) , self.img_raw_right)

        #self.rate.sleep()

    def imageCallBackLeft(self, img_topic):
        try:
            self.img_raw_left = self.bridge.imgmsg_to_cv2(img_topic, "mono8")
            self.has_left = True
        except CvBridgeError as e:
            print(e)
        
        if rospy.has_param('/save_flag'):
            self.save_flag = rospy.get_param("/save_flag")

        if self.img_raw_left is None:
            self.has_left = False
            print('frame dropped.')
        else:        
            self.imageProcessorLeft()



    def imageCallBackRight(self, img_topic):
        try:
            self.img_raw_right = self.bridge.imgmsg_to_cv2(img_topic, "mono8")
            self.has_right = True
        except CvBridgeError as e:
            print(e)
        
        if rospy.has_param('/save_flag'):
            self.save_flag = rospy.get_param("/save_flag")

        if self.img_raw_right is None:
            print('frame dropped.')
        elif self.has_left:        
            self.imageProcessorRight()



    def imageProcessorLeft(self):   
        # print("test")

        input_key = cv2.waitKey(1) & 0xFF
        
        if input_key == ord('q'):
            rospy.delete_param('/save_flag')
            print('Quitting the program.')
            #cv2.destroyAllWindows()
            rospy.signal_shutdown("Q is pressed.")


        #cv2.imshow('Output', self.img_raw_left)        
        if self.save_flag:
            self.img_counter_left += 1
            print("saving.")
            cv2.imwrite(os.path.join(self.out_folder, 'img_{:03d}_left.png'.format(self.img_counter_left) ) , self.img_raw_left)

        self.rate.sleep()
        
    def imageProcessorRight(self):   
        # print("test")

        input_key = cv2.waitKey(1) & 0xFF
        # self.img_counter += 1
        
        if input_key == ord('q'):
            rospy.delete_param('/save_flag')
            print('Quitting the program.')
            #cv2.destroyAllWindows()
            rospy.signal_shutdown("Q is pressed.")


        #cv2.imshow('Output', self.img_raw_left)        
        if self.save_flag:
            self.img_counter_right += 1
            print("saving.")
            cv2.imwrite(os.path.join(self.out_folder, 'img_{:03d}_right.png'.format(self.img_counter_right) ) , self.img_raw_right) 
            self.has_left = False   

        self.rate.sleep()

    
BagExtractor()
