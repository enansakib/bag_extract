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
        self.out_folder = '/home/enan0001/zedbag_2023-04-25-16-14-29_3'  
        self.out_folder_left = self.out_folder+'_left'
        self.out_folder_right = self.out_folder+'_right'
        os.makedirs(self.out_folder_left, exist_ok=True)     
        os.makedirs(self.out_folder_right, exist_ok=True)     
        self.save_flag = False
        self.rate = rospy.Rate(3) # ~number of frames saved per second
        self.bridge = CvBridge()
        self.image_sub_left = message_filters.Subscriber('/zedm/zed_node/left_raw/image_raw_color', Image)
        self.image_sub_right = message_filters.Subscriber('/zedm/zed_node/right_raw/image_raw_color', Image)
        self.time_sync = message_filters.ApproximateTimeSynchronizer([self.image_sub_left, self.image_sub_right], queue_size=10, allow_headerless=True, slop=0.001)
        self.time_sync.registerCallback(self.imageCallback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.delete_param('/save_flag')
            print("Rospy shutting down.")

    def imageCallback(self, img_topic_left, img_topic_right):
        try:
            self.img_raw_left = self.bridge.imgmsg_to_cv2(img_topic_left, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            self.img_raw_right = self.bridge.imgmsg_to_cv2(img_topic_right, "bgr8")
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

        if input_key == ord('c'):
            if rospy.has_param('/save_flag'):
                self.save_flag = rospy.get_param("/save_flag")
                if self.save_flag:
                    print("Setting /save_flag from "+str(self.save_flag),end="")
                    rospy.set_param("/save_flag",False)
                    self.save_flag = rospy.get_param("/save_flag")
                    print(" to "+ str(self.save_flag))
                else: 
                    print("Setting /save_flag from "+str(self.save_flag),end="")
                    rospy.set_param("/save_flag",True)
                    self.save_flag = rospy.get_param("/save_flag")
                    print(" to "+ str(self.save_flag))
            else: 
                print("Setting /save_flag from "+str(self.save_flag),end="")
                rospy.set_param("/save_flag",True)
                self.save_flag = rospy.get_param("/save_flag")
                print(" to "+ str(self.save_flag))


        cv2.imshow('Output', self.img_raw_left) 
        if rospy.has_param('/save_flag'):      
            self.save_flag = rospy.get_param("/save_flag") 
        if self.save_flag:
            self.img_counter += 1
            # print("saving...")
            cv2.imwrite(os.path.join(self.out_folder_left, 'img_{:03d}_left.png'.format(self.img_counter) ) , self.img_raw_left)
            cv2.imwrite(os.path.join(self.out_folder_right, 'img_{:03d}_right.png'.format(self.img_counter) ) , self.img_raw_right)

        self.rate.sleep()

        
BagExtractor()
