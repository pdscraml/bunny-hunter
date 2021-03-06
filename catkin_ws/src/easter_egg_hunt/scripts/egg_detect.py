#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#            Final Project
#          Philip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.5

# imports
import rospy
import sys
import time
import roslaunch
import os
import cv_bridge
import cv2
import smach
import smach_ros
import numpy as np
from sensor_msgs.msg import Image


# defining class for egg detection
class bunny_egg_detect(smach.State):
    # init state machine
    def __init__(self):
        smach.State.__init__(self, outcomes=['EGGS_DETECTED', 'EGGS_NOT_DETECTED'])

    # define executation stage
    def execute(self, userdata):
        # init status variable
        self.done = 0

        # cvBridge is a ROS library that provides an interface between ROS
        # and OpenCV.
        self.bridge = cv_bridge.CvBridge()
        # create node for listening to twist messages
        # rospy.init_node("egg_detect")

        # subscribe to raw images
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.Callback)
        # msg = rospy.wait_for_message("my_topic", MyType)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            # complete?
            if self.done:
                rospy.signal_shutdown("Detected all eggs!")
                return 'EGGS_DETECTED'
            # else:
            #     return 'EGGS_NOT_DETECTED'

            rate.sleep()

        # # complete?
        # if self.done:
        #     rospy.signal_shutdown("Detected all eggs!")
        #     return '1'
        # else:
        #     return '0'


    # egg detection algorithm
    def Callback(self, data):
        # iterator variable
        i = 0

        # define window
        cv2.namedWindow("results", cv2.WINDOW_NORMAL)
        # load the image
        # imgmsg_to_cv2 converts an image message pointer to an OpenCV
        # message.
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # cv2.imshow("results", image)
        # cv2.waitKey(0)

        # # define the list of boundaries (BGR)
        # boundaries = [([160, 255, 255], [60, 190, 205]), # yellow
        #               ([230, 150, 60], [255, 190, 100]), # blue
        #               ([80, 170, 80], [130, 210, 120]),  # green
        #               ([1, 180, 255], [20, 200, 255]),   # orange
        #               ([70, 120, 215], [130, 175, 255]), # red
        #               ([160, 120, 160], [180, 150, 180])]# violet

        boundaries = [([160, 255, 255], [60, 190, 205]), # yellow
                      ([218, 127, 79], [223, 115, 54]),  # blue
                      ([175, 175, 108], [104, 139, 93]), # green
                      ([57, 169, 235], [13, 136, 209]),  # orange
                      ([136, 136, 200], [67, 83, 159]),  # red
                      ([218, 171, 183], [119, 34, 82])]  # violet

        # # define the list of boundaries
        # # boundaries = [([20, 100, 100], [30, 255, 255]), # yellow
        # boundaries = [([20, 100, 100], [30, 255, 255]), # yellow
        #               ([90, 100, 100], [110, 255, 255]), # blue
        #               ([60, 100, 100], [95, 255, 255]),  # green
        #               ([5, 100, 100], [30, 255, 255]),   # orange
        #               ([160, 100, 100], [185, 255, 255]), # red
        #               ([128, 100, 100], [164, 255, 255])] # violet

        # loop over the boundaries
        for (lower, upper) in boundaries:

            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

            # find the colors within the specified boundaries and apply the mask
            mask = cv2.inRange(image, lower, upper)
            output = cv2.bitwise_and(image, image, mask = mask)

            img = output
            # cv2.imshow("results", output)
            # cv2.waitKey(0)

            # Median Blurring to remove Salt-Pepper noise
            blur_img = cv2.medianBlur(output,9)
            blur_img = cv2.medianBlur(blur_img,9)
            # cv2.imshow("results", blur_img)
            # cv2.waitKey(0)

            # convert to gray-scale to eliminate lighting effects
            gray = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)
            # cv2.imshow("results", gray)
            # cv2.waitKey(0)

            # apply thresholding to define gradients
            ret, thresh = cv2.threshold(gray,10,255,0)
            # cv2.imshow("results", thresh)
            # cv2.waitKey(0)

            # define and draw contours around detected objects
            contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(blur_img, contours, -1, (0,255,0), 3)
            # cv2.imshow("results", blur_img)
            # cv2.waitKey(0)

            # Display color's and number of eggs:
            if i==0:
                print('Eggs found:')
                print('Yellow: %d'%len(contours))
            elif i==1:
              print('Blue: %d'%len(contours))
            elif i==2:
              print('Green: %d'%len(contours))
            elif i==3:
              print('Orange: %d'%len(contours))
            elif i==4:
              print('Red: %d'%len(contours))
            elif i==5:
              print('Violet: %d'%len(contours))
            else:
              print('New Color!?')

            i += 1

        # stop subscription
        self.sub.unregister()
        self.done = 1
