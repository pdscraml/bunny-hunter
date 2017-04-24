#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#            Final Project
#          Philip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.4

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
        # cv2.imshow("results", image)
        # cv2.waitKey(0)

        # define the list of boundaries
        boundaries = [([70,  170, 180], [130, 215, 225]), # yellow
                      ([170, 60,  01],  [225, 110, 20]),  # blue
                      ([65,  125, 25],  [125, 210, 120]), # green
                      ([15,  100, 160], [30,  130, 225]), # orange
                      ([50,  05,  110], [120, 70,  255])] # red

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
            else:
              print('New Color!?')

            i += 1

        # stop subscription
        self.sub.unregister()
        self.done = 1

        # complete?
        if self.done:
            rospy.signal_shutdown("Detected all eggs!")
            return 'EGGS_DETECTED'
        else:
            return 'EGGS_NOT_DETECTED'