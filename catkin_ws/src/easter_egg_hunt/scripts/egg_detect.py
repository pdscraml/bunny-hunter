#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#             Final Project
#          Philip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.2


# imports
import rospy
import sys
import time
import roslaunch
import os
import argparse
import cv2
import numpy as np


# defining class for discrete movement algorithm
class egg_detect(object):

    # define setup and run routine
    def __init__(self):

        # construct and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-i", "--image", help = "path to the image")
        args = vars(ap.parse_args())

        # define window
        cv2.namedWindow("results", cv2.WINDOW_NORMAL)
        # load the image
        image = cv2.imread(args["image"])

        i = 0

        # define the list of boundaries
        boundaries = [([40,  210, 230], [90,  235, 255]), # yellow
                      ([150, 90,  10],  [200, 120, 30]),  # blue
                      ([65,  125, 25],  [125, 210, 120]), # green
                      ([35,  45,  200], [80,  100, 255]), # orange
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

            # Median Blurring to remove Salt-Pepper noise
            blur_img = cv2.medianBlur(output,9)
            blur_img = cv2.medianBlur(blur_img,9)

            # convert to gray-scale to eliminate lighting effects
            gray = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)

            # apply thresholding to define gradients
            ret, thresh = cv2.threshold(gray,10,255,0)

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


# standard ros boilerplate
if __name__ == "__main__":
    try:
        cv = egg_detect()
    except rospy.ROSInterruptException:
        pass

