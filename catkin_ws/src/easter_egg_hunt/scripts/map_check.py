#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#             Final Project
#          Phillip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.0


# imports
import rospy
import sys
import time
import roslaunch
import os
import argparse
import cv_bridge
import cv2
import numpy as np
from sys import argv

# defining class for map checker algorithm
class map_checker(object):

    # define setup and run routine
    def __init__(self):
        rospy.init_node("map_checker")

        # self.bridge = cv_bridge.CvBridge()
        # construct and parse the arguments
        # ap = argparse.ArgumentParser()
        # ap.add_argument("-i", "--image", help = "path to the image")
        # args = vars(ap.parse_args())
        # script, image_filename = argv
        # Read in map file
        rate = rospy.Rate(0.09)
        while not rospy.is_shutdown():
            self.check_map()
            rate.sleep()

    def check_map(self):
        max_area = 400
        map_image = cv2.imread(str(os.path.dirname(os.path.realpath(__file__)))[:-7]+"maps/map.pgm",1)

        # hsv = cv2.cvtColor(map_image, cv2.COLOR_BGR2HSV)
        # Define black pixel range
        blackLower = np.array([0, 0, 0], np.uint8)
        blackUpper = np.array([2, 2, 2], np.uint8)
        # Build mask and dilate black pixels
        mask = cv2.inRange(map_image, blackLower, blackUpper)
        # res = cv2.bitwise_and(map_image,map_image, mask= mask)

        kernel = np.ones((5,5),np.uint8)
        # erosion = cv2.erode(mask,kernel,iterations = 1)
        dilation = cv2.dilate(mask,kernel,iterations = 3)
        # Invert mask
        masked = 255 - dilation
        # Maskout dilated portions
        andop_output = cv2.bitwise_and(map_image, map_image, mask=masked)

        cv2.imshow('Original',map_image)
        cv2.waitKey(0)
        # cv2.imshow('Mask',mask)
        # cv2.waitKey(0)
        # cv2.imshow('Erosion',erosion)
        # cv2.waitKey(0)
        # cv2.imshow('Dilation',dilation)
        # cv2.waitKey(0)
        # cv2.imshow('Masked', masked)
        # cv2.waitKey(0)
        # cv2.imshow('After AND Operation', andop_output)
        # cv2.waitKey(0)

        # Calculate map gradient
        ddepth = cv2.CV_32F
        dx = cv2.Sobel(andop_output, ddepth, 1, 0)
        dy = cv2.Sobel(andop_output, ddepth, 0, 1)
        dxabs = cv2.convertScaleAbs(dx)
        dyabs = cv2.convertScaleAbs(dy)
        mag = cv2.addWeighted(dxabs, 0.5, dyabs, 0.5, 0)
        # cv2.imshow('Gradient', mag)
        # cv2.waitKey(0)

        # Calculate threshold of non-explored areas in map
        ret, thresh = cv2.threshold(mag, 100, 150, 100)
        # cv2.imshow('thresh', thresh)
        # cv2.waitKey(0)
        # Blur image to remove noise
        blur_img = cv2.medianBlur(thresh,5)
        contour_img = cv2.medianBlur(thresh,3)
        # erosion = cv2.erode(mask,kernel,iterations = 1)
        gray = cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)
        # Build contours of unexplored areas
        contours, hierarchy = cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(contour_img, contours, -1, (255,0,0), 20)
        # cv2.imshow('drawContours', contour_img)
        # cv2.waitKey(0)
        # Define blue contour color range
        blueLower = np.array([250,0,0], np.uint8)
        blueUpper = np.array([255,0,0], np.uint8)
        # Build mask covering blue range
        mask = cv2.inRange(contour_img, blueLower, blueUpper)
        # Erode for noise removal
        erosion = cv2.erode(mask,kernel,iterations = 1)
        # Build final contour
        final_contour = cv2.bitwise_and(contour_img, contour_img, mask=erosion)
        gray_two = cv2.cvtColor(final_contour, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('final_contour', final_contour)
        # cv2.waitKey(0)
        cv2.imshow('contour', contour_img)
        contours_two, hierarchy = cv2.findContours(gray_two,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(final_contour, contours_two, -1, (0,255,0), 3)
        # Determine if unexplored areas are large enough
        keep_hunting = 1
        print len(contours_two)
        for cr in contours_two:
            area = cv2.contourArea(cr)
            print area
            if area > max_area:
                keep_hunting = 0

        if keep_hunting == 0:
            rospy.loginfo("map still unexplored, keep hunting")
        else:
            rospy.loginfo("Hunting complete, map fully explored")

        # print contours

        cv2.imshow('contour_post', final_contour)
        # load the image

        cv2.waitKey(3)


# standard ros boilerplate
if __name__ == "__main__":
    try:
        cv = map_checker()
    except rospy.ROSInterruptException:
        pass
