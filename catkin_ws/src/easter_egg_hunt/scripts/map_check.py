#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#             Final Project
#          Phillip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.2

# Map checker script
# Subscribes to Occupancy grid topic.  Builds map based on grid data.
#   Analyzes map data for unexplored areas.  These areas can be found
#   using gradient and threshold image processing functions of OpenCV.
#   Publishes MapDone message containing boolean value representing the
#   map completeness.
#
# msg.mapdone == False: when mapping is unfinished
# msg.mapdone == True: when mapping is finished
#
# max_area: Used for determining max allowable threshold
#   for ignoring noise.  If unexplored areas exceed the script will
#   stil determine map to be unexplored

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
from nav_msgs.msg import *
from std_msgs.msg import Bool
from easter_egg_hunt.msg import MapDone

# defining class for map checker algorithm
class map_checker(object):

    # define setup and run routine
    def __init__(self):
        rospy.init_node("map_checker")
        # subscribe to Occupancy Grid data
        self.sub = rospy.Subscriber("/map", OccupancyGrid, self._rawMap)
        # Publish MapDone, containing mapdone boolean variable
        pub = rospy.Publisher("map_status", MapDone, queue_size=10)
        # Initialize image parameters for building map image
        self.raw_map = []
        self.height = 0
        self.width = 0
        # MapDone message, False if map unfinished
        msg = MapDone()
        msg.mapdone = False
        # max area allowed for ignoring unexplored map areas
        max_area = 600

        rate = rospy.Rate(0.035)
        while not rospy.is_shutdown():

            if self.height != 0:
                # Build blank map
                rospy.loginfo("Check the map...")
                map_image = np.zeros(( self.height, self.width, 3), np.uint8)
                map_image[:,0:0.5*self.width] = (255,0,0) # (B, G, R)
                map_image[:,0.5*self.width:self.width] = (0,255,0)

                k = 0
                for i in range(0, self.height):
                    for j in range(0, self.width):
                        map_image[i][j] = self.raw_map[k]

                        k = k + 1
                # cv2.imshow('Original_rawsize',map_image)
                # cv2.waitKey(0)
                map_image = cv2.resize(map_image, (0,0), fx=0.5, fy=0.5)

                # cv2.imshow("Raw Resized", map_image)

                map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2HSV)
                # Define black pixel range
                blackLower = np.array([0, 0, 0], np.uint8)
                blackUpper = np.array([2, 2, 2], np.uint8)
                # Build mask and dilate black pixels

                mask = cv2.inRange(map_image, blackLower, blackUpper)

                kernel = np.ones((5,5),np.uint8)

                dilation = cv2.dilate(mask,kernel,iterations = 3)
                # Invert mask
                masked = 255 - dilation

                # Maskout dilated portions
                andop_output = cv2.bitwise_and(map_image, map_image, mask=masked)

                # cv2.imshow('Original',map_image)
                # cv2.waitKey(0)
                # cv2.imshow('Mask',mask)
                # cv2.waitKey(0)
                # cv2.imshow('Dilation',dilation)


                # cv2.waitKey(0)
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
                blur_img = cv2.medianBlur(thresh,3)
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
                # cv2.imshow('contour', contour_img)
                contours_two, hierarchy = cv2.findContours(gray_two,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(final_contour, contours_two, -1, (0,255,0), 3)
                # Determine if unexplored areas are large enough
                keep_hunting = 1
                # print len(contours_two)
                for cr in contours_two:
                    area = cv2.contourArea(cr)
                    # print area
                    if area > max_area:
                        keep_hunting = 0
                # Publishes map completeness status
                if keep_hunting == 0:
                    rospy.loginfo("Map still unexplored, Keep hunting wabbits")
                    msg.mapdone = False
                    rospy.loginfo(msg)
                    pub.publish(msg)
                else:
                    rospy.loginfo("Map fully explored, Wabbit hunt complete")
                    msg.mapdone = True
                    rospy.loginfo(msg)
                    pub.publish(msg)

                # print contours
                # cv2.imshow('contour_post', final_contour)

                # cv2.waitKey(0)


                # rospy.loginfo("Done making raw_map")
            rate.sleep()

    # Callback function for retrieving Occupancy grid data
    # Used to determine if map is complete
    def _rawMap(self, data):
        self.height = data.info.height
        self.width = data.info.width

        print self.height
        print self.width
        print len(data.data)
        for i in range(0, len(data.data)):
            if data.data[i] == -1:
                self.raw_map.append(205)
            elif data.data[i] == 0:
                self.raw_map.append(255)
            else:
                self.raw_map.append(0)


# standard ros boilerplate
if __name__ == "__main__":
    try:
        cv = map_checker()
    except rospy.ROSInterruptException:
        pass
