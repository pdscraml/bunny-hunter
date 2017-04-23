#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#            Final Project
#          Philip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.1

# imports
import rospy
import actionlib
import random
import sys
import time
import roslaunch
import os
import math
import cv_bridge
import cv2
import smach
import smach_ros
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# wall following routine
class wallFollow(smach.State):
    # init state machine
    def __init__(self):
        smach.State.__init__(self, outcomes=['EXPLORATION_COMPLETE','EXPLORATION_INCOMPLETE'])

    # define executation stage
    def execute(self, userdata):
        # init status variable
        self.done = 0
        self.i = 0
        self.j = 0

        # define speed and turning coefficients
        self.turnCoef = [(x ** 2 - 9000) / 5000000.0 for x in xrange(-90, 0)] + [(-x ** 2 + 9000) / 5000000.0 for x in xrange(0, 91)]
        self.speedCoef = [(-x ** 2 + 9900) / 7500000.0 for x in xrange(-90,91)]

        # subscribers and publishers
        rospy.Subscriber("/scan", LaserScan, self.function)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.spin()

        # complete?
        if self.done:
            return 'EXPLORATION_COMPLETE'
        else:
            return 'EXPLORATION_INCOMPLETE'


    # function to switch between states
    def function(self, data):
        self.i += 1
        if self.i > random.randrange(90, 100):
            self.jackalSpin(data)
            # self.Callback(data)
        else:
            self.Callback(data)


    # spin jackal 360 deg
    def jackalSpin(self, data):
        print ('jackalSpin')
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0.45
        self.pub.publish(self.cmd)
        self.j += 1
        if self.j > 130:
            self.i = 0


    def Callback(self, data):
        # define initial varibles
        turnVal = 0
        speedVal = 0
        # define scan zones
        right_zone = data.ranges[0:65]
        front_zone = data.ranges[65:115]
        left_zone = data.ranges[115:180]

        front_zone_avg = sum(front_zone) / len(front_zone)
        right_zone_avg = sum(right_zone) / len(right_zone)
        left_zone_avg  = sum(left_zone) / len(left_zone)

        # If average is really REALLY close, might want to back up instead
        if front_zone_avg < 1 or min(front_zone) < 0.6:
            speedVal = -0.1
            if left_zone_avg > right_zone_avg and min(left_zone) > min(right_zone):
                # rospy.loginfo("Backing up to the left...")
                turnVal = 0.4
            else:
                # rospy.loginfo("Backing up to the right...")
                turnVal = -0.4

        else:
            # rospy.loginfo("Normal hallway")
            for p in range(0, 181):
                # Inf range return means its over 10m from the LIDAR
                if math.isinf(data.ranges[p]) or math.isnan(data.ranges[p]):
                    speedVal = speedVal + (self.speedCoef[p] * 10)
                    # Don't account long ranges into turn calcs
                else:
                    speedVal = speedVal + (self.speedCoef[p] * data.ranges[p])

                    # Turn away from walls
                    turnVal = turnVal + (self.turnCoef[p] * data.ranges[p])

            speedVal = min(speedVal * 1.2, 0.35) # sets max speed
            turnVal = turnVal * 1.5

            if front_zone_avg < 2.2:
                turnVal = turnVal * 2.0
                speedVal = speedVal * 1.1

        # rospy.loginfo('left_zone_avg:'+str(left_zone_avg))
        # rospy.loginfo('right_zone_avg:'+str(right_zone_avg))

        rospy.loginfo('SpeedVal: ' + str(speedVal))
        rospy.loginfo('turnVal: ' + str(turnVal))

        self.cmd = Twist()
        self.cmd.linear.x = speedVal
        self.cmd.angular.z = turnVal

        # rospy.loginfo(self.cmd)
        self.pub.publish(self.cmd)
        self.j = 0
