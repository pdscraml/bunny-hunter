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


# class WallAvoid(object):
class WallAvoid(smach.State):
        # init state machine
    def __init__(self):
        smach.State.__init__(self, outcomes=['0','1'])

    # define executation stage
    def execute(self, userdata):
        # init status variable
        self.done = 0
        timeout = 0

        self.turnCoef = [(x ** 2 - 8100) / 10000000.0 for x in range(-90, 0)] + [(-x ** 2 + 8100) / 10000000.0 for x in range(0, 91)]
        self.speedCoef = [(-x ** 2 + 8100) / 10000000.0 for x in range(-90,91)]

        rospy.Subscriber("/scan", LaserScan, self._latestScan)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Ready to get out there and avoid some walls!")
        rospy.logdebug(self.turnCoef)
        rospy.logdebug(self.speedCoef)

        self.timeout = 0
        if timeout:
            self.timeout = time.time() + timeout

        rospy.spin()

        # complete?
        if self.done:
            return '1'
        else:
            return '0'


    def isTimedout(self):
        return self.timeout <= time.time()

    def _latestScan(self, data):
        if self.timeout and self.timeout <= time.time():
            rospy.signal_shutdown("Execution timer expired")

        turnVal = 0
        speedVal = 0
        right_zone = data.ranges[0:65]
        front_zone = data.ranges[65:115]
        left_zone = data.ranges[115:180]

        front_zone_avg = sum(front_zone) / len(front_zone)
        right_zone_avg = sum(right_zone) / len(right_zone)
        left_zone_avg = sum(left_zone) / len(left_zone)

        # If average is really REALLY close, might want to back up instead
        if front_zone_avg < 1.5 or min(front_zone) < 0.8:
            speedVal = -0.1
            if left_zone_avg > right_zone_avg and min(left_zone) > min(right_zone):
                rospy.loginfo("Backing up to the left...")
                turnVal = 0.5
            else:
                rospy.loginfo("Backing up to the right...")
                turnVal = -0.3

        else:
            rospy.loginfo("Normal hallway")
            for p in range(0, 181):
                # Inf range return means its over 10m from the LIDAR
                if math.isinf(data.ranges[p]) or math.isnan(data.ranges[p]):
                    speedVal = speedVal + (self.speedCoef[p] * 10)
                    # Don't account long ranges into turn calcs
                else:
                    speedVal = speedVal + (self.speedCoef[p] * data.ranges[p])

                    # Turn away from walls
                    turnVal = turnVal + (self.turnCoef[p] * data.ranges[p])

            speedVal = min(speedVal * 1.2, 0.4) # sets max speed
            turnVal = turnVal * 1.4

            if front_zone_avg < 2.0:
                turnVal = turnVal * 2.0
                speedVal = speedVal * 1.1

        cmd = Twist()
        cmd.linear.x = speedVal
        cmd.angular.z = turnVal

        rospy.loginfo(cmd)

        self.pub.publish(cmd)
