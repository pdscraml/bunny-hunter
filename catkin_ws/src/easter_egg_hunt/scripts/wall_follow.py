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
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import MapMetaData, Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


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
        self.k = 0

        # define speed and turning coefficients
        self.turnCoef = [(x ** 2 - 8750) / 5000000.0 for x in xrange(-90, 0)] + [(-x ** 2 + 8750) / 5000000.0 for x in xrange(0, 91)]
        self.speedCoef = [(-x ** 2 + 8750) / 7500000.0 for x in xrange(-90,91)]

        # define random spin time
        # self.rand = random.randrange(10, 40)
        self.rand = random.randrange(100, 140)

        # define publisher to Twist
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # subscribe to laserscan
        self.lidar = rospy.Subscriber("/scan", LaserScan, self.function)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            # completed exploration?
            if self.done:
                # define node params
                package ='map_server'
                executable ='map_saver'

                rospy.loginfo("-f "+str(os.path.dirname(os.path.realpath(__file__)))+"/map")

                # init node
                node = roslaunch.core.Node(package, executable, args="-f "+str(os.path.dirname(os.path.realpath(__file__)))+"/map")

                # launch the node
                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()

                # escelate to process
                process = launch.launch(node)
                while process.is_alive():
                    print process.is_alive()

                print('EXPLORATION_COMPLETE')
                return 'EXPLORATION_COMPLETE'

            # else:
            #     return 'EXPLORATION_INCOMPLETE'

            rate.sleep()


    # function to switch between states
    def function(self, data):
        self.i += 1

        if self.i > self.rand:
            self.jackalSpin(data)
            # self.Callback(data)
        else:
            self.Callback(data)

        if self.k >= 5:
            self.lidar.unregister()
            print ('unregistered')
            self.done = 1


    # spin jackal 360 deg
    def jackalSpin(self, data):
        # publish spin msg
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0.35
        self.pub.publish(self.cmd)

        # increment j on every iteration
        self.j += 1

        # reset 'i' to get out of spin
        if self.j > 175:
            self.i = 0
            self.k += 1


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
            for p in range(0, 180):
                # Inf range return means its over 10m from the LIDAR
                if math.isinf(data.ranges[p]) or math.isnan(data.ranges[p]):
                    speedVal = speedVal + (self.speedCoef[p] * 10)
                    # Don't account long ranges into turn calcs
                else:
                    speedVal = speedVal + (self.speedCoef[p] * data.ranges[p])

                    # Turn away from walls
                    turnVal = turnVal + (self.turnCoef[p] * data.ranges[p])

            speedVal = min(speedVal * 1.2, 0.33) # sets max speed
            turnVal = turnVal * 1

            if front_zone_avg < 2.2:
                turnVal = turnVal * 2.0
                speedVal = speedVal * 1.1

        # rospy.loginfo('left_zone_avg:'+str(left_zone_avg))
        # rospy.loginfo('right_zone_avg:'+str(right_zone_avg))
        # rospy.loginfo('SpeedVal: ' + str(speedVal))
        # rospy.loginfo('turnVal: ' + str(turnVal))

        self.cmd = Twist()
        self.cmd.linear.x = speedVal
        self.cmd.angular.z = turnVal

        # rospy.loginfo(self.cmd)
        self.pub.publish(self.cmd)
        self.j = 0
