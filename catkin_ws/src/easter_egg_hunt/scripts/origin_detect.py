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
import sys
import time
import os
import smach
import smach_ros

from nav_msgs.msg import Odometry


# detect origin
class jackal_origin_detect(smach.State):
    # init state machine
    def __init__(self):
        smach.State.__init__(self, outcomes=['ORIGIN_DETECTED','ORIGIN_NOT_DETECTED'], output_keys=['self.origin'])

    # define executation stage
    def execute(self, userdata):
        # init status variable
        self.done = 0

        # subscribe to odometry to get initial pose
        self.origin_pose = rospy.Subscriber("/odometry/filtered", Odometry, self.origin_detect)

        rospy.spin()

        # complete?
        if self.done:
            userdata.origin = self.origin
            return 'ORIGIN_DETECTED'
        else:
            return 'ORIGIN_NOT_DETECTED'


    def origin_detect(self, data):
        # get origin pose and position
        print('origin at:')
        self.origin = data.pose.pose
        self.origin_pose.unregister()
        print self.origin

        # set status
        self.done = 1
