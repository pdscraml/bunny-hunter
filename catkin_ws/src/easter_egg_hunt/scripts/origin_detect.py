#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#            Final Project
#          Philip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.2

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
        smach.State.__init__(self, outcomes=['ORIGIN_DETECTED','ORIGIN_NOT_DETECTED'], output_keys=['origin'])

    # define executation stage
    def execute(self, userdata):
        # Wait for initial pose from odometry data
        payload = rospy.wait_for_message("/odometry/filtered", Odometry, timeout=None)
        # if pose recieved:
        if payload:
            userdata.origin = payload.pose.pose
            print payload.pose.pose
            return 'ORIGIN_DETECTED'
        else:
            return 'ORIGIN_NOT_DETECTED'
