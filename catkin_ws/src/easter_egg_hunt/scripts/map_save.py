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
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import MapMetaData, Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


# map saver routine
class bunny_map_save(smach.State):
    # init state machine
    def __init__(self):
        smach.State.__init__(self, outcomes=['MAP_COMPLETE','MAP_INCOMPLETE'])


    # define executation stage
    def execute(self, userdata):

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
            pass
        #     print process.is_alive()

        process.stop()

        return 'MAP_COMPLETE'
