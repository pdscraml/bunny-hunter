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

from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


# detect origin
class jackal_start_goal(smach.State):
    # init state machine
    def __init__(self):
        smach.State.__init__(self, outcomes=['GOAL_REACHED','GOAL_NOT_REACHED'], input_keys=['destination'])

    # define executation stage
    def execute(self, userdata):
        # init status variable
        self.done = 0

        # define destination as move base
        dest = []
        dest = MoveBaseGoal()
        # set initial position as goal
        dest.target_pose.header.frame_id = 'map'
        dest.target_pose.header.stamp = rospy.Time.now()
        # dest.target_pose.pose.position = Point(0.0, 0.0, 0.0)
        # dest.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.7, 0.7)

        dest.target_pose.pose = userdata.destination

        print('recieved dest:')
        print dest.target_pose.pose

        # actionlib as client for move base
        mvbs = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        mvbs.wait_for_server()
        # set move_base goal
        mvbs.send_goal(dest)
        mvbs.wait_for_result()
        # mvbs.cancel_goal()
        rospy.loginfo('Back to the Start!')

        self.done = 1

        # complete?
        if self.done:
            return 'GOAL_REACHED'
        else:
            return 'GOAL_NOT_REACHED'
