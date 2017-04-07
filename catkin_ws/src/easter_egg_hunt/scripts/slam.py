#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#          Final Project
#        Philip (Team Lead)
#             Ian
#            Akhil
#
# Revision: v1.1

# imports
import rospy
import sys
import time
import roslaunch
import os
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# defining class for discrete movement algorithm
class slam(object):
    # define setup and run routine
    def __init__(self):

        self.motion = Twist()
        self.wall_found = 0
        self.loop_counter = 1
        self.direction = 0
        self.closest_value = 0.0

        # create node for listening to twist messages
        rospy.init_node("slam")

        # subscribe to twist_key
        # rospy.Subscriber("/front/scan", LaserScan, self.Callback)
        rospy.Subscriber("/scan", LaserScan, self.Callback)
        rate = rospy.Rate(10)

        # publish to cmd_vel of the jackal
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():

            # publish Twist
            pub.publish(self.motion)

            self.loop_counter += 1
            # self.wall_found = 0

            rate.sleep()


    # motion algorithm in Callback routine for Jackal motion
    def Callback(self, data):

        # if wall found
        if (min(data.ranges) < 1.5) :
            self.motion.linear.x  = 0
            self.motion.angular.z  = 0

            # go towards the wall
            self.get_direction(data)
            rospy.loginfo(self.direction)

            # rospy.loginfo('going towards wall..')
            # self.motion.linear.x  = 1
            # self.motion.angular.z  = 0

            # stop if too close to the wall
            # if (min(data.ranges) < 0.5) :
            #     # stop
            #     self.motion.linear.x  = 0
            #     self.motion.angular.z  = 0

        # if wall not found
        else :
            # move in a spiral to minimize the time to find a wall
            rospy.loginfo('spiral..')
            self.motion.linear.x = 0.4
            self.motion.angular.z = math.exp(-self.loop_counter/75.0)
            rospy.loginfo(self.motion.linear.x)
            rospy.loginfo(self.motion.angular.z)


    def get_direction(self, data):
        self.closest_value = min(data.ranges)
        self.direction = data.ranges.index(self.closest_value)


# standard ros boilerplate
if __name__ == "__main__":
    try:
        move = slam()
    except rospy.ROSInterruptException:
        pass

