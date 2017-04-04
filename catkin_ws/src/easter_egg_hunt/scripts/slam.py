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

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# defining class for discrete movement algorithm
class slam(object):
    # define setup and run routine
    def __init__(self):

        self.motion = Twist()

        # create node for listening to twist messages
        rospy.init_node("slam")

        # subscribe to twist_key
        rospy.Subscriber("/scan", LaserScan, self.Callback)
        rate = rospy.Rate(10)

        loop_counter = 1
        # publish to cmd_vel of the jackal
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():

            # rate = rospy.Rate(100.0/counter)

            # # push Twist msgs to override Callback algorithm
            self.motion.linear.x = loop_counter/100.0
            self.motion.angular.z = 0.4

            # publish Twist
            pub.publish(self.motion)

            loop_counter += 1

            rate.sleep()


    # motion algorithm in Callback routine for Jackal motion
    def Callback(self, data):

        # move forward
        rospy.loginfo('publishing to /cmd_vel..\n')
        self.motion.linear.x  = 1
        self.motion.angular.z = 1


# standard ros boilerplate
if __name__ == "__main__":
    try:
        move = slam()
    except rospy.ROSInterruptException:
        pass
