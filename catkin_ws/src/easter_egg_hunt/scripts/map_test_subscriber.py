#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Bool
from easter_egg_hunt.msg import MapDone

# Simple Subscriber for testing mapdone messages from map_checker script

def callback(data):

    rospy.loginfo("Wabbit Hunt condition: " + str(data.mapdone))

def map_test_sub():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("map_status", MapDone, callback)

    rospy.spin()

if __name__ == '__main__':
    map_test_sub()
