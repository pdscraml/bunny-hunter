#!/usr/bin/env python

import rospy
import smach
import smach_ros
from sensor_msgs.msg import Joy

class JoystickButtonPause(smach.State):
    def __init__(self, topic, button, timeout=None):
        super(JoystickPause, self).__init__(outcomes=['BUTTON_PRESSED', 'BUTTON_NEVER_PRESSED'])
        self.button_pause = button
        self.timeout = timeout

    def execute(self, userdata):
        while not rospy.is_shutdown():
            try:
                payload = rospy.wait_for_message(self.topic, Joy, self.timeout)
            except ROSException, ROSInterruptException as e:
                pass

            if payload[self.button_pause]:
                return 'BUTTON_PRESSED'

        return 'BUTTON_NEVER_PRESSED'
