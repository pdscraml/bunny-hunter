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
import smach
import smach_ros
from sensor_msgs.msg import Joy

class JoystickButtonPause(smach.State):
    def __init__(self, topic, button, timeout=None):
        super(JoystickButtonPause, self).__init__(outcomes=['BUTTON_PRESSED', 'BUTTON_NEVER_PRESSED'])
        self.button_pause = button
        self.topic = topic
        self.timeout = timeout

    def execute(self, userdata):
        while not rospy.is_shutdown():
            try:
                payload = rospy.wait_for_message(self.topic, Joy, timeout=self.timeout)

                if payload.buttons[self.button_pause]:
                    return 'BUTTON_PRESSED'
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                rospy.logwarn(e)
                continue

        return 'BUTTON_NEVER_PRESSED'

# standard ros boilerplate
if __name__ == "__main__":
    try:
        rospy.init_node('joystick_state')
        joy = JoystickButtonPause('/bluetooth_teleop/joy', 0)
        joy.execute([])
    except rospy.ROSInterruptException:
        pass
