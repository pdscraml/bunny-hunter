#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#             Final Project
#          Philip (Team Lead)
#                 Ian
#                Akhil
#
# Revision: v1.2

# imports
import rospy
import smach
import smach_ros
import egg_detect

def main():
    rospy.init_node('master_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['0', '1'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Egg_Detect', egg_detect.bunny_egg_detect(),
                               transitions={'0':'',
                                            '1':''}
                              )

    # Execute SMACH plan
    outcome = sm.execute()


# standard boilerplate
if __name__ == '__main__':
    main()
