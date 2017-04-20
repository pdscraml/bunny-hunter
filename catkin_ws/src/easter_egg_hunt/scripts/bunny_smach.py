#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#             Final Project
#          Philip (Team Lead)
#               Ian
#              Akhil
#
# Revision: v1.1

# imports
import rospy
import smach
import smach_ros
import egg_detect

# define state img_proc
class img_proc(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             # 0 -> busy; 1 -> completed
                             outcomes=['0','1'],
                             input_keys=[''],
                             output_keys=['']
                             )

    def execute(self, userdata):
        rospy.loginfo('Executing img proc..')

        # define flag to check status
        self.done = 0

        # invoke egg detection routine
        egg_detect.bunny_egg_detect()

        # return complete status
        if self.done:
            return '1'
        else:
            return '0'


def main():
    rospy.init_node('master_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['0', '1'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Egg_Detect', img_proc(),
                               transitions={'0':'',
                                            '1':''}
                              )

    # Execute SMACH plan
    outcome = sm.execute()


# standard boilerplate
if __name__ == '__main__':
    main()
