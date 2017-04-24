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
import joystick
import wall_follow
import discovery
import origin_detect
import start_goal

def main():
    rospy.init_node('master_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['0', '1'])
    sm.userdata.sm_origin = 0.0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Start_Pause', joystick.JoystickButtonPause('/bluetooth_teleop/joy', 0), # X
                               transitions={'BUTTON_PRESSED':'ENABLE_DISCOVERY',
                                            'BUTTON_NEVER_PRESSED': '0'})


        smach.StateMachine.add('Start_Pause1', joystick.JoystickButtonPause('/bluetooth_teleop/joy', 1), # O
                               transitions={'BUTTON_PRESSED':'ORIGIN_DETECT',
                                            'BUTTON_NEVER_PRESSED': '0'})

        smach.StateMachine.add('ORIGIN_DETECT', origin_detect.jackal_origin_detect(),
                               transitions={'ORIGIN_DETECTED':'START_GOAL',
                                            'ORIGIN_NOT_DETECTED':'0'},
                               remapping={'origin':'sm_origin'})

        smach.StateMachine.add('START_GOAL', start_goal.(),
                               transitions={'GOAL_REACHED':'0',
                                            'GOAL_NOT_REACHED':'0'},
                               remapping={'destination':'sm_origin'})


        smach.StateMachine.add('ENABLE_DISCOVERY', discovery.EnableWaypointDiscovery(),
                               transitions={'WAYPOINTS_ENABLED':'Exploration'})


        smach.StateMachine.add('Exploration', wall_follow.wallFollow(),
                               transitions={'EXPLORATION_COMPLETE':'Egg_Detect',
                                            'EXPLORATION_INCOMPLETE':'0'})

        smach.StateMachine.add('DISABLE_DISCOVERY', discovery.DisableWaypointDiscovery(),
                               transitions={'WAYPOINTS_DISABLED':'Egg_Detect'})

        smach.StateMachine.add('Egg_Detect', egg_detect.bunny_egg_detect(),
                               transitions={'0':'',
                                            '1':''})

    # Execute SMACH plan
    outcome = sm.execute()


# standard boilerplate
if __name__ == '__main__':
    main()
