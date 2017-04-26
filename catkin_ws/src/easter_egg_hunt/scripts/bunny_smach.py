#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
#             Final Project
#          Philip (Team Lead)
#                 Ian
#                Akhil
#
# Revision: v1.3

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
import waypoint_states
import map_save

def main():
    rospy.init_node('master_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['0'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Start_Pause', joystick.JoystickButtonPause('/bluetooth_teleop/joy', 0), # X
                               transitions={'BUTTON_PRESSED':'ENABLE_DISCOVERY',
                                            'BUTTON_NEVER_PRESSED': '0'})

        smach.StateMachine.add('ENABLE_DISCOVERY', waypoint_states.EnableWaypointDiscovery(),
                               transitions={'WAYPOINTS_ENABLED':'ORIGIN_DETECT'})

        smach.StateMachine.add('ORIGIN_DETECT', origin_detect.jackal_origin_detect(),
                               transitions={'ORIGIN_DETECTED':'EXPLORATION',
                                            'ORIGIN_NOT_DETECTED':'0'},
                               remapping={'origin':'sm_origin'})

        smach.StateMachine.add('EXPLORATION', wall_follow.wallFollow(),
                               transitions={'EXPLORATION_COMPLETE':'START_GOAL',
                                            'EXPLORATION_INCOMPLETE':'0'})

        smach.StateMachine.add('START_GOAL', start_goal.jackal_start_goal(),
                               transitions={'GOAL_REACHED':'MAP_SAVER',
                                            'GOAL_NOT_REACHED':'0'},
                               remapping={'destination':'sm_origin'})

        smach.StateMachine.add('MAP_SAVER', map_save.bunny_map_save(),
                               transitions={'MAP_COMPLETE':'DISABLE_DISCOVERY',
                                            'MAP_INCOMPLETE':'0'})

        smach.StateMachine.add('DISABLE_DISCOVERY', waypoint_states.DisableWaypointDiscovery(),
                               transitions={'WAYPOINTS_DISABLED':'MARKER_DISPLAY_WAIT'})

        smach.StateMachine.add('MARKER_DISPLAY_WAIT', joystick.JoystickButtonPause('/bluetooth_teleop/joy', 0), # X
                               transitions={'BUTTON_PRESSED':'SELECT_BUNNY',
                                            'BUTTON_NEVER_PRESSED': '0'})

        smach.StateMachine.add('SELECT_BUNNY', waypoint_states.WaypointSelect(),
                               transitions={'WAYPOINT_SELECTED':'EGG_DETECT',
                                            'WAYPOINT_UNAVAILABLE':'0'},
                               remapping={'marker_dest':'marker_dest',
                                          'marker_ID':'marker_ID'})

        smach.StateMachine.add('BUNNY_NAV', waypoint_states.WaypointNav(),
                               transitions={'WAYPOINT_REACHED':'EGG_DETECT',
                                            'FAILED_WAYPOINT':'START_GOAL'},
                               remapping={'marker_dest':'marker_dest'})

        smach.StateMachine.add('EGG_DETECT', egg_detect.bunny_egg_detect(),
                               transitions={'EGGS_DETECTED':'START_GOAL',
                                            'EGGS_NOT_DETECTED':'START_GOAL'})


    # Execute SMACH plan
    outcome = sm.execute()


# standard boilerplate
if __name__ == '__main__':
    main()
