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
import time
import actionlib

from easter_egg_hunt.srv import EnableDiscovery
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from ar_track_alvar_msgs.msg._AlvarMarkers import AlvarMarkers
from easter_egg_hunt.msg import DiscoveredWaypoints

class EnableWaypointDiscovery(smach.State):
    def __init__(self):
        super(EnableWaypointDiscovery, self).__init__(outcomes=['WAYPOINTS_ENABLED'])
        self.enabler = rospy.ServiceProxy('WaypointManager/enable_discovery', EnableDiscovery)

    def execute(self, userdata):
        was_enabled = self.enabler(True)
        return 'WAYPOINTS_ENABLED'

class DisableWaypointDiscovery(smach.State):
    def __init__(self):
        super(DisableWaypointDiscovery, self).__init__(outcomes=['WAYPOINTS_DISABLED'])
        self.enabler = rospy.ServiceProxy('WaypointManager/enable_discovery', EnableDiscovery)

    def execute(self, userdata):
        was_enabled = self.enabler(False)
        return 'WAYPOINTS_DISABLED'

class WaypointSelect(smach.State):
    def __init__(self):
        super(WaypointSelect, self).__init__(outcomes=['WAYPOINT_SELECTED', 'WAYPOINT_UNAVAILABLE'], output_keys=["marker_dest", "marker_ID"])

    def execute(self, userdata):
        selected_marker = None
        while not selected_marker:
            try:
                selected_marker = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers, timeout=0.2).markers
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                rospy.logwarn(e)
                continue

        try:
            waypoints = rospy.wait_for_message('WaypointManager/waypoints', DiscoveredWaypoints, timeout=3)

            waypoints = {x.ID:x.pose for x in waypoints.waypoints}
            waypoint = waypoints[selected_marker[0].id]
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            return "WAYPOINT_UNAVAILABLE"

        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = 'map'
        dest.target_pose.pose = waypoint

        userdata.marker_dest = dest
        userdata.marker_ID = selected_marker[0].id

        rospy.loginfo(dest)

        time.sleep(5)
        return 'WAYPOINT_SELECTED'

class WaypointNav(smach.State):
    def __init__(self):
        super(WaypointNav, self).__init__(outcomes=["WAYPOINT_REACHED", "FAILED_WAYPOINT"], input_keys=["marker_dest"])

    def execute(self, userdata):
        # try:
        mvbs = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        mvbs.wait_for_server()

        mvbs.send_goal(userdata.marker_dest)
        mvbs.wait_for_result()

        return 'WAYPOINT_REACHED'
        # except Exception as e:
        #     return 'FAILED_WAYPOINT'
