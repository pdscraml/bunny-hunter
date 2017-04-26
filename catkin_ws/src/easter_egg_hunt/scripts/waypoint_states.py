#!/usr/bin/env python
import rospy
import smach
import smach_ros
import time

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
        super(WaypointSelect, self).__init__(outcomes=['WAYPOINT_SELECTED', 'WAYPOINT_UNAVAILABLE'])

    def execute(self, userdata):
        selected_marker = None
        while not selected_marker:
            try:
                selected_marker = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers, timeout=rospy.Duration(0.2)).markers
            except (rospy.ROSException, rospy.ROSInterruptException) as e:
                rospy.logwarn(e)
                continue

        try:
            waypoint = rospy.wait_for_message('WaypointManager/waypoints', DiscoveredWaypoints, timeout=rospy.Duration(3)).waypoints[0]
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            return "WAYPOINT_UNAVAILABLE"

        dest = MoveBaseGoal()
        dest.target_pose.header.frame_id = 'map'
        dest.target_pose.pose = waypoint.pose

        userdata.marker_dest = dest
        userdata.marker_ID = waypoint.ID

        sleep(5)
        return 'WAYPOINT_SELECTED'

class WaypointNav(smach.State):
    def __init__(self):
        super(WaypointNav, self).__init__(outcomes=["WAYPOINT_REACHED", "FAILED_WAYPOINT"])

    def execute(self, userdata):
        try:
            mvbs = actionlib.SimpleActionClient('move_base', MoveBaseAction)

            mvbs.wait_for_server()

            mvbs.send_goal(userdata.marker_dest)
            mvbs.wait_for_result()

            return 'WAYPOINT_REACHED'
        except Exception as e:
            return 'FAILED_WAYPOINT'
