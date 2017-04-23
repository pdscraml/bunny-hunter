#!/usr/bin/env python
import rospy
import smach
import smach_ros
import sys
import tf

from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from ar_track_alvar_msgs.msg._AlvarMarkers import AlvarMarkers

global manager

class EnableWaypointDiscovery(smach.State):
    def __init__(self):
        super(EnableWaypointDiscovery, self).__init__(outcomes=['WAYPOINTS_ENABLED'])

    def execute(self, userdata):
        global manager
        manager.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, manager.marker_callback)
        return 'WAYPOINTS_ENABLED'

class DisableWaypointDiscovery(smach.State):
    def __init__(self):
        super(DisableWaypointDiscovery, self).__init__(outcomes=['WAYPOINTS_DISABLED'])

    def execute(self, userdata):
        global manager
        manager.sub.unregister()
        return 'WAYPOINTS_DISABLED'

class WaypointNav(smach.State):
    def __init__(self):
        super(WaypointNav, self).__init__(outcomes=["WAYPOINT_REACHED", "FAILED_WAYPOINT"])

    def execute(self, userdata):
        global manager


class WaypointManager(object):
    _waypoints = {}
    _marker_filter = (1,2,3)
    def __init__(self, marker_distance=1):
        rospy.init_node("WaypointManager", anonymous=False)
        self.marker_distance = marker_distance
        self.tf_list = tf.TransformListener()
        self.sub = None

        viz_pub = rospy.Publisher("WaypointManager/waypoints", PoseArray, queue_size=10)
        viz_update = rospy.Rate(2)
        # self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)

        while not rospy.is_shutdown():
            header = Header(frame_id='/map')
            viz_pub.publish(PoseArray(header, self._waypoints.values()))

            rospy.loginfo("marker update {}".format(self._waypoints.keys()))
            viz_update.sleep()

    def saveMarkerWaypoint(self, marker):
        if self.tf_list.canTransform('map', 'head_camera', rospy.Time(0)):
            pose_st = PoseStamped()
            pose_st = marker.pose

            quat = (pose_st.pose.orientation.x,
                    pose_st.pose.orientation.y,
                    pose_st.pose.orientation.z,
                    pose_st.pose.orientation.w)

            (roll, pitch, yaw) = (tf.transformations.euler_from_quaternion(quat))
            converted = tf.transformations.quaternion_from_euler(3.14159, pitch - 1.570796327, 0.0)

            # print(roll, pitch, yaw)

            pose_st.pose.orientation.x = converted[0]
            pose_st.pose.orientation.y = converted[1]
            pose_st.pose.orientation.z = converted[2]
            pose_st.pose.orientation.w = converted[3]

            # Moves point to a distance infront of the Alvar marker
            pose_st.pose.position.z = pose_st.pose.position.z - self.marker_distance

            pose_st.header.frame_id = 'head_camera'
            # print(pose_st)

            transd_pose = self.tf_list.transformPose("map", pose_st)

            self._waypoints[marker.id] = transd_pose.pose
        else:
            rospy.logwarn("Had problems transforming the waypoint.")

    def marker_callback(self, data):
        for marker in data.markers:
            if marker.id in self._marker_filter:
                # print(marker)
                self.saveMarkerWaypoint(marker)
            else:
                rospy.logwarn("Marker {} was filtered out...".format(marker.id))

# standard ros boilerplate
if __name__ == "__main__":
    try:
        global manager
        manager = WaypointManager()
    except rospy.ROSInterruptException:
        pass
