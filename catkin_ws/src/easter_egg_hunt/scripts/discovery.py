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

class WaypointManager(object):
    _waypoints = {}
    _marker_filter = (1,2,3)
    def __init__(self, marker_distance=1):
        rospy.init_node("WaypointManager", anonymous=False)
        self.marker_distance = marker_distance
        self.marker_callback = self.saveMarkerWaypoint
        self.tf_list = tf.TransformListener()

        viz_pub = rospy.Publisher("WaypointManager/waypoints", PoseArray, queue_size=10)
        viz_update = rospy.Rate(2)

        sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)

        while not rospy.is_shutdown():
            header = Header(frame_id='/map')
            viz_pub.publish(PoseArray(header, self._waypoints.values()))

            rospy.loginfo("marker update {}".format(self._waypoints.keys()))
            viz_update.sleep()

    def saveMarkerWaypoint(self, marker):
        if self.tf_list.canTransform('map', 'head_camera', rospy.Time(0)):
            pose_st = PoseStamped()
            pose_st = marker.pose
            pose_st.pose.orientation.x = 0
            pose_st.pose.orientation.z = 0
            pose_st.pose.orientation.w = pose_st.pose.orientation.y
            pose_st.header.frame_id = 'head_camera'

            transd_pose = self.tf_list.transformPose("map", pose_st)

            self._waypoints[marker.id] = transd_pose.pose

        else:
            rospy.logwarn("Had problems transforming the waypoint.")

    def callback(self, data):
        for marker in data.markers:
            if marker.id in self._marker_filter:
                # print(marker)
                self.marker_callback(marker)
            else:
                rospy.logwarn("Marker {} was filtered out...".format(marker.id))

# standard ros boilerplate
if __name__ == "__main__":
    try:
        manager = WaypointManager()
    except rospy.ROSInterruptException:
        pass
