#!/usr/bin/env python
import rospy
import sys
import tf
import math

from easter_egg_hunt.srv import EnableDiscovery
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
from ar_track_alvar_msgs.msg._AlvarMarkers import AlvarMarkers

class WaypointManager(object):
    _waypoints = {}
    _marker_filter = (1,2,3)
    def __init__(self, marker_distance=1, enable_on_start=True):
        rospy.init_node("WaypointManager", anonymous=False)
        self.marker_distance = marker_distance
        self.tf_list = tf.TransformListener()
        self.sub = None

        viz_pub = rospy.Publisher("WaypointManager/waypoints", PoseArray, queue_size=10)
        viz_update = rospy.Rate(2)

        self.srv = rospy.Service("WaypointManager/enable_discovery", EnableDiscovery, self.enable_discovery)

        if enable_on_start:
            self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)

        while not rospy.is_shutdown():
            header = Header(frame_id='/map')
            viz_pub.publish(PoseArray(header, self._waypoints.values()))

            rospy.loginfo("marker update {}".format(self._waypoints.keys()))
            viz_update.sleep()

    def enable_discovery(self, data):
        print(data)

    def saveMarkerWaypoint(self, marker):
        marker_frame = "ar_marker_{}".format(marker.id)
        now = rospy.Time(0)
        if self.tf_list.canTransform('map', 'head_camera', now):
            try:
                pose_st = PoseStamped()
                pose_st = marker.pose

                quat = (pose_st.pose.orientation.x,
                        pose_st.pose.orientation.y,
                        pose_st.pose.orientation.z,
                        pose_st.pose.orientation.w)

                roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)

                # Moves point to a distance infront of the Alvar marker
                yaw = yaw + math.pi/2
                pose_st.pose.position.x = pose_st.pose.position.x - (self.marker_distance * math.cos(yaw))
                pose_st.pose.position.z = pose_st.pose.position.z - (self.marker_distance * math.sin(yaw))

                converted = tf.transformations.quaternion_from_euler(0, -math.pi/2, yaw)
                pose_st.pose.orientation.x = converted[0]
                pose_st.pose.orientation.y = converted[1]
                pose_st.pose.orientation.z = converted[2]
                pose_st.pose.orientation.w = converted[3]

                pose_st.header.frame_id = 'head_camera'
                # print(pose_st)

                transd_pose = self.tf_list.transformPose("map", pose_st)

                self._waypoints[marker.id] = transd_pose.pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
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
        manager = WaypointManager(0.45)
    except rospy.ROSInterruptException:
        pass
