#!/usr/bin/env python
import rospy
import smach
import smach_ros
import sys
import tf

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from ar_track_alvar_msgs.msg._AlvarMarkers import AlvarMarkers



class WaypointManager(object):
    _waypoints = {}
    def __init__(self, marker_distance=1):
        rospy.init_node("WaypointManager", anonymous=False)
        self.marker_distance = marker_distance
        self.marker_callback = self.saveMarkerWaypoint
        self.tf_list = tf.TransformListener()

        viz_pub = rospy.Publisher("WaypointManager/waypoint_visualization", MarkerArray, queue_size=10)
        viz_update = rospy.Rate(2)

        sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)

        while not rospy.is_shutdown():
            mark_viz = []
            for ID in self._waypoints:
                temp = Marker()

                temp.id = ID
                temp.action = Marker.ADD
                temp.ns = "WaypointManager"
                temp.type = Marker.ARROW
                temp.pose = self._waypoints[ID].pose
                temp.header = self._waypoints[ID].header

                temp.scale.x = 0.02
                temp.scale.y = 0.5
                temp.scale.z = 0.02

                temp.color.r = 1
                temp.color.g = 0
                temp.color.b = 0
                temp.color.a = 1

                mark_viz.append(temp)

            viz_pub.publish(markers=mark_viz)
            print("marker update {}".format(self._waypoints.keys()))
            viz_update.sleep()

    def saveMarkerWaypoint(self, marker):
        trans = None
        rot = None

        if self.tf_list.canTransform('map', 'head_camera', rospy.Time(0)):
            pose_st = PoseStamped()
            pose_st = marker.pose
            pose_st.header.frame_id = 'head_camera'

            transd_pose = self.tf_list.transformPose("map", pose_st)

            self._waypoints[marker.id] = transd_pose

        else:
            rospy.logwarn("Had problems transforming the waypoint.")

    def callback(self, data):
        for marker in data.markers:
            # print(marker)
            self.marker_callback(marker)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        manager = WaypointManager()
    except rospy.ROSInterruptException:
        pass
