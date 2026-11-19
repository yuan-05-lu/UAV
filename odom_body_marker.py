#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
在 RViz 用 Marker 显示机体位置（bag 无 Gazebo 时比 odom_visualization 更直观）。

Add → Marker → /body_pose_marker
Fixed Frame = world
"""
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class OdomBodyMarker(object):
    def __init__(self):
        rospy.init_node("odom_body_marker", anonymous=False)
        self.frame = rospy.get_param("~frame_id", "world")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.pub = rospy.Publisher("/body_pose_marker", Marker, queue_size=1, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self._cb, queue_size=10)
        rospy.loginfo("body marker: %s -> /body_pose_marker (frame=%s)", self.odom_topic, self.frame)

    def _cb(self, msg):
        p = msg.pose.pose.position
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.frame
        m.ns = "body"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = msg.pose.pose
        if m.header.frame_id != msg.header.frame_id:
            m.pose.position = p
        m.scale.x = m.scale.y = m.scale.z = rospy.get_param("~scale", 1.0)
        m.color.r = 1.0
        m.color.g = 0.15
        m.color.b = 0.15
        m.color.a = 0.95
        self.pub.publish(m)

        arrow = Marker()
        arrow.header = m.header
        arrow.ns = "body"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = msg.pose.pose
        arrow.scale.x = 2.0
        arrow.scale.y = 0.25
        arrow.scale.z = 0.25
        arrow.color.r = 1.0
        arrow.color.g = 0.85
        arrow.color.b = 0.0
        arrow.color.a = 0.9
        self.pub.publish(arrow)


if __name__ == "__main__":
    try:
        OdomBodyMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass