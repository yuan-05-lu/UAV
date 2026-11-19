#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
将雷达 PointCloud2 用 TF 变换到 world（EGO grid_map 需要世界系点云时）。

bag 无 /tf 时：先起 path_to_odom_bridge，并用 static_transform_publisher 补 base_link->雷达。

用法：
  python points_to_world.py \
    _cloud_in:=/os1_cloud_node1/points _cloud_out:=/ego/points_world _target_frame:=world
"""
import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2


class PointsToWorld(object):
    def __init__(self):
        rospy.init_node("points_to_world")
        self.cloud_in = rospy.get_param("~cloud_in", "/os1_cloud_node1/points")
        self.cloud_out = rospy.get_param("~cloud_out", "/ego/points_world")
        self.target_frame = rospy.get_param("~target_frame", "world")

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._pub = rospy.Publisher(self.cloud_out, PointCloud2, queue_size=2)
        rospy.Subscriber(self.cloud_in, PointCloud2, self._cb, queue_size=2)
        rospy.loginfo(
            "points_to_world: %s -> %s (%s)",
            self.cloud_in,
            self.cloud_out,
            self.target_frame,
        )

    def _cb(self, msg):
        src = msg.header.frame_id or "base_link"
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time(0)
        try:
            trans = self._tf_buffer.lookup_transform(
                self.target_frame, src, stamp, rospy.Duration(0.15)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as err:
            rospy.logwarn_throttle(2.0, "TF %s -> %s: %s", src, self.target_frame, err)
            if msg.header.frame_id == self.target_frame:
                self._pub.publish(msg)
            return
        try:
            out = tf2_sensor_msgs.do_transform_cloud(msg, trans)
            out.header.frame_id = self.target_frame
            self._pub.publish(out)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "transform cloud: %s", e)


if __name__ == "__main__":
    try:
        PointsToWorld()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass