#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
将 rosbag 里的 nav_msgs/Path（如 /node_pos_sc）转成 EGO 用的 nav_msgs/Odometry。

配合 sbs_01 等无 odom、无 /tf 的 bag：
  - Path 约 5 Hz，点云约 10 Hz；EGO 用最新位姿，并对外发 odom->base_link 静态链路的动态段。

用法：
  python path_to_odom_bridge.py _path_topic:=/node_pos_sc _odom_topic:=/mavros/odom_convert

若点云 frame_id 为雷达系，另启 static TF（示例）：
  rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link os1_cloud_node1 100
"""
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros


class PathToOdomBridge(object):
    def __init__(self):
        rospy.init_node("path_to_odom_bridge")
        self.path_topic = rospy.get_param("~path_topic", "/node_pos_sc")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.publish_static_world = rospy.get_param("~publish_static_world", True)

        self._latest = None
        self._path_frame = self.odom_frame

        self._odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=5)
        self._tf_br = tf2_ros.TransformBroadcaster()
        if self.publish_static_world:
            self._send_static_world_map()

        rospy.Subscriber(self.path_topic, Path, self._path_cb, queue_size=5)
        rospy.loginfo(
            "path_to_odom: %s -> %s (child=%s parent=%s)",
            self.path_topic,
            self.odom_topic,
            self.base_frame,
            self.odom_frame,
        )

    def _send_static_world_map(self):
        static_br = tf2_ros.StaticTransformBroadcaster()
        now = rospy.Time.now()
        msgs = []
        for parent, child in ((self.world_frame, "map"), ("map", self.odom_frame)):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            msgs.append(t)
        static_br.sendTransform(msgs)

    def _path_cb(self, msg):
        if not msg.poses:
            return
        if msg.header.frame_id:
            self._path_frame = msg.header.frame_id
        ps = msg.poses[-1]
        if not isinstance(ps, PoseStamped):
            return
        self._latest = ps
        self._publish_odom(ps)

    def _publish_odom(self, ps):
        stamp = ps.header.stamp if ps.header.stamp.to_sec() > 0 else rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = ps.pose
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        self._odom_pub.publish(odom)

        if not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = ps.pose.position.x
        tf.transform.translation.y = ps.pose.position.y
        tf.transform.translation.z = ps.pose.position.z
        tf.transform.rotation = ps.pose.orientation
        self._tf_br.sendTransform(tf)


if __name__ == "__main__":
    try:
        PathToOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass