#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
将 rosbag 里的 nav_msgs/Path（如 /node_pos_sc）转成 EGO 用的 nav_msgs/Odometry。

配合 sbs_01 等无 odom、无 /tf 的 bag：
  - Path 约 5 Hz，点云约 10 Hz；EGO 用最新位姿，并对外发 odom->base_link 静态链路的动态段。

用法：
  python path_to_odom_bridge.py _path_topic:=/node_pos_sc _odom_topic:=/mavros/odom_convert

若点云 frame_id 为雷达系，另启 static TF（示例）：
  rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link sensor1/os_sensor 100
"""
import math

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import tf2_ros


def _quat_valid(q):
    n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    return n2 > 1e-8


def _quat_identity():
    q = Quaternion()
    q.w = 1.0
    return q


def _quat_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


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
        self.force_identity_yaw = rospy.get_param("~force_identity_yaw", True)
        # rosbag 回放：用当前 sim time，别用 Path 里 2020 年的 stamp（防 TF_OLD_DATA / loop 回跳）
        self.use_bag_stamp = rospy.get_param("~use_bag_stamp", False)

        self._latest = None
        self._path_frame = self.odom_frame
        self._prev_x = None
        self._prev_y = None
        self._warned_bad_quat = False
        self._last_tf = None
        self._tf_rate = rospy.get_param("~tf_republish_hz", 30.0)

        self._odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=5)
        self._tf_br = tf2_ros.TransformBroadcaster()
        if self.publish_static_world:
            self._send_static_world_map()

        rospy.Subscriber(self.path_topic, Path, self._path_cb, queue_size=5)
        if self.publish_tf and self._tf_rate > 0:
            rospy.Timer(
                rospy.Duration(1.0 / self._tf_rate), self._tf_timer, oneshot=False
            )
        rospy.loginfo(
            "path_to_odom: %s -> %s (child=%s parent=%s)",
            self.path_topic,
            self.odom_topic,
            self.base_frame,
            self.odom_frame,
        )

    def _send_static_world_map(self):
        static_br = tf2_ros.StaticTransformBroadcaster()
        msgs = []
        for parent, child in ((self.world_frame, "map"), ("map", self.odom_frame)):
            t = TransformStamped()
            t.header.stamp = rospy.Time(0)
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            msgs.append(t)
        static_br.sendTransform(msgs)
        rospy.loginfo("Static TF latched: %s -> map -> %s", self.world_frame, self.odom_frame)

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

    def _orientation_for_pose(self, ps):
        if self.force_identity_yaw:
            return _quat_identity()
        q = ps.pose.orientation
        if _quat_valid(q):
            return q
        if not self._warned_bad_quat:
            rospy.logwarn(
                "Path pose has invalid quaternion (0,0,0,0); "
                "use yaw from motion or identity w=1"
            )
            self._warned_bad_quat = True
        x, y = ps.pose.position.x, ps.pose.position.y
        if self._prev_x is not None:
            dx, dy = x - self._prev_x, y - self._prev_y
            if dx * dx + dy * dy > 1e-8:
                return _quat_yaw(math.atan2(dy, dx))
        return _quat_identity()

    def _publish_odom(self, ps):
        if self.use_bag_stamp and ps.header.stamp.to_sec() > 0:
            stamp = ps.header.stamp
        else:
            stamp = rospy.Time.now()
        ori = self._orientation_for_pose(ps)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position = ps.pose.position
        odom.pose.pose.orientation = ori
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        self._odom_pub.publish(odom)

        self._last_tf = (ps.pose.position, ori)
        self._prev_x = ps.pose.position.x
        self._prev_y = ps.pose.position.y
        if self.publish_tf:
            self._send_base_tf(stamp, ps.pose.position, ori)

    def _send_base_tf(self, stamp, position, ori):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = position.x
        tf.transform.translation.y = position.y
        tf.transform.translation.z = position.z
        tf.transform.rotation = ori
        self._tf_br.sendTransform(tf)

    def _tf_timer(self, _evt):
        if not self.publish_tf or self._last_tf is None:
            return
        try:
            pos, ori = self._last_tf
            self._send_base_tf(rospy.Time.now(), pos, ori)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass


if __name__ == "__main__":
    try:
        PathToOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass