#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
仅可视化：用 traj_server 的 /planning/pos_cmd 驱动假机体，不参与 EGO 反馈。

EGO 仍使用 path_to_odom_bridge 的 /mavros/odom_convert；本节点另发：
  - /viz/odom
  - /viz/executed_path（累积已“跟踪”的规划位置）
  - /viz_planner_marker（洋红球 + 箭头）

RViz（Fixed Frame = world）：
  - Odometry → /viz/odom
  - Path → /viz/executed_path
  - Marker → /viz_planner_marker

可选：用 odom_visualization 看 mesh（勿改 EGO 的 odom 参数）：
  rosrun odom_visualization odom_visualization _frame_id:=world \
    odom:=/viz/odom

历史相机（与规划无关，随 bag 时间播放）：
  rosrun rqt_image_view rqt_image_view
  选 /left/image_raw 或 /right/image_raw

用法：
  python poscmd_to_viz_odom.py
"""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from visualization_msgs.msg import Marker
from std_msgs.msg import Header


def _yaw_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PosCmdToVizOdom(object):
    def __init__(self):
        rospy.init_node("poscmd_to_viz_odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/planning/pos_cmd")
        self.odom_topic = rospy.get_param("~odom_topic", "/viz/odom")
        self.path_topic = rospy.get_param("~path_topic", "/viz/executed_path")
        self.marker_topic = rospy.get_param("~marker_topic", "/viz_planner_marker")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.child_frame = rospy.get_param("~child_frame", "viz_base_link")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.min_step = rospy.get_param("~path_min_step", 0.08)
        self.marker_scale = rospy.get_param("~marker_scale", 0.55)

        self._path = Path()
        self._path.header.frame_id = self.world_frame
        self._last_append = None
        self._traj_id = None
        self._last_pos = None

        self._odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=5)
        self._path_pub = rospy.Publisher(self.path_topic, Path, queue_size=2)
        self._marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=2)
        self._tf_br = tf2_ros.TransformBroadcaster()

        rospy.Subscriber(self.cmd_topic, PositionCommand, self._cmd_cb, queue_size=20)
        rospy.loginfo(
            "viz planner odom: %s -> %s, path %s (EGO odom unchanged)",
            self.cmd_topic,
            self.odom_topic,
            self.path_topic,
        )

    def _maybe_clear_path(self, msg):
        tid = getattr(msg, "trajectory_id", None)
        if tid is None:
            return
        if self._traj_id is not None and tid != self._traj_id:
            self._path.poses = []
            self._last_append = None
            rospy.loginfo("new trajectory_id=%s, cleared %s", tid, self.path_topic)
        self._traj_id = tid

    def _append_path(self, stamp, x, y, z, q):
        if self._last_append is not None:
            dx = x - self._last_append[0]
            dy = y - self._last_append[1]
            dz = z - self._last_append[2]
            if dx * dx + dy * dy + dz * dz < self.min_step * self.min_step:
                return
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.world_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation = q
        self._path.header.stamp = stamp
        self._path.poses.append(ps)
        self._last_append = (x, y, z)
        self._path_pub.publish(self._path)

    def _publish_markers(self, stamp, x, y, z, q):
        base = Header(stamp=stamp, frame_id=self.world_frame)
        sphere = Marker()
        sphere.header = base
        sphere.ns = "viz_planner"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = z
        sphere.pose.orientation = q
        sphere.scale.x = sphere.scale.y = sphere.scale.z = self.marker_scale
        sphere.color.r = 0.95
        sphere.color.g = 0.1
        sphere.color.b = 0.75
        sphere.color.a = 0.95

        arrow = Marker()
        arrow.header = base
        arrow.ns = "viz_planner"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = z
        arrow.pose.orientation = q
        arrow.scale.x = 1.6
        arrow.scale.y = 0.22
        arrow.scale.z = 0.22
        arrow.color.r = 1.0
        arrow.color.g = 0.85
        arrow.color.b = 0.15
        arrow.color.a = 0.9

        self._marker_pub.publish(sphere)
        self._marker_pub.publish(arrow)

    def _send_tf(self, stamp, x, y, z, q):
        if not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation = q
        self._tf_br.sendTransform(tf)

    def _cmd_cb(self, msg):
        stamp = rospy.Time.now()
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        q = _yaw_quat(msg.yaw)

        self._maybe_clear_path(msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = q
        odom.twist.twist.linear = msg.velocity
        self._odom_pub.publish(odom)

        self._append_path(stamp, x, y, z, q)
        self._publish_markers(stamp, x, y, z, q)
        self._send_tf(stamp, x, y, z, q)
        self._last_pos = (x, y, z)


if __name__ == "__main__":
    try:
        PosCmdToVizOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass