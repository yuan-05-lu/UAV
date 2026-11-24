#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
评指标 / 离线规划：固定起点位姿持续发布 odom + TF（不依赖 rosbag）。

配合：全局 PCD + pcd_map_publisher、use_sim_time=false、run_ego_offline_eval.launch

用法：
  rosparam set use_sim_time false
  python static_odom_publisher.py _x:=1.0 _y:=2.0 _z:=1.2
"""
import math

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion


def _yaw_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class StaticOdomPublisher(object):
    def __init__(self):
        rospy.init_node("static_odom_publisher")
        self.x = rospy.get_param("~x", 0.0)
        self.y = rospy.get_param("~y", 0.0)
        self.z = rospy.get_param("~z", 1.0)
        self.yaw = rospy.get_param("~yaw", 0.0)
        self.rate_hz = rospy.get_param("~rate_hz", 30.0)
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.publish_static_world = rospy.get_param("~publish_static_world", True)

        self._q = _yaw_quat(self.yaw)
        self._pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=2)
        self._tf_br = tf2_ros.TransformBroadcaster()
        if self.publish_static_world:
            self._latch_world_chain()

        rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._tick, oneshot=False
        )
        rospy.loginfo(
            "static odom @ (%.2f, %.2f, %.2f) yaw=%.2f -> %s",
            self.x,
            self.y,
            self.z,
            self.yaw,
            self.odom_topic,
        )

    def _latch_world_chain(self):
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

    def _tick(self, _evt):
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = self._q
        self._pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation = self._q
        self._tf_br.sendTransform(tf)


if __name__ == "__main__":
    try:
        StaticOdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass