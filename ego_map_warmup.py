#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
静态全局 PCD 发布时，EGO grid_map 仍以当前 odom 为射线原点且只更新局部体素。
若 odom 一直停在任务起点，远处障碍不会被写入栅格 → 规划像在“空地”里飞、路径偏低。

本节点在正式评指标/发 goal 前，沿若干位姿依次发布 odom（地图预热），再落到起点。

用法（与 pcd_map_publisher、use_sim_time=false 同时）：
  python ego_map_warmup.py \\
    _pcd_file:=/path/sbs_01_global.pcd \\
    _finish_x:=15.8 _finish_y:=-6.31 _finish_z:=0.68
"""
import math
import os

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from pcd_map_publisher import load_pcd


def _yaw_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _bbox(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), max(xs), min(ys), max(ys)


def _grid_centers(x0, x1, y0, y1, n_x, n_y, z, margin):
    if n_x < 1:
        n_x = 1
    if n_y < 1:
        n_y = 1
    xs = [x0 + margin + (x1 - x0 - 2 * margin) * i / max(n_x - 1, 1) for i in range(n_x)]
    ys = [y0 + margin + (y1 - y0 - 2 * margin) * j / max(n_y - 1, 1) for j in range(n_y)]
    return [(x, y, z) for x in xs for y in ys]


class EgoMapWarmup(object):
    def __init__(self):
        rospy.init_node("ego_map_warmup")
        self.pcd_file = rospy.get_param("~pcd_file")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.dwell = rospy.get_param("~dwell_sec", 2.0)
        self.rate_hz = rospy.get_param("~rate_hz", 30.0)
        self.grid_nx = rospy.get_param("~grid_nx", 4)
        self.grid_ny = rospy.get_param("~grid_ny", 4)
        self.margin = rospy.get_param("~bbox_margin", 1.0)
        self.finish_x = rospy.get_param("~finish_x", 0.0)
        self.finish_y = rospy.get_param("~finish_y", 0.0)
        self.finish_z = rospy.get_param("~finish_z", 1.0)
        self.finish_yaw = rospy.get_param("~finish_yaw", 0.0)
        self.warmup_z = rospy.get_param("~warmup_z", self.finish_z)

        if not os.path.isfile(self.pcd_file):
            rospy.logfatal("missing pcd: %s", self.pcd_file)
            raise SystemExit(1)

        pts = load_pcd(self.pcd_file)
        x0, x1, y0, y1 = _bbox(pts)
        self._poses = _grid_centers(
            x0, x1, y0, y1, self.grid_nx, self.grid_ny, self.warmup_z, self.margin
        )
        self._poses.append((self.finish_x, self.finish_y, self.finish_z))

        self._pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=2)
        self._tf_br = tf2_ros.TransformBroadcaster()
        self._static_br = tf2_ros.StaticTransformBroadcaster()
        self._latch_world()
        rospy.loginfo(
            "warmup %d poses on bbox [%.1f,%.1f]x[%.1f,%.1f], finish=(%.2f,%.2f,%.2f)",
            len(self._poses),
            x0,
            x1,
            y0,
            y1,
            self.finish_x,
            self.finish_y,
            self.finish_z,
        )
        rospy.Timer(rospy.Duration(0.1), self._run_once, oneshot=True)

    def _latch_world(self):
        msgs = []
        for parent, child in ((self.world_frame, "map"), ("map", self.odom_frame)):
            t = TransformStamped()
            t.header.stamp = rospy.Time(0)
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            msgs.append(t)
        self._static_br.sendTransform(msgs)

    def _send(self, x, y, z, yaw):
        stamp = rospy.Time.now()
        q = _yaw_quat(yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = q
        self._pub.publish(odom)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation = q
        self._tf_br.sendTransform(tf)

    def _run_once(self, _evt):
        r = rospy.Rate(self.rate_hz)
        for i, (x, y, z) in enumerate(self._poses):
            rospy.loginfo("warmup pose %d/%d (%.2f, %.2f, %.2f)", i + 1, len(self._poses), x, y, z)
            t_end = rospy.Time.now() + rospy.Duration(self.dwell)
            while rospy.Time.now() < t_end and not rospy.is_shutdown():
                self._send(x, y, z, self.finish_yaw)
                r.sleep()
        rospy.logwarn(
            "warmup done @ finish pose. NOTE: EGO map scrolls with odom — "
            "cells fused far away may clear when returning here; "
            "prefer large local_update_range + suggest_map_params.py"
        )
        rospy.loginfo("keep publishing finish pose for planning")
        rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz),
            lambda e: self._send(
                self.finish_x, self.finish_y, self.finish_z, self.finish_yaw
            ),
            oneshot=False,
        )


if __name__ == "__main__":
    try:
        EgoMapWarmup()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass