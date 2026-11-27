#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Rosbag + EGO：按 YAML 顺序发布 Nav Goal，完成航点任务。

前提（你已具备）：
  - 播 bag、path_to_odom_bridge、points_to_world、EGO、traj_server
  - flight_type=1（Nav Goal）；waypoint_generator 已起

本节点：
  1. 读 config/*.yaml 航点列表
  2. 在 /waypoint_mission/final_goal 发布**最后一航点**（给指标脚本判到达）
  3. 依次发 /move_base_simple/goal，直到 odom 进入 arrive 阈值

用法：
  python waypoint_mission_bag.py \\
    _config:=/path/to/UAV/config/waypoints_sbs01_bag.yaml

与指标一起：
  终端1: python ego_plan_metrics.py _csv_path:=/tmp/run.csv
  终端2: python waypoint_mission_bag.py _config:=...
"""
from __future__ import print_function

import math
import os

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry


def _yaw_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WaypointMissionBag(object):
    def __init__(self):
        rospy.init_node("waypoint_mission_bag")
        cfg_path = rospy.get_param(
            "~config",
            os.path.join(
                os.path.dirname(__file__), "config/waypoints_sbs01_bag.yaml"
            ),
        )
        with open(cfg_path, "r") as fh:
            self.cfg = yaml.safe_load(fh)

        self.frame_id = self.cfg.get("frame_id", "world")
        self.arrive_h = float(self.cfg.get("arrive_horiz", 1.2))
        self.arrive_z = float(self.cfg.get("arrive_z", 0.35))
        self.dwell = float(self.cfg.get("dwell_sec", 0.5))
        self.odom_topic = rospy.get_param(
            "~odom_topic", self.cfg.get("odom_topic", "/viz/odom")
        )
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/move_base_simple/goal"
        )
        self.waypoints = self.cfg.get("waypoints", [])
        if not self.waypoints:
            rospy.logerr("no waypoints in %s", cfg_path)
            raise rospy.ROSException("empty waypoints")

        self._odom = None
        self._goal_pub = rospy.Publisher(
            self.goal_topic, PoseStamped, queue_size=1, latch=True
        )
        self._final_pub = rospy.Publisher(
            "/waypoint_mission/final_goal",
            PoseStamped,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=30)

        rospy.loginfo(
            "mission: %d waypoints, odom=%s, cfg=%s",
            len(self.waypoints),
            self.odom_topic,
            cfg_path,
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._odom = (p.x, p.y, p.z)

    def _pose_goal(self, wp):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = wp["x"]
        ps.pose.position.y = wp["y"]
        ps.pose.position.z = wp.get("z", 1.0)
        ps.pose.orientation = _yaw_quat(wp.get("yaw", 0.0))
        return ps

    def _wait_odom(self, timeout):
        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self._odom is None:
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("timeout waiting odom on %s", self.odom_topic)
                return False
            rate.sleep()
        return True

    def _at_goal(self, wp):
        if self._odom is None:
            return False
        ox, oy, oz = self._odom
        h = math.hypot(ox - wp["x"], oy - wp["y"])
        dz = abs(oz - wp.get("z", 1.0))
        return h <= self.arrive_h and dz <= self.arrive_z

    def _wait_reach(self, wp, timeout):
        rate = rospy.Rate(10)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._at_goal(wp):
                rospy.loginfo(
                    "reached wp (%.2f, %.2f, %.2f)", wp["x"], wp["y"], wp.get("z", 1.0)
                )
                rospy.sleep(self.dwell)
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn(
                    "timeout wp (%.2f, %.2f, %.2f)", wp["x"], wp["y"], wp.get("z", 1.0)
                )
                return False
            rate.sleep()
        return False

    def run(self):
        timeout = float(rospy.get_param("~wp_timeout_sec", 120.0))
        if not self._wait_odom(min(timeout, 30.0)):
            return

        final = self.waypoints[-1]
        self._final_pub.publish(self._pose_goal(final))
        rospy.loginfo(
            "latched final_goal (%.2f, %.2f, %.2f)",
            final["x"],
            final["y"],
            final.get("z", 1.0),
        )

        for i, wp in enumerate(self.waypoints):
            rospy.loginfo(
                ">>> waypoint %d/%d (%.2f, %.2f, %.2f)",
                i + 1,
                len(self.waypoints),
                wp["x"],
                wp["y"],
                wp.get("z", 1.0),
            )
            self._goal_pub.publish(self._pose_goal(wp))
            self._wait_reach(wp, timeout)

        rospy.loginfo("mission finished (%d waypoints)", len(self.waypoints))


if __name__ == "__main__":
    try:
        WaypointMissionBag().run()
    except rospy.ROSInterruptException:
        pass