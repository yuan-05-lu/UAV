#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Rosbag + EGO（flight_type=1，无 waypoint_generator）：

从 launch 写入的 /ego_planner_node/fsm/waypoint* 读航点，依次发 /move_base_simple/goal。
launch：run_ego_bag_scripted.launch（只改 point_num / point*_*）。

可选 ~config:=xxx.yaml 覆盖 launch 航点。

指标：先起 ego_plan_metrics.py，再跑本脚本；终点为 /waypoint_mission/final_goal。

用法：
  python waypoint_mission_bag.py
  python waypoint_mission_bag.py _config:=config/waypoints_sbs01_bag.yaml
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


def _load_from_fsm(fsm_ns):
    n = int(rospy.get_param(fsm_ns + "/waypoint_num", 0))
    wps = []
    for i in range(n):
        x = float(rospy.get_param(fsm_ns + "/waypoint%d_x" % i))
        y = float(rospy.get_param(fsm_ns + "/waypoint%d_y" % i))
        z = float(rospy.get_param(fsm_ns + "/waypoint%d_z" % i))
        wps.append({"x": x, "y": y, "z": z, "yaw": 0.0})
    return wps


class WaypointMissionBag(object):
    def __init__(self):
        rospy.init_node("waypoint_mission_bag")
        self.fsm_ns = rospy.get_param("~fsm_param_ns", "/ego_planner_node/fsm")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.arrive_h = float(rospy.get_param("~arrive_horiz", 1.2))
        self.arrive_z = float(rospy.get_param("~arrive_z", 0.35))
        self.dwell = float(rospy.get_param("~dwell_sec", 0.5))
        self.odom_topic = rospy.get_param("~odom_topic", "/viz/odom")
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/move_base_simple/goal"
        )

        cfg_path = rospy.get_param("~config", "")
        if cfg_path and os.path.isfile(cfg_path):
            with open(cfg_path, "r") as fh:
                cfg = yaml.safe_load(fh)
            self.frame_id = cfg.get("frame_id", self.frame_id)
            self.arrive_h = float(cfg.get("arrive_horiz", self.arrive_h))
            self.arrive_z = float(cfg.get("arrive_z", self.arrive_z))
            self.dwell = float(cfg.get("dwell_sec", self.dwell))
            self.odom_topic = rospy.get_param(
                "~odom_topic", cfg.get("odom_topic", self.odom_topic)
            )
            self.waypoints = cfg.get("waypoints", [])
            rospy.loginfo("waypoints from yaml: %s", cfg_path)
        else:
            rospy.sleep(1.0)
            self.waypoints = _load_from_fsm(self.fsm_ns)
            rospy.loginfo(
                "waypoints from %s waypoint_num=%d",
                self.fsm_ns,
                len(self.waypoints),
            )

        if not self.waypoints:
            raise rospy.ROSException("no waypoints (launch fsm or ~config)")

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
                rospy.sleep(self.dwell)
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rate.sleep()
        return False

    def run(self):
        timeout = float(rospy.get_param("~wp_timeout_sec", 180.0))
        if not self._wait_odom(min(timeout, 30.0)):
            rospy.logerr("no odom on %s", self.odom_topic)
            return

        final = self.waypoints[-1]
        self._final_pub.publish(self._pose_goal(final))
        rospy.loginfo(
            "final goal (%.2f, %.2f, %.2f)", final["x"], final["y"], final.get("z", 1.0)
        )

        for i, wp in enumerate(self.waypoints):
            rospy.loginfo(
                "waypoint %d/%d -> (%.2f, %.2f, %.2f)",
                i + 1,
                len(self.waypoints),
                wp["x"],
                wp["y"],
                wp.get("z", 1.0),
            )
            self._goal_pub.publish(self._pose_goal(wp))
            ok = self._wait_reach(wp, timeout)
            if not ok:
                rospy.logwarn("skip timeout at wp %d", i + 1)

        rospy.loginfo("mission done")


if __name__ == "__main__":
    try:
        WaypointMissionBag().run()
    except rospy.ROSInterruptException:
        pass