#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
播 bag + path 桥时的 EGO 指标汇总。

旧脚本常用「odom 终态 vs goal」判到达；RViz 里紫球/红绿线多来自 poscmd_to_viz_odom，
与 path_to_odom_bridge 的 odom 不是同一条轨迹 → 会「界面到了、脚本没到」。

arrival_mode（~arrival_mode）：
  recorded  — 用录制 odom（/mavros/odom_convert），真轨迹是否到 goal（bag 回放通常 FAIL）
  plan_end  — 用 optimal_list 末端 vs goal（推荐：评规划是否朝 goal 画到位）
  viz_track — 用 /viz/odom 末端 vs goal（仅当开了 poscmd_to_viz_odom）

用法：
  rosrun uav_pkg ego_bag_plan_metrics.py _goal_x:=12.7 _goal_y:=-34.5 _goal_z:=1.0 \
    _arrival_mode:=plan_end
  # Ctrl+C 结束采集后打印 summary
"""
from __future__ import print_function

import math
import sys

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


def _dist2d(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _path_len_odom(samples):
    if len(samples) < 2:
        return 0.0
    s = 0.0
    for i in range(1, len(samples)):
        x0, y0, z0 = samples[i - 1]
        x1, y1, z1 = samples[i]
        s += math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
    return s


def _marker_endpoint(marker):
    if marker.points:
        p = marker.points[-1]
        return p.x, p.y, p.z
    p = marker.pose.position
    return p.x, p.y, p.z


class EgoBagPlanMetrics(object):
    def __init__(self):
        rospy.init_node("ego_bag_plan_metrics")
        self.goal = (
            rospy.get_param("~goal_x", 0.0),
            rospy.get_param("~goal_y", 0.0),
            rospy.get_param("~goal_z", 1.0),
        )
        self.arrival_mode = rospy.get_param("~arrival_mode", "plan_end")
        self.goal_eps = rospy.get_param("~goal_eps_m", 0.8)
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.viz_odom_topic = rospy.get_param("~viz_odom_topic", "/viz/odom")

        self._bspline_count = 0
        self._marker_updates = 0
        self._last_marker = None
        self._odom_trail = []
        self._viz_trail = []
        self._last_odom = None
        self._last_viz = None
        self.bspline_topic = rospy.get_param(
            "~bspline_topic", "/planning/bspline"
        )

        try:
            from ego_planner.msg import Bspline

            rospy.Subscriber(
                self.bspline_topic,
                Bspline,
                self._bspline_cb,
                queue_size=50,
            )
            self._bspline_ok = True
        except Exception:
            rospy.logwarn(
                "no Bspline on %s; 规划次数用 optimal_list 更新次数",
                self.bspline_topic,
            )
            self._bspline_ok = False
        rospy.Subscriber(
            "/ego_planner_node/optimal_list",
            Marker,
            self._marker_cb,
            queue_size=10,
        )
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=30)
        rospy.Subscriber(
            self.viz_odom_topic, Odometry, self._viz_cb, queue_size=30
        )
        rospy.on_shutdown(self._print_summary)
        rospy.loginfo(
            "ego_bag_plan_metrics goal=(%.2f,%.2f,%.2f) arrival_mode=%s",
            self.goal[0],
            self.goal[1],
            self.goal[2],
            self.arrival_mode,
        )

    def _bspline_cb(self, _msg):
        self._bspline_count += 1

    def _marker_cb(self, msg):
        self._last_marker = msg
        self._marker_updates += 1

    def _sample_odom(self, trail, msg):
        p = msg.pose.pose.position
        if trail:
            x0, y0, z0 = trail[-1]
            if (
                abs(p.x - x0) < 1e-4
                and abs(p.y - y0) < 1e-4
                and abs(p.z - z0) < 1e-4
            ):
                return
        trail.append((p.x, p.y, p.z))

    def _odom_cb(self, msg):
        self._last_odom = msg
        self._sample_odom(self._odom_trail, msg)

    def _viz_cb(self, msg):
        self._last_viz = msg
        self._sample_odom(self._viz_trail, msg)

    def _planned_polyline_len(self):
        if self._last_marker is None:
            return 0.0
        pts = self._last_marker.points
        if len(pts) < 2:
            return 0.0
        s = 0.0
        for i in range(1, len(pts)):
            s += math.sqrt(
                (pts[i].x - pts[i - 1].x) ** 2
                + (pts[i].y - pts[i - 1].y) ** 2
                + (pts[i].z - pts[i - 1].z) ** 2
            )
        return s

    def _arrival(self):
        mode = self.arrival_mode
        if mode == "viz_track":
            if self._last_viz is None:
                return False, float("nan"), "no /viz/odom"
            p = self._last_viz.pose.pose.position
            err = _dist2d(p.x, p.y, self.goal[0], self.goal[1])
            ok = err <= self.goal_eps
            return ok, err, "viz odom (%s)" % self.viz_odom_topic
        if mode == "plan_end":
            if self._last_marker is None:
                return False, float("nan"), "no optimal_list"
            ex, ey, ez = _marker_endpoint(self._last_marker)
            err = _dist2d(ex, ey, self.goal[0], self.goal[1])
            ok = err <= self.goal_eps
            return ok, err, "optimal_list last point"
        # recorded
        if self._last_odom is None:
            return False, float("nan"), "no odom"
        p = self._last_odom.pose.pose.position
        err = _dist2d(p.x, p.y, self.goal[0], self.goal[1])
        ok = err <= self.goal_eps
        return ok, err, "recorded odom (%s)" % self.odom_topic

    def _print_summary(self):
        ok, err, src = self._arrival()
        act_len = _path_len_odom(self._odom_trail)
        viz_len = _path_len_odom(self._viz_trail)
        plan_len = self._planned_polyline_len()

        print("\n======== EGO metrics summary (UAV) ========")
        plan_n = (
            self._bspline_count
            if getattr(self, "_bspline_ok", False)
            else self._marker_updates
        )
        print("Planning count 规划次数: %d" % plan_n)
        print("Replan count 重规划次数: %d" % max(0, plan_n - 1))
        print("Planned path (last ctrl polyline) m: %.2f" % plan_len)
        print("Actual path length (recorded odom) m: %.2f" % act_len)
        if self._viz_trail:
            print("Viz track path length (/viz/odom) m: %.2f" % viz_len)
        print("Goal (x,y,z): (%.2f, %.2f, %.2f)" % self.goal)
        if math.isnan(err):
            print("Arrival 到达结果: FAIL (%s)" % src)
        elif ok:
            print(
                "Arrival 到达结果: OK (%s, horiz err %.2fm)"
                % (src, err)
            )
        else:
            print(
                "Arrival 到达结果: FAIL (not reached, %s horiz err %.2fm)"
                % (src, err)
            )
        print("arrival_mode=%s  goal_eps=%.2fm" % (self.arrival_mode, self.goal_eps))
        print("注: bag+path 回放时 recorded 到达常 FAIL；评规划请用 arrival_mode:=plan_end")
        print("===========================================\n")
        sys.stdout.flush()


if __name__ == "__main__":
    try:
        EgoBagPlanMetrics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass