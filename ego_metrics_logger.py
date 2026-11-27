#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
EGO 规划指标（与 Gazebo 下 ego_metrics_logger 同款输出 + CSV）。

适用于：播 rosbag + path_to_odom_bridge + EGO（含 flight_type=2 航点模式）。

与原版一致：
  - 规划次数 / 耗时 / 规划折线长 / 实际 odom 弧长 / 最小障碍距离 / 到达判定
  - arrive_horiz、arrive_z 阈值

Rosbag 差异（参数可选，不改默认指标名）：
  - goal_source=auto：flight_type=2 时取 EGO fsm **最后一个航点** 为任务终点
    （不再要求手写 _goal_x/_goal_y/_goal_z）
  - flight_type=1 且 auto_goal_from_topic 时仍用 Nav Goal
  - 若用 poscmd_to_viz_odom 演示且希望到达与 RViz 紫球一致：
    _odom_topic:=/viz/odom（默认仍为 /mavros/odom_convert，与 Gazebo 一致）

用法：
  python ego_metrics_logger.py
  python ego_metrics_logger.py _csv_path:=/tmp/ego_run.csv _auto_stop_sec:=0
  # 航点模式 launch 已设 fsm/waypoint* 时无需再设 goal
"""
from __future__ import print_function

import csv
import math
import os
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

try:
    from ego_planner.msg import Bspline
except ImportError:
    Bspline = None


def dist3(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


class EgoMetricsLogger(object):
    def __init__(self):
        rospy.init_node("ego_metrics_logger")

        self.bspline_topic = rospy.get_param("~bspline_topic", "/planning/bspline")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.occ_topic = rospy.get_param(
            "~occ_topic", "/grid_map/occupancy_inflate"
        )
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/move_base_simple/goal"
        )
        self.goal_source = rospy.get_param("~goal_source", "auto")
        self.fsm_ns = rospy.get_param("~fsm_param_ns", "/ego_planner_node/fsm")
        self.goal_point_topic = rospy.get_param(
            "~goal_point_topic", "/ego_planner_node/goal_point"
        )

        self.auto_goal = rospy.get_param("~auto_goal_from_topic", True)
        self.arrive_h = rospy.get_param("~arrive_horiz", 1.2)
        self.arrive_z = rospy.get_param("~arrive_z", 0.35)

        self.goal = (
            rospy.get_param("~goal_x", 0.0),
            rospy.get_param("~goal_y", 0.0),
            rospy.get_param("~goal_z", 1.0),
        )

        self.csv_path = rospy.get_param("~csv_path", "")
        self.auto_stop_sec = float(rospy.get_param("~auto_stop_sec", 0.0))

        self.bspline_count = 0
        self.plan_durations_ms = []
        self.pending_plan_since = None
        self.last_planned_len = 0.0
        self.actual_path_len = 0.0
        self.min_clearance = float("inf")
        self.arrived = False
        self.final_err = None
        self.mission_goal_set = False
        self.last_odom = None
        self.occ_points = []
        self._printed = False
        self._session_start = rospy.Time.now()

        if Bspline is not None:
            rospy.Subscriber(
                self.bspline_topic, Bspline, self._bspline_cb, queue_size=50
            )
        else:
            rospy.logwarn(
                "ego_planner.msg.Bspline not found; plan count/latency/len disabled"
            )

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber(self.occ_topic, PointCloud2, self._occ_cb, queue_size=2)

        if self.auto_goal and self.goal_source in ("auto", "nav_goal"):
            rospy.Subscriber(
                self.goal_topic, PoseStamped, self._goal_cb, queue_size=5
            )

        if self.goal_source in ("auto", "ego_goal_point"):
            rospy.Subscriber(
                self.goal_point_topic, Marker, self._goal_point_cb, queue_size=5
            )

        rospy.Timer(rospy.Duration(2.0), self._try_load_fsm_waypoints, oneshot=True)
        rospy.Timer(
            rospy.Duration(5.0), self._try_load_fsm_waypoints, oneshot=False
        )

        if self.auto_stop_sec > 0:
            rospy.Timer(
                rospy.Duration(self.auto_stop_sec),
                self._auto_stop,
                oneshot=True,
            )

        rospy.on_shutdown(self._report)
        self._log_goal_state("init")

    def _log_goal_state(self, tag):
        rospy.loginfo(
            "[%s] mission_goal_set=%s goal=(%.2f,%.2f,%.2f) odom=%s "
            "goal_source=%s",
            tag,
            self.mission_goal_set,
            self.goal[0],
            self.goal[1],
            self.goal[2],
            self.odom_topic,
            self.goal_source,
        )

    def _load_fsm_waypoints(self):
        try:
            flight_type = int(rospy.get_param(self.fsm_ns + "/flight_type", 1))
            n = int(rospy.get_param(self.fsm_ns + "/waypoint_num", 0))
        except (KeyError, ValueError, rospy.ROSException):
            return flight_type, []

        wps = []
        for i in range(n):
            try:
                x = float(rospy.get_param(self.fsm_ns + "/waypoint%d_x" % i))
                y = float(rospy.get_param(self.fsm_ns + "/waypoint%d_y" % i))
                z = float(rospy.get_param(self.fsm_ns + "/waypoint%d_z" % i))
                wps.append((x, y, z))
            except (KeyError, ValueError, rospy.ROSException):
                break
        return flight_type, wps

    def _should_use_fsm_goal(self, flight_type, wps):
        if not wps:
            return False
        if self.goal_source == "fsm_waypoints":
            return True
        if self.goal_source == "auto" and flight_type == 2:
            return True
        return False

    def _try_load_fsm_waypoints(self, _evt=None):
        flight_type, wps = self._load_fsm_waypoints()
        if not self._should_use_fsm_goal(flight_type, wps):
            return
        last = wps[-1]
        self.goal = last
        self.mission_goal_set = True
        self.pending_plan_since = rospy.Time.now()
        self.arrived = False
        rospy.loginfo(
            "FSM waypoint mission (%d pts, flight_type=%d); "
            "task goal = last waypoint (%.2f, %.2f, %.2f)",
            len(wps),
            flight_type,
            last[0],
            last[1],
            last[2],
        )

    def _set_mission_goal(self, x, y, z, source):
        self.goal = (x, y, z)
        self.mission_goal_set = True
        self.pending_plan_since = rospy.Time.now()
        self.arrived = False
        self.final_err = None
        rospy.loginfo(
            "Mission goal from %s: (%.2f, %.2f, %.2f); planning timer started",
            source,
            x,
            y,
            z,
        )

    def _goal_cb(self, msg):
        flight_type, wps = self._load_fsm_waypoints()
        if self._should_use_fsm_goal(flight_type, wps):
            return
        p = msg.pose.position
        self._set_mission_goal(p.x, p.y, p.z, "nav_goal")

    def _goal_point_cb(self, msg):
        flight_type, wps = self._load_fsm_waypoints()
        if self._should_use_fsm_goal(flight_type, wps):
            return
        p = msg.pose.position
        if p.x == 0.0 and p.y == 0.0 and p.z == 0.0 and not msg.points:
            return
        self._set_mission_goal(p.x, p.y, p.z, "ego_goal_point")

    def _bspline_cb(self, msg):
        now = rospy.Time.now()
        if self.pending_plan_since is not None:
            dt = (now - self.pending_plan_since).to_sec() * 1000.0
            if dt >= 0:
                self.plan_durations_ms.append(dt)
        self.pending_plan_since = now
        self.bspline_count += 1

        pts = msg.pos_pts
        length = 0.0
        prev = None
        for pt in pts:
            cur = (pt.x, pt.y, pt.z)
            if prev is not None:
                length += dist3(prev, cur)
            prev = cur
        self.last_planned_len = length

        rospy.loginfo(
            "Plan #%d latency=%.1fms traj_id=%s ctrl_pts=%d",
            self.bspline_count,
            self.plan_durations_ms[-1] if self.plan_durations_ms else 0.0,
            getattr(msg, "traj_id", "?"),
            len(pts),
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        cur = (p.x, p.y, p.z)
        if self.last_odom is not None:
            self.actual_path_len += dist3(self.last_odom, cur)
        self.last_odom = cur
        self._update_clearance(cur)

        if self.mission_goal_set and not self.arrived:
            h = math.hypot(cur[0] - self.goal[0], cur[1] - self.goal[1])
            dz = abs(cur[2] - self.goal[2])
            if h <= self.arrive_h and dz <= self.arrive_z:
                self.arrived = True
                self.final_err = (h, dz)
                rospy.loginfo(
                    "Arrival OK horiz=%.2fm dz=%.2fm", h, dz
                )

    def _occ_cb(self, msg):
        try:
            self.occ_points = list(
                pc2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            )
        except Exception as err:
            rospy.logwarn_throttle(5.0, "occ cloud parse: %s", err)

    def _update_clearance(self, pos):
        if not self.occ_points:
            return
        px, py, pz = pos
        best = float("inf")
        stride = 3
        for i, (x, y, z) in enumerate(self.occ_points):
            if i % stride:
                continue
            d = math.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2)
            if d < best:
                best = d
        if best < 50.0:
            self.min_clearance = min(self.min_clearance, best)

    def _auto_stop(self, _evt):
        self._report()
        rospy.signal_shutdown("metrics auto_stop")

    def _report(self):
        if self._printed:
            return
        self._printed = True

        replans = max(0, self.bspline_count - 1)
        if self.plan_durations_ms:
            avg_plan = sum(self.plan_durations_ms) / len(self.plan_durations_ms)
            max_plan = max(self.plan_durations_ms)
        else:
            avg_plan = max_plan = float("nan")

        if self.arrived:
            arrival = "SUCCESS"
        elif self.mission_goal_set and self.last_odom is not None:
            h = math.hypot(
                self.last_odom[0] - self.goal[0],
                self.last_odom[1] - self.goal[1],
            )
            arrival = "FAIL (not reached, horiz err %.2fm)" % h
        else:
            arrival = "UNKNOWN (no goal)"

        min_clr = (
            self.min_clearance
            if math.isfinite(self.min_clearance)
            else float("nan")
        )

        lines = [
            "",
            "======== EGO metrics summary ========",
            "Planning count (bspline pub) 规划次数: %d" % self.bspline_count,
            "Replan count (count - 1) 重规划次数: %d" % replans,
            "Plan latency avg / max ms 规划耗时 (平均/最大) : %.1f / %.1f"
            % (avg_plan, max_plan),
            "Planned path (last ctrl polyline) m 规划路径长度: %.2f"
            % self.last_planned_len,
            "Actual path length (odom) m 实际路径长度: %.2f"
            % self.actual_path_len,
            "Min obstacle distance m 最小障碍距离: %s"
            % ("%.3f" % min_clr if math.isfinite(min_clr) else "N/A"),
            "Arrival 到达结果: %s" % arrival,
            "Goal (x,y,z) 目标点: (%.2f, %.2f, %.2f)"
            % (self.goal[0], self.goal[1], self.goal[2]),
            "=====================================",
            "",
        ]
        text = "\n".join(lines)
        print(text)
        rospy.loginfo("\n%s", text)

        if self.csv_path:
            row = {
                "bspline_count": self.bspline_count,
                "replan_count": replans,
                "plan_latency_avg_ms": avg_plan,
                "plan_latency_max_ms": max_plan,
                "planned_polyline_m": self.last_planned_len,
                "actual_path_m": self.actual_path_len,
                "min_clearance_m": min_clr,
                "arrival": arrival,
                "goal_x": self.goal[0],
                "goal_y": self.goal[1],
                "goal_z": self.goal[2],
            }
            write_header = not os.path.isfile(self.csv_path)
            with open(self.csv_path, "a") as fh:
                w = csv.DictWriter(fh, fieldnames=list(row.keys()))
                if write_header:
                    w.writeheader()
                w.writerow(row)
            rospy.loginfo("wrote csv row -> %s", self.csv_path)

        sys.stdout.flush()


if __name__ == "__main__":
    try:
        EgoMetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass