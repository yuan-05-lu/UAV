#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
收集 EGO rosbag 实验指标（与 Gazebo recorder 同款字段）：

  - 规划耗时（平均/最大 ms）
  - 规划次数 / 重规划次数
  - 规划路径长度（最后一帧 Bspline 控制折线）
  - 实际路径长度（odom 弧长）
  - 最小障碍距离（occupancy_inflate 点云）
  - 到达结果（相对任务终点）

任务终点：优先 /waypoint_mission/final_goal（waypoint_mission_bag.py），
否则 Nav Goal，否则 fsm 最后一航点。

用法：
  python ego_plan_metrics.py _csv_path:=/tmp/metrics.csv
  # 再跑 waypoint_mission_bag.py；任务结束 Ctrl+C 或等 bag 播完打 summary
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

try:
    from ego_planner.msg import Bspline
except ImportError:
    Bspline = None


def dist3(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


def _finite(x):
    """Python2 无 math.isfinite."""
    try:
        if x != x:
            return False
        if abs(x) == float("inf"):
            return False
        return True
    except (TypeError, ValueError):
        return False


class EgoPlanMetrics(object):
    def __init__(self):
        rospy.init_node("ego_plan_metrics")

        self.bspline_topic = rospy.get_param("~bspline_topic", "/planning/bspline")
        self.odom_topic = rospy.get_param("~odom_topic", "/viz/odom")
        self.occ_topic = rospy.get_param(
            "~occ_topic", "/grid_map/occupancy_inflate"
        )
        self.arrive_h = rospy.get_param("~arrive_horiz", 1.2)
        self.arrive_z = rospy.get_param("~arrive_z", 0.35)
        self.csv_path = rospy.get_param("~csv_path", "")
        self.fsm_ns = rospy.get_param("~fsm_param_ns", "/ego_planner_node/fsm")

        self.goal = (0.0, 0.0, 1.0)
        self.mission_goal_set = False
        self.bspline_count = 0
        self.plan_durations_ms = []
        self.pending_plan_since = None
        self.last_planned_len = 0.0
        self.actual_path_len = 0.0
        self.min_clearance = float("inf")
        self.arrived = False
        self.last_odom = None
        self.occ_points = []
        self._printed = False

        if Bspline is not None:
            rospy.Subscriber(
                self.bspline_topic, Bspline, self._bspline_cb, queue_size=50
            )
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber(self.occ_topic, PointCloud2, self._occ_cb, queue_size=2)
        rospy.Subscriber(
            "/waypoint_mission/final_goal",
            PoseStamped,
            self._mission_goal_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self._nav_goal_cb,
            queue_size=5,
        )

        rospy.Timer(rospy.Duration(3.0), self._try_fsm_goal, oneshot=True)
        rospy.on_shutdown(self._report)

        rospy.loginfo(
            "ego_plan_metrics odom=%s bspline=%s", self.odom_topic, self.bspline_topic
        )

    def _set_goal(self, x, y, z, src):
        self.goal = (x, y, z)
        self.mission_goal_set = True
        self.pending_plan_since = rospy.Time.now()
        self.arrived = False
        rospy.loginfo("metrics goal from %s: (%.2f, %.2f, %.2f)", src, x, y, z)

    def _mission_goal_cb(self, msg):
        p = msg.pose.position
        self._set_goal(p.x, p.y, p.z, "waypoint_mission/final_goal")

    def _nav_goal_cb(self, msg):
        if self.mission_goal_set:
            return
        p = msg.pose.position
        self._set_goal(p.x, p.y, p.z, "nav_goal")

    def _try_fsm_goal(self, _evt):
        if self.mission_goal_set:
            return
        try:
            n = int(rospy.get_param(self.fsm_ns + "/waypoint_num", 0))
            if n < 1:
                return
            i = n - 1
            x = float(rospy.get_param(self.fsm_ns + "/waypoint%d_x" % i))
            y = float(rospy.get_param(self.fsm_ns + "/waypoint%d_y" % i))
            z = float(rospy.get_param(self.fsm_ns + "/waypoint%d_z" % i))
            self._set_goal(x, y, z, "fsm_last_waypoint")
        except (KeyError, ValueError, rospy.ROSException):
            pass

    def _bspline_cb(self, msg):
        now = rospy.Time.now()
        if self.pending_plan_since is not None:
            dt = (now - self.pending_plan_since).to_sec() * 1000.0
            if dt >= 0:
                self.plan_durations_ms.append(dt)
        self.pending_plan_since = now
        self.bspline_count += 1

        length = 0.0
        prev = None
        for pt in msg.pos_pts:
            cur = (pt.x, pt.y, pt.z)
            if prev is not None:
                length += dist3(prev, cur)
            prev = cur
        self.last_planned_len = length

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
                rospy.loginfo("Arrival OK horiz=%.2fm dz=%.2fm", h, dz)

    def _occ_cb(self, msg):
        try:
            self.occ_points = list(
                pc2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            )
        except Exception as err:
            rospy.logwarn_throttle(5.0, "occ: %s", err)

    def _update_clearance(self, pos):
        if not self.occ_points:
            return
        px, py, pz = pos
        best = float("inf")
        for i, (x, y, z) in enumerate(self.occ_points):
            if i % 3:
                continue
            d = math.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2)
            if d < best:
                best = d
        if best < 50.0:
            self.min_clearance = min(self.min_clearance, best)

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

        if _finite(self.min_clearance):
            min_clr = self.min_clearance
        else:
            min_clr = float("nan")

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
            % ("%.3f" % min_clr if _finite(min_clr) else "N/A"),
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
            new_file = not os.path.isfile(self.csv_path)
            with open(self.csv_path, "a") as fh:
                w = csv.DictWriter(fh, fieldnames=list(row.keys()))
                if new_file:
                    w.writeheader()
                w.writerow(row)

        sys.stdout.flush()


if __name__ == "__main__":
    try:
        EgoPlanMetrics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass