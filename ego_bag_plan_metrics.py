#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
与终端「EGO metrics summary」同款输出（bag + path 桥 / 仿真通用）。

指标含义与旧脚本对齐：
  - 规划次数 / 重规划：Bspline 发布次数（无则用 optimal_list 更新次数）
  - 规划耗时：每次 Bspline 相对上一帧的 wall 间隔（ms）平均/最大
  - 规划路径长度：最后一帧 optimal_list 折线长
  - 实际路径长度：odom 轨迹弧长（默认 /mavros/odom_convert）
  - 最小障碍距离：轨迹点与点云最近距离的最小值
  - 到达：录制 odom 水平距 goal（与旧脚本一致，非 RViz /viz 轨迹）

用法：
  python ego_bag_plan_metrics.py _goal_x:=12.7 _goal_y:=-34.5 _goal_z:=1.0
  # 发 goal 后开始采；Ctrl+C 或 _auto_stop_sec:=30 结束并打印 summary

可选 _arrival_mode:=plan_end|viz_track 仅用于对照 RViz，不改变默认打印字段。
"""
from __future__ import print_function

import math
import sys
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker


def _dist2d(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _path_len(samples):
    if len(samples) < 2:
        return 0.0
    s = 0.0
    for i in range(1, len(samples)):
        x0, y0, z0 = samples[i - 1]
        x1, y1, z1 = samples[i]
        s += math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)
    return s


def _marker_polyline(marker):
    if marker is None:
        return []
    return [(p.x, p.y, p.z) for p in marker.points]


def _min_dist_traj_to_cloud(traj, cloud_pts, stride=3):
    """traj: [(x,y,z),...]; cloud_pts list with stride subsample."""
    if not traj or not cloud_pts:
        return float("nan")
    obs = cloud_pts[:: max(1, stride)]
    best = 1e9
    for x, y, z in traj[:: max(1, stride)]:
        for ox, oy, oz in obs:
            d = math.sqrt((x - ox) ** 2 + (y - oy) ** 2 + (z - oz) ** 2)
            if d < best:
                best = d
    return best if best < 1e8 else float("nan")


class EgoMetricsSummary(object):
    def __init__(self):
        rospy.init_node("ego_bag_plan_metrics")
        self.goal = (
            rospy.get_param("~goal_x", 0.0),
            rospy.get_param("~goal_y", 0.0),
            rospy.get_param("~goal_z", 1.0),
        )
        self.goal_eps = rospy.get_param("~goal_eps_m", 0.5)
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/ego/points_world")
        self.bspline_topic = rospy.get_param("~bspline_topic", "/planning/bspline")
        self.arrival_mode = rospy.get_param(
            "~arrival_mode", "recorded"
        )
        self.auto_stop_sec = rospy.get_param("~auto_stop_sec", 0.0)
        self.cloud_stride = int(rospy.get_param("~cloud_stride", 4))
        self.traj_stride = int(rospy.get_param("~traj_stride", 2))

        self._bspline_count = 0
        self._marker_updates = 0
        self._last_marker = None
        self._odom_trail = []
        self._last_odom = None
        self._cloud_pts = []
        self._plan_latencies_ms = []
        self._last_bspline_wall = None
        self._session_start = time.time()
        self._bspline_ok = False

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
                "Bspline unavailable on %s; plan count from optimal_list",
                self.bspline_topic,
            )

        rospy.Subscriber(
            "/ego_planner_node/optimal_list",
            Marker,
            self._marker_cb,
            queue_size=10,
        )
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=30)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self._cloud_cb, queue_size=2)
        rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self._goal_cb,
            queue_size=5,
        )

        if self.goal_eps and abs(self.goal[0]) + abs(self.goal[1]) < 1e-6:
            rospy.logwarn(
                "goal 为默认 (0,0); 请设 _goal_x/_goal_y/_goal_z 或与 Nav Goal 一致"
            )

        if self.auto_stop_sec > 0:
            rospy.Timer(
                rospy.Duration(self.auto_stop_sec),
                self._auto_stop,
                oneshot=True,
            )

        rospy.on_shutdown(self._print_summary)
        rospy.loginfo(
            "ego metrics: goal=(%.2f,%.2f,%.2f) odom=%s cloud=%s",
            self.goal[0],
            self.goal[1],
            self.goal[2],
            self.odom_topic,
            self.cloud_topic,
        )

    def _goal_cb(self, msg):
        self.goal = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self._last_bspline_wall = None
        rospy.loginfo(
            "goal from RViz: (%.2f, %.2f, %.2f)", self.goal[0], self.goal[1], self.goal[2]
        )

    def _bspline_cb(self, _msg):
        now = time.time()
        if self._last_bspline_wall is not None:
            self._plan_latencies_ms.append((now - self._last_bspline_wall) * 1000.0)
        elif self._session_start:
            self._plan_latencies_ms.append((now - self._session_start) * 1000.0)
        self._last_bspline_wall = now
        self._bspline_count += 1

    def _marker_cb(self, msg):
        self._last_marker = msg
        self._marker_updates += 1

    def _sample_odom(self, msg):
        p = msg.pose.pose.position
        if self._odom_trail:
            x0, y0, z0 = self._odom_trail[-1]
            if (
                abs(p.x - x0) < 1e-4
                and abs(p.y - y0) < 1e-4
                and abs(p.z - z0) < 1e-4
            ):
                return
        self._odom_trail.append((p.x, p.y, p.z))

    def _odom_cb(self, msg):
        self._last_odom = msg
        self._sample_odom(msg)

    def _cloud_cb(self, msg):
        pts = []
        for x, y, z in pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            pts.append((x, y, z))
        self._cloud_pts = pts

    def _auto_stop(self, _evt):
        rospy.loginfo("auto_stop_sec reached, printing summary")
        self._print_summary()
        rospy.signal_shutdown("metrics done")

    def _horiz_arrival_recorded(self):
        if self._last_odom is None:
            return False, float("nan")
        p = self._last_odom.pose.pose.position
        err = _dist2d(p.x, p.y, self.goal[0], self.goal[1])
        return err <= self.goal_eps, err

    def _print_summary(self):
        if getattr(self, "_printed", False):
            return
        self._printed = True

        plan_n = self._bspline_count if self._bspline_ok else self._marker_updates
        plan_len = _path_len(_marker_polyline(self._last_marker))
        act_len = _path_len(self._odom_trail)

        if self._plan_latencies_ms:
            lat_avg = sum(self._plan_latencies_ms) / len(self._plan_latencies_ms)
            lat_max = max(self._plan_latencies_ms)
        else:
            lat_avg = lat_max = 0.0

        check_traj = list(self._odom_trail)
        if self._last_marker is not None:
            check_traj.extend(_marker_polyline(self._last_marker))
        min_obs = _min_dist_traj_to_cloud(
            check_traj, self._cloud_pts, stride=self.cloud_stride
        )
        if math.isnan(min_obs):
            min_obs = 0.0

        ok, err = self._horiz_arrival_recorded()
        if self.arrival_mode != "recorded" and self._last_marker is not None:
            pts = _marker_polyline(self._last_marker)
            if pts:
                ex, ey, _ = pts[-1]
                err = _dist2d(ex, ey, self.goal[0], self.goal[1])
                ok = err <= self.goal_eps

        print("")
        print("======== EGO metrics summary ========")
        print(
            "Planning count (bspline pub) 规划次数: %d" % plan_n
        )
        print("Replan count (count - 1) 重规划次数: %d" % max(0, plan_n - 1))
        print(
            "Plan latency avg / max ms 规划耗时(平均/最大): %.1f / %.1f"
            % (lat_avg, lat_max)
        )
        print(
            "Planned path (last ctrl polyline) m 规划路径长度: %.2f" % plan_len
        )
        print(
            "Actual path length (odom) m 实际路径长度: %.2f" % act_len
        )
        print("Min obstacle distance m 最小障碍距离: %.3f" % min_obs)
        if math.isnan(err):
            print("Arrival 到达结果: FAIL (no odom)")
        elif ok:
            print("Arrival 到达结果: OK (horiz err %.2fm)" % err)
        else:
            print(
                "Arrival 到达结果: FAIL (not reached, horiz err %.2fm)" % err
            )
        print(
            "Goal (x,y,z) 目标点: (%.2f, %.2f, %.2f)"
            % (self.goal[0], self.goal[1], self.goal[2])
        )
        print("=====================================")
        print("")
        sys.stdout.flush()


if __name__ == "__main__":
    try:
        EgoMetricsSummary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass