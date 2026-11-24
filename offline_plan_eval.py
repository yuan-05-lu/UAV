#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
离线评指标：按 eval_tasks.yaml 发 goal，记录规划路径长度与耗时（不循环 rosbag）。

前提：use_sim_time=false、static_odom_publisher、pcd_map_publisher、EGO 已启动。

用法：
  python offline_plan_eval.py _config:=/path/to/UAV/config/eval_tasks_sbs01.yaml
"""
from __future__ import print_function

import csv
import math
import os
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker


def _yaw_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _path_length(pts):
    if len(pts) < 2:
        return 0.0
    s = 0.0
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        dz = pts[i][2] - pts[i - 1][2]
        s += math.sqrt(dx * dx + dy * dy + dz * dz)
    return s


def _marker_points(marker):
    pts = []
    for p in marker.points:
        pts.append((p.x, p.y, p.z))
    if not pts and marker.pose.position:
        pts.append(
            (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            )
        )
    return pts


class OfflinePlanEval(object):
    def __init__(self):
        rospy.init_node("offline_plan_eval")
        cfg_path = rospy.get_param(
            "~config",
            os.path.join(os.path.dirname(__file__), "config/eval_tasks_sbs01.yaml"),
        )
        with open(cfg_path, "r") as fh:
            self.cfg = yaml.safe_load(fh)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1
        )
        self.out_csv = rospy.get_param(
            "~out_csv",
            os.path.splitext(cfg_path)[0] + "_results.csv",
        )
        self.wait_sec = rospy.get_param("~wait_after_goal", 4.0)
        self._last_marker = None
        rospy.Subscriber(
            "/ego_planner_node/optimal_list",
            Marker,
            self._marker_cb,
            queue_size=5,
        )

    def _marker_cb(self, msg):
        self._last_marker = msg

    def _publish_goal(self, goal):
        ps = PoseStamped()
        ps.header.frame_id = self.cfg.get("frame_id", "world")
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = goal["x"]
        ps.pose.position.y = goal["y"]
        ps.pose.position.z = goal.get("z", 1.0)
        ps.pose.orientation = _yaw_quat(goal.get("yaw", 0.0))
        self.goal_pub.publish(ps)

    def run(self):
        scene = self.cfg.get("scene", "unknown")
        rows = []
        for task in self.cfg.get("tasks", []):
            tid = task.get("id", "T?")
            rospy.loginfo("task %s: publish goal", tid)
            self._last_marker = None
            t0 = time.time()
            self._publish_goal(task["goal"])
            rospy.sleep(self.wait_sec)
            plan_sec = time.time() - t0

            marker = self._last_marker
            if marker is None:
                rospy.logwarn("task %s: no optimal_list received", tid)
                rows.append(
                    {
                        "scene": scene,
                        "task": tid,
                        "success": 0,
                        "plan_wall_sec": round(plan_sec, 3),
                        "path_length_m": "",
                        "goal_dist_m": "",
                    }
                )
                continue

            pts = _marker_points(marker)
            plen = _path_length(pts)
            goal = task["goal"]
            if pts:
                ex, ey, ez = pts[-1]
            else:
                ex = ey = ez = 0.0
            gx, gy, gz = goal["x"], goal["y"], goal.get("z", 1.0)
            gdist = math.sqrt(
                (ex - gx) ** 2 + (ey - gy) ** 2 + (ez - gz) ** 2
            )
            eps = float(self.cfg.get("goal_eps_m", 0.5))
            success = int(gdist <= eps and plen > 0.05)

            rows.append(
                {
                    "scene": scene,
                    "task": tid,
                    "success": success,
                    "plan_wall_sec": round(plan_sec, 3),
                    "path_length_m": round(plen, 3),
                    "goal_dist_m": round(gdist, 3),
                }
            )
            rospy.loginfo(
                "task %s: len=%.2f goal_dist=%.2f success=%s",
                tid,
                plen,
                gdist,
                success,
            )

        fields = ["scene", "task", "success", "plan_wall_sec", "path_length_m", "goal_dist_m"]
        with open(self.out_csv, "w") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
        rospy.loginfo("wrote %s", self.out_csv)


if __name__ == "__main__":
    try:
        OfflinePlanEval().run()
    except rospy.ROSInterruptException:
        pass