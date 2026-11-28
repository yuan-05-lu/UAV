#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
航点任务（与 Gazebo waypoint_sequential.py 同逻辑，用于 rosbag + EGO flight_type=1）。

作用：按 launch 里 fsm/waypoint* 依次发目标，替代 RViz 2D Nav Goal。

依赖：
  - launch 里 fsm/flight_type=1，point_num / waypoint0..N 已配置
  - 可不起 waypoint_generator（本节点直发 /waypoint_generator/waypoints）
  - path_to_odom_bridge；演示跟规划时 poscmd_to_viz_odom + _odom_topic:=/viz/odom

用法：
  source devel/setup.bash
  python waypoint_mission_bag.py
  python waypoint_mission_bag.py _ego_input:=both _reach_radius:=1.5 _frame_id:=world
"""
from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

DEFAULT_WAYPOINTS = [
    #(34.7, -7.59, 1.0),
    (25.7, 3.29, 1.0),
]


def load_waypoints_from_params():
    n = rospy.get_param("~waypoint_num", 0)
    if n <= 0:
        n = rospy.get_param("/ego_planner_node/fsm/waypoint_num", 0)
    if n <= 0:
        return list(DEFAULT_WAYPOINTS)

    wps = []
    for i in range(n):
        x = rospy.get_param("~wp%d_x" % i, None)
        if x is None:
            x = rospy.get_param(
                "/ego_planner_node/fsm/waypoint%d_x" % i, 0.0
            )
            y = rospy.get_param(
                "/ego_planner_node/fsm/waypoint%d_y" % i, 0.0
            )
            z = rospy.get_param(
                "/ego_planner_node/fsm/waypoint%d_z" % i, 1.0
            )
        else:
            y = rospy.get_param("~wp%d_y" % i, 0.0)
            z = rospy.get_param("~wp%d_z" % i, 1.0)
        wps.append((float(x), float(y), float(z)))
    return wps


class WaypointSequential(object):
    def __init__(self):
        rospy.init_node("waypoint_sequential")

        self.frame_id = rospy.get_param("~frame_id", "world")
        self.reach_radius = rospy.get_param("~reach_radius", 1.2)
        self.reach_z_tol = rospy.get_param("~reach_z_tol", 0.35)
        self.start_delay = rospy.get_param("~start_delay", 2.0)
        self.odom_wait_timeout = rospy.get_param("~odom_wait_timeout", 60.0)
        self.publish_without_odom = rospy.get_param("~publish_without_odom", False)
        self.goal_republish = int(rospy.get_param("~goal_republish", 3))
        self.ego_input = rospy.get_param("~ego_input", "both").strip().lower()

        self.odom_topic = rospy.get_param(
            "~odom_topic", "/mavros/odom_convert"
        )
        self.odom_fallback_topic = rospy.get_param(
            "~odom_fallback_topic", "/viz/odom"
        )
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/move_base_simple/goal"
        )
        self.waypoints_topic = rospy.get_param(
            "~waypoints_topic", "/waypoint_generator/waypoints"
        )

        self.waypoints = load_waypoints_from_params()
        self.index = 0
        self.odom = None
        self.sent_index = -1
        self.mission_done = False

        use_goal = self.ego_input in ("goal", "both")
        use_wps = self.ego_input in ("waypoints", "both")

        self.goal_pub = None
        self.waypoints_pub = None
        if use_goal:
            self.goal_pub = rospy.Publisher(
                self.goal_topic, PoseStamped, queue_size=1, latch=True
            )
        if use_wps:
            self.waypoints_pub = rospy.Publisher(
                self.waypoints_topic, Path, queue_size=1, latch=True
            )

        self._final_pub = rospy.Publisher(
            "/waypoint_mission/final_goal",
            PoseStamped,
            queue_size=1,
            latch=True,
        )

        for t in (self.odom_topic, self.odom_fallback_topic):
            if t:
                rospy.Subscriber(
                    t, Odometry, self._make_odom_cb(t), queue_size=1
                )

        rospy.loginfo(
            "waypoints=%d ego_input=%s odom=%s fallback=%s",
            len(self.waypoints),
            self.ego_input,
            self.odom_topic,
            self.odom_fallback_topic,
        )

    def _make_odom_cb(self, topic):
        def _cb(msg):
            if self.odom is None:
                p = msg.pose.pose.position
                rospy.loginfo(
                    "odom from %s: start (%.2f, %.2f, %.2f)",
                    topic,
                    p.x,
                    p.y,
                    p.z,
                )
            self.odom = msg

        return _cb

    def _dist_to(self, wp):
        p = self.odom.pose.pose.position
        horiz = math.hypot(p.x - wp[0], p.y - wp[1])
        dz = abs(p.z - wp[2])
        return horiz, dz

    def _pose_goal(self, wp):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        pose.pose.position.z = wp[2]
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_goal(self, wp):
        pose = self._pose_goal(wp)
        path = Path()
        path.header = pose.header
        path.poses.append(pose)

        for _ in range(max(1, self.goal_republish)):
            if self.goal_pub is not None:
                self.goal_pub.publish(pose)
            if self.waypoints_pub is not None:
                self.waypoints_pub.publish(path)
            rospy.sleep(0.15)

        self.sent_index = self.index
        rospy.loginfo(
            "Sent goal %d/%d -> (%.2f, %.2f, %.2f)",
            self.index + 1,
            len(self.waypoints),
            wp[0],
            wp[1],
            wp[2],
        )

    def _wait_for_odom(self):
        t0 = rospy.Time.now()
        while self.odom is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > self.odom_wait_timeout:
                if self.publish_without_odom:
                    rospy.logwarn("odom timeout; publish without odom")
                    return
                raise RuntimeError(
                    "odom timeout - start path_to_odom_bridge or "
                    "poscmd_to_viz_odom, or set _publish_without_odom:=true"
                )
            rospy.sleep(0.1)

    def run(self):
        if not self.waypoints:
            rospy.logerr("no waypoints")
            return

        final = self.waypoints[-1]
        self._final_pub.publish(self._pose_goal(final))
        rospy.loginfo(
            "mission final goal (%.2f, %.2f, %.2f)", final[0], final[1], final[2]
        )

        self._wait_for_odom()
        rospy.sleep(self.start_delay)
        self._publish_goal(self.waypoints[0])

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.mission_done:
            if self.odom is None or self.sent_index < 0:
                rate.sleep()
                continue

            wp = self.waypoints[self.index]
            horiz, dz = self._dist_to(wp)
            if horiz <= self.reach_radius and dz <= self.reach_z_tol:
                if self.index >= len(self.waypoints) - 1:
                    rospy.loginfo(
                        "Mission complete (%d waypoints).", len(self.waypoints)
                    )
                    self.mission_done = True
                    break
                self.index += 1
                rospy.sleep(1.0)
                self._publish_goal(self.waypoints[self.index])

            rate.sleep()


if __name__ == "__main__":
    try:
        WaypointSequential().run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
