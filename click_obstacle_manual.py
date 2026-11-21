#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
仅「RViz Publish Point」→ Gazebo 障碍移动（无巡逻）。用于排查点击坐标/方向问题。

用法（Gazebo 已启动，先 source 工作空间 /opt/ros/melodic）：
  python click_obstacle_manual.py
  python click_obstacle_manual.py _model_name:=moving_box _default_z:=0.75

RViz：
  - Global Options → Fixed Frame = world（推荐）
  - 工具选 Publish Point（不要 2D Nav Goal）

调试：
  - 终端会打印 raw 点击 / world 目标 / 到达后 get_model_state 与目标差值
  - 发布 ~/debug_marker（Marker CUBE），可在 RViz 添加 Marker 与 Gazebo 对照

依赖：ros-melodic-tf2-geometry-msgs
"""
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker


def quat_yaw(yaw):
    q = Quaternion()
    q.x = q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ClickObstacleManual(object):
    def __init__(self):
        rospy.init_node("click_obstacle_manual", anonymous=False)

        self.model_name = rospy.get_param("~model_name", "moving_box")
        self.speed = rospy.get_param("~speed", 0.6)
        self.default_z = rospy.get_param("~default_z", 0.75)
        self.use_click_z = rospy.get_param("~use_click_z", False)
        self.click_topic = rospy.get_param("~click_topic", "/clicked_point")
        self.target_frame = rospy.get_param("~target_frame", "world")
        self.default_click_frame = rospy.get_param("~default_click_frame", "world")
        self.rate_hz = rospy.get_param("~rate_hz", 30.0)
        self.arrive_dist = rospy.get_param("~arrive_dist", 0.10)
        self.max_move_sec = rospy.get_param("~max_move_sec", 90.0)
        self.stall_sec = rospy.get_param("~stall_sec", 2.5)
        self.publish_debug_marker = rospy.get_param("~publish_debug_marker", True)
        self.marker_size = rospy.get_param("~marker_size", 0.55)
        self.marker_height = rospy.get_param("~marker_height", 1.5)

        rospy.wait_for_service("/gazebo/get_model_state", timeout=60.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=60.0)
        self._get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._target_lock = threading.Lock()
        self._set_lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._move_thread = None
        self._click_count = 0
        self._target_world = None
        self._last_raw = None

        if self.publish_debug_marker:
            self._marker_pub = rospy.Publisher(
                "~debug_marker", Marker, queue_size=1, latch=True
            )
        else:
            self._marker_pub = None

        rospy.Subscriber(
            self.click_topic, PointStamped, self._on_click, queue_size=10
        )

        rospy.loginfo(
            "click_obstacle_manual READY | model=%s topic=%s target_frame=%s "
            "| RViz: Fixed Frame=%s, tool=Publish Point",
            self.model_name,
            self.click_topic,
            self.target_frame,
            self.target_frame,
        )
        self._log_model_pose("STARTUP")

    def _log_model_pose(self, tag):
        pose = self._query_pose()
        if pose is None:
            rospy.logwarn("[%s] get_model_state failed for '%s'", tag, self.model_name)
            return
        p = pose.position
        rospy.loginfo(
            "[%s] gazebo '%s' pose world (%.3f, %.3f, %.3f)",
            tag,
            self.model_name,
            p.x,
            p.y,
            p.z,
        )

    def _query_pose(self):
        try:
            resp = self._get_state(self.model_name, "")
        except rospy.ServiceException as e:
            rospy.logerr_throttle(1.0, "get_model_state: %s", e)
            return None
        if not resp.success:
            rospy.logerr_throttle(1.0, "get_model_state: model '%s' not found", self.model_name)
            return None
        return resp.pose

    def _click_to_world(self, msg):
        if not msg.header.frame_id:
            msg.header.frame_id = self.default_click_frame
        if msg.header.frame_id == self.target_frame:
            return msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id, False
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(2.0),
            )
            out = do_transform_point(msg, trans)
            return out.point.x, out.point.y, out.point.z, msg.header.frame_id, True
        except Exception as e:
            rospy.logwarn(
                "TF %s -> %s failed (%s); using raw click coords",
                msg.header.frame_id,
                self.target_frame,
                e,
            )
            return msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id, False

    def _apply_state(self, x, y, z, yaw=0.0):
        st = ModelState()
        st.model_name = self.model_name
        st.pose.position.x = x
        st.pose.position.y = y
        st.pose.position.z = z
        st.pose.orientation = quat_yaw(yaw)
        st.twist = Twist()
        st.reference_frame = self.target_frame
        with self._set_lock:
            try:
                resp = self._set_state(st)
            except rospy.ServiceException as e:
                rospy.logerr("set_model_state: %s", e)
                return False
        if not resp.success:
            rospy.logerr_throttle(1.0, "set_model_state returned success=False")
            return False
        if self._marker_pub is not None:
            self._publish_marker(x, y, z, yaw)
        return True

    def _publish_marker(self, x, y, z, yaw):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "click_obstacle_debug"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation = quat_yaw(yaw)
        m.scale.x = self.marker_size
        m.scale.y = self.marker_size
        m.scale.z = self.marker_height
        m.color.r = 0.1
        m.color.g = 0.85
        m.color.b = 0.2
        m.color.a = 0.75
        self._marker_pub.publish(m)

    def _stop_move_thread(self):
        self._stop_evt.set()
        th = self._move_thread
        if th is not None and th.is_alive():
            th.join(timeout=2.0)
        self._stop_evt.clear()

    def _on_click(self, msg):
        self._click_count += 1
        wx, wy, wz_raw, src_frame, tf_used = self._click_to_world(msg)
        z = wz_raw if self.use_click_z else self.default_z
        self._last_raw = (msg.point.x, msg.point.y, msg.point.z, src_frame)

        rospy.loginfo(
            "CLICK #%d | raw(%.3f,%.3f,%.3f) frame=%s | world(%.3f,%.3f,%.3f) z=%.3f tf=%s",
            self._click_count,
            msg.point.x,
            msg.point.y,
            msg.point.z,
            src_frame,
            wx,
            wy,
            z,
            z,
            tf_used,
        )

        with self._target_lock:
            self._target_world = (wx, wy, z)

        self._stop_move_thread()
        self._move_thread = threading.Thread(target=self._move_loop)
        self._move_thread.daemon = True
        self._move_thread.start()

    def _move_loop(self):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now().to_sec()
        last_ok_dist = 1e9
        last_progress_t = t0

        while not rospy.is_shutdown() and not self._stop_evt.is_set():
            now = rospy.Time.now().to_sec()
            if now - t0 > self.max_move_sec:
                rospy.logwarn("MOVE timeout %.0fs", self.max_move_sec)
                break

            with self._target_lock:
                tgt = self._target_world
            if tgt is None:
                break

            pose = self._query_pose()
            if pose is None:
                rospy.logerr("MOVE abort: cannot read model pose")
                break

            cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
            tx, ty, tz = tgt
            dx, dy, dz = tx - cx, ty - cy, tz - cz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < last_ok_dist - 0.015:
                last_ok_dist = dist
                last_progress_t = now
            elif dist > 0.05 and (now - last_progress_t) > self.stall_sec:
                rospy.logwarn(
                    "STALL %.1fs dist=%.3fm -> snap to target", self.stall_sec, dist
                )
                if not self._apply_state(tx, ty, tz, math.atan2(dy, dx)):
                    break
                last_ok_dist = 0.0
                dist = 0.0
                cx, cy, cz = tx, ty, tz

            if dist < self.arrive_dist:
                if not self._apply_state(tx, ty, tz, math.atan2(ty - cy, tx - cx)):
                    break
                final = self._query_pose()
                if final is not None:
                    fp = final.position
                    ex, ey, ez = fp.x - tx, fp.y - ty, fp.z - tz
                    rospy.loginfo(
                        "ARRIVED | target(%.3f,%.3f,%.3f) gazebo(%.3f,%.3f,%.3f) "
                        "err(%.3f,%.3f,%.3f) dist_xy=%.3f",
                        tx,
                        ty,
                        tz,
                        fp.x,
                        fp.y,
                        fp.z,
                        ex,
                        ey,
                        ez,
                        math.sqrt(ex * ex + ey * ey),
                    )
                else:
                    rospy.loginfo("ARRIVED | target(%.3f, %.3f, %.3f)", tx, ty, tz)
                with self._target_lock:
                    self._target_world = None
                break

            step = min(self.speed / self.rate_hz, dist)
            if dist < 1e-9:
                nx, ny, nz = tx, ty, tz
                yaw = 0.0
            else:
                nx = cx + dx * (step / dist)
                ny = cy + dy * (step / dist)
                nz = cz + dz * (step / dist)
                yaw = math.atan2(dy, dx)

            if not self._apply_state(nx, ny, nz, yaw):
                break
            rate.sleep()


if __name__ == "__main__":
    try:
        ClickObstacleManual()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass