#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
手动点击障碍 + 两条自动往复障碍（单节点）。

手动：与 click_obstacle_manual.py 相同（Publish Point → moving_box）。
自动 1：X 固定，Y 在 [2, 5] 往复（默认 patrol_obstacle_1）。
自动 2：Y 固定，X 在 [-7, -3] 往复（默认 patrol_obstacle_2，world 里需有对应模型）。

用法（Gazebo 已启动）：
  python dynamic_obstacles.py

关闭某一项：
  python dynamic_obstacles.py _click_enable:=false
  python dynamic_obstacles.py _patrol_y_enable:=false _patrol_x_enable:=false

world 里每个动态障碍 link 建议：
  <kinematic>1</kinematic>
  <gravity>0</gravity>
  pose.z =  box高度/2（如高1.5 → z=0.75）
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

WORLD_FRAME = "world"


def quat_yaw(yaw):
    q = Quaternion()
    q.x = q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class GazeboMover(object):
    """Shared get/set_model_state with lock."""

    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state", timeout=60.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=60.0)
        self._get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self._set = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self._lock = threading.Lock()

    def get_pose(self, model_name):
        try:
            resp = self._get(model_name, "")
        except rospy.ServiceException as e:
            rospy.logerr_throttle(1.0, "get_model_state: %s", e)
            return None
        if not resp.success:
            rospy.logerr_throttle(1.0, "no model '%s'", model_name)
            return None
        return resp.pose

    def set_pose(self, model_name, x, y, z, yaw=0.0):
        st = ModelState()
        st.model_name = model_name
        st.pose.position.x = x
        st.pose.position.y = y
        st.pose.position.z = z
        st.pose.orientation = quat_yaw(yaw)
        st.twist = Twist()
        st.reference_frame = WORLD_FRAME
        with self._lock:
            try:
                resp = self._set(st)
            except rospy.ServiceException as e:
                rospy.logerr_throttle(1.0, "set_model_state: %s", e)
                return False
        return resp.success


class PatrolAxis(object):
    def __init__(self, gazebo, name, axis, pmin, pmax, fixed_x, fixed_y, fixed_z, speed, rate_hz):
        self.gazebo = gazebo
        self.name = name
        self.axis = axis
        self.pmin = min(pmin, pmax)
        self.pmax = max(pmin, pmax)
        self.fixed_x = fixed_x
        self.fixed_y = fixed_y
        self.fixed_z = fixed_z
        self.speed = speed
        self.rate_hz = rate_hz
        self.sign = 1.0
        self._pos = self._read_pos()
        if self._pos is None:
            self._pos = 0.5 * (self.pmin + self.pmax)
        else:
            self._pos = max(self.pmin, min(self.pmax, self._pos))

    def _read_pos(self):
        pose = self.gazebo.get_pose(self.name)
        if pose is None:
            return None
        if self.axis == "x":
            return pose.position.x
        return pose.position.y

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        step = self.speed / self.rate_hz
        rospy.loginfo(
            "PATROL '%s' axis=%s [%.2f, %.2f] fixed=(%.2f,%.2f,%.2f) speed=%.2f",
            self.name,
            self.axis,
            self.pmin,
            self.pmax,
            self.fixed_x,
            self.fixed_y,
            self.fixed_z,
            self.speed,
        )
        while not rospy.is_shutdown():
            self._pos += self.sign * step
            if self._pos >= self.pmax:
                self._pos = self.pmax
                self.sign = -1.0
            elif self._pos <= self.pmin:
                self._pos = self.pmin
                self.sign = 1.0
            if self.axis == "x":
                ax, ay = self._pos, self.fixed_y
            else:
                ax, ay = self.fixed_x, self._pos
            self.gazebo.set_pose(self.name, ax, ay, self.fixed_z, 0.0)
            rate.sleep()


class ClickMover(object):
    def __init__(self, gazebo):
        self.gazebo = gazebo
        self.model_name = rospy.get_param("~click_model", "moving_box")
        self.speed = rospy.get_param("~click_speed", 0.6)
        self.default_z = rospy.get_param("~click_z", 0.75)
        self.use_click_z = rospy.get_param("~click_use_point_z", False)
        self.click_topic = rospy.get_param("~click_topic", "/clicked_point")
        self.target_frame = rospy.get_param("~target_frame", WORLD_FRAME)
        self.default_click_frame = rospy.get_param("~default_click_frame", WORLD_FRAME)
        self.rate_hz = rospy.get_param("~rate_hz", 30.0)
        self.arrive_dist = rospy.get_param("~click_arrive_dist", 0.10)
        self.max_move_sec = rospy.get_param("~click_max_move_sec", 90.0)
        self.stall_sec = rospy.get_param("~click_stall_sec", 2.5)
        self.publish_debug_marker = rospy.get_param("~publish_debug_marker", True)
        self.marker_size = rospy.get_param("~marker_size", 0.55)
        self.marker_height = rospy.get_param("~marker_height", 1.5)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._target_lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._move_thread = None
        self._target = None
        self._click_count = 0

        if self.publish_debug_marker:
            self._marker_pub = rospy.Publisher(
                "~click_debug_marker", Marker, queue_size=1, latch=True
            )
        else:
            self._marker_pub = None

        rospy.Subscriber(self.click_topic, PointStamped, self._on_click, queue_size=10)
        rospy.loginfo(
            "CLICK model=%s topic=%s | RViz: Fixed Frame=%s, Publish Point",
            self.model_name,
            self.click_topic,
            self.target_frame,
        )

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
                "TF %s->%s: %s; use raw", msg.header.frame_id, self.target_frame, e
            )
            return msg.point.x, msg.point.y, msg.point.z, msg.header.frame_id, False

    def _publish_marker(self, x, y, z, yaw):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "click_obstacle"
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
        m.color.r = 0.2
        m.color.g = 0.85
        m.color.b = 0.2
        m.color.a = 0.75
        self._marker_pub.publish(m)

    def _apply(self, x, y, z, yaw):
        ok = self.gazebo.set_pose(self.model_name, x, y, z, yaw)
        if ok and self._marker_pub is not None:
            self._publish_marker(x, y, z, yaw)
        return ok

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
        rospy.loginfo(
            "CLICK #%d raw(%.3f,%.3f,%.3f) [%s] -> world(%.3f,%.3f,%.3f) tf=%s",
            self._click_count,
            msg.point.x,
            msg.point.y,
            msg.point.z,
            src_frame,
            wx,
            wy,
            z,
            tf_used,
        )
        with self._target_lock:
            self._target = (wx, wy, z)
        self._stop_move_thread()
        self._move_thread = threading.Thread(target=self._move_loop)
        self._move_thread.daemon = True
        self._move_thread.start()

    def _move_loop(self):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now().to_sec()
        best = 1e9
        last_prog = t0

        while not rospy.is_shutdown() and not self._stop_evt.is_set():
            if rospy.Time.now().to_sec() - t0 > self.max_move_sec:
                rospy.logwarn("CLICK move timeout")
                break
            with self._target_lock:
                tgt = self._target
            if tgt is None:
                break

            pose = self.gazebo.get_pose(self.model_name)
            if pose is None:
                break
            cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
            tx, ty, tz = tgt
            dx, dy, dz = tx - cx, ty - cy, tz - cz
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            now = rospy.Time.now().to_sec()

            if dist < best - 0.015:
                best = dist
                last_prog = now
            elif dist > 0.05 and (now - last_prog) > self.stall_sec:
                rospy.logwarn("CLICK stall -> snap")
                self._apply(tx, ty, tz, math.atan2(dy, dx))
                best = 0.0
                dist = 0.0

            if dist < self.arrive_dist:
                self._apply(tx, ty, tz, math.atan2(ty - cy, tx - cx))
                final = self.gazebo.get_pose(self.model_name)
                if final is not None:
                    fp = final.position
                    rospy.loginfo(
                        "CLICK ARRIVED err_xy=%.3f",
                        math.sqrt((fp.x - tx) ** 2 + (fp.y - ty) ** 2),
                    )
                with self._target_lock:
                    self._target = None
                break

            step = min(self.speed / self.rate_hz, dist)
            if dist > 1e-9:
                nx = cx + dx * (step / dist)
                ny = cy + dy * (step / dist)
                nz = cz + dz * (step / dist)
                yaw = math.atan2(dy, dx)
            else:
                nx, ny, nz, yaw = tx, ty, tz, 0.0
            if not self._apply(nx, ny, nz, yaw):
                break
            rate.sleep()


class DynamicObstacles(object):
    def __init__(self):
        rospy.init_node("dynamic_obstacles", anonymous=False)
        self.gazebo = GazeboMover()
        rate_hz = rospy.get_param("~rate_hz", 30.0)

        click_enable = rospy.get_param("~click_enable", True)
        patrol_y_enable = rospy.get_param("~patrol_y_enable", True)
        patrol_x_enable = rospy.get_param("~patrol_x_enable", True)

        self._threads = []

        if click_enable:
            ClickMover(self.gazebo)

        if patrol_y_enable:
            py = PatrolAxis(
                self.gazebo,
                rospy.get_param("~patrol_y_model", "patrol_obstacle_1"),
                "y",
                rospy.get_param("~patrol_y_min", 2.0),
                rospy.get_param("~patrol_y_max", 5.0),
                rospy.get_param("~patrol_y_fixed_x", 5.0),
                rospy.get_param("~patrol_y_fixed_y", 0.0),
                rospy.get_param("~patrol_z", 0.75),
                rospy.get_param("~patrol_y_speed", 0.6),
                rate_hz,
            )
            t = threading.Thread(target=py.run)
            t.daemon = True
            t.start()
            self._threads.append(t)

        if patrol_x_enable:
            px = PatrolAxis(
                self.gazebo,
                rospy.get_param("~patrol_x_model", "patrol_obstacle_2"),
                "x",
                rospy.get_param("~patrol_x_min", -7.0),
                rospy.get_param("~patrol_x_max", -3.0),
                rospy.get_param("~patrol_x_fixed_x", 0.0),
                rospy.get_param("~patrol_x_fixed_y", 0.0),
                rospy.get_param("~patrol_z", 0.75),
                rospy.get_param("~patrol_x_speed", 0.6),
                rate_hz,
            )
            t = threading.Thread(target=px.run)
            t.daemon = True
            t.start()
            self._threads.append(t)

        rospy.loginfo(
            "dynamic_obstacles: click=%s patrol_y=%s patrol_x=%s",
            click_enable,
            patrol_y_enable,
            patrol_x_enable,
        )


if __name__ == "__main__":
    try:
        DynamicObstacles()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass