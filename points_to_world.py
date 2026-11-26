#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
将雷达 PointCloud2 用 TF 变换到 world（EGO grid_map 需要世界系点云时）。

bag 无 /tf 时：先起 path_to_odom_bridge（已修复四元数），再补 base_link -> 点云 header.frame_id。

sbs_01 点云 frame 多为 sensor1/os_sensor（不是 os1_cloud_node1），例如：
  rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link sensor1/os_sensor 100

用法：
  python points_to_world.py \
    _cloud_in:=/os1_cloud_node1/points _cloud_out:=/ego/points_world \
    _clear_radius_xy:=1.8
  # clear_radius_xy：去掉机体水平半径内点云，缓解 LiDAR 同心环陷障（与 min_ray 互补）
"""
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


class PointsToWorld(object):
    def __init__(self):
        rospy.init_node("points_to_world")
        self.cloud_in = rospy.get_param("~cloud_in", "/os1_cloud_node1/points")
        self.cloud_out = rospy.get_param("~cloud_out", "/ego/points_world")
        self.target_frame = rospy.get_param("~target_frame", "world")
        # rosbag -l 会时间回跳，必须用最新 TF，不要用点云旧 stamp
        self.use_latest_tf = rospy.get_param("~use_latest_tf", True)
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odom_convert")
        self.clear_radius_xy = rospy.get_param("~clear_radius_xy", 1.8)
        self._odom_xy = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._pub = rospy.Publisher(self.cloud_out, PointCloud2, queue_size=2)
        self._seen_frame = False
        rospy.Subscriber(self.cloud_in, PointCloud2, self._cb, queue_size=2)
        if self.clear_radius_xy > 0:
            rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.loginfo(
            "points_to_world: %s -> %s (%s) clear_radius_xy=%.2f",
            self.cloud_in,
            self.cloud_out,
            self.target_frame,
            self.clear_radius_xy,
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._odom_xy = (p.x, p.y)

    def _filter_near_body(self, cloud):
        if self.clear_radius_xy <= 0 or self._odom_xy is None:
            return cloud
        ox, oy = self._odom_xy
        r2 = self.clear_radius_xy * self.clear_radius_xy
        kept = []
        for x, y, z in pc2.read_points(cloud, skip_nans=True):
            dx, dy = x - ox, y - oy
            if dx * dx + dy * dy < r2:
                continue
            kept.append((x, y, z))
        if not kept:
            return None
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        header = Header(frame_id=self.target_frame, stamp=cloud.header.stamp)
        return pc2.create_cloud(header, fields, kept)

    def _lookup(self, src, stamp):
        times = (rospy.Time(0),) if self.use_latest_tf else (stamp, rospy.Time(0))
        last_err = None
        for t in times:
            try:
                return self._tf_buffer.lookup_transform(
                    self.target_frame, src, t, rospy.Duration(0.5)
                )
            except (
                tf2_ros.ExtrapolationException,
                tf2_ros.LookupException,
            ) as err:
                last_err = err
                continue
        raise last_err or tf2_ros.LookupException(
            "no transform %s -> %s" % (src, self.target_frame)
        )

    def _cb(self, msg):
        src = msg.header.frame_id or "base_link"
        if not self._seen_frame:
            self._seen_frame = True
            rospy.loginfo(
                "point cloud frame_id='%s' — static TF example:\n"
                "  rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 "
                "base_link %s 100",
                src,
                src,
            )
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time(0)
        try:
            trans = self._lookup(src, stamp)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn_throttle(
                5.0, "sim time jumped (rosbag loop); skip one cloud frame"
            )
            return
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as err:
            rospy.logwarn_throttle(
                2.0,
                "TF %s -> %s: %s (path_to_odom_bridge 已更新? base_link->%s 静态 TF?)",
                src,
                self.target_frame,
                err,
                src,
            )
            if msg.header.frame_id == self.target_frame:
                self._pub.publish(msg)
            return
        try:
            out = tf2_sensor_msgs.do_transform_cloud(msg, trans)
            out.header.frame_id = self.target_frame
            out = self._filter_near_body(out)
            if out is not None:
                self._pub.publish(out)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "transform cloud: %s", e)


if __name__ == "__main__":
    try:
        PointsToWorld()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass