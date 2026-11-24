#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
将拼接好的全局 PCD（world 系）周期性发布到 /ego/points_world，供 EGO 融合静态地图。

评指标时不播 rosbag；use_sim_time=false。

用法：
  python pcd_map_publisher.py _pcd_file:=/path/sbs_01_global.pcd
"""
import struct

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _read_pcd_ascii(path):
    points = []
    data = False
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if line.startswith("DATA"):
                data = "ascii" in line.lower()
                continue
            if data and line and line[0] not in "#":
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        points.append(
                            (float(parts[0]), float(parts[1]), float(parts[2]))
                        )
                    except ValueError:
                        pass
    return points


def _read_pcd_binary(path, header_lines):
    """Minimal binary PCD reader when DATA binary."""
    fields = {}
    width = height = 0
    for ln in header_lines:
        if ln.startswith("FIELDS"):
            fields["names"] = ln.split()[1:]
        if ln.startswith("SIZE"):
            fields["size"] = list(map(int, ln.split()[1:]))
        if ln.startswith("TYPE"):
            fields["type"] = ln.split()[1:]
        if ln.startswith("WIDTH"):
            width = int(ln.split()[1])
        if ln.startswith("HEIGHT"):
            height = int(ln.split()[1])
    n = width * max(height, 1)
    names = fields.get("names", ["x", "y", "z"])
    sizes = fields.get("size", [4, 4, 4])
    types = fields.get("type", ["F", "F", "F"])
    point_step = sum(sizes)
    with open(path, "rb") as fh:
        text = fh.read(4096).decode("latin-1", errors="ignore")
        data_idx = text.find("DATA binary")
        if data_idx < 0:
            return []
        fh.seek(0)
        raw = fh.read()
        bin_start = raw.find(b"DATA binary") + len(b"DATA binary\n")
        buf = raw[bin_start:]
        points = []
        for i in range(n):
            off = i * point_step
            if off + point_step > len(buf):
                break
            vals = []
            o = 0
            for name, sz, tp in zip(names, sizes, types):
                if name in ("x", "y", "z") and tp == "F" and sz == 4:
                    vals.append(struct.unpack_from("f", buf, off + o)[0])
                o += sz
            if len(vals) == 3:
                points.append(tuple(vals))
        return points


def load_pcd(path):
    header = []
    with open(path, "r") as fh:
        for line in fh:
            header.append(line.strip())
            if line.strip().startswith("DATA"):
                if "binary" in line.lower():
                    return _read_pcd_binary(path, header)
                break
    return _read_pcd_ascii(path)


class PcdMapPublisher(object):
    def __init__(self):
        rospy.init_node("pcd_map_publisher")
        self.pcd_file = rospy.get_param("~pcd_file")
        self.cloud_out = rospy.get_param("~cloud_out", "/ego/points_world")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.rate_hz = rospy.get_param("~rate_hz", 2.0)

        pts = load_pcd(self.pcd_file)
        if not pts:
            rospy.logfatal("empty PCD: %s", self.pcd_file)
            raise SystemExit(1)
        rospy.loginfo("loaded %d points from %s", len(pts), self.pcd_file)

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        self._msg = pc2.create_cloud(
            Header(frame_id=self.frame_id), fields, pts
        )
        self._pub = rospy.Publisher(self.cloud_out, PointCloud2, queue_size=1)
        rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz), self._pub_once, oneshot=False
        )
        self._pub_once(None)

    def _pub_once(self, _evt):
        self._msg.header.stamp = rospy.Time.now()
        self._pub.publish(self._msg)


if __name__ == "__main__":
    try:
        PcdMapPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass