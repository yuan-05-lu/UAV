#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
已知 Path 位姿 + 雷达外参，将 rosbag 拼成 world 系全局点云（无需 SLAM）。

不启动 ROS。输出 ASCII PCD，供 pcd_map_publisher / CloudCompare 使用。

用法：
  python bag_stitch_pcd.py \\
    --bag /path/sbs_01_vis.bag \\
    --out /path/sbs_01_global.pcd \\
    --voxel 0.08 \\
    --sensor-tz 0.2
"""
from __future__ import print_function

import argparse
import bisect
import math
import struct
import sys

import numpy as np

try:
    import rosbag
    from sensor_msgs import point_cloud2
except ImportError as exc:
    sys.stderr.write("需要 ROS Python: rosbag, sensor_msgs\n")
    raise exc


def _euler_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz.dot(ry).dot(rx)


def _make_se3(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _load_path(bag, topic):
    stamps = []
    pos = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        if not msg.poses:
            continue
        ps = msg.poses[-1]
        stamps.append(t.to_sec())
        p = ps.pose.position
        pos.append((p.x, p.y, p.z))
    return stamps, pos


def _nearest_pose(stamps, pos, t_sec):
    if not stamps:
        return None
    i = bisect.bisect_left(stamps, t_sec)
    if i == 0:
        return pos[0]
    if i >= len(stamps):
        return pos[-1]
    if abs(stamps[i] - t_sec) < abs(stamps[i - 1] - t_sec):
        return pos[i]
    return pos[i - 1]


def _voxel_downsample(points, voxel):
    if voxel <= 0 or len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


def _write_pcd_ascii(path, points):
    n = len(points)
    with open(path, "w") as fh:
        fh.write("# stitched global map\n")
        fh.write("VERSION 0.7\n")
        fh.write("FIELDS x y z\n")
        fh.write("SIZE 4 4 4\n")
        fh.write("TYPE F F F\n")
        fh.write("COUNT 1 1 1\n")
        fh.write("WIDTH %d\n" % n)
        fh.write("HEIGHT 1\n")
        fh.write("POINTS %d\n" % n)
        fh.write("DATA ascii\n")
        for x, y, z in points:
            fh.write("%.6f %.6f %.6f\n" % (x, y, z))


def stitch(args):
    R_bs = _euler_matrix(args.sensor_roll, args.sensor_pitch, args.sensor_yaw)
    t_bs = np.array([args.sensor_tx, args.sensor_ty, args.sensor_tz])
    T_bs = _make_se3(R_bs, t_bs)

    bag = rosbag.Bag(args.bag, "r")
    path_stamps, path_pos = _load_path(bag, args.path_topic)
    if not path_stamps:
        bag.close()
        raise RuntimeError("no poses on topic %s" % args.path_topic)

    chunks = []
    count = 0
    for _, msg, t in bag.read_messages(topics=[args.cloud_topic]):
        pose = _nearest_pose(path_stamps, path_pos, t.to_sec())
        if pose is None:
            continue
        T_wb = _make_se3(np.eye(3), np.array(pose))
        T_ws = T_wb.dot(T_bs)
        try:
            pts = list(
                point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            )
        except Exception:
            continue
        if not pts:
            continue
        block = np.array(pts, dtype=np.float64)
        ones = np.ones((block.shape[0], 1))
        hom = np.hstack([block, ones])
        world = (T_ws.dot(hom.T)).T[:, :3]
        chunks.append(world)
        count += 1
        if args.max_scans and count >= args.max_scans:
            break
        if count % 50 == 0:
            print("scans fused:", count)

    bag.close()
    if not chunks:
        raise RuntimeError("no cloud points read from %s" % args.cloud_topic)

    all_pts = np.vstack(chunks)
    if args.world_z_offset:
        all_pts[:, 2] += args.world_z_offset
    all_pts = _voxel_downsample(all_pts, args.voxel)
    _write_pcd_ascii(args.out, all_pts)
    print("wrote %s  points=%d  scans=%d" % (args.out, len(all_pts), count))


def main():
    p = argparse.ArgumentParser(description="Stitch bag lidar to global PCD")
    p.add_argument("--bag", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--path-topic", default="/node_pos_sc")
    p.add_argument("--cloud-topic", default="/os1_cloud_node1/points")
    p.add_argument("--voxel", type=float, default=0.08, help="downsample voxel m")
    p.add_argument("--max-scans", type=int, default=0, help="0 = all")
    p.add_argument("--sensor-tx", type=float, default=0.0)
    p.add_argument("--sensor-ty", type=float, default=0.0)
    p.add_argument("--sensor-tz", type=float, default=0.2)
    p.add_argument("--sensor-roll", type=float, default=0.0)
    p.add_argument("--sensor-pitch", type=float, default=0.0)
    p.add_argument("--sensor-yaw", type=float, default=0.0)
    p.add_argument(
        "--world-z-offset",
        type=float,
        default=0.0,
        help="add to all stitched z (align Path height with cloud)",
    )
    args = p.parse_args()
    stitch(args)


if __name__ == "__main__":
    main()