#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
世界系障碍物点云 → 体素占据栅格（含膨胀），再转为 EGO 可吃的 PointCloud2。

流程：
  1. 读全局 PCD（world）
  2. 高度带 / 去地面过滤
  3. 体素化 + 3D 膨胀（对齐 grid_map/resolution、obstacles_inflation）
  4. 发布体素中心点云 → EGO /grid_map/cloud 当作「已占据」点
  5. 发布 2D 投影 OccupancyGrid → RViz Map（全景查看栅格）

EGO 仍用自带 grid_map 融合；输入改为体素障碍点云后，不再依赖远处 raw 点是否落在局部窗。

用法：
  rosparam set use_sim_time false
  python obstacle_grid_from_pcd.py _pcd_file:=/path/sbs_01_global.pcd

  仅离线导出（不启 rospy 图）：
  python obstacle_grid_from_pcd.py --pcd /path/in.pcd --export-only --out-npz /path/grid.npz
"""
from __future__ import division, print_function

import argparse
import math
import sys

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from pcd_map_publisher import load_pcd


def _filter_points(pts, z_min, z_max, ground_margin, ground_percentile):
    arr = np.asarray(pts, dtype=np.float64)
    if arr.size == 0:
        return arr.reshape(0, 3)
    z_floor = np.percentile(arr[:, 2], ground_percentile) + ground_margin
    mask = (arr[:, 2] >= max(z_min, z_floor)) & (arr[:, 2] <= z_max)
    return arr[mask]


def _voxel_keys(points, origin, resolution):
    rel = (points - origin) / resolution
    return np.floor(rel).astype(np.int32)


def _inflate_keys(keys, inflation_cells):
    if inflation_cells <= 0:
        return np.unique(keys, axis=0)
    off = []
    r = inflation_cells
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            for dz in range(-r, r + 1):
                if dx * dx + dy * dy + dz * dz <= r * r + r:
                    off.append((dx, dy, dz))
    off = np.asarray(off, dtype=np.int32)
    chunks = []
    step = 50000
    for i in range(0, len(keys), step):
        block = keys[i : i + step]
        exp = block[:, None, :] + off[None, :, :]
        chunks.append(exp.reshape(-1, 3))
    all_k = np.vstack(chunks)
    return np.unique(all_k, axis=0)


def _keys_to_centers(keys, origin, resolution):
    return (keys.astype(np.float64) + 0.5) * resolution + origin


def _build_2d_occ(keys, origin, resolution, z_slice, z_band):
    z0 = (z_slice - z_band - origin[2]) / resolution
    z1 = (z_slice + z_band - origin[2]) / resolution
    ki = keys[:, 2]
    sel = keys[(ki >= math.floor(z0)) & (ki <= math.ceil(z1))]
    if len(sel) == 0:
        sel = keys
    ij = np.unique(sel[:, :2], axis=0)
    ix, iy = ij[:, 0], ij[:, 1]
    min_i, max_i = ix.min(), ix.max()
    min_j, max_j = iy.min(), iy.max()
    w = int(max_i - min_i + 1)
    h = int(max_j - min_j + 1)
    grid = np.zeros((h, w), dtype=np.int8)
    for i, j in zip(ix, iy):
        grid[int(j - min_j), int(i - min_i)] = 100
    meta_origin_x = origin[0] + min_i * resolution
    meta_origin_y = origin[1] + min_j * resolution
    return grid, meta_origin_x, meta_origin_y, w, h


def build_grid(
    pts,
    resolution=0.5,
    inflation=0.1,
    z_min=0.2,
    z_max=2.8,
    ground_margin=0.15,
    ground_percentile=5.0,
    slice_z=1.2,
    slice_band=0.6,
):
    filt = _filter_points(pts, z_min, z_max, ground_margin, ground_percentile)
    if len(filt) < 10:
        raise ValueError("too few points after filter (%d)" % len(filt))
    origin = np.array(
        [filt[:, 0].min(), filt[:, 1].min(), filt[:, 2].min()], dtype=np.float64
    )
    keys = _voxel_keys(filt, origin, resolution)
    keys = np.unique(keys, axis=0)
    inflate_cells = int(math.ceil(inflation / resolution))
    occ_keys = _inflate_keys(keys, inflate_cells)
    centers = _keys_to_centers(occ_keys, origin, resolution)
    grid2d, ox, oy, w, h = _build_2d_occ(
        occ_keys, origin, resolution, slice_z, slice_band
    )
    meta = {
        "origin": origin,
        "resolution": resolution,
        "inflation": inflation,
        "n_raw": len(filt),
        "n_voxels": len(occ_keys),
        "centroid": centers.mean(axis=0),
        "grid2d_shape": (w, h),
        "grid2d_origin_xy": (ox, oy),
    }
    return centers, grid2d, ox, oy, w, h, meta, occ_keys


def export_npz(path, meta, occ_keys):
    np.savez_compressed(
        path,
        origin=meta["origin"],
        resolution=meta["resolution"],
        keys=occ_keys,
        **{k: meta[k] for k in ("centroid", "inflation") if k in meta}
    )


def _cloud_msg(frame_id, centers):
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(Header(frame_id=frame_id), fields, centers.tolist())


def _occ_grid_msg(frame_id, grid, ox, oy, resolution, w, h):
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.info = MapMetaData()
    msg.info.resolution = resolution
    msg.info.width = w
    msg.info.height = h
    msg.info.origin = Pose()
    msg.info.origin.position.x = ox
    msg.info.origin.position.y = oy
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.flatten().tolist()
    return msg


class ObstacleGridFromPcd(object):
    def __init__(self):
        rospy.init_node("obstacle_grid_from_pcd")
        pcd = rospy.get_param("~pcd_file")
        self.frame = rospy.get_param("~frame_id", "world")
        self.resolution = rospy.get_param("~resolution", 0.5)
        self.inflation = rospy.get_param("~inflation", 0.1)
        self.z_min = rospy.get_param("~z_min", 0.2)
        self.z_max = rospy.get_param("~z_max", 2.8)
        self.slice_z = rospy.get_param("~slice_z", 1.2)
        self.rate_hz = rospy.get_param("~rate_hz", 2.0)
        self.cloud_ego = rospy.get_param("~cloud_out", "/ego/obstacle_voxels")
        self.cloud_viz = rospy.get_param("~cloud_viz", "/map/obstacle_voxel_cloud")
        self.map_topic = rospy.get_param("~map_topic", "/map/projected_occupancy")

        pts = load_pcd(pcd)
        centers, grid2d, ox, oy, w, h, meta, keys = build_grid(
            pts,
            resolution=self.resolution,
            inflation=self.inflation,
            z_min=self.z_min,
            z_max=self.z_max,
            slice_z=self.slice_z,
        )
        self._centers = centers
        self._grid2d = grid2d
        self._ox, self._oy, self._w, self._h = ox, oy, w, h
        self._meta = meta
        rospy.loginfo(
            "obstacle grid: raw=%d voxels=%d resolution=%.2f inflate=%.2f centroid=(%.2f,%.2f,%.2f)",
            meta["n_raw"],
            meta["n_voxels"],
            self.resolution,
            self.inflation,
            meta["centroid"][0],
            meta["centroid"][1],
            meta["centroid"][2],
        )
        rospy.loginfo(
            "建议 odom 放在场景内；地图质心 z≈%.2f；2D 切片 z=%.2f",
            meta["centroid"][2],
            self.slice_z,
        )

        self._pub_ego = rospy.Publisher(self.cloud_ego, PointCloud2, queue_size=1)
        self._pub_viz = rospy.Publisher(self.cloud_viz, PointCloud2, queue_size=1)
        self._pub_map = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick, oneshot=False)
        self._tick(None)

    def _tick(self, _):
        stamp = rospy.Time.now()
        ego = _cloud_msg(self.frame, self._centers)
        ego.header.stamp = stamp
        viz = _cloud_msg(self.frame, self._centers)
        viz.header.stamp = stamp
        occ = _occ_grid_msg(
            self.frame, self._grid2d, self._ox, self._oy, self.resolution, self._w, self._h
        )
        occ.header.stamp = stamp
        self._pub_ego.publish(ego)
        self._pub_viz.publish(viz)
        self._pub_map.publish(occ)


def _main_cli():
    p = argparse.ArgumentParser()
    p.add_argument("--pcd", required=True)
    p.add_argument("--export-only", action="store_true")
    p.add_argument("--out-npz", default="")
    p.add_argument("--resolution", type=float, default=0.5)
    p.add_argument("--inflation", type=float, default=0.1)
    args, unknown = p.parse_known_args()
    if args.export_only:
        pts = load_pcd(args.pcd)
        centers, _, _, _, _, _, meta, keys = build_grid(
            pts, resolution=args.resolution, inflation=args.inflation
        )
        print("voxels", meta["n_voxels"], "centroid", meta["centroid"])
        if args.out_npz:
            export_npz(args.out_npz, meta, keys)
            print("wrote", args.out_npz)
        return
    if unknown or "--pcd" in sys.argv:
        rospy.init_node("obstacle_grid_from_pcd")
        # rospy path uses private params; if called with only --pcd, set param
        if "--pcd" in sys.argv:
            idx = sys.argv.index("--pcd")
            if idx + 1 < len(sys.argv):
                rospy.set_param("~pcd_file", sys.argv[idx + 1])
    ObstacleGridFromPcd()
    rospy.spin()


if __name__ == "__main__":
    if "--export-only" in sys.argv:
        _main_cli()
    elif "--pcd" in sys.argv:
        i = sys.argv.index("--pcd")
        if i + 1 < len(sys.argv):
            import rospkg  # noqa: F401

            rospy.init_node("obstacle_grid_from_pcd", anonymous=True)
            rospy.set_param("~pcd_file", sys.argv[i + 1])
        try:
            ObstacleGridFromPcd()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    else:
        try:
            ObstacleGridFromPcd()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass