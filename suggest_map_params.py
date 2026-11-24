#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
根据全局 PCD 与规划起点，估算 run_ego_offline_eval 需要的 map_size / local_update_range。

EGO 体素图随 odom 居中；只有落在「起点周围局部盒」内的点云才会进栅格。
warmup 走到远处再回起点时，远处融进去的格子可能被滚掉。

用法：
  python suggest_map_params.py --pcd /path/global.pcd --start-x 15.8 --start-y -6.31 --start-z 0.68
"""
from __future__ import print_function

import argparse
import math

from pcd_map_publisher import load_pcd


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pcd", required=True)
    p.add_argument("--start-x", type=float, required=True)
    p.add_argument("--start-y", type=float, required=True)
    p.add_argument("--start-z", type=float, default=1.0)
    p.add_argument("--margin", type=float, default=8.0)
    args = p.parse_args()

    pts = load_pcd(args.pcd)
    xs = [q[0] for q in pts]
    ys = [q[1] for q in pts]
    zs = [q[2] for q in pts]
    dx = max(abs(max(xs) - args.start_x), abs(min(xs) - args.start_x)) + args.margin
    dy = max(abs(max(ys) - args.start_y), abs(min(ys) - args.start_y)) + args.margin
    dz = max(max(zs) - min(zs), 2.5) + 1.0
    span_xy = max(max(xs) - min(xs), max(ys) - min(ys))
    range_xy = max(dx, dy, span_xy * 0.55 + args.margin)

    map_size = math.ceil(max(2 * range_xy, span_xy + 2 * args.margin))
    print("=== 点云范围 (world) ===")
    print("  x: %.2f .. %.2f  span=%.2f" % (min(xs), max(xs), max(xs) - min(xs)))
    print("  y: %.2f .. %.2f  span=%.2f" % (min(ys), max(ys), max(ys) - min(ys)))
    print("  z: %.2f .. %.2f  span=%.2f  (点云是 3D)" % (min(zs), max(zs), max(zs) - min(zs)))
    print("")
    print("=== 相对起点 (%.2f, %.2f, %.2f) 的建议 ===" % (args.start_x, args.start_y, args.start_z))
    print("  grid_map/local_update_range_x  >= %.1f" % range_xy)
    print("  grid_map/local_update_range_y  >= %.1f" % range_xy)
    print("  grid_map/local_update_range_z  >= %.1f" % max(5.0, dz))
    print("  grid_map/map_size_x / map_size_y      >= %.1f" % map_size)
    print("  grid_map/map_size_z                   >= %.1f" % max(3.0, dz))
    print("  grid_map/max_ray_length               >= %.1f" % (range_xy + 5))
    print("")
    print("roslaunch 示例:")
    print(
        "  roslaunch ... run_ego_offline_eval.launch "
        "map_size_x:=%.0f map_size_y:=%.0f map_size_z:=3.0"
        % (map_size, map_size)
    )
    print("")
    print("若 goal 距起点超过 local_update_range，远处障碍不会进栅格（不是点云不是 3D）。")


if __name__ == "__main__":
    main()