#!/usr/bin/env bash
# rosbag 避障已单独起好时，本脚本只起「指标 + 航点任务」。
set -euo pipefail
UAV_REPO="${UAV_REPO:-$(cd "$(dirname "$0")/.." && pwd)}"
CFG="${1:-${UAV_REPO}/config/waypoints_sbs01_bag.yaml}"
CSV="${2:-/tmp/ego_bag_metrics.csv}"

echo "metrics csv -> ${CSV}"
echo "waypoints   -> ${CFG}"
echo "先起: python ${UAV_REPO}/ego_plan_metrics.py _csv_path:=${CSV}"
echo "再起: python ${UAV_REPO}/waypoint_mission_bag.py _config:=${CFG}"
echo "需要 poscmd_to_viz_odom 时，yaml 里 odom_topic 用 /viz/odom"