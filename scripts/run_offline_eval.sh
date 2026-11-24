#!/usr/bin/env bash
# 离线评指标一键编排（需已 source ROS + ego_planner catkin）
set -euo pipefail

UAV_REPO="${UAV_REPO:-$(cd "$(dirname "$0")/.." && pwd)}"
export UAV_REPO

PCD_FILE="${1:-}"
START_X="${START_X:-0.0}"
START_Y="${START_Y:-0.0}"
START_Z="${START_Z:-1.2}"
START_YAW="${START_YAW:-0.0}"

if [[ -z "$PCD_FILE" || ! -f "$PCD_FILE" ]]; then
  echo "用法: PCD_FILE=/path/map.pcd START_X=.. START_Y=.. $0"
  echo "  或: $0 /path/map.pcd"
  exit 1
fi

rosparam set use_sim_time false

roscore &
ROC_PID=$!
sleep 2
trap 'kill $ROC_PID 2>/dev/null || true' EXIT

python "${UAV_REPO}/pcd_map_publisher.py" _pcd_file:="${PCD_FILE}" &
python "${UAV_REPO}/static_odom_publisher.py" \
  _x:="${START_X}" _y:="${START_Y}" _z:="${START_Z}" _yaw:="${START_YAW}" &
sleep 2

rosrun tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link sensor1/os_sensor 100 &
roslaunch "${UAV_REPO}/launch_amended/run_ego_offline_eval.launch" &
sleep 4

python "${UAV_REPO}/offline_plan_eval.py" \
  _config:="${UAV_REPO}/config/eval_tasks_sbs01.yaml"

echo "完成。可在 RViz 查看 /ego/points_world 与 optimal_list；CSV 见 config/eval_tasks_sbs01_results.csv"
wait