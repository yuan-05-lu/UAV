# Bag + EGO 与离线评指标流程

## 模式 A：在线回放（调试、看相机）

1. `roscore`
2. `rosparam set use_sim_time true`
3. `rosbag play -l /path/sbs_01_vis.bag --clock`（调试可 `-l`，**评指标不要用 -l**）
4. `python path_to_odom_bridge.py`
5. `rosrun tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link sensor1/os_sensor 100`
6. `python points_to_world.py`
7. `roslaunch launch_amended/run_ego_bag.launch`
8. 可选：`python poscmd_to_viz_odom.py`；`rosrun rqt_image_view rqt_image_view`

## 模式 B：静态全局图 + 离线评指标（阶段二推荐）

### B1 拼接地图（不用 ROS，播一遍或不播均可）

```bash
python bag_stitch_pcd.py \
  --bag /path/sbs_01_vis.bag \
  --out /path/sbs_01_global.pcd \
  --voxel 0.08 \
  --sensor-tz 0.2
```

外参与在线 `static_transform_publisher` 一致。

### B2 跑规划与指标（不播 rosbag）

```bash
export UAV_REPO=/path/to/UAV
rosparam set use_sim_time false
```

按顺序：

| 终端 | 命令 |
|------|------|
| 1 | `roscore` |
| 2 | `python pcd_map_publisher.py _pcd_file:=/path/sbs_01_global.pcd` |
| 3 | `python static_odom_publisher.py _x:=... _y:=... _z:=1.2` |
| 4 | `rosrun tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link sensor1/os_sensor 100` |
| 5 | `roslaunch $UAV_REPO/launch_amended/run_ego_offline_eval.launch` |
| 6 | `python offline_plan_eval.py _config:=$UAV_REPO/config/eval_tasks_sbs01.yaml` |

或：`scripts/run_offline_eval.sh /path/sbs_01_global.pcd`（需先设 `START_X/Y/Z`）。

输出 CSV：`config/eval_tasks_sbs01_results.csv`（路径长、规划耗时、到 goal 距离）。

### RViz（模式 B）

- Fixed Frame：`world`
- PointCloud2：`/ego/points_world`（整图）
- Marker：`/ego_planner_node/optimal_list`
- 障碍物：`/grid_map/occupancy_inflate`

## 评价语义

- **真 odom（评指标）**：`static_odom_publisher` 或录制 Path 的起点。
- **规划演示**：`poscmd_to_viz_odom`（模式 A 可选）。
- **相机**：仅模式 A 播 bag 时有用；模式 B 不参与规划与指标。

## 与阶段二条目对应

| 要求 | 本仓库实现 |
|------|------------|
| 真实场景数据包 | `sbs_01_vis.bag` + 第二场景 |
| 点云/占据 → 规划器 | `bag_stitch_pcd` + `pcd_map_publisher` + EGO grid |
| 离线规划 | `run_ego_offline_eval.launch` |
| 指标 | `offline_plan_eval.py`（路径长、耗时、到达；碰撞/最小距待扩展） |
| 仿真对比 | Gazebo 同任务表另跑 |