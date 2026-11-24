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

## 常见问题

### 固定点云有了，但栅格不跟着“规划路径”走

EGO 的 `grid_map` **不是**把整幅 PCD 一次性灌满；它在 **当前 odom 附近** 用射线更新局部体素。  
**规划路径**只是最优解，**不会移动 odom** 时，远处体素可能一直是空的。

**处理：** 用 `ego_map_warmup.py` 在 PCD 包络内网格走一圈再落回起点；或在线模式让 Path 持续更新 odom。  
评指标仍以 **`occupancy_inflate` 是否覆盖规划走廊** 为准，不要期望栅格随红线移动（静态图模式下栅格应一次铺满）。

### 规划线在点云“下面”、`final_plan_success=0`

1. **Z 不对齐**：拼接外参 / `--world-z-offset` 与 `static_odom` 的 `z`、`goal` 的 `z` 统一（与 Path 一致）。  
2. **栅格未铺满**：路径下方在规划器里是“自由空间”。先 warmup。  
3. **`ground_height` / `virtual_ceil_height`**：见 `advanced_param_static_map.xml`。

### `last_progress_time_ ERROR` / `first_optimize_step_success=0`

多为 **优化失败**（碰撞、不可行、地图空）。确认 `use_sim_time` 与当前模式一致（离线应用 `false`）。  
不要混用仍在播的 bag 时钟与 `static_odom`。

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