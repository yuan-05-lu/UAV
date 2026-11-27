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

### B1.5 障碍物点云 → 障碍栅格（推荐）

```bash
python obstacle_grid_from_pcd.py _pcd_file:=/path/sbs_01_global.pcd \
  _resolution:=0.5 _inflation:=0.1 _slice_z:=1.2
```

| 话题 | 含义 |
|------|------|
| `/ego/obstacle_voxels` | 体素中心点云 → 接 EGO `cloud_topic` |
| `/map/projected_occupancy` | 某一高度层 2D 栅格 → RViz **Map** 全景 |
| `/map/obstacle_voxel_cloud` | 同体素点云，便于对照点云 |

EGO launch 默认 `cloud_topic=/ego/obstacle_voxels`。RViz 同时加 **Map**（`/map/projected_occupancy`）与 **`/grid_map/occupancy_inflate`**，前者为离线投影全景，后者为规划器内部融合结果。

### B2 跑规划与指标（不播 rosbag）

```bash
export UAV_REPO=/path/to/UAV
rosparam set use_sim_time false
```

按顺序：

| 终端 | 命令 |
|------|------|
| 1 | `roscore` |
| 2 | `python obstacle_grid_from_pcd.py _pcd_file:=/path/sbs_01_global.pcd` |
| 3 | `python ego_map_warmup.py _pcd_file:=... _finish_x:=... _finish_y:=... _finish_z:=...`（代替一直停在起点的 static_odom） |
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

### `last_progress_time_ ERROR` / `final_plan_success=0`（已显示 Close to goal）

**与“规划在点云下方”不是同一类问题。** 日志里起点已到 goal 附近（例如 0.1～0.2 m），仍刷 `last_progress_time_ ERROR`，是 EGO **全局轨迹进度变量** 和 **当前 odom 起点** 不同步：

- `getLocalTarget()` 在全局轨迹时间 `last_progress_time_` 上取点，若该点距当前 `start` 仍 **大于 `planning_horizon`**（launch 里常为 7～10 m），就会连打 5 行 `last_progress_time_ ERROR` 并 **`final_plan_success=0`**。
- 典型触发：**odom 来自 `path_to_odom_bridge`（bag 里的真轨迹）**，而你在 RViz 点了 **另一附近 goal**；全局多项式是按某次 odom 切的，但 odom 仍随 bag 在变 → `last_progress_time_` 卡住，重规划次数暴涨（上千次）。
- 若同时见 `terminal point ... in obstacle`，末端在膨胀栅格里，优化也会 `first_optimize_step_success=0`，与上面错误常一起出现。

**处理：**

1. **调试对齐**：先 `rosparam set use_sim_time true`，播 bag 时 **暂停或关掉 path 桥** 的起终点，改用 **`static_odom_publisher`** 固定 start，再点 goal；或 goal 选在 Path **前方** 至少 `planning_horizon` 一段，不要贴在当前机体旁。
2. **缩短视线**：`rosparam set /ego_planner_node/fsm/planning_horizon 5`（或 launch 改小），近程 goal 不易撞上该 bug。
3. **新目标前重启** `ego_planner_node`，清空 `global_data.last_progress_time_`。
4. **评指标**：离线以 **`/ego_planner_node/optimal_list`** 与 `occupancy_inflate` 为准；**不必**把 FSM 的 `final_plan_success` 当真机“到达”信号（odom 不跟 `pos_cmd` 时 FSM 很难进 SUCCESS）。
5. **时钟**：全 ROS 图同一 `use_sim_time`；混用 bag 时刻与 wall 时刻会在时间相关逻辑里放大异常（与 ERROR 行里两个量级差很多的时间戳类似）。

其它 **优化失败**（地图空、碰撞）仍先查 `use_sim_time`、静态 PCD 是否铺满局部图。

## 评价语义

- **真 odom（评指标）**：`static_odom_publisher` 或录制 Path 的起点。
- **规划演示**：`poscmd_to_viz_odom`（模式 A 可选）。
- **相机**：仅模式 A 播 bag 时有用；模式 B 不参与规划与指标。

### RViz「到了」但脚本报 `Arrival FAIL`（horiz err 很大）

**不必换脚本逻辑也能理解；旧脚本默认语义在 bag 模式下会误杀。**

| RViz 里你在看什么 | 代表什么 |
|------------------|----------|
| 紫球 + 红/绿线（`/viz/executed_path`） | `poscmd_to_viz_odom` 跟着 **规划指令** 走的**假机体** |
| 绿 odom / `/node_pos_sc` | **bag 录制真轨迹**（`path_to_odom_bridge`） |

旧指标脚本里的 **Actual path length (odom)** 与 **到达判定** 几乎总是用 **`/mavros/odom_convert`**（录制 Path）。  
播 bag 时录像往往 **只走了零点几米**（你图里 **0.52 m**），相对 goal 仍差 **14.71 m** → **FAIL**。  
RViz 里「到了」= **规划演示轨迹** 画到目标附近，**不是** 录制 odom 真飞到了。

**做法：**

### 阶段二 rosbag：航点任务 + 指标

**推荐：launch 写航点，`flight_type=1`，不起 waypoint_generator**

1. `launch_amended/run_ego_bag_scripted.launch`：改 `point_num`、`point0_*`…`pointN_*`
2. `waypoint_mission_bag.py`（同 Gazebo `waypoint_sequential.py`）：读 fsm 航点，`_ego_input:=both` 发 `/move_base_simple/goal` + `/waypoint_generator/waypoints`
3. `ego_plan_metrics.py`：采指标；终点以 `/waypoint_mission/final_goal` 为准

```bash
roslaunch … run_ego_bag_scripted.launch   # 无 waypoint_generator
python ego_plan_metrics.py _csv_path:=/tmp/run.csv _odom_topic:=/viz/odom
python waypoint_mission_bag.py            # 无需 yaml
```

若 EGO 收不到 goal，在 `ego_planner_node` 上增加  
`<remap from="/goal" to="/move_base_simple/goal"/>`（视你 fork 订阅名而定）。
- **评「规划有没有画到 goal」**（对照 RViz）：加 `_arrival_mode:=plan_end`。
- **仍用旧脚本**：到达判定改用 **`plan_end`** 或订阅 **`/viz/odom`** 仅作演示说明，**不要**在实验报告里写成真机到达。
- **评「真机/录制是否到达」**：只有 odom 真跟着飞（或 Path 本身终点就是 goal）时，才用 **recorded odom** 判定。

## 与阶段二条目对应

| 要求 | 本仓库实现 |
|------|------------|
| 真实场景数据包 | `sbs_01_vis.bag` + 第二场景 |
| 点云/占据 → 规划器 | `bag_stitch_pcd` + `pcd_map_publisher` + EGO grid |
| 离线规划 | `run_ego_offline_eval.launch` |
| 指标 | `offline_plan_eval.py`（路径长、耗时、到达；碰撞/最小距待扩展） |
| 仿真对比 | Gazebo 同任务表另跑 |