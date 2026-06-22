# 成功版本配置快照

本目录只保存上游工程中与本次单墙避障成功直接相关的修改文件，不包含整个 PX4-Autopilot 或 ego-planner-swarm。

## 文件映射

| 本仓库路径 | 运行机器上的目标路径 | 成功版本要点 |
|---|---|---|
| `ego_planner/single_run_in_sim.launch.py` | `~/ego_jazzy_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim.launch.py` | 目标点 `(20, 12, 1)` |
| `ego_planner/advanced_param.launch.py` | `~/ego_jazzy_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.launch.py` | local update range `10/10/3 m`，obstacle inflation `0.2 m` |
| `gazebo/OakD-Lite/model.sdf` | `~/drone_ws/px4_clean/Tools/simulation/gz/models/OakD-Lite/model.sdf` | 关闭 RGB；depth `80×60`、10 Hz、0.2～19.1 m |

`simulator.launch.py` 已检查，相对 ego-planner-swarm 的 `ros2_version` 分支没有修改，因此没有复制进来。

## 安装

覆盖上游文件前先自行备份：

```bash
cp config/ego_planner/single_run_in_sim.launch.py \
  ~/ego_jazzy_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/
cp config/ego_planner/advanced_param.launch.py \
  ~/ego_jazzy_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/
cp config/gazebo/OakD-Lite/model.sdf \
  ~/drone_ws/px4_clean/Tools/simulation/gz/models/OakD-Lite/model.sdf
cp worlds/simple_wall.sdf \
  ~/drone_ws/px4_clean/Tools/simulation/gz/worlds/simple_wall.sdf
```

EGO launch 文件改变后要重新执行 `colcon build` 并重新 source；仅修改源码但继续使用旧 `build/install` 产物，会表现得像配置没有生效。
