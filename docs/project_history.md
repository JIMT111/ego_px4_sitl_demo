# 项目演进历史

以下按问题解决顺序记录从基础仿真到单墙避障成功的主要里程碑。

## 1. PX4 SITL 跑通

先验证 PX4-Autopilot 可以启动 Gazebo Sim 与 `gz_x500_depth`，确认飞控、仿真时钟和深度相机模型能够工作，为后续 ROS 2 与规划器接入建立稳定底座。

## 2. ROS 2 通信跑通

启动 `MicroXRCEAgent udp4 -p 8888`，打通 PX4 uXRCE-DDS client 与 ROS 2 Jazzy DDS 图，确认 `/fmu/out/*` 状态和 `/fmu/in/*` 控制 topic 可见。

## 3. EGO Planner 跑通

在 ego-planner-swarm `ros2_version` 上完成编译和单机仿真启动，验证 EGO 能接收 odom、建立 grid map、生成 bspline 与 `/drone_0_planning/pos_cmd`。

## 4. PX4 ↔ EGO bridge

建立 `ego_px4_bridge`：

- `px4_to_ego_odom.py` 将 PX4 `VehicleLocalPosition` 从 NED 转为 EGO/ROS ENU odom。
- `ego_to_px4_setpoint.py` 将 EGO `PositionCommand` 转为 PX4 `TrajectorySetpoint`。
- `px4_offboard_arm.py` 持续发布 Offboard 心跳并可靠请求 Offboard + Arm。

## 5. Depth Camera 接入

通过 `ros_gz_bridge` 把 Gazebo `/depth_camera/points` 转成 ROS 2 `PointCloud2`，再由 `gazebo_cloud_to_ego_cloud.py` 做外参、world 坐标变换、距离抽样和地面过滤，输出 EGO 所需 `/drone_0_pcl_render_node/cloud`。

## 6. 点云漂移问题定位

早期版本使用 PX4 `vehicle_local_position.heading` 旋转点云。实测发现同一面静态墙会随飞机运动而漂移，occupancy map 中产生幽灵障碍物，说明点云 world 变换的航向来源不可靠。

## 7. Gazebo dynamic_pose 修复

提交 `214ab1f` 把位姿来源切换为 `/world/simple_wall/dynamic_pose/info` 的 Gazebo Ground Truth Pose，使用平移和归一化四元数构造刚体变换。修复后墙体点云与 `occupancy_inflate` 基本对齐且不再漂移。

## 8. yaw follow velocity

提交 `2b2d9e9` 使用 EGO 水平速度方向生成 yaw，并完成 ENU→NED 航向转换。低速时保持上一次有效 yaw，避免抖动；Gazebo 中机头开始沿轨迹方向转向，前视相机持续看向飞行方向。

## 9. simple_wall 避障成功

最终成功版本组合如下：

- bridge 分支：`yaw_follow_velocity`，最新源码提交 `d6173ae`；
- EGO 目标点：`(20, 12, 1)`；local map 范围 `10/10/3 m`；膨胀半径 `0.2 m`；
- OakD-Lite：关闭 RGB，depth `80×60`、10 Hz、量程 0.2～19.1 m；
- 点云 pose：Gazebo `simple_wall` dynamic pose；
- Offboard：解锁后 hold 6 秒，再跟随 EGO 轨迹；yaw 跟随速度方向。

验证结果：点云稳定、墙体进入 occupancy、EGO 产生绕墙轨迹、PX4 Offboard 成功跟踪、机头随轨迹转向，最终到达目标且没有撞墙。
