# 调试记录

## 1. 现象：RTF 降至 0.04～0.05，PX4 飞行不稳定

### 原因

OakD-Lite 模型同时运行高负载 RGB 与 depth camera，Gazebo 计算量过大。仿真时钟变慢后，PX4 local position 与控制链路的有效频率同步下降。

### 处理

- 在 PX4 本地的 `Tools/simulation/gz/models/OakD-Lite/model.sdf` 中关闭 IMX214 RGB camera。
- 保留 depth camera。
- 将 depth 分辨率降低至 `80x60`。
- 调整 `update_rate` 和 `clip far`。

### 结果

RTF 恢复到约 1.0，位置消息与控制链路频率恢复可用。该模型文件属于 PX4 本地修改，不在本仓库上传范围内。

## 2. 现象：点云随飞机漂移，地图中出现幽灵障碍物

### 原因

最初使用 PX4 `vehicle_local_position.heading` 旋转点云，但该 heading 在当前仿真链路中不可靠。错误航向把同一面墙投到不同 world 位置，旧点又保留在 occupancy map 中，形成漂移与重影。

### 处理

- 启动 Gazebo dynamic pose bridge。
- 订阅 `/world/simple_wall/dynamic_pose/info`。
- 使用 Gazebo Ground Truth 四元数与平移执行完整刚体变换。

### 结果

墙体点云在 world 中稳定，`occupancy_inflate` 与实际墙体基本重合。

## 3. 现象：使用 optical-frame 旋转后点云方向错误

### 原因

实际 `/depth_camera/points` 的 `camera_link` 数据表现为 body-like 坐标，不符合预期 optical frame 轴向；无条件套用 REP-103 optical 旋转造成二次错误变换。

### 处理

根据定向测试将 `r_base_from_camera` 设为单位矩阵，只保留相机相对机体的平移外参。

## 4. 现象：偶尔无法进入 Offboard 或 Arm

### 原因

PX4 要求切入 Offboard 前已连续收到外部控制模式与有效 setpoint。只发送一次模式/解锁命令时，命令可能早于数据流建立或被 preflight 条件拒绝。

### 处理

- setpoint bridge 在 Arm 前持续发送悬停设定值。
- arm bridge 前 1 秒只发送 `OffboardControlMode`。
- 随后 4 秒内每 0.5 秒重复请求 Offboard 与 Arm。
- 操作上等待 EGO 已出现规划线再启动 arm bridge。

## 5. 现象：起飞后马上追轨迹，姿态变化过快

### 处理

检测到 `VehicleStatus.arming_state == 2` 后记录时间，先在固定 ENU 点悬停 6 秒，再切入 EGO 轨迹。

## 6. 现象：机头不跟路径或在低速时抖动

### 处理

- 用 EGO 水平速度方向 `atan2(vy, vx)` 生成 ENU yaw。
- 转成 PX4 NED yaw：`π/2 - yaw_enu`。
- 速度小于 0.05 m/s 时保持最近一次有效 yaw，避免对接近零的噪声做 `atan2`。

## 7. 推荐的逐层检查顺序

1. Gazebo RTF 接近 1.0。
2. `/depth_camera/points` 约 10 Hz。
3. `/world/simple_wall/dynamic_pose/info` 持续更新。
4. `/drone_0_visual_slam/odom` 约 50 Hz，轴向与实际运动一致。
5. world 点云不漂移，地面基本被滤除。
6. `occupancy_inflate` 与墙体对齐。
7. EGO 有 bspline 和 `pos_cmd`。
8. `/fmu/in/trajectory_setpoint` 达到 10～20 Hz。
9. 最后才运行 Offboard/Arm 节点。

## 8. 当前已知技术债

- odom 姿态尚未做完整 FRD/NED → FLU/ENU 四元数变换。
- dynamic pose 目前读取 `transforms[0]`，应改为按模型/子 frame 名筛选。
- 点云外参、抽样率、距离阈值、地面阈值仍是代码常量。
- 缺少自动测试、录包回放测试和故障注入测试。
