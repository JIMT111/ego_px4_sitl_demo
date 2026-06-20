# 后续工作

## P0：真机前必须完成

- 完整实现并测试 PX4 FRD/NED 与 ROS FLU/ENU 的姿态四元数转换。
- 为 odom、点云、EGO `pos_cmd` 增加时间戳新鲜度检查；超时立即退出轨迹跟随并进入安全模式。
- 增加速度、加速度、jerk、yaw rate、飞行高度与地理围栏限制。
- 增加独立急停、RC 人工接管、失联处理、降落策略和 PX4 failsafe 联调。
- 将 Arm 从“重复请求”升级为显式状态机：检查 preflight、Offboard、Arm ACK 和超时。
- 对坐标系、外参、符号和单位做硬件在环验证，不能仅凭 RViz 视觉判断。

## P1：提高仿真鲁棒性

- dynamic pose 按 Gazebo 实体名/子 frame 精确选择，取消 `transforms[0]` 假设。
- 把 topic 名、相机外参、滤波阈值、悬停时长与目标频率改成 ROS 2 参数。
- 为消息超时、无效值、NaN、PX4 版本差异提供清晰日志。
- 加入 rosbag/mcap 固定数据集，建立点云变换与 NED/ENU 单元测试。
- 增加多墙、狭窄通道、动态障碍、低 RTF 和消息丢包场景。

## P2：D455 接入

1. 使用 `realsense2_camera` 发布 depth/point cloud。
2. 标定 D455 相对机体的外参，并通过 TF 发布。
3. 用 TF2 把点云转换到状态估计器的 world/map frame。
4. 处理深度噪声、空洞、最小/最大距离、体素降采样和地面分割。
5. 保持输出接口 `/drone_0_pcl_render_node/cloud`，让 EGO 上层尽量不变。
6. 测量端到端延迟和实际更新率，按速度重新评估安全距离。

## P3：MID360 + FAST-LIO2 接入

1. MID360 驱动发布点云与 IMU，完成时间同步和 LiDAR-IMU 外参标定。
2. FAST-LIO2 输出稳定的里程计/地图坐标系与去畸变点云。
3. 将 FAST-LIO2 odom 适配为 EGO 所需 `/drone_0_visual_slam/odom`。
4. 将去畸变、裁剪、降采样后的障碍点云送入 EGO cloud topic。
5. 明确 map/odom/base_link/lidar frame 关系，避免把局部漂移误当作障碍运动。
6. 若将外部视觉/里程计融合进 PX4 EKF2，还要处理 MAVLink/ROS 2 接口、协方差、延迟和重置计数。

## P4：工程化

- 建立 launch 文件，一次启动并检查所有 bridge。
- 增加容器或可复现依赖清单，固定 PX4、`px4_msgs`、EGO 和 ROS 2 版本。
- CI 执行 Python 静态检查、XML/SDF 检查和离线消息变换测试。
- 输出结构化运行指标：RTF、topic 频率、setpoint age、规划耗时和最小障碍距离。
- 将演示视频压缩成浏览器普遍支持的 H.264 MP4，并保留原始证据哈希。
