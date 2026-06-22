# 面试准备：EGO Planner + PX4 SITL Bridge

以下问答以本仓库的实际实现为基础。回答时先讲“为什么”，再讲接口与验证证据。

当前成功版本的面试证据链：bridge 分支 `yaw_follow_velocity` 包含 Gazebo pose 修复提交 `214ab1f` 与 yaw 跟随提交 `2b2d9e9`；EGO 的成功配置快照位于 `config/ego_planner/`；OakD-Lite depth-only 配置位于 `config/gazebo/OakD-Lite/model.sdf`；最终效果是 simple_wall 绕障、到点且不碰撞。

接口速记：`VehicleLocalPosition` 提供 PX4 本地 NED 位置/速度和有效性标志；`VehicleStatus` 提供解锁与导航状态，bridge 用它检测首次 Arm、启动 6 秒 hold 计时，并在 Disarm 后复位。

仿真桥接速记：`ros_gz_bridge` 把 Gazebo depth cloud 和 `dynamic_pose` 转成 ROS 2 的 `PointCloud2`/`TFMessage`；它只负责消息系统转换，world 坐标变换由自写 cloud bridge 完成。

## 1. 这个项目解决了什么问题？

它打通 Gazebo 深度传感器、EGO Planner 与 PX4 Offboard：把 PX4 状态转换成 EGO 可用的 ENU odom，把深度点云转换到稳定 world 坐标，再把 EGO 轨迹转换为 PX4 NED 设定值，实现单墙自主绕障。

## 2. 为什么不把 PX4 和 EGO 的完整源码放进仓库？

它们是独立上游项目，体积大、版本复杂。仓库只保留自写适配层与可复现实验资产，依赖通过文档和版本约束说明，边界更清楚，也避免复制上游历史和构建产物。

## 3. 为什么需要 `px4_msgs`？

ROS 2 节点必须使用与 PX4 uORB 对应的强类型消息，例如 `VehicleLocalPosition`、`TrajectorySetpoint`、`VehicleCommand`。`px4_msgs` 提供这些 `.msg` 定义和生成后的语言类型。

## 4. 为什么 `px4_msgs` 必须与 PX4 版本匹配？

PX4 的消息字段和版本后缀可能变化。定义不匹配时，轻则 topic 名不同，重则 DDS 类型哈希或字段布局不兼容，节点即使能编译也收不到正确数据。

## 5. 为什么需要 `px4_ros_com`？

它提供 PX4 与 ROS 2 通信的配套示例、模板和桥接开发参考。当前运行链路的数据传输核心是 PX4 uXRCE-DDS client + Micro XRCE-DDS Agent，bridge 代码本身主要直接依赖 `px4_msgs`；`px4_ros_com` 更多承担开发与集成参考角色，而不是每个节点都必须 import 的运行时库。

## 6. Micro XRCE-DDS Agent 的作用是什么？

PX4 内的 uXRCE-DDS client 通过 UDP 与 Agent 通信，Agent 把 PX4 侧数据接入 DDS/ROS 2 域。没有 Agent，`/fmu/in/*` 和 `/fmu/out/*` 通常不会在 ROS 2 图里正常出现。

## 7. PX4 Offboard 模式的核心约束是什么？

外部控制器必须持续发送控制模式心跳和有效设定值，PX4 才允许进入并保持 Offboard。信号低于要求或超时会触发退出/ failsafe，因此不能只发一次 setpoint。

## 8. `OffboardControlMode` 表示什么？

它告诉 PX4 外部控制器准备控制哪一层。本项目设置 `position=True`，其他控制层为 false，表示提供位置设定值。

## 9. `TrajectorySetpoint` 是什么？

它是 PX4 轨迹控制器的输入，可包含位置、速度、加速度、jerk、yaw 和 yaw speed。本项目只控制位置与 yaw，其余未控制字段填 `NaN`，避免错误地同时约束多个层级。

## 10. 为什么 setpoint 以 20 Hz 发布？

Offboard 需要连续数据流，20 Hz 高于最低心跳要求，并能平滑承接 EGO 轨迹。还要通过实测确认系统负载下频率稳定，而不是只看定时器配置。

## 11. 为什么 Arm 前也发布悬停 setpoint？

PX4 切入 Offboard 前需要先检测到稳定的外部控制信号。提前发布安全悬停点能建立数据流，并避免一解锁就使用尚未准备好的轨迹。

## 12. 为什么解锁后还要 hold 6 秒？

给起飞和状态收敛留出时间，避免刚离地就追逐横向轨迹。6 秒是当前演示调出的策略参数，工程化后应参数化并用状态条件替代纯固定时间。

## 13. 为什么重复发送 Offboard 与 Arm 命令？

一次性命令可能早于 Offboard 信号建立或遇到 preflight 条件未满足。当前实现通过有限时间内重复请求提高演示成功率；真机应读取 ACK 和状态，做有界、可诊断的状态机。

## 14. EGO Planner 的主要输入是什么？

本项目给它 ENU world 坐标下的里程计 `/drone_0_visual_slam/odom` 和障碍点云 `/drone_0_pcl_render_node/cloud`，另外还需要目标点与对应配置。

## 15. EGO Planner 的主要输出是什么？

包括规划轨迹/bspline、位置控制命令 `/drone_0_planning/pos_cmd`，以及用于观察的 grid map/occupancy 相关 topic。bridge 使用 `PositionCommand` 作为 PX4 设定值来源。

## 16. 点云如何变成 occupancy map？

传感器点先转到一致的 world/map frame，再做范围过滤、降采样和地面过滤。EGO 的 grid map 模块把落入栅格的障碍观测更新为占据状态，供轨迹优化查询碰撞距离。

## 17. `occupancy_inflate` 是什么？

它是膨胀后的占据区域：在原始障碍周围按安全半径扩张，等价于把有体积的无人机简化为点来规划。膨胀过小会擦碰，过大则可能把可行通道封死。

## 18. 为什么点云必须转换到 world 坐标？

如果每帧都留在随飞机运动的相机坐标，历史点无法与当前位置一致融合，静态墙会在地图中移动。统一到 world 后，同一静态表面应落在稳定位置。

## 19. 为什么使用 Gazebo Pose 替代 PX4 heading？

实测发现当前仿真里的 PX4 heading 不可靠，导致点云旋转错误、墙体漂移和幽灵障碍。Gazebo Ground Truth 提供完整且稳定的仿真位姿，适合验证点云处理链路。

## 20. Gazebo Ground Truth 能直接用于真机吗？

不能。它是仿真特权信息。真机应由 VIO、LiDAR-inertial odometry、GNSS/INS 或融合状态估计器提供可靠位姿，并包含时间同步、协方差和失效检测。

## 21. 为什么不直接套用 optical-frame 旋转？

对实际 topic 做定向测试后发现其点坐标表现为 body-like frame。坐标变换必须以数据和 frame 定义为准，不能只凭传感器名称假设；否则会把正确数据再错误旋转一次。

## 22. 点云过滤做了什么？

保留 0.1～20 m 范围，每 5 个点取 1 个，并过滤 world 高度低于 0.1 m 的点。这降低负载并减少地面进入地图，但阈值当前仍是演示参数。

## 23. 为什么要过滤地面？

EGO 若把地面当障碍，机体附近会形成大面积 occupancy，导致无路可走或轨迹被向上推。过滤必须在正确 world 坐标和外参基础上进行。

## 24. NED 与 ENU 如何转换？

位置和速度使用 `x_enu=y_ned`、`y_enu=x_ned`、`z_enu=-z_ned`。它表示交换北/东轴并反转上下轴。发送设定值回 PX4 时应用对应逆变换。

## 25. yaw 如何从 ENU 转到 NED？

本项目使用 `yaw_ned = π/2 - yaw_enu`。转换后还应测试东、北、西、南四个基准方向，防止只凭公式遗漏机体系约定或角度包络问题。

## 26. 为什么 yaw 跟随速度方向？

让机头沿轨迹切线方向转动，视觉效果自然，也适合前视深度相机持续观察前方路径。若相机全向或任务要求固定朝向，也可以使用独立 yaw 规划。

## 27. 为什么低速时保持上一次 yaw？

接近零速度时，`atan2(vy,vx)` 对噪声极敏感，微小速度变化会让角度大幅跳动。设置 0.05 m/s 阈值并保持上次有效值可抑制抖动。

## 28. 为什么 PX4 topic 使用 Best Effort QoS？

PX4 高频状态流通常采用传感器数据风格 QoS。订阅端使用 Best Effort、Volatile 能与发布端兼容，并避免可靠传输在慢订阅者情况下累积历史数据。

## 29. 如何验证整个链路而不是只看“飞机飞了”？

逐层检查 RTF、原始点云频率、pose、odom 轴向、world 点云稳定性、occupancy 对齐、EGO `pos_cmd`、setpoint 频率和 PX4 状态。最终用无碰撞到达目标与视频作为系统级证据。

## 30. 这个项目最关键的调试方法是什么？

把问题按数据链路拆层，先验证坐标和频率，再接规划和控制；用静态墙作为可测基准，区分传感器坐标错误、位姿错误、地图错误与控制错误，而不是同时改多个参数。

## 31. D455 替换 Gazebo Depth Camera 怎么做？

用 `realsense2_camera` 获取点云，标定 D455 到机体的外参，用 TF2 和真实状态估计把点云转到 world/map，增加深度噪声与空洞过滤，然后保持 EGO cloud topic 接口不变。还必须测量延迟和实际视场。

## 32. MID360 + FAST-LIO2 后续怎么接入？

先完成 LiDAR-IMU 时间同步与外参标定；FAST-LIO2 输出 odom 和去畸变点云；将 odom 适配为 EGO odom，将过滤点云送入 EGO cloud。若还要融合进 PX4 EKF2，需要处理外部视觉消息、协方差、延迟和 reset counter。

## 33. 真机部署还缺什么安全模块？

至少缺 setpoint/odom 超时、限速限高、地理围栏、碰撞紧急制动、RC 接管、独立急停、失联降落、传感器健康监测、状态机 ACK、日志审计和系统级故障注入测试。

## 34. 当前 odom 姿态为什么是单位四元数？

完整 PX4 FRD/NED 到 ROS FLU/ENU 姿态转换尚未实现。发布单位四元数比发布一个符号/轴向错误的四元数更可控，但只是临时限制，依赖真实姿态的功能不能据此上线。

## 35. 当前 cloud bridge 有什么脆弱假设？

它取 `TFMessage.transforms[0]` 作为机体 pose。如果 Gazebo 输出顺序变化，就可能取错实体。下一步应按模型名或 child frame 精确匹配，并对 pose 时间戳和点云时间戳做同步。

## 36. 如何把这个 demo 提升为可维护工程？

固定依赖版本；参数化 topic、外参与阈值；用 launch 管理启动；增加坐标转换单元测试和 rosbag 回放；CI 做 Python/XML/SDF 检查；记录频率、延迟和 failsafe 指标；明确 SITL、HITL 与真机的验证门槛。
