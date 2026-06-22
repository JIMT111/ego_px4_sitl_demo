# 调试记录

本文件记录 simple_wall 成功版本中真实遇到的问题、判断依据和最终处理方法。

## 1. GitHub clone 出现 TLS/握手失败

### 现象

Ubuntu 上 `git clone`、`git fetch` 或依赖下载报 TLS handshake、connection reset、连接超时等错误。

### 处理

Mac 代理软件打开 **Allow LAN**，Ubuntu 使用 Mac 局域网代理 `192.168.1.106:7897`：

```bash
git config --global http.proxy http://192.168.1.106:7897
git config --global https.proxy http://192.168.1.106:7897
git config --global http.version HTTP/1.1
```

先用 `curl -I -x http://192.168.1.106:7897 https://github.com` 验证代理确实可达，不要只设置变量后直接重试。

## 2. Gazebo Fuel 模型下载失败

### 现象

Gazebo 启动时 Fuel 资源下载超时，模型或材质缺失。

### 处理

同样走 Mac 的 Allow LAN 代理：

```bash
export http_proxy=http://192.168.1.106:7897
export https_proxy=http://192.168.1.106:7897
export HTTP_PROXY=$http_proxy
export HTTPS_PROXY=$https_proxy
```

在启动 PX4/Gazebo 的同一终端导出变量，并先测试 Fuel/GitHub HTTPS 可达性。

## 3. RTF 降至 0.04～0.05，PX4 飞行不稳定

### 原因

OakD-Lite 同时运行 IMX214 RGB 与 depth camera，渲染和传感器计算量过大。仿真时钟变慢后，PX4 local position 与控制链路的有效频率一起下降。

### 处理

- 注释/关闭 IMX214 RGB camera。
- 只保留 StereoOV7251 depth camera。
- depth 分辨率改为 `80x60`，更新率 `10 Hz`，裁剪范围 `0.2～19.1 m`。
- 成功配置保存于 `config/gazebo/OakD-Lite/model.sdf`。

### 结果

RTF 从约 0.04～0.05 恢复到约 1.0，状态和控制频率恢复可用。

## 4. 点云漂移并产生幽灵障碍物

### 原因

早期版本使用 PX4 `vehicle_local_position.heading` 旋转点云。该 heading 在当前仿真链路中不可靠，同一面静态墙被投到不同 world 位置；历史 occupancy 未立即消失，于是出现漂移和幽灵墙。

### 处理

- 不再使用 PX4 heading 作为点云姿态。
- 通过 `ros_gz_bridge` 桥接 `/world/simple_wall/dynamic_pose/info`，消息类型为 `tf2_msgs/msg/TFMessage`。
- `gazebo_cloud_to_ego_cloud.py` 使用 Gazebo Ground Truth 平移和归一化四元数构造 `R_world_from_base`。
- 输出点云固定为 `frame_id=world`。

### 结果

墙体点云在 world 中稳定，`occupancy_inflate` 与实际墙体基本重合。

## 5. simple_wall / walls 切换后 pose 错误

dynamic pose topic 含 world 名。切换 world 时必须同步切换：

```text
simple_wall → /world/simple_wall/dynamic_pose/info
walls       → /world/walls/dynamic_pose/info
```

Gazebo 启动参数、`ros_gz_bridge parameter_bridge` 命令和 cloud 节点订阅必须使用同一个 world 名；只改其中一处会导致 pose 无数据或使用错误世界的位姿。

## 6. `/drone_0_pcl_render_node/cloud` 无输出

cloud 节点只有同时收到 depth cloud 和 dynamic pose 后才发布。按顺序检查：

```bash
ros2 topic hz /depth_camera/points
ros2 topic hz /world/simple_wall/dynamic_pose/info
ros2 topic info /depth_camera/points -v
ros2 topic info /world/simple_wall/dynamic_pose/info -v
ros2 topic hz /drone_0_pcl_render_node/cloud
```

若前两个 topic 之一无数据，检查对应 `ros_gz_bridge` 是否启动、消息类型是否写对、world 名是否一致。修改源码后还要重建 `ego_px4_bridge_ws` 并重新 source，避免继续运行旧 install 产物。

## 7. `ground_z_min` 太大，墙底缺失

### 现象

RViz 中墙体点云下沿被切掉，occupancy 墙底出现缺口，规划线可能从墙底穿过。

### 原因与处理

地面过滤在 world z 上执行。`ground_z_min` 过大不仅滤掉地面，也滤掉墙体靠近地面的有效点。成功版本使用 `ground_z_min = 0.1`；调整时必须同时观察原始 cloud、过滤后 cloud、`occupancy_inflate` 和规划线，不能只看点云是否“干净”。

## 8. yaw 固定为 0，相机不看飞行方向

### 现象

位置轨迹能绕墙，但机头始终固定，前视深度相机没有持续朝向运动方向。

### 处理

- 取 EGO 水平速度方向：`yaw_enu = atan2(vy, vx)`。
- 转为 PX4 NED yaw：`yaw_ned = π/2 - yaw_enu`。
- 水平速度小于 `0.05 m/s` 时保持最近一次有效 yaw，避免低速噪声引起机头抖动。

提交 `2b2d9e9` 完成 yaw follow velocity 后，Gazebo 机头能随轨迹转向。

## 9. PX4/Gazebo 旧进程残留，世界和 topic 混乱

### 现象

重启后出现多个 world、旧模型仍存在、topic 发布者数量异常或 PX4 连接到旧 Gazebo 实例。

### 处理

先确认没有需要保留的仿真任务，再清理残留进程，然后从 PX4/Gazebo 开始按顺序重启：

```bash
pkill -f px4 || true
pkill -f 'gz sim' || true
pkill -f MicroXRCEAgent || true
pkill -f ros_gz_bridge || true
```

清理后用 `ps`、`ros2 node list` 和 `ros2 topic info -v` 确认没有重复实例。

## 10. `PX4_GZ_MODEL_POSE` 生效，但 RViz odom 起点不变

### 原因

`PX4_GZ_MODEL_POSE` 设置的是 Gazebo world 中的模型初始 pose；PX4 `VehicleLocalPosition` 使用启动时建立的本地 NED 原点，启动后通常归零。因此 Gazebo world pose 与 PX4 local position 不是同一个原点，RViz 中由 PX4 local position 转出的 odom 不会自动显示 Gazebo 的绝对初始偏移。

### 结论

不要把“RViz odom 从 0 开始”误判为 `PX4_GZ_MODEL_POSE` 未生效。分别检查 Gazebo dynamic pose 和 PX4 local position；若系统需要统一绝对坐标，应显式维护两者之间的原点变换。

## 11. 偶尔无法进入 Offboard 或 Arm

PX4 要求切入 Offboard 前已连续收到控制模式心跳和有效 setpoint。成功版本采用：

- setpoint bridge 在 Arm 前持续发送安全悬停点；
- arm bridge 前 1 秒只发送 `OffboardControlMode`；
- 随后 4 秒每 0.5 秒重复请求 Offboard 与 Arm；
- 等 EGO 已出现规划线后再启动 arm bridge；
- 解锁后 hold 6 秒，再跟随 EGO 轨迹。

## 12. 推荐的逐层检查顺序

1. 只有一套 PX4/Gazebo/Agent/bridge 进程。
2. Gazebo RTF 接近 1.0。
3. `/depth_camera/points` 约 10 Hz。
4. 当前 world 的 dynamic pose 持续更新。
5. `/drone_0_visual_slam/odom` 约 50 Hz，NED→ENU 轴向正确。
6. world 点云不漂移，墙底没有被错误滤掉。
7. `occupancy_inflate` 与墙体对齐。
8. EGO 有 bspline 和 `/drone_0_planning/pos_cmd`。
9. `/fmu/in/trajectory_setpoint` 约 20 Hz。
10. 最后启动 Offboard/Arm，观察 hold、跟踪、yaw 和到点结果。

## 13. 当前已知技术债

- odom 姿态尚未做完整 FRD/NED → FLU/ENU 四元数转换。
- dynamic pose 目前读取 `transforms[0]`，应改为按模型/子 frame 名筛选。
- 点云外参、抽样率、距离阈值、地面阈值仍是代码常量。
- 缺少 setpoint/odom 超时、地理围栏、限速限高、RC 接管和独立急停。
- 缺少自动测试、rosbag 回放测试和故障注入测试。
