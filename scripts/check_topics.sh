#!/usr/bin/env bash
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 不在 PATH 中；请先 source /opt/ros/jazzy/setup.bash 和相关工作区。" >&2
  exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "缺少 coreutils timeout，无法做有界频率采样。" >&2
  exit 1
fi

topics=(
  /depth_camera/points
  /world/simple_wall/dynamic_pose/info
  /drone_0_visual_slam/odom
  /drone_0_pcl_render_node/cloud
  /drone_0_grid/grid_map/occupancy_inflate
  /drone_0_planning/pos_cmd
  /fmu/in/trajectory_setpoint
  /fmu/in/offboard_control_mode
)

echo "每个 topic 采样 6 秒；无数据会显示超时。"
for topic in "${topics[@]}"; do
  echo
  echo "=== ${topic} ==="
  timeout 6 ros2 topic hz "${topic}" || true
done
