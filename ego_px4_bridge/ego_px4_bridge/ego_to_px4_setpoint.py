#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry


class EgoToPx4Setpoint(Node):
    def __init__(self):
        super().__init__('ego_to_px4_setpoint_bridge')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_odom = None
        self.latest_cmd = None

        self.is_armed = False
        self.armed_time_ns = None
        self.last_pub_ns = 0

        self.hold_x = None
        self.hold_y = None
        self.hold_z = 1.0
        self.hold_seconds_after_arm = 6.0

        # 最近一次有效 ENU 航向。低速时保持它，避免 yaw 抖动或回正。
        self.last_yaw_enu = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone_0_visual_slam/odom',
            self.odom_callback,
            10,
        )

        self.cmd_sub = self.create_subscription(
            PositionCommand,
            '/drone_0_planning/pos_cmd',
            self.cmd_callback,
            10,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            px4_qos,
        )

        self.pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('EGO -> PX4 setpoint bridge started.')
        self.get_logger().info('Before arm: publish safe hold setpoint.')
        self.get_logger().info('After arm: hold 6s, then follow EGO trajectory.')
        self.get_logger().info('Follow mode: yaw follows EGO velocity direction.')

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

        if self.hold_x is None:
            self.hold_x = float(msg.pose.pose.position.x)
            self.hold_y = float(msg.pose.pose.position.y)
            self.get_logger().info(
                f'Set hold point ENU: x={self.hold_x:.2f}, '
                f'y={self.hold_y:.2f}, z={self.hold_z:.2f}'
            )

    def cmd_callback(self, msg: PositionCommand):
        self.latest_cmd = msg

    def status_callback(self, msg: VehicleStatus):
        armed_now = msg.arming_state == 2

        if armed_now and not self.is_armed:
            self.armed_time_ns = self.get_clock().now().nanoseconds
            self.get_logger().info('PX4 armed. Start post-arm hold timer.')

        if not armed_now and self.is_armed:
            self.armed_time_ns = None
            self.get_logger().info('PX4 disarmed. Reset hold timer.')

        self.is_armed = armed_now

    def timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_pub_ns < 50_000_000:
            return
        self.last_pub_ns = now_ns

        if self.latest_odom is None or self.hold_x is None:
            return

        if not self.is_armed or self.armed_time_ns is None:
            self.publish_hold_setpoint()
            return

        elapsed_after_arm = (now_ns - self.armed_time_ns) / 1e9

        if elapsed_after_arm < self.hold_seconds_after_arm:
            self.publish_hold_setpoint()
            return

        if self.latest_cmd is not None:
            self.publish_ego_setpoint(self.latest_cmd)
        else:
            self.publish_hold_setpoint()

    def publish_hold_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # EGO / ROS ENU -> PX4 NED
        msg.position = [
            float(self.hold_y),
            float(self.hold_x),
            float(-self.hold_z),
        ]

        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]

        # 起飞悬停阶段先固定 yaw，避免原地乱转。
        msg.yaw = 0.0
        msg.yawspeed = math.nan

        self.pub.publish(msg)

    def publish_ego_setpoint(self, cmd: PositionCommand):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # EGO / ROS ENU -> PX4 NED
        msg.position = [
            float(cmd.position.y),
            float(cmd.position.x),
            float(-cmd.position.z),
        ]

        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]

        # EGO速度方向作为机头方向。
        # 速度太小时保持最近一次有效 yaw，避免 atan2 低速抖动。
        vx = float(cmd.velocity.x)
        vy = float(cmd.velocity.y)
        speed = math.sqrt(vx * vx + vy * vy)

        if speed > 0.05:
            self.last_yaw_enu = math.atan2(vy, vx)

        # ENU yaw -> PX4 NED yaw
        msg.yaw = float(math.pi / 2.0 - self.last_yaw_enu)
        msg.yawspeed = math.nan

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EgoToPx4Setpoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
