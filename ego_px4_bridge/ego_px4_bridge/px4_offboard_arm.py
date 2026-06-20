#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, VehicleCommand


class Px4OffboardArm(Node):
    def __init__(self):
        super().__init__('px4_offboard_arm')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )

        self.counter = 0
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info('PX4 robust offboard arm node started.')
        self.get_logger().info('Continuously publishing offboard_control_mode.')
        self.get_logger().info('Repeatedly sending Offboard + Arm commands.')

    def timer_callback(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        mode = OffboardControlMode()
        mode.timestamp = now_us
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False
        self.offboard_pub.publish(mode)

        self.counter += 1

        # 前 1 秒只发 offboard_control_mode，让 PX4 先检测到 offboard 信号。
        if self.counter < 20:
            return

        # 第 1 秒到第 5 秒，每 0.5 秒重复发一次 Offboard 和 Arm。
        # 避免只发一次时机错过。
        if self.counter <= 100 and self.counter % 10 == 0:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )
            self.get_logger().info('Sent command: set Offboard mode')

            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )
            self.get_logger().info('Sent command: arm')

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Px4OffboardArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
