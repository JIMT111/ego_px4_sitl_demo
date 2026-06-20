#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


class Px4ToEgoOdom(Node):
    def __init__(self):
        super().__init__('px4_to_ego_odom')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_attitude = None

        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos,
        )

        self.att_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            px4_qos,
        )

        self.pub = self.create_publisher(
            Odometry,
            '/drone_0_visual_slam/odom',
            10,
        )

        self.get_logger().info('PX4 -> EGO odom bridge started.')
        self.get_logger().info('Subscribe: /fmu/out/vehicle_local_position_v1')
        self.get_logger().info('Subscribe: /fmu/out/vehicle_attitude')
        self.get_logger().info('Publish: /drone_0_visual_slam/odom')

    def attitude_callback(self, msg: VehicleAttitude):
        self.latest_attitude = msg

    def position_callback(self, msg: VehicleLocalPosition):
        if not msg.xy_valid or not msg.z_valid:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'

        # PX4 NED -> ROS/EGO ENU
        odom.pose.pose.position.x = float(msg.y)
        odom.pose.pose.position.y = float(msg.x)
        odom.pose.pose.position.z = float(-msg.z)

        if msg.v_xy_valid:
            odom.twist.twist.linear.x = float(msg.vy)
            odom.twist.twist.linear.y = float(msg.vx)

        if msg.v_z_valid:
            odom.twist.twist.linear.z = float(-msg.vz)

        # Temporary orientation handling:
        # PX4 VehicleAttitude quaternion is FRD/NED based.
        # For now, keep it identity until we add the full NED->ENU attitude conversion.
        # This avoids publishing a wrong quaternion.


        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = Px4ToEgoOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
