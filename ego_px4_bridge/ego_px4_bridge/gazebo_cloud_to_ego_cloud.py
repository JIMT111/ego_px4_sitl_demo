#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage


class GazeboCloudToEgoCloud(Node):
    def __init__(self):
        super().__init__('gazebo_cloud_to_ego_cloud')

        self.latest_transform = None

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.cloud_callback,
            10,
        )

        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/simple_wall/dynamic_pose/info',
            self.pose_callback,
            10,
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/drone_0_pcl_render_node/cloud',
            10,
        )

        # Confirmed by Test A:
        # /depth_camera/points camera_link is body-like, not optical.
        # Use identity instead of REP-103 optical rotation.
        self.r_base_from_camera = np.eye(3, dtype=np.float64)

        self.camera_translation_base = np.array(
            [0.13233, 0.0, 0.26078],
            dtype=np.float64,
        )

        self.min_range = 0.1
        self.max_range = 20.0
        self.ground_z_min =  0.1
        self.keep_every = 5

    def pose_callback(self, msg: TFMessage):
        if msg.transforms:
            self.latest_transform = msg.transforms[0].transform

    def cloud_callback(self, msg: PointCloud2):
        if self.latest_transform is None:
            return

        transform = self.latest_transform
        world_pos = np.array(
            [
                float(transform.translation.x),
                float(transform.translation.y),
                float(transform.translation.z),
            ],
            dtype=np.float64,
        )

        quaternion = np.array(
            [
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            ],
            dtype=np.float64,
        )
        quaternion_norm = np.linalg.norm(quaternion)
        if quaternion_norm < np.finfo(np.float64).eps:
            return

        x, y, z, w = quaternion / quaternion_norm

        r_world_from_base = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

        points_world = []

        for i, p in enumerate(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)):
            if i % self.keep_every != 0:
                continue

            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            r = math.sqrt(x * x + y * y + z * z)
            if r < self.min_range or r > self.max_range:
                continue

            p_camera = np.array([x, y, z], dtype=np.float64)
            p_base = self.r_base_from_camera @ p_camera + self.camera_translation_base
            p_world = world_pos + r_world_from_base @ p_base

            if p_world[2] < self.ground_z_min:
                continue

            points_world.append((float(p_world[0]), float(p_world[1]), float(p_world[2])))

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'

        out = point_cloud2.create_cloud_xyz32(header, points_world)
        self.cloud_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCloudToEgoCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
