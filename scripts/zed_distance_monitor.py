"""ROS 2 node to monitor nearest obstacle distance from a ZED point cloud stream.

This script is designed for Windows installations that run the ZED SDK ROS 2
wrapper. It subscribes to a ``sensor_msgs/msg/PointCloud2`` topic, extracts a
sample of the incoming points, and reports the minimum Euclidean distance to
any detected point.  The output can be logged to the console or optionally sent
as a ROS 2 parameter event.
"""

from __future__ import annotations

import argparse
import math
from typing import Iterable, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ZEDDistanceMonitor(Node):
    """Node that reports the minimum range detected in a point cloud stream."""

    def __init__(self, topic: str, sample_ratio: float, frame_id: str | None) -> None:
        super().__init__("zed_distance_monitor")
        self._sample_ratio = float(np.clip(sample_ratio, 0.0, 1.0))
        self._expected_frame = frame_id

        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self._on_point_cloud,
            qos_profile=10,
        )

        self.get_logger().info(
            "Listening for point clouds on %s (sample ratio %.2f)",
            topic,
            self._sample_ratio,
        )

    def _iter_points(self, msg: PointCloud2) -> Iterable[Tuple[float, float, float]]:
        if self._sample_ratio <= 0.0:
            return []

        for idx, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if self._sample_ratio < 1.0 and idx % max(int(1.0 / self._sample_ratio), 1) != 0:
                continue
            yield float(point[0]), float(point[1]), float(point[2])

    def _on_point_cloud(self, msg: PointCloud2) -> None:
        if self._expected_frame and msg.header.frame_id != self._expected_frame:
            self.get_logger().debug(
                "Ignoring point cloud from unexpected frame '%s'", msg.header.frame_id
            )
            return

        min_distance = math.inf
        for x, y, z in self._iter_points(msg):
            distance = math.sqrt(x * x + y * y + z * z)
            if distance < min_distance:
                min_distance = distance

        if math.isinf(min_distance):
            self.get_logger().warn("No valid points received in point cloud frame.")
        else:
            self.get_logger().info("Nearest detected point: %.2f m", min_distance)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor nearest point distance from ZED.")
    parser.add_argument(
        "--topic",
        default="/zed/zed_node/point_cloud/cloud_registered",
        help="PointCloud2 topic published by the ZED SDK wrapper.",
    )
    parser.add_argument(
        "--sample-ratio",
        type=float,
        default=0.1,
        help="Fraction of points to sample (0-1). Lower values use less CPU.",
    )
    parser.add_argument(
        "--frame-id",
        default=None,
        help="Optional frame_id to filter point clouds (e.g., 'zed_camera_center').",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = ZEDDistanceMonitor(args.topic, args.sample_ratio, args.frame_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down distance monitor.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
