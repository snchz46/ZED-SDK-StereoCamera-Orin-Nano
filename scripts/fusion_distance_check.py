#!/usr/bin/env python3
#
# fusion_distance_check.py
#
# Subscribes to both LiDAR and ZED depth data and compares
# average front distances to verify alignment and consistency.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import numpy as np

try:
    from cv_bridge import CvBridge
    bridge = CvBridge()
except ImportError:
    bridge = None
    print("[INFO] cv_bridge not found, using manual decoding for ZED depth.")

class FusionCheck(Node):
    def __init__(self):
        super().__init__('fusion_distance_check')

        self.lidar_range = None
        self.depth_range = None

        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cb_lidar, 10)
        self.sub_depth = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.cb_depth, 10)
        self.timer = self.create_timer(1.0, self.compare)

    def cb_lidar(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        valid = np.logical_and(ranges > msg.range_min, ranges < msg.range_max)

        # front-facing ±5° window
        center_idx = len(ranges) // 2
        width = int((5.0 * np.pi / 180.0) / msg.angle_increment)
        window = ranges[center_idx - width:center_idx + width]
        window = window[np.isfinite(window)]

        if len(window) > 0:
            self.lidar_range = np.mean(window)

    def cb_depth(self, msg: Image):
        if bridge:
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        else:
            data = np.frombuffer(msg.data, dtype=np.float32)
            depth = data.reshape(msg.height, msg.width)

        # Take the central vertical slice
        row = depth[depth.shape[0] // 2, :]
        valid = (row > 0.1) & (row < 20.0)
        row = row[valid]

        if len(row) > 0:
            self.depth_range = np.nanmean(row[depth.shape[1]//2 - 10 : depth.shape[1]//2 + 10])

    def compare(self):
        if self.lidar_range is None or self.depth_range is None:
            return

        diff = abs(self.lidar_range - self.depth_range)
        self.get_logger().info(
            f"LIDAR: {self.lidar_range:.2f} m | ZED: {self.depth_range:.2f} m | Δ = {diff:.2f} m"
        )


def main():
    rclpy.init()
    node = FusionCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
