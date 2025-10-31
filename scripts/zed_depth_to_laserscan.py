#!/usr/bin/env python3
#
# zed_depth_to_laserscan.py
#
# Converts ZED depth images to a simulated 2D LaserScan message.
# Works as a rough LiDAR substitute for testing and visualization in RViz.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
import numpy as np

try:
    from cv_bridge import CvBridge
    USE_BRIDGE = True
    bridge = CvBridge()
except ImportError:
    USE_BRIDGE = False
    print("[INFO] cv_bridge not found, using manual decoding instead.")


class DepthToScan(Node):
    def __init__(self):
        super().__init__('zed_depth_to_scan')

        # Subscribe to ZED depth topic
        topic_depth = '/zed/zed_node/depth/depth_registered'
        self.sub_depth = self.create_subscription(Image, topic_depth, self.cb_depth, 10)
        self.get_logger().info(f"Subscribed to {topic_depth}")

        # Create publisher for simulated LaserScan
        self.pub_scan = self.create_publisher(LaserScan, '/zed/fake_laserscan', 10)

        # Camera intrinsics or approximations
        self.hfov_deg = 90.0   # typical horizontal field of view
        self.hfov = np.deg2rad(self.hfov_deg)

    def decode_depth(self, msg: Image):
        if USE_BRIDGE:
            try:
                return bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            except Exception as e:
                self.get_logger().warn(f"CvBridge failed: {e}")
                return None
        data = np.frombuffer(msg.data, dtype=np.float32)
        return data.reshape(msg.height, msg.width)

    def cb_depth(self, msg: Image):
        depth = self.decode_depth(msg)
        if depth is None:
            return

        # Clean invalid values
        depth = np.where((depth > 0.1) & (depth < 20.0), depth, np.nan)

        # Take one horizontal scan line (e.g. middle of image)
        row = depth[depth.shape[0] // 2, :]
        valid = ~np.isnan(row)
        row = row[valid]

        if row.size < 5:
            self.get_logger().warn("Not enough valid depth points.")
            return

        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = "zed_laserscan"
        scan.angle_min = -self.hfov / 2
        scan.angle_max = self.hfov / 2
        scan.angle_increment = self.hfov / len(row)
        scan.range_min = 0.1
        scan.range_max = 20.0
        scan.ranges = row.tolist()

        self.pub_scan.publish(scan)
        self.get_logger().info(f"Published scan with {len(row)} rays (min {np.nanmin(row):.2f} m)")

def main():
    rclpy.init()
    node = DepthToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
