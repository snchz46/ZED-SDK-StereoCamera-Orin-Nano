#!/usr/bin/env python3
#
# zed_depth_listener.py
#
# Subscribes to the ZED depth topic and prints / visualizes distance information.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

try:
    from cv_bridge import CvBridge
    USE_BRIDGE = True
    bridge = CvBridge()
except ImportError:
    USE_BRIDGE = False
    print("[INFO] cv_bridge not found, using manual decoding instead.")


class DepthViewer(Node):
    def __init__(self):
        super().__init__('zed_depth_viewer')

        topic = '/zed/zed_node/depth/depth_registered'
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.get_logger().info(f"Subscribed to {topic}")

    def decode_depth(self, msg: Image):
        if USE_BRIDGE:
            try:
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                return depth
            except Exception as e:
                self.get_logger().warn(f"CvBridge failed: {e}")
                return None
        # Manual fallback
        data = np.frombuffer(msg.data, dtype=np.float32)
        depth = data.reshape(msg.height, msg.width)
        return depth

    def cb(self, msg: Image):
        depth = self.decode_depth(msg)
        if depth is None:
            return

        # Mask invalid or too far values
        depth = np.where((depth > 0.1) & (depth < 20.0), depth, np.nan)

        # Compute some quick stats
        min_dist = np.nanmin(depth)
        mean_dist = np.nanmean(depth)
        self.get_logger().info(f"Nearest object: {min_dist:.2f} m | Mean: {mean_dist:.2f} m")

        # Optional visualization (grayscale depth map)
        disp = np.nan_to_num(depth, nan=0.0, posinf=0.0)
        disp = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
        disp = np.uint8(disp)
        cv2.imshow("ZED Depth Map", disp)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = DepthViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
