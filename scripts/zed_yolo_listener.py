"""Apply YOLO detections to images published by the ZED SDK ROS 2 wrapper."""

from __future__ import annotations

import argparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover - informative message for runtime users
    raise SystemExit(
        "The 'ultralytics' package is required for YOLO inference. Install it with "
        "'pip install ultralytics'."
    ) from exc


class ZEDYOLOListener(Node):
    """Node that runs YOLO inference on incoming ROS 2 image messages."""

    def __init__(self, topic: str, model_path: str, score_threshold: float, device: str):
        super().__init__("zed_yolo_listener")
        self._bridge = CvBridge()
        self._model = YOLO(model_path)
        self._model.to(device)
        self._score_threshold = score_threshold

        self.subscription = self.create_subscription(
            Image,
            topic,
            self._on_image,
            qos_profile=10,
        )

        self.get_logger().info(
            "Running YOLO model '%s' on topic %s (score ≥ %.2f)",
            model_path,
            topic,
            self._score_threshold,
        )

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self._model.predict(frame, verbose=False)

        annotated = frame.copy()
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                score = float(box.conf.cpu().numpy())
                if score < self._score_threshold:
                    continue

                cls_id = int(box.cls.cpu().numpy())
                label = self._model.names.get(cls_id, str(cls_id))
                x1, y1, x2, y2 = box.xyxy.cpu().numpy().astype(int).ravel()

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f"{label}: {score:.2f}",
                    (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        cv2.imshow("ZED YOLO Detections", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("'q' pressed - exiting YOLO listener.")
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run YOLO on ZED ROS 2 image topics.")
    parser.add_argument(
        "--topic",
        default="/zed/zed_node/rgb/image_rect_color",
        help="Image topic published by the ZED SDK wrapper.",
    )
    parser.add_argument(
        "--model",
        default="yolov8n.pt",
        help="Path to a YOLO model file compatible with ultralytics.",
    )
    parser.add_argument(
        "--score-threshold",
        type=float,
        default=0.25,
        help="Discard detections below this confidence.",
    )
    parser.add_argument(
        "--device",
        default="cpu",
        help="Device string passed to ultralytics (e.g., 'cpu', 'cuda:0').",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ZEDYOLOListener(args.topic, args.model, args.score_threshold, args.device)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user - shutting down.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
