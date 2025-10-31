# Scripts Overview

This reference covers the utility scripts that live in the `scripts/` directory. Each
entry summarizes what the script does, the primary dependencies it expects, and how
to launch it. Unless stated otherwise, run Python helpers with `python3` from the
root of the workspace while your ROS 2 environment is sourced.

## `export_to_tensorrt.py`

Exports a trained model into a TensorRT engine. The function is currently a
placeholder that raises `NotImplementedError`, making it a scaffold for wiring in
your own PyTorch → ONNX → TensorRT toolchain. Provide the path to the trained model
and optionally tweak the output directory and precision flag.

```bash
python3 scripts/export_to_tensorrt.py <path/to/checkpoint>.pt --output deploy/tensorrt --precision fp16
```

## `fusion_distance_check.py`

Spins up a ROS 2 node that subscribes to `/scan` (LiDAR) and the ZED depth topic
(`/zed/zed_node/depth/depth_registered`). It samples a narrow front-facing window
from both sensors, computes the average range, and logs the delta once per second.
Optional `cv_bridge` support is used when available; otherwise the depth image is
manually decoded.

```bash
python3 scripts/fusion_distance_check.py
```

## `rosbag_to_coco.py`

Stub for converting ROS 2 bag files into a COCO-style dataset. It establishes the
CLI structure—accepting a bag path and output directory—but leaves the conversion
logic for your application. Replace the `NotImplementedError` with processing that
iterates over the topics you need and writes COCO JSON/imagery.

```bash
python3 scripts/rosbag_to_coco.py <path/to/bag.db3> --output datasets/coco
```

## `rosbag_to_kitti.py`

Similar to the COCO helper, this script scaffolds ROS bag export into KITTI format.
Fill in the conversion routine to emit RGB frames, depth, and poses that match your
topics before running it on recorded data.

```bash
python3 scripts/rosbag_to_kitti.py <path/to/bag.db3> --output datasets/kitti
```

## `windows_yolov8_cam_sub_.py`

Lightweight YOLOv8 visualizer intended for quick experiments. It subscribes to
`/zed/zed_node/left_raw/image_raw_color`, converts the byte buffer into a NumPy
array, runs YOLOv8 Nano, and displays the annotated frame. This variant does not
handle varying encodings and assumes `bgr8` image data, making it best suited for
controlled setups.

Dependencies: `rclpy`, `sensor_msgs`, `opencv-python`, `numpy`, `ultralytics`.

```bash
python3 scripts/windows_yolov8_cam_sub_.py
```

## `windows_yolov8_cam_sub_ZED.py`

A more robust Windows-friendly subscriber for ZED image topics. It loads a YOLOv8
model, optionally uses `cv_bridge` for decoding, and falls back to manual handling
of several common encodings (`bgr8`, `rgb8`, `rgba8`, `yuv422`). Bounding boxes and
labels are rendered in an OpenCV window.

Dependencies: `rclpy`, `sensor_msgs`, `opencv-python`, `numpy`, `ultralytics`,
(optional) `cv_bridge`.

```bash
python3 scripts/windows_yolov8_cam_sub_ZED.py
```

## `zed_distance_monitor.py`

ROS 2 node that listens to a ZED point cloud (default
`/zed/zed_node/point_cloud/cloud_registered`). It optionally filters by frame ID,
randomly samples points according to `--sample-ratio`, and logs the minimum
Euclidean distance observed. Use it to watch for nearby obstacles on Windows
setups.

Dependencies: `rclpy`, `sensor_msgs_py`, `numpy`.

```bash
python3 scripts/zed_distance_monitor.py --topic /zed/zed_node/point_cloud/cloud_registered --sample-ratio 0.1
```

## `zed_depth_listener.py`

Subscribes to the ZED depth image topic and prints basic statistics (nearest object
and mean distance) while optionally visualizing the depth map. When `cv_bridge` is
not available it decodes the raw float buffer directly.

Dependencies: `rclpy`, `sensor_msgs`, `opencv-python`, `numpy`, (optional)
`cv_bridge`.

```bash
python3 scripts/zed_depth_listener.py
```

## `zed_depth_to_laserscan.py`

Creates a synthetic 2D `LaserScan` from the horizontal centerline of the ZED depth
image. Useful for prototyping when a LiDAR is not available. The generated scan is
published on `/zed/fake_laserscan` and reuses the depth message timestamp so you
can visualize it in RViz alongside other data.

Dependencies: `rclpy`, `sensor_msgs`, `numpy`, (optional) `cv_bridge`.

```bash
python3 scripts/zed_depth_to_laserscan.py
```

## `zed_yolo_listener.py`

Configurable YOLO inference node for ZED image topics. Command-line flags let you
pick the source topic, model file, score threshold, and device (`cpu`/`cuda`). Each
frame is annotated with bounding boxes and confidence scores, and pressing `q`
closes the OpenCV window.

Dependencies: `rclpy`, `sensor_msgs`, `opencv-python`, `ultralytics`, `cv_bridge`.

```bash
python3 scripts/zed_yolo_listener.py --topic /zed/zed_node/rgb/image_rect_color --model yolov8n.pt --device cuda:0
```

## `replay_bag_pipeline.sh`

Shell helper that rebuilds the workspace, sources the overlay, loops a ROS 2 bag
file, and launches `launch/mission_stack.launch.py` with simulated time. It expects
a rosbag path as the first argument and terminates the playback automatically when
the script exits.

```bash
bash scripts/replay_bag_pipeline.sh <path/to/bag>
```
