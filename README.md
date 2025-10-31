# ZED Mini Autonomous Vehicle Platform on Jetson Orin Nano

A comprehensive workspace for building an autonomous ground vehicle using the **ZED Mini stereo camera** paired with an **NVIDIA Jetson Orin Nano** running **ROS 2 Humble**. The repository captures setup steps, launch files, utility scripts, and ongoing experiments for perception, mapping, planning, and control that leverage the ZED SDK.

## 🚗 Project Vision
- Deliver a reliable perception stack based on the ZED Mini for depth, visual odometry, and semantic understanding.
- Fuse camera data with onboard sensors to enable localization, obstacle avoidance, and navigation.
- Provide a reference architecture for small-scale autonomous vehicles (UGV/RC car) powered by the Jetson Orin Nano.
- Maintain reproducible workflows for data collection, training, and deployment of machine-learning models.

## 📦 Repository Contents
- `launch/` – ROS 2 launch files for the ZED Mini, perception pipeline, and vehicle bring-up.
- `config/` – Camera calibration, robot description, and parameter YAML files.
- `scripts/` – Helper tools for calibration, dataset recording, bag conversion, and diagnostics.
- `src/` – Custom ROS 2 packages for perception, sensor fusion, planning, and control.
- `notebooks/` – Jupyter notebooks for dataset exploration, neural network training, and algorithm prototyping.
- `docs/` – Additional guides, diagrams, and experimental notes.

> **Note:** Some directories will be added progressively as new capabilities are implemented.

## 🧰 Hardware
| Component | Notes |
|-----------|-------|
| NVIDIA Jetson Orin Nano (8 GB) | Host computer running Ubuntu 22.04 / JetPack 5.x |
| ZED Mini Stereo Camera | Provides RGB + depth, IMU data |
| Waveshare or custom RC chassis | Differential drive with motor controller (e.g., VESC, RoboClaw) |
| Wheel encoders / IMU | Optional for improved state estimation |
| LiPo battery + BEC | Power supply for Jetson and actuators |

## 🧪 Software Stack
- **OS:** Ubuntu 22.04 LTS (JetPack 5.x)
- **ROS 2 Humble** with `ros-base` and `ros-dev-tools`
- **ZED SDK** (>= 4.0) and `zed-ros2-wrapper`
- **CUDA**, **TensorRT**, and `torch` / `onnxruntime` for AI models
- **Colcon** for building ROS 2 workspaces
- Optional: `nav2`, `slam_toolbox`, `rclpy`, `rclcpp`, `foxglove_bridge`, `micro-ros-agent`

## 🛠️ Setup Instructions
1. **Flash Jetson Orin Nano** with JetPack 5.x and complete initial configuration.
2. **Install ROS 2 Humble** using the `ros-humble-desktop` meta-package or minimal `ros-base` variant.
3. **Install ZED SDK** following Stereolabs' instructions, then clone and build the [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-wrapper).
4. **Clone this repository** inside your ROS 2 workspace (e.g., `~/ros2_ws/src/`).
5. **Install dependencies**:
   ```bash
   sudo apt update && sudo apt install python3-vcstool python3-colcon-common-extensions ros-humble-navigation2
   rosdep install --from-paths src --ignore-src -r -y
   ```
6. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
7. **Configure udev rules** for the ZED Mini (optional but recommended).

## 📡 Bringing Up the ZED Mini
1. Connect the ZED Mini via USB-C and ensure the device enumerates (`lsusb`).
2. Launch the camera driver:
   ```bash
   ros2 launch zed_wrapper zedm.launch.py
   ```
3. Visualize streams in RViz2 or Foxglove Studio to verify RGB, depth, point cloud, and IMU data.
4. Record rosbags for dataset creation:
   ```bash
   ros2 bag record /zed/zed_node/rgb_raw/image_raw_color /zed/zed_node/depth/depth_registered
   ```

## 📋 ROS 2 Cheat Sheet para el ZED Wrapper

Consulta la [chuleta completa de comandos](docs/ros2-cheat-sheet.md) para ver combinaciones habituales del `zed-ros2-wrapper` y comandos complementarios para depuración.

## 🔧 Utility Scripts
The `scripts/` directory includes helper nodes that can be run directly on
Windows installations using the [ZED SDK ROS 2 wrapper](https://www.stereolabs.com/docs/ros2/).

- `zed_distance_monitor.py` subscribes to the registered point cloud topic and
  prints the minimum detected distance for quick obstacle awareness.
  ```powershell
  python scripts/zed_distance_monitor.py --topic /zed/zed_node/point_cloud/cloud_registered --sample-ratio 0.05
  ```
- `zed_yolo_listener.py` applies an [Ultralytics YOLO](https://docs.ultralytics.com/) model to the rectified RGB feed and
  displays annotated detections.
  ```powershell
  python scripts/zed_yolo_listener.py --model yolov8n.pt --device cuda:0
  ```

Both scripts require the ROS 2 Humble Python environment on Windows and the
dependencies listed in their docstrings (e.g., `ultralytics`, `opencv-python`,
`cv_bridge`).

## 🧭 Autonomous Vehicle Pipeline
### 1. Perception
- Stereo depth, visual odometry, and pose provided by `zed-ros2-wrapper`.
- Semantic segmentation or object detection using TensorRT-optimized models.
- Optional occupancy grid generation via `stereo_image_proc` or custom node.

### 2. Localization & Mapping
- Fuse ZED odometry with wheel encoders/IMU using an **EKF** (`robot_localization`).
- Optional SLAM integration (`slam_toolbox`, `rtabmap_ros`) for map building.

### 3. Planning & Control
- Use **Nav2** for global/local path planning.
- Differential drive controller via `ros2_control` or custom PID node.
- Safety checks: virtual bumper, emergency stop topics.

### 4. Teleoperation & Monitoring
- Joystick teleop for manual override.
- Foxglove Studio dashboard for visualization and remote supervision.
- ROS 2 diagnostics to monitor sensor health, temperature, and power draw.

## 🧬 Machine Learning Workflow
- Capture datasets with rosbag and convert to training formats (e.g., COCO, KITTI).
- Use notebooks for labeling, augmentation, and training custom models.
- Export optimized engines (TensorRT / ONNX) and deploy inference nodes within ROS 2.

## 🛠️ Development Workflow
1. Create feature branches per module (perception, control, etc.).
2. Use `colcon test` and `ament_lint` for quality assurance.
3. Document experiments and configurations under `docs/`.
4. Use Git LFS for large datasets or neural network weights when necessary.

## 🐛 Troubleshooting Tips
- **Camera not detected:** Check USB-C cable, power, and `udev` rules. Run `nvidia-smi` to verify Jetson GPU availability.
- **Driver crashes:** Ensure ZED SDK version matches JetPack release. Reinstall `zed-ros2-wrapper` dependencies.
- **Low FPS:** Disable unused topics, reduce resolution, and leverage GPU accelerated inference.
- **Nav2 instability:** Tune local planner parameters, ensure tf tree consistency, and verify odometry noise values.

## 🗺️ Roadmap
- [ ] Integrate wheel encoder + IMU fusion with `robot_localization`.
- [ ] Implement obstacle detection node publishing costmap layers.
- [ ] Add waypoint-following mission manager with Nav2.
- [ ] Deploy semantic segmentation model optimized with TensorRT.
- [ ] Create CI pipeline for linting and simulation tests.

## 📚 References
- [Stereolabs ZED Mini](https://www.stereolabs.com/zed-mini/)
- [ZED ROS 2 Wrapper Documentation](https://www.stereolabs.com/docs/ros2/)
- [Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [ROS 2 Navigation Stack (Nav2)](https://navigation.ros.org/)

## 📄 License
Specify licensing terms here (e.g., MIT, Apache-2.0) once finalized.

