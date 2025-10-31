# ZED Mini Autonomous Vehicle Platform on Jetson Orin Nano

An end-to-end workspace for building a ground autonomous vehicle with the **ZED Mini stereo camera** and an **NVIDIA Jetson Orin Nano** running **ROS 2 Humble**. The repository captures configuration guides, launchers, scripts, and now includes complete flows for simulation, dataset handling, sensor fusion, advanced navigation, teleoperation, and CI/CD automation.

## 🚗 Project Vision
- Deliver a reliable perception stack based on the ZED Mini for depth, visual odometry, and semantic understanding.
- Fuse camera data with onboard sensors to enable localization, obstacle avoidance, and autonomous navigation.
- Provide a reference architecture for small-scale autonomous vehicles (UGV/RC) with the Jetson Orin Nano.
- Maintain reproducible workflows for data capture, training, and deployment of machine learning models.

## 📦 Repository Contents
- `launch/` – ROS 2 launch files for simulation, full mission, and the web panel (`ignition_zed_world.launch.py`, `mission_stack.launch.py`, `web_dashboard.launch.py`).
- `config/` – Shared parameters, health rules, `robot_localization` setups, missions, and RViz layouts.
- `scripts/` – Tools for rosbag conversion (COCO/KITTI), TensorRT export, pipeline replay, and bespoke utilities.
- `src/` – Custom packages for sensor fusion (`fusion/`), datasets/inference, navigation/missions, and teleoperation.
- `notebooks/` – Labeling and training templates for PyTorch/Lightning, plus an experiment log.
- `docs/` – Specialized guides: [simulation](docs/simulation.md), [datasets](docs/dataset_workflow.md), [sensor fusion](docs/sensor_fusion.md), [missions](docs/navigation_mission.md), [teleoperation](docs/teleoperation.md), [scripts overview](docs/scripts_overview.md), and [CI/CD](docs/ci_cd.md).
- `.github/workflows/` – GitHub Actions pipeline running linters, tests, and publishing reference artifacts.

> **Note:** Many paths contain placeholders so you can quickly adapt the logic to your robot. Each file describes its purpose and expected extension points.

## 🧰 Reference Hardware
| Component | Notes |
|-----------|-------|
| NVIDIA Jetson Orin Nano (8 GB) | Ubuntu 22.04 host / JetPack 5.x |
| ZED Mini Stereo Camera | RGB + depth + IMU |
| Differential chassis (Waveshare or similar) | VESC/RoboClaw-style controller |
| Encoders and IMU | Optional to improve estimation |
| LiPo battery + BEC | System power |

## 🧪 Software Stack
- **OS:** Ubuntu 22.04 LTS (JetPack 5.x)
- **ROS 2 Humble** with `ros-base` + development tools
- **ZED SDK** (>= 4.0) and `zed-ros2-wrapper`
- **CUDA**, **TensorRT**, `torch`/`onnxruntime` for AI models
- **Nav2**, `robot_localization`, `slam_toolbox`, `foxglove_bridge`
- **Colcon** to build ROS 2 workspaces

## 🛠️ Quick Setup
1. Flash the Jetson Orin Nano with JetPack 5.x.
2. Install ROS 2 Humble (`ros-humble-desktop` or `ros-base`).
3. Install the ZED SDK and build [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-wrapper).
4. Clone this repository into your workspace (`~/ros2_ws/src/`).
5. Install dependencies:
   ```bash
   sudo apt update && sudo apt install python3-vcstool python3-colcon-common-extensions ros-humble-navigation2
   rosdep install --from-paths src --ignore-src -r -y
   ```
6. Build and source:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
7. Configure udev rules for the ZED Mini.

## 🌐 Simulation and Virtual Testing
The guide [docs/simulation.md](docs/simulation.md) details how to use `launch/ignition_zed_world.launch.py` with URDF/SDFormat models, replay bags, and validate Nav2 in virtual environments. The script `scripts/replay_bag_pipeline.sh` automates regressions against recorded rosbags.

## 📊 Dataset Management and Training
See [docs/dataset_workflow.md](docs/dataset_workflow.md) to convert rosbags to COCO/KITTI (`scripts/rosbag_to_coco.py`, `scripts/rosbag_to_kitti.py`), label with `notebooks/label-assistant.ipynb`, and export models (`scripts/export_to_tensorrt.py`). Deployment nodes live in `src/datasets/inference_nodes/`.

## 🔗 Advanced Sensor Fusion
`src/fusion/` contains `robot_localization` setups, point cloud filters, and occupancy mapping. Adjust `config/robot_localization.yaml` and `config/robot_localization_sim.yaml` to combine IMU, encoders, and visual odometry. More details in [docs/sensor_fusion.md](docs/sensor_fusion.md).

## 🧭 Navigation and Missions
The extended navigation stack is documented in [docs/navigation_mission.md](docs/navigation_mission.md). `launch/mission_stack.launch.py` brings up Nav2 with the mission manager (`src/navigation/mission_manager/`) and health monitor (`src/navigation/health_monitor/`). Define waypoints in `config/missions/` and dynamic conditions via plugins in `src/navigation/plugins/`.

## 🎮 Teleoperation and User Experience
The guide [docs/teleoperation.md](docs/teleoperation.md) covers RViz/Foxglove panels, the web interface (`src/teleop/web_dashboard/`), and the joystick node (`src/teleop/joystick/`). `launch/web_dashboard.launch.py` serves ROSBridge plus the React UI.

## 🤖 CI/CD Pipeline
`.github/workflows/ci.yml` introduces a GitHub Actions flow for linters, tests, and reference artifacts. See [docs/ci_cd.md](docs/ci_cd.md) for additional repository governance recommendations.

## 📡 ZED Mini Bring-up
1. Connect the ZED Mini and verify `lsusb`.
2. Launch the driver:
   ```bash
   ros2 launch zed_wrapper zedm.launch.py
   ```
3. Visualize RGB, depth, and IMU in RViz or Foxglove.
4. Record rosbags for datasets:
   ```bash
   ros2 bag record /zed/zed_node/rgb_raw/image_raw_color /zed/zed_node/depth/depth_registered
   ```

## 🔧 Highlighted Scripts
- `scripts/fusion_distance_check.py`: cross-checks front-facing ranges from LiDAR and ZED depth data.
- `scripts/zed_distance_monitor.py`: monitors minimum distances in point clouds.
- `scripts/zed_depth_listener.py`: prints depth statistics and optional visualizations.
- `scripts/zed_depth_to_laserscan.py`: generates a synthetic `LaserScan` from ZED depth images.
- `scripts/zed_yolo_listener.py`: runs YOLO on the RGB stream with configurable models and devices.
- `scripts/windows_yolov8_cam_sub_.py` / `scripts/windows_yolov8_cam_sub_ZED.py`: Windows-friendly YOLOv8 visualizers.
- `scripts/replay_bag_pipeline.sh`: launches the full stack against a rosbag.
- `scripts/rosbag_to_coco.py` / `scripts/rosbag_to_kitti.py`: dataset conversion.
- `scripts/export_to_tensorrt.py`: exports trained models to TensorRT.
- See [docs/scripts_overview.md](docs/scripts_overview.md) for a complete, continuously updated reference of script behavior, dependencies, and usage examples.

## 🧬 Machine Learning Workflow
- Capture datasets with rosbag and convert them to standard formats.
- Label and train from `notebooks/` using PyTorch/Lightning.
- Export to ONNX/TensorRT and deploy nodes in `src/datasets/inference_nodes/`.

## 🛠️ Development Flow
1. Create feature branches (perception, control, etc.).
2. Run `colcon test` and linters before opening a PR.
3. Document changes in `docs/` and update dashboards when applicable.
4. Use Git LFS for large artifacts.

## 🐛 Troubleshooting
- **Camera not detected:** verify cabling, udev rules, and `nvidia-smi`.
- **Driver crashes:** ensure ZED SDK ↔ JetPack compatibility.
- **Low FPS:** disable unnecessary topics and use optimized models.
- **Nav2 instability:** review the `tf` tree and covariances in `robot_localization`.

## 🗺️ Updated Roadmap
- [x] Integrate simulation flows with Ignition.
- [x] Incorporate dataset conversion and TensorRT export.
- [x] Add advanced sensor fusion with `robot_localization`.
- [x] Deploy the mission manager and health monitoring.
- [x] Publish teleoperation tools and dashboards.
- [x] Configure a basic CI/CD pipeline.

## 📚 References
- [Stereolabs ZED Mini](https://www.stereolabs.com/zed-mini/)
- [ZED ROS 2 Wrapper Documentation](https://www.stereolabs.com/docs/ros2/)
- [Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [ROS 2 Navigation Stack (Nav2)](https://navigation.ros.org/)

## 📄 License
Specify license terms (e.g., MIT, Apache-2.0) when finalized.
