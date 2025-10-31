# Advanced Sensor Fusion

The packages in `src/fusion/` combine the ZED Mini's visual odometry with additional sensors.

## Core packages
- `src/fusion/robot_localization/`: configurations and nodes for EKF/UKF using `robot_localization`.
- `src/fusion/pointcloud_filters/`: voxel filters, outlier removal, and downsampling for point clouds.

## Key integrations
1. Configure IMU and encoder topics in `config/robot_localization.yaml`.
2. Publish visual odometry from `zed_wrapper` and fuse it via `robot_localization`.
3. Tune latency and covariance weights with `rqt_reconfigure` or YAML parameters.

## Occupancy maps
Use `src/fusion/occupancy_mapping_node.cpp` to generate filtered 2D/3D maps before feeding Nav2.
