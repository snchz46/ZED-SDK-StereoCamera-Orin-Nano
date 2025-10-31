# Simulation and Virtual Testing

This guide describes how to prepare Gazebo/Ignition scenarios to validate the perception and navigation stack without hardware.

## Reference scenarios
- **`launch/ignition_zed_world.launch.py`**: launches a base world with the robot, the ZED Mini camera, and Nav2.
- **`config/nav2_params.yaml`**: shared parameters for planners in simulation and on hardware.
- **`config/robot_localization_sim.yaml`**: extended configuration to fuse visual odometry and simulated sensors.

## Suggested workflows
1. Create or import a URDF/SDFormat model of the vehicle in `config/urdf/`.
2. Tune the virtual sensors (IMU, encoders, camera) and export the package as an Ignition plugin.
3. Run `ros2 launch launch/ignition_zed_world.launch.py use_sim_time:=true` to validate topics and transforms.
4. Record `ros2 bag` from the simulation for regression tests.

## Data replay
Use the script `scripts/replay_bag_pipeline.sh` to launch the full stack against a recorded rosbag and compare performance metrics.
