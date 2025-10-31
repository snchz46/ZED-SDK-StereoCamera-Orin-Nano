# Teleoperation and User Experience

The resources in `src/teleop/` allow controlling the robot and visualizing data in real time.

## Ready-to-use panels
- `config/rviz/zed_nav2.rviz`: RViz panel with preconfigured topics.
- `docs/dashboards/foxglove_panel.json`: layout for Foxglove Studio.

## Web interface
- `src/teleop/web_dashboard/`: React application that consumes ROSBridge (`rosbridge_suite`).
- Launch with `launch/web_dashboard.launch.py` to serve the interface and bridge.

## Manual controls
`src/teleop/joystick/joy_teleop_node.py` translates joystick commands and exposes an emergency stop mode.
