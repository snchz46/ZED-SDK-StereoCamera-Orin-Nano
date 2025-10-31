# ROS 2 Cheat Sheet for the ZED Wrapper

Quick commands to launch the most common publishers from the [`zed-ros2-wrapper`](https://www.stereolabs.com/docs/ros2/) package. Set the `camera_model` parameter to match your device (`zedm`, `zed2`, `zed2i`, etc.).

| ROS 2 Command | Key Published Data | Purpose |
|---------------|-------------------|---------|
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_rgb:=true publish_depth:=true` | `rgb/image_rect_color`, `depth/depth_registered`, `stereo/image_rect_*` | Start the camera with rectified RGB and depth streams ready for visualization or processing. |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm sensors:=true` | `imu/data`, `imu/mag`, `temperature/imu` | Enable the onboard IMU and sensors so they can be fused with estimators such as `robot_localization`. |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_odom_tf:=true publish_pose:=true` | `odom`, `pose`, `tf` | Publish visual odometry and TF transforms for navigation and SLAM. |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm point_cloud_freq:=15.0` | `point_cloud/cloud_registered` | Generate XYZRGB point clouds at the selected frequency for mapping or obstacle detection. |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm object_detection:=true` | `obj_det/objects`, `obj_det/markers` | Activate the SDK's built-in object detection to emit 3D bounding boxes and RViz markers. |

## Complementary Commands

- Preview an image topic: `ros2 run rqt_image_view rqt_image_view` and select `/zed/zed_node/rgb/image_rect_color`.
- Inspect IMU data in the console: `ros2 topic echo /zed/zed_node/imu/data`.
- Record synchronized data: `ros2 bag record /zed/zed_node/rgb/image_rect_color /zed/zed_node/depth/depth_registered /zed/zed_node/imu/data`.
- View the TF tree: `ros2 run tf2_tools view_frames && evince frames.pdf`.

> 💡 Many parameters in `zed_camera.launch.py` can be combined. For example:
> ```bash
> ros2 launch zed_wrapper zed_camera.launch.py \
>   camera_model:=zedm publish_rgb:=true publish_depth:=true \
>   sensors:=true publish_pose:=true publish_odom_tf:=true
> ```
