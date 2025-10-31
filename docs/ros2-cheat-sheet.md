# ROS 2 Cheat Sheet para el ZED Wrapper

Comandos rápidos para arrancar los nodos de publicación más habituales del paquete [`zed-ros2-wrapper`](https://www.stereolabs.com/docs/ros2/). Ajusta el parámetro `camera_model` a tu dispositivo (`zedm`, `zed2`, `zed2i`, etc.).

| Comando ROS 2 | Datos publicados clave | ¿Para qué sirve? |
|---------------|-----------------------|-------------------|
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_rgb:=true publish_depth:=true` | `rgb/image_rect_color`, `depth/depth_registered`, `stereo/image_rect_*` | Inicializa la cámara con streams RGB y de profundidad rectificados listos para visualización o procesamiento. |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm sensors:=true` | `imu/data`, `imu/mag`, `temperature/imu` | Activa la IMU y sensores integrados para fusionarlos con otros estimadores (por ejemplo `robot_localization`). |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_odom_tf:=true publish_pose:=true` | `odom`, `pose`, `tf` | Publica la odometría visual y las transformaciones TF para navegación y SLAM. |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm point_cloud_freq:=15.0` | `point_cloud/cloud_registered` | Genera nubes de puntos XYZRGB a la frecuencia indicada para mapeo o detección de obstáculos. |
| `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm object_detection:=true` | `obj_det/objects`, `obj_det/markers` | Activa la detección de objetos integrada del SDK para enviar bounding boxes 3D y marcadores de RViz. |

## Comandos complementarios

- Previsualizar un tópico de imagen: `ros2 run rqt_image_view rqt_image_view` y seleccionar `/zed/zed_node/rgb/image_rect_color`.
- Inspeccionar la IMU en consola: `ros2 topic echo /zed/zed_node/imu/data`.
- Grabar datos sincronizados: `ros2 bag record /zed/zed_node/rgb/image_rect_color /zed/zed_node/depth/depth_registered /zed/zed_node/imu/data`.
- Ver la estructura TF: `ros2 run tf2_tools view_frames && evince frames.pdf`.

> 💡 Muchos parámetros del `zed_camera.launch.py` se pueden combinar. Por ejemplo:
> ```bash
> ros2 launch zed_wrapper zed_camera.launch.py \
>   camera_model:=zedm publish_rgb:=true publish_depth:=true \
>   sensors:=true publish_pose:=true publish_odom_tf:=true
> ```
