# Fusión Sensorial Avanzada

Los paquetes en `src/fusion/` combinan la odometría visual de la ZED Mini con sensores adicionales.

## Paquetes principales
- `src/fusion/robot_localization/`: configuraciones y nodos para EKF y UKF basados en `robot_localization`.
- `src/fusion/pointcloud_filters/`: filtros voxel, outlier removal y downsampling para nubes de puntos.

## Integraciones clave
1. Configure los tópicos de IMU y encoders en `config/robot_localization.yaml`.
2. Publique la odometría visual desde `zed_wrapper` y combínela mediante `robot_localization`.
3. Ajuste la latencia y los pesos de covarianza con `rqt_reconfigure` o parámetros YAML.

## Mapas de ocupación
Use `src/fusion/occupancy_mapping_node.cpp` para generar mapas 2D/3D filtrados antes de alimentar Nav2.
