# Simulación y Pruebas Virtuales

Esta guía describe cómo preparar escenarios de Gazebo/Ignition para validar la pila de percepción y navegación sin hardware.

## Escenarios de referencia
- **`launch/ignition_zed_world.launch.py`**: lanza un mundo base con el robot, la cámara ZED Mini y Nav2.
- **`config/nav2_params.yaml`**: parámetros compartidos para planificadores en simulación y hardware real.
- **`config/robot_localization_sim.yaml`**: configuración extendida para fusionar odometría visual y sensores simulados.

## Flujos de trabajo sugeridos
1. Cree o importe un modelo URDF/SDFormat del vehículo en `config/urdf/`.
2. Ajuste los sensores virtuales (IMU, encoders, cámara) y exporte el paquete como un plugin de Ignition.
3. Ejecute `ros2 launch launch/ignition_zed_world.launch.py use_sim_time:=true` para validar tópicos y transformaciones.
4. Registre `ros2 bag` desde la simulación para pruebas de regresión.

## Reproducción de datos
Utilice el script `scripts/replay_bag_pipeline.sh` para lanzar la pila completa contra un rosbag grabado y comparar métricas de desempeño.
