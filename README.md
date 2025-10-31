# ZED Mini Autonomous Vehicle Platform on Jetson Orin Nano

Un workspace integral para construir un vehículo autónomo terrestre con la **cámara estéreo ZED Mini** y un **NVIDIA Jetson Orin Nano** ejecutando **ROS 2 Humble**. El repositorio captura guías de configuración, lanzadores, scripts y ahora incorpora flujos completos de simulación, manejo de datasets, fusión sensorial, navegación avanzada, teleoperación y automatización CI/CD.

## 🚗 Visión del Proyecto
- Entregar una pila de percepción confiable basada en la ZED Mini para profundidad, odometría visual y comprensión semántica.
- Fusionar datos de cámara con sensores a bordo para habilitar localización, evasión de obstáculos y navegación autónoma.
- Proveer una arquitectura de referencia para vehículos autónomos de pequeña escala (UGV/RC) con Jetson Orin Nano.
- Mantener workflows reproducibles para captura de datos, entrenamiento y despliegue de modelos de machine learning.

## 📦 Contenido del Repositorio
- `launch/` – Lanzadores ROS 2 para simulación, misión completa y panel web (`ignition_zed_world.launch.py`, `mission_stack.launch.py`, `web_dashboard.launch.py`).
- `config/` – Parámetros compartidos, reglas de salud, configuraciones de `robot_localization`, misiones y layouts de RViz.
- `scripts/` – Herramientas para conversión de rosbags (COCO/KITTI), exportación a TensorRT, reproducción de pipelines y utilidades originales.
- `src/` – Paquetes personalizados para fusión sensorial (`fusion/`), datasets/inferencia, navegación/misiones y teleoperación.
- `notebooks/` – Plantillas de etiquetado y entrenamiento para PyTorch/Lightning, junto con un registro de experimentos.
- `docs/` – Guías especializadas: [simulación](docs/simulation.md), [datasets](docs/dataset_workflow.md), [fusión sensorial](docs/sensor_fusion.md), [misiones](docs/navigation_mission.md), [teleoperación](docs/teleoperation.md) y [CI/CD](docs/ci_cd.md).
- `.github/workflows/` – Pipeline de GitHub Actions que ejecuta linters, pruebas y publica artefactos de referencia.

> **Nota:** Muchas rutas contienen placeholders para que adaptes rápidamente la lógica a tu robot. Cada archivo describe el propósito y puntos de extensión esperados.

## 🧰 Hardware de Referencia
| Componente | Notas |
|-----------|-------|
| NVIDIA Jetson Orin Nano (8 GB) | Host Ubuntu 22.04 / JetPack 5.x |
| ZED Mini Stereo Camera | RGB + profundidad + IMU |
| Chasis diferencial (Waveshare u otro) | Controlador tipo VESC/RoboClaw |
| Encoders e IMU | Opcional para mejorar estimación |
| Batería LiPo + BEC | Alimentación del sistema |

## 🧪 Stack de Software
- **OS:** Ubuntu 22.04 LTS (JetPack 5.x)
- **ROS 2 Humble** con `ros-base` + herramientas de desarrollo
- **ZED SDK** (>= 4.0) y `zed-ros2-wrapper`
- **CUDA**, **TensorRT**, `torch`/`onnxruntime` para modelos IA
- **Nav2**, `robot_localization`, `slam_toolbox`, `foxglove_bridge`
- **Colcon** para construir workspaces ROS 2

## 🛠️ Configuración Rápida
1. Flashea el Jetson Orin Nano con JetPack 5.x.
2. Instala ROS 2 Humble (`ros-humble-desktop` o `ros-base`).
3. Instala ZED SDK y compila [`zed-ros2-wrapper`](https://github.com/stereolabs/zed-ros2-wrapper).
4. Clona este repositorio dentro de tu workspace (`~/ros2_ws/src/`).
5. Instala dependencias:
   ```bash
   sudo apt update && sudo apt install python3-vcstool python3-colcon-common-extensions ros-humble-navigation2
   rosdep install --from-paths src --ignore-src -r -y
   ```
6. Construye y sourcea:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
7. Configura reglas udev para la ZED Mini.

## 🌐 Simulación y Pruebas Virtuales
La guía [docs/simulation.md](docs/simulation.md) detalla cómo emplear `launch/ignition_zed_world.launch.py` con modelos URDF/SDFormat, reproducir bags y validar Nav2 en entornos virtuales. El script `scripts/replay_bag_pipeline.sh` automatiza regresiones contra rosbags grabados.

## 📊 Gestión de Datasets y Entrenamiento
Consulta [docs/dataset_workflow.md](docs/dataset_workflow.md) para convertir rosbags a COCO/KITTI (`scripts/rosbag_to_coco.py`, `scripts/rosbag_to_kitti.py`), etiquetar con `notebooks/label-assistant.ipynb` y exportar modelos (`scripts/export_to_tensorrt.py`). Los nodos de despliegue viven en `src/datasets/inference_nodes/`.

## 🔗 Fusión Sensorial Avanzada
`src/fusion/` contiene configuraciones de `robot_localization`, filtros de nubes y mapeo de ocupación. Ajusta `config/robot_localization.yaml` y `config/robot_localization_sim.yaml` para combinar IMU, encoders y odometría visual. Más detalles en [docs/sensor_fusion.md](docs/sensor_fusion.md).

## 🧭 Navegación y Misiones
El stack de navegación extendida se documenta en [docs/navigation_mission.md](docs/navigation_mission.md). `launch/mission_stack.launch.py` levanta Nav2 con el mission manager (`src/navigation/mission_manager/`) y el monitor de salud (`src/navigation/health_monitor/`). Define waypoints en `config/missions/` y condiciones dinámicas mediante plugins en `src/navigation/plugins/`.

## 🎮 Teleoperación y Experiencia de Usuario
La guía [docs/teleoperation.md](docs/teleoperation.md) cubre paneles RViz/Foxglove, la interfaz web (`src/teleop/web_dashboard/`) y el nodo de joystick (`src/teleop/joystick/`). `launch/web_dashboard.launch.py` sirve ROSBridge más la UI React.

## 🤖 Pipeline CI/CD
`.github/workflows/ci.yml` introduce un flujo de GitHub Actions para linters, pruebas y artefactos de referencia. Siga [docs/ci_cd.md](docs/ci_cd.md) para recomendaciones adicionales de gobierno de repositorio.

## 📡 Puesta en Marcha de la ZED Mini
1. Conecta la ZED Mini y verifica `lsusb`.
2. Lanza el driver:
   ```bash
   ros2 launch zed_wrapper zedm.launch.py
   ```
3. Visualiza RGB, profundidad e IMU en RViz o Foxglove.
4. Graba rosbags para datasets:
   ```bash
   ros2 bag record /zed/zed_node/rgb_raw/image_raw_color /zed/zed_node/depth/depth_registered
   ```

## 🔧 Scripts Destacados
- `scripts/zed_distance_monitor.py`: monitorea distancias mínimas en nubes de puntos.
- `scripts/zed_yolo_listener.py`: ejecuta YOLO sobre el stream RGB.
- `scripts/replay_bag_pipeline.sh`: lanza la pila completa contra un rosbag.
- `scripts/rosbag_to_coco.py` / `scripts/rosbag_to_kitti.py`: conversión de datasets.
- `scripts/export_to_tensorrt.py`: exporta modelos entrenados a TensorRT.

## 🧬 Workflow de Machine Learning
- Captura datasets con rosbag y convierte a formatos estándar.
- Etiqueta y entrena desde `notebooks/` empleando PyTorch/Lightning.
- Exporta a ONNX/TensorRT y despliega nodos en `src/datasets/inference_nodes/`.

## 🛠️ Flujo de Desarrollo
1. Crea ramas por funcionalidad (percepción, control, etc.).
2. Ejecuta `colcon test` y linters antes de abrir un PR.
3. Documenta cambios en `docs/` y actualiza dashboards si aplica.
4. Usa Git LFS para grandes artefactos.

## 🐛 Troubleshooting
- **Cámara no detectada:** verifica cableado, reglas udev y `nvidia-smi`.
- **Caídas del driver:** asegura compatibilidad ZED SDK ↔ JetPack.
- **Bajo FPS:** deshabilita tópicos innecesarios y usa modelos optimizados.
- **Inestabilidad en Nav2:** revisa la `tf` y covarianzas en `robot_localization`.

## 🗺️ Roadmap Actualizado
- [x] Integrar flujos de simulación con Ignition.
- [x] Incorporar conversión de datasets y exportación a TensorRT.
- [x] Añadir fusión sensorial avanzada con `robot_localization`.
- [x] Desplegar mission manager y monitoreo de salud.
- [x] Publicar herramientas de teleoperación y dashboards.
- [x] Configurar pipeline CI/CD básico.

## 📚 Referencias
- [Stereolabs ZED Mini](https://www.stereolabs.com/zed-mini/)
- [ZED ROS 2 Wrapper Documentation](https://www.stereolabs.com/docs/ros2/)
- [Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [ROS 2 Navigation Stack (Nav2)](https://navigation.ros.org/)

## 📄 Licencia
Especifica los términos de licencia (p.ej. MIT, Apache-2.0) cuando estén definidos.
