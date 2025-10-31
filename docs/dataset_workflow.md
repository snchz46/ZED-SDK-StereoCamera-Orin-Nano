# Gestión de Datasets y Entrenamiento

Esta sección resume herramientas para preparar, etiquetar y entrenar modelos con datos de la ZED Mini.

## Conversión de datos
- `scripts/rosbag_to_coco.py`: convierte rosbags a formato COCO para detección de objetos.
- `scripts/rosbag_to_kitti.py`: genera muestras estilo KITTI con información de profundidad y poses.

## Etiquetado asistido
Abra el notebook `notebooks/label-assistant.ipynb` para etiquetar con ayuda de detecciones previas y exportar a COCO.

## Entrenamiento reproducible
1. Prepare un entorno con PyTorch/Lightning (ver `notebooks/training-template.ipynb`).
2. Ejecute los experimentos y registre resultados en `notebooks/experiments/`.
3. Exporte el modelo a ONNX/TensorRT usando `scripts/export_to_tensorrt.py`.

## Integración en ROS 2
Los modelos exportados se despliegan mediante nodos en `src/datasets/inference_nodes/` que publican detecciones o segmentaciones.
