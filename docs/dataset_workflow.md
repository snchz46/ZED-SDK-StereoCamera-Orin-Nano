# Dataset Management and Training

This section summarizes tools to prepare, label, and train models with ZED Mini data.

## Data conversion
- `scripts/rosbag_to_coco.py`: converts rosbags to COCO format for object detection.
- `scripts/rosbag_to_kitti.py`: generates KITTI-style samples with depth and pose information.

## Assisted labeling
Open the notebook `notebooks/label-assistant.ipynb` to label with help from prior detections and export to COCO.

## Reproducible training
1. Prepare an environment with PyTorch/Lightning (see `notebooks/training-template.ipynb`).
2. Run experiments and log results in `notebooks/experiments/`.
3. Export the model to ONNX/TensorRT using `scripts/export_to_tensorrt.py`.

## Integration in ROS 2
Exported models are deployed through nodes in `src/datasets/inference_nodes/` that publish detections or segmentations.
