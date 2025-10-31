"""Export trained models to TensorRT."""

import argparse
from pathlib import Path


def export_model(model_path: Path, output_dir: Path, precision: str = "fp16") -> None:
    """Export placeholder; wire up PyTorch/ONNX depending on your stack."""
    raise NotImplementedError("Integrate your TensorRT export pipeline")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export a model to TensorRT")
    parser.add_argument("model", type=Path, help="Path to the trained checkpoint")
    parser.add_argument("--output", type=Path, default=Path("deploy/tensorrt"), help="Output directory")
    parser.add_argument("--precision", choices=["fp32", "fp16", "int8"], default="fp16", help="Target precision")
    args = parser.parse_args()
    export_model(args.model, args.output, args.precision)
