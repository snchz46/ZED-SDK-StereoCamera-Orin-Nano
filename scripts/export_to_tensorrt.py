"""Exporta modelos entrenados a TensorRT."""

import argparse
from pathlib import Path


def export_model(model_path: Path, output_dir: Path, precision: str = "fp16") -> None:
    """Placeholder de exportación; conecte PyTorch/ONNX según su stack."""
    raise NotImplementedError("Integra tu pipeline de exportación a TensorRT")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Exporta un modelo a TensorRT")
    parser.add_argument("model", type=Path, help="Ruta al checkpoint entrenado")
    parser.add_argument("--output", type=Path, default=Path("deploy/tensorrt"), help="Directorio de salida")
    parser.add_argument("--precision", choices=["fp32", "fp16", "int8"], default="fp16", help="Precisión objetivo")
    args = parser.parse_args()
    export_model(args.model, args.output, args.precision)
