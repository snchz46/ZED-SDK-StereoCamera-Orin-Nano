"""Herramientas para convertir rosbags a anotaciones COCO."""

import argparse
from pathlib import Path


def convert_rosbag_to_coco(bag_path: Path, output_dir: Path) -> None:
    """Placeholder de conversión; implemente la lógica específica según su dataset."""
    raise NotImplementedError("Implementa la conversión a COCO utilizando tus tópicos personalizados")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convierte un rosbag a formato COCO")
    parser.add_argument("bag", type=Path, help="Ruta al archivo .db3 del rosbag")
    parser.add_argument("--output", type=Path, default=Path("datasets/coco"), help="Directorio de salida")
    args = parser.parse_args()
    convert_rosbag_to_coco(args.bag, args.output)
