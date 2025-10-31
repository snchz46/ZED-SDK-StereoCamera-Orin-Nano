"""Conversión básica de rosbags a datasets estilo KITTI."""

import argparse
from pathlib import Path


def convert_rosbag_to_kitti(bag_path: Path, output_dir: Path) -> None:
    """Placeholder para generar frames RGB, profundidad y poses."""
    raise NotImplementedError("Implementa la conversión a KITTI con tus tópicos de interés")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convierte un rosbag a formato KITTI")
    parser.add_argument("bag", type=Path, help="Ruta al archivo .db3 del rosbag")
    parser.add_argument("--output", type=Path, default=Path("datasets/kitti"), help="Directorio de salida")
    args = parser.parse_args()
    convert_rosbag_to_kitti(args.bag, args.output)
