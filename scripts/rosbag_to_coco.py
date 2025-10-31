"""Tools to convert rosbags into COCO annotations."""

import argparse
from pathlib import Path


def convert_rosbag_to_coco(bag_path: Path, output_dir: Path) -> None:
    """Conversion placeholder; implement dataset-specific logic."""
    raise NotImplementedError("Implement COCO conversion using your custom topics")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a rosbag into COCO format")
    parser.add_argument("bag", type=Path, help="Path to the rosbag .db3 file")
    parser.add_argument("--output", type=Path, default=Path("datasets/coco"), help="Output directory")
    args = parser.parse_args()
    convert_rosbag_to_coco(args.bag, args.output)
