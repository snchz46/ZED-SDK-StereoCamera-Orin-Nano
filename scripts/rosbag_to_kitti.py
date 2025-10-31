"""Basic conversion from rosbags to KITTI-style datasets."""

import argparse
from pathlib import Path


def convert_rosbag_to_kitti(bag_path: Path, output_dir: Path) -> None:
    """Placeholder to generate RGB frames, depth, and poses."""
    raise NotImplementedError("Implement KITTI conversion with your topics of interest")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a rosbag into KITTI format")
    parser.add_argument("bag", type=Path, help="Path to the rosbag .db3 file")
    parser.add_argument("--output", type=Path, default=Path("datasets/kitti"), help="Output directory")
    args = parser.parse_args()
    convert_rosbag_to_kitti(args.bag, args.output)
