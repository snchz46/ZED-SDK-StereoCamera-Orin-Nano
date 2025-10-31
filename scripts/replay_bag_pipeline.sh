#!/usr/bin/env bash
set -euo pipefail

BAG_PATH=${1:-}
if [[ -z "${BAG_PATH}" ]]; then
  echo "Uso: $0 <ruta_al_rosbag>"
  exit 1
fi

colcon build --symlink-install
source install/setup.bash
ros2 bag play "${BAG_PATH}" --loop &
BAG_PID=$!
trap "kill ${BAG_PID}" EXIT
ros2 launch launch/mission_stack.launch.py use_sim_time:=true
