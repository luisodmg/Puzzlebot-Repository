#!/usr/bin/env bash
set -euo pipefail

JETSON_USER="${JETSON_USER:-puzzlebot}"
JETSON_HOST="${JETSON_HOST:-10.10.0.100}"
REMOTE_WS="${REMOTE_WS:-/home/${JETSON_USER}/ros2_ws}"
REMOTE_SRC="${REMOTE_WS}/src"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

xhost +local: || true

ssh "${JETSON_USER}@${JETSON_HOST}" \
  "rm -rf '${REMOTE_SRC}/computer_vision' && mkdir -p '${REMOTE_SRC}'"

scp -r "${PACKAGE_DIR}" "${JETSON_USER}@${JETSON_HOST}:${REMOTE_SRC}/computer_vision"

ssh -X "${JETSON_USER}@${JETSON_HOST}" \
  "bash -lc '
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    if [ -f ~/ros2_ws/env_jetson.sh ]; then
      source ~/ros2_ws/env_jetson.sh
    fi
    colcon build --packages-select computer_vision
    source install/setup.bash
    ros2 run computer_vision live_undistort.py
  '"
