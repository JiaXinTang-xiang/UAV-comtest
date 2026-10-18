#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/px4_msgs_overlay_ws/install/setup.bash"
source "$PROJECT_DIR/uav_competition_ws/install/setup.bash"
exec ros2 launch uav_competition px4_sensors.launch.py "$@"
