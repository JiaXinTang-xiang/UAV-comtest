#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# A previous launch can leave sensor/preflight nodes alive after the terminal
# was closed.  Those duplicate publishers make the next run look random.
pkill -f 'ros2 launch uav_competition px4_regions.launch.py' 2>/dev/null || true
pkill -f 'uav_competition/.*/px4_offboard' 2>/dev/null || true
pkill -f 'uav_competition/.*/px4_sensor_adapter' 2>/dev/null || true
pkill -f 'uav_competition/.*/preflight_check' 2>/dev/null || true
pkill -f 'uav_competition/.*/px4_region_mission' 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/px4_msgs_overlay_ws/install/setup.bash"
source "$PROJECT_DIR/uav_competition_ws/install/setup.bash"
exec ros2 launch uav_competition px4_regions.launch.py "$@"
