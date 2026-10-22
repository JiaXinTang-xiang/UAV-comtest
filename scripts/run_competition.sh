#!/usr/bin/env bash
# One ROS Launch owns the simulation, agent and mission nodes. No auto arming.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/px4_msgs_overlay_ws/install/setup.bash"
source "$ROOT/uav_competition_ws/install/setup.bash"
exec ros2 launch uav_competition competition.launch.py project_dir:="$ROOT" "$@"
