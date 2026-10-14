#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../uav_competition_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "Workspace sourced. Try: ros2 launch uav_competition simulation.launch.py use_mock:=true"
