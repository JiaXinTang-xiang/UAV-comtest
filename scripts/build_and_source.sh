#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../uav_competition_ws"
source /opt/ros/humble/setup.bash
source ../px4_msgs_overlay_ws/install/setup.bash
# Build messages in the separate overlay; avoid regenerating them under this
# workspace's non-ASCII source path.
colcon build --symlink-install --packages-select uav_competition
source install/setup.bash
echo "Workspace sourced. Try: ros2 launch uav_competition simulation.launch.py use_mock:=true"
