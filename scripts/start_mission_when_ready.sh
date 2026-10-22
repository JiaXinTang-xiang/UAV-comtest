#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/px4_msgs_overlay_ws/install/setup.bash"
source "$ROOT/uav_competition_ws/install/setup.bash"
echo "等待 PX4 和传感器自检进入 READY，然后请求开始任务..."
exec python3 -m uav_competition.runtime start "$@"
