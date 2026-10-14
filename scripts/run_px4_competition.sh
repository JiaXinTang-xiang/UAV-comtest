#!/usr/bin/env bash
set -eo pipefail

ROOT="/home/jiaxintang/桌面/bao"
PX4_ROOT="/home/jiaxintang/PX4-Autopilot"
WORLD="$ROOT/uav_competition_ws/src/uav_competition/worlds/competition_course.world"

if [ ! -f "$WORLD" ]; then
  echo "World file not found: $WORLD" >&2
  exit 1
fi

# Close only stale simulation processes from an earlier run.
pkill -x gzclient 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
pkill -x px4 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
set -u
export PX4_SITL_WORLD="$WORLD"
# Do not make Gazebo attach the camera to Iris; keep the overview camera in the world file.
export PX4_NO_FOLLOW_MODE=1

echo "PX4_SITL_WORLD=$PX4_SITL_WORLD"
echo "WORLD_EXISTS=$(test -f "$PX4_SITL_WORLD" && echo yes || echo no)"

cd "$PX4_ROOT"
make px4_sitl gazebo-classic
