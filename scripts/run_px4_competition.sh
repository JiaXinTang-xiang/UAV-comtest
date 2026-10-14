#!/usr/bin/env bash
set -eo pipefail

ROOT="/home/jiaxintang/桌面/bao"
PX4_ROOT="/home/jiaxintang/PX4-Autopilot"
WORLD="$ROOT/uav_competition_ws/src/uav_competition/worlds/competition_course.world"

if [ ! -f "$WORLD" ]; then
  echo "World file not found: $WORLD" >&2
  exit 1
fi

# Stop the full stale PX4 launch chain before starting a new instance. A stale
# sitl_run process can otherwise reconnect to the new Gazebo master.
pkill -f "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_run.sh" 2>/dev/null || true
pkill -f '^gz model ' 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -x gzserver 2>/dev/null || true
pkill -x px4 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
set -u
export PX4_SITL_WORLD="$WORLD"
export PX4_SITL_X=5.0
export PX4_SITL_Y=9.0
export PX4_SITL_Z=0.83
# Do not make Gazebo attach the camera to Iris; keep the overview camera in the world file.
export PX4_NO_FOLLOW_MODE=1

# All models used by this exercise are installed locally.
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:$HOME/.gazebo/models:$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

# Use the installed NVIDIA driver. Software OpenGL made gzclient unresponsive
# on this computer.
unset LIBGL_ALWAYS_SOFTWARE

echo "PX4_SITL_WORLD=$PX4_SITL_WORLD"
echo "PX4_SITL_START=(${PX4_SITL_X},${PX4_SITL_Y},${PX4_SITL_Z})"
echo "WORLD_EXISTS=$(test -f "$PX4_SITL_WORLD" && echo yes || echo no)"

# Keep the GUI outside PX4's process lifecycle. Let gzserver spawn Iris first,
# then connect the client. The local URI fails quickly instead of blocking on
# the retired online Gazebo model database.
(
  sleep 6
  exec env GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9" gzclient --verbose
) &
GUI_PID=$!
cleanup() {
  kill "$GUI_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "$PX4_ROOT"
HEADLESS=1 make px4_sitl gazebo-classic
