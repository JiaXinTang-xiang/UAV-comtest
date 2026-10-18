#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="$ROOT/PX4-Autopilot"
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
# Lockstep can block a polite shutdown while its simulator is gone.
pkill -9 -x px4 2>/dev/null || true
pkill -9 -x gzserver 2>/dev/null || true

source /opt/ros/humble/setup.bash
set -u

# PX4's native ROS 2 bridge needs a live Micro XRCE-DDS Agent. Without it PX4
# starts briefly, then exits with "No ping response, disconnecting".
if ! pgrep -x MicroXRCEAgent >/dev/null; then
  echo "Micro XRCE-DDS Agent is not running." >&2
  echo "Start it in another terminal with:" >&2
  echo "  cd '$ROOT' && ./scripts/run_xrce_agent.sh" >&2
  exit 2
fi

export PX4_SITL_WORLD="$WORLD"
export PX4_SITL_X=5.0
export PX4_SITL_Y=9.0
export PX4_SITL_Z=0.83
# Do not make Gazebo attach the camera to Iris; keep the overview camera in the world file.
export PX4_NO_FOLLOW_MODE=1
# Prevent gz model (Iris spawn) from waiting for the online Gazebo model
# database. All required models are local.
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:9"
export GAZEBO_MASTER_URI="http://127.0.0.1:11345"
# Keep Gazebo transport on the local loopback interface.  Using the active
# LAN address can make gzclient receive malformed/empty transport packets and
# appear frozen while gzserver and PX4 continue running.
export GZ_IP="127.0.0.1"

# All models used by this exercise are installed locally.
export GAZEBO_MODEL_PATH="$ROOT/uav_competition_ws/src/uav_competition/models:/usr/share/gazebo-11/models:$HOME/.gazebo/models:$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="$ROOT/uav_competition_ws/src/uav_competition/models:/usr/share/gazebo-11:/usr/share/gazebo-11/media:$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"

# Use the installed NVIDIA driver. Software OpenGL made gzclient unresponsive
# on this computer.
unset LIBGL_ALWAYS_SOFTWARE

echo "PX4_SITL_WORLD=$PX4_SITL_WORLD"
echo "PX4_SITL_START=(${PX4_SITL_X},${PX4_SITL_Y},${PX4_SITL_Z})"
echo "WORLD_EXISTS=$(test -f "$PX4_SITL_WORLD" && echo yes || echo no)"

# Build without the upstream unbounded model-discovery/spawn loop.
cd "$PX4_ROOT"
DONT_RUN=1 make px4_sitl gazebo-classic
set +u
source "$PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
set -u
RUN_DIR="$ROOT/logs/simulation"
mkdir -p "$RUN_DIR"
EXPANDED_MODEL=$(mktemp --suffix=.sdf)
SERVER_PID=""
GUI_PID=""
PX4_PID=""
cleanup() {
  for child in "$PX4_PID" "$GUI_PID" "$SERVER_PID"; do
    if [ -n "$child" ]; then kill "$child" 2>/dev/null || true; fi
  done
  rm -f "$EXPANDED_MODEL"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# Expand includes before spawning; use the actual world name explicitly.
gz sdf -p "$ROOT/uav_competition_ws/src/uav_competition/models/iris/iris.sdf" > "$EXPANDED_MODEL"
WORLD_NAME=$(python3 -c 'import sys,xml.etree.ElementTree as E; print(E.parse(sys.argv[1]).getroot().find("world").get("name"))' "$WORLD")
gzserver --verbose "$WORLD" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > "$RUN_DIR/gzserver.log" 2>&1 &
SERVER_PID=$!
spawned=0
for attempt in $(seq 1 12); do
  kill -0 "$SERVER_PID" || { cat "$RUN_DIR/gzserver.log"; exit 1; }
  if timeout 8s gz model -w "$WORLD_NAME" --spawn-file="$EXPANDED_MODEL" -m iris \
      -x "$PX4_SITL_X" -y "$PX4_SITL_Y" -z "$PX4_SITL_Z" > "$RUN_DIR/spawn.log" 2>&1; then
    spawned=1
    break
  fi
  sleep 1
done
if [ "$spawned" != 1 ]; then cat "$RUN_DIR/spawn.log"; echo 'Iris spawn timed out' >&2; exit 1; fi
if [ "${HEADLESS:-0}" != 1 ]; then
  # Start the GUI in a separate process group. Gazebo Classic can keep the
  # client alive while it is still initializing its Qt/OpenGL window.
  # Set GAZEBO_GUI_WAIT=0 to skip it for headless runs.
  setsid env \
    GAZEBO_MASTER_URI="$GAZEBO_MASTER_URI" \
    GAZEBO_MODEL_DATABASE_URI="$GAZEBO_MODEL_DATABASE_URI" \
    GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH" \
    GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH" \
    GZ_IP="$GZ_IP" \
    QT_X11_NO_MITSHM=1 \
    gzclient --verbose > "$RUN_DIR/gzclient.log" 2>&1 &
  GUI_PID=$!
  # Give the client a short initialization window, but do not make the PX4
  # server depend on GUI startup. The server remains usable if the client
  # driver is unavailable.
  sleep 2
  if ! kill -0 "$GUI_PID" 2>/dev/null; then
    echo "WARNING: gzclient exited; PX4/Gazebo server is still running. See $RUN_DIR/gzclient.log" >&2
  fi
fi
export PATH="$ROOT/config:$PATH"
export PX4_SIM_MODEL=gazebo-classic_iris
mkdir -p "$PX4_ROOT/build/px4_sitl_default/rootfs"
cd "$PX4_ROOT/build/px4_sitl_default/rootfs"
# -d prevents a noninteractive stdin from generating an endless pxh prompt.
if [ "${NO_PXH:-0}" = 1 ]; then
  ../bin/px4 -d ../etc &
  PX4_PID=$!
  wait "$PX4_PID"
else
  ../bin/px4 ../etc
fi
