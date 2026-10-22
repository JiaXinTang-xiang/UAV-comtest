#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/px4_msgs_overlay_ws/install/setup.bash"
source "$ROOT/uav_competition_ws/install/setup.bash"
case "${1:-}" in
  dropout) ros2 service call /uav/test/sensor_dropout std_srvs/srv/SetBool '{data: true}' ;;
  restore) ros2 service call /uav/test/sensor_dropout std_srvs/srv/SetBool '{data: false}' ;;
  corridor) ros2 service call /uav/test/corridor std_srvs/srv/Trigger '{}' ;;
  geofence) ros2 service call /uav/test/geofence std_srvs/srv/Trigger '{}' ;;
  timeout) ros2 service call /uav/test/timeout std_srvs/srv/Trigger '{}' ;;
  obstacle) python3 "$ROOT/scripts/training_obstacle.py" add "${@:2}" ;;
  remove-obstacle) python3 "$ROOT/scripts/training_obstacle.py" remove "${@:2}" ;;
  *) echo "Usage: $0 dropout|restore|corridor|geofence|timeout|obstacle|remove-obstacle" >&2; exit 2 ;;
esac
