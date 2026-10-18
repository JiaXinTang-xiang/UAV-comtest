#!/usr/bin/env bash
set -euo pipefail

# ROS 2 setup scripts intentionally read optional unset variables.  Source
# them with nounset disabled, then restore the strict shell mode.
set +u
source /opt/ros/humble/setup.bash
set -u

echo "等待 PX4 自检完成并进入 READY..."
for _ in $(seq 1 120); do
  state=$(timeout 2s ros2 topic echo /uav/flight/state std_msgs/msg/String --once 2>/dev/null \
    | sed -n 's/^data: //p' | tail -1 || true)
  if [[ "$state" == "READY" ]]; then
    echo "PX4 状态 READY，开始任务。"
    exec ros2 service call /uav/mission/start std_srvs/srv/Trigger '{}'
  fi
  if [[ "$state" == "FAILED" ]]; then
    echo "PX4 已进入 FAILED，停止启动任务。" >&2
    exit 2
  fi
  sleep 1
done

echo "等待 PX4 READY 超时；请检查任务节点终端日志。" >&2
exit 3
