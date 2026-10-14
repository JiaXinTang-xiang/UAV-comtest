#!/usr/bin/env bash
set -u
printf 'Ubuntu: '; . /etc/os-release; echo "${PRETTY_NAME}"
printf 'ROS_DISTRO: '; echo "${ROS_DISTRO:-not sourced}"
printf 'Gazebo: '; gazebo --version 2>/dev/null | head -1 || echo missing
printf 'colcon: '; command -v colcon || echo missing
printf 'px4_msgs: '; ros2 pkg prefix px4_msgs 2>/dev/null || echo missing
printf 'PX4 source: '; if [ -d "${PX4_SRC_DIR:-$HOME/PX4-Autopilot}" ]; then echo found; else echo missing; fi
printf 'XRCE agent: '; command -v MicroXRCEAgent || echo missing
