#!/usr/bin/env bash
set -euo pipefail
# Run only when network access and permission to install dependencies are available.
ROOT="${PX4_SRC_DIR:-/home/jiaxintang/桌面/bao/PX4-Autopilot}"
if [ ! -d "$ROOT/.git" ]; then git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$ROOT"; fi
cd "$ROOT"
bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo-classic
