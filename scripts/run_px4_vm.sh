#!/usr/bin/env bash
# Low-resource entry point; the normal competition launcher stays available.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export SIM_PROFILE=vm
# Avoid OpenCV / BLAS spawning more workers than a small VM can schedule.
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENCV_FOR_THREADS_NUM=1
export BUILD_JOBS="${BUILD_JOBS:-1}"
exec bash "$ROOT/scripts/run_px4_competition.sh" "$@"
